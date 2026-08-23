#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

namespace oscadeval {

// Mirrors _split_by_role: background/show_only bodies are excluded from a
// CSG merge entirely (rather than participating in it), so `!`/`%` isolate
// their subtree correctly even nested inside union/difference/intersection.
//
// ponytail: the reference retags the highlight subset "highlight_ghost" --
// a renderer-only marker meaning "draw this again as a separate translucent
// overlay, in addition to it already being merged into the foreground CSG
// result." This port keeps those bodies tagged plain Highlight instead of
// introducing a 5th BodyRole value purely for that overlay-pass distinction
// -- there's no renderer yet for it to matter to. Revisit if/when a
// consuming renderer needs to tell "merged, also draw a ghost" apart from
// "merged, highlighted, draw once."
RoleSplit splitByRole(const std::vector<ColoredBody>& bodies) {
    RoleSplit r;
    for (const ColoredBody& c : bodies) {
        if (c.role == BodyRole::Background) r.background.push_back(c);
    }
    // Checked before the foreground sweep below and excluded from it: a
    // display-only body has an empty Manifold, so letting it reach a
    // boolean would silently zero the whole operation (the same failure
    // mode invalid operands caused before they were filtered out).
    for (const ColoredBody& c : bodies) {
        if (c.isDisplayOnly() && c.role != BodyRole::Background && c.role != BodyRole::ShowOnly) {
            r.displayOnly.push_back(c);
        }
    }
    for (const ColoredBody& c : bodies) {
        if (c.role != BodyRole::Background && c.role != BodyRole::ShowOnly && !c.isDisplayOnly()) {
            r.foreground.push_back(c);
        }
    }
    for (const ColoredBody& c : r.foreground) {
        if (c.role == BodyRole::Highlight) r.highlight.push_back(c);
    }
    for (const ColoredBody& c : bodies) {
        if (c.role == BodyRole::ShowOnly) r.showOnly.push_back(c);
    }
    return r;
}

// Unions every 2D child into a single CrossSection; nullopt if there are no
// 2D children at all. Mirrors _to_cross_section. Shared by
// linear_extrude/rotate_extrude/roof (extrude.cpp, roof.cpp).
std::optional<manifold::CrossSection> toCrossSection(const std::vector<ColoredBody>& bodies) {
    std::optional<manifold::CrossSection> cs;
    for (const ColoredBody& b : bodies) {
        if (!b.section) continue;
        cs = cs ? (*cs + *b.section) : *b.section;
    }
    return cs;
}

// union()/difference()/intersection() -- evaluates each top-level geometry
// statement separately (group_sizes) so their body groups are preserved:
// for difference(), all bodies from the FIRST statement form the positive
// operand (unioned together), and each subsequent statement's bodies are
// unioned then subtracted -- a flat evaluation would lose this grouping and
// misbehave when e.g. BOSL2's attachable() returns multiple bodies (parent
// + attached children) as one operand.
//
// One statement can still turn into several: children(separate=true) is
// expanded into one `children(k)` statement per child it forwards, before
// this loop ever sees the block (Evaluator::expandChildStatements), which is
// how `difference() children(separate=true)` subtracts children 1..n from
// child 0. Nothing special happens here -- they are simply statements. Mirrors _resolve_csg/_generate_csg,
// minus _attach_tri_colors' multi-color-merge provenance (ponytail: a
// merged body always takes its first contributing child's color, matching
// this evaluator's own pre-tri_colors baseline behavior -- revisit
// alongside real ManifoldCache/provenance work if per-triangle color
// through a merge is needed).

CSGParams resolveCsg(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    const std::string& op = node.name->name;
    // union/difference/intersection take no positional arguments in real
    // OpenSCAD, but this still routes through resolveCallArgs (not a bare
    // no-op) so a $-prefixed named arg (e.g. `difference($fn=8) {...}`,
    // unusual but legal) still propagates into children the same way every
    // other builtin honors it.
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    (void)args;

    std::vector<const oscad::ASTNode*> assignNodes;
    std::vector<const oscad::ASTNode*> geoNodes;
    for (const oscad::ASTNode* c : ev.expandChildStatements(node.children, effCtx)) {
        if (c->kind() == oscad::NodeKind::Assignment) {
            assignNodes.push_back(c);
        } else if (c->kind() != oscad::NodeKind::ModuleDeclaration && c->kind() != oscad::NodeKind::FunctionDeclaration) {
            geoNodes.push_back(c);
        }
    }
    if (!assignNodes.empty()) ev.evalChildren(assignNodes, effCtx);

    std::vector<Value> groupSizes;
    groupSizes.reserve(geoNodes.size());
    for (const oscad::ASTNode* geoNode : geoNodes) {
        const size_t before = ev.currentTreeFrameSize();
        ev.evalChildren(std::vector<const oscad::ASTNode*>{geoNode}, effCtx);
        groupSizes.push_back(Value{static_cast<double>(ev.currentTreeFrameSize() - before)});
    }

    CSGParams params;
    params["op"] = Value{op};
    params["group_sizes"] = Value{std::make_shared<const ValueList>(ValueList{std::move(groupSizes)})};
    return params;
}

namespace {

// Matches the reference's own SceneRenderer._default_color -- shown for
// geometry with no explicit color() override. ColoredBody::color normally
// stays nullopt for uncolored geometry so a live renderer can resolve it
// against its own color theme, but a per-triangle triColors override has
// no such live-resolution mechanism, so a concrete fallback is needed here
// specifically.
constexpr std::array<float, 4> kDefaultGeometryColor{0.9f, 0.85f, 0.1f, 1.0f};

// After a real boolean merge, `cb.color` is just one arbitrary child's
// color (the first contributing operand above) -- every other child's own
// color is otherwise lost, e.g. union()-ing an opaque cube with a
// translucent sphere silently rendered the whole result fully opaque.
// Manifold preserves per-triangle provenance through boolean ops via each
// merged mesh's own runOriginalID/runIndex (already relied on for
// idToNode/WYSIWYG ray-cast picking -- see tagGenerated()); this reuses
// the same mechanism to recover each triangle's real originating color
// from ev.idToColor (populated by tagGenerated() when each child was
// itself first generated, before being merged away). If every triangle
// resolves to the same color, this is a no-op (leaves triColors unset) --
// the common single-material case pays no extra cost and keeps following
// live color-theme changes for uncolored geometry, same as before this
// existed. Mirrors _attach_tri_colors.
void attachTriColors(Evaluator& ev, ColoredBody& cb) {
    if (!cb.body) return;
    const manifold::MeshGL mesh = cb.body->GetMeshGL();
    const std::vector<uint32_t>& runIds = mesh.runOriginalID;
    const std::vector<uint32_t>& runIdx = mesh.runIndex;
    const size_t numTris = mesh.triVerts.size() / 3;
    if (numTris == 0 || runIds.size() <= 1) return;

    std::vector<std::optional<std::array<float, 4>>> perRunColor;
    perRunColor.reserve(runIds.size());
    for (uint32_t rid : runIds) {
        auto it = ev.idToColor.find(rid);
        perRunColor.push_back(it != ev.idToColor.end() ? it->second : cb.color);
    }
    const bool allSame =
        std::all_of(perRunColor.begin(), perRunColor.end(), [&](const auto& c) { return c == perRunColor.front(); });
    if (allSame) return;

    std::vector<std::array<float, 4>> triColors(numTris, kDefaultGeometryColor);
    for (size_t i = 0; i + 1 < runIdx.size(); ++i) {
        const size_t start = runIdx[i] / 3;
        const size_t end = std::min<size_t>(runIdx[i + 1] / 3, numTris);
        if (start >= numTris) continue;
        const std::array<float, 4> color = perRunColor[i].value_or(kDefaultGeometryColor);
        for (size_t t = start; t < end; ++t) triColors[t] = color;
    }
    cb.triColors = std::move(triColors);
}

} // namespace

std::vector<ColoredBody> generateCsg(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>& children,
                                      const oscad::ASTNode&) {
    const std::string& op = std::get<std::string>(params.at("op"));
    const auto& groupSizes = std::get<ListPtr>(params.at("group_sizes"))->items;

    std::vector<ColoredBody> allBg, allHi, allSo, allDo;
    std::optional<ColoredBody> csgResult;
    size_t idx = 0;

    for (const Value& sizeVal : groupSizes) {
        const size_t size = static_cast<size_t>(std::get<double>(sizeVal));
        std::vector<ColoredBody> stmtBodies = flattenCsgTree(children, idx, size);
        idx += size;

        RoleSplit split = splitByRole(stmtBodies);
        allBg.insert(allBg.end(), split.background.begin(), split.background.end());
        allHi.insert(allHi.end(), split.highlight.begin(), split.highlight.end());
        allSo.insert(allSo.end(), split.showOnly.begin(), split.showOnly.end());
        allDo.insert(allDo.end(), split.displayOnly.begin(), split.displayOnly.end());

        std::vector<ColoredBody> bodies3d, sections2d;
        for (const ColoredBody& c : split.foreground) {
            // A body whose own Manifold::Status() isn't NoError (e.g.
            // NonFiniteVertex, from a degenerate accumulated transform deep
            // in an unrelated ancestor's positioning math -- found via a
            // real user script, snappy-reprap's z_tower_assembly chain,
            // where one already-invalid sub-part silently zeroed out an
            // entire 65-operand union of otherwise-valid geometry) must
            // never reach Manifold's own `+`/`-`/`^` operators: unlike a
            // genuinely empty operand (0 triangles, NoError), Manifold
            // propagates a non-NoError status through boolean ops onto the
            // WHOLE result instead of treating it as a no-op contributor,
            // so a single bad part silently discards every valid sibling.
            // Real OpenSCAD's CGAL backend doesn't hit this at all here
            // (more robust to the same input) -- dropping the one invalid
            // operand and unioning everything else is the closest match to
            // its own behavior available without new numerical-robustness
            // work on Manifold's own boolean ops.
            if (c.body && c.body->Status() == manifold::Manifold::Error::NoError) bodies3d.push_back(c);
        }
        for (const ColoredBody& c : split.foreground) {
            if (c.section) sections2d.push_back(c);
        }

        if (bodies3d.empty() && sections2d.empty()) {
            // Empty statement: intersection(empty, B) discards any
            // csg_result already built; difference(empty, B) only
            // discards while no positive operand has been established yet;
            // union just skips the empty contributor and keeps going.
            if (op == "intersection") {
                csgResult.reset();
                break;
            }
            if (op == "difference" && !csgResult) break;
            continue;
        }

        if (!bodies3d.empty()) {
            manifold::Manifold grp = *bodies3d.front().body;
            for (size_t i = 1; i < bodies3d.size(); ++i) grp = grp + *bodies3d[i].body;
            if (!csgResult) {
                ColoredBody cb;
                cb.body = std::move(grp);
                cb.color = bodies3d.front().color;
                csgResult = std::move(cb);
            } else if (op == "union") {
                csgResult->body = *csgResult->body + grp;
            } else if (op == "difference") {
                csgResult->body = *csgResult->body - grp;
            } else if (op == "intersection") {
                csgResult->body = *csgResult->body ^ grp;
            }
        } else {
            manifold::CrossSection grp = *sections2d.front().section;
            for (size_t i = 1; i < sections2d.size(); ++i) grp = grp + *sections2d[i].section;
            if (!csgResult) {
                ColoredBody cb;
                cb.section = std::move(grp);
                cb.color = sections2d.front().color;
                csgResult = std::move(cb);
            } else if (op == "union") {
                csgResult->section = *csgResult->section + grp;
            } else if (op == "difference") {
                csgResult->section = *csgResult->section - grp;
            } else if (op == "intersection") {
                csgResult->section = *csgResult->section ^ grp;
            }
        }
    }

    if (csgResult && csgResult->body) attachTriColors(ev, *csgResult);

    std::vector<ColoredBody> result;
    if (csgResult) result.push_back(std::move(*csgResult));
    result.insert(result.end(), allBg.begin(), allBg.end());
    result.insert(result.end(), allHi.begin(), allHi.end());
    result.insert(result.end(), allSo.begin(), allSo.end());
    result.insert(result.end(), allDo.begin(), allDo.end());
    return result;
}

} // namespace oscadeval
