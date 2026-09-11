#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <unordered_map>

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

    // ONE scope around both passes -- see Evaluator::blockScope. The
    // assignment pass writes into the same block scope the geometry pass
    // then reads from, so it cannot be scoped per-pass.
    EvalContext blockCtx = ev.blockScope(effCtx);

    std::vector<const oscad::ASTNode*> assignNodes;
    std::vector<const oscad::ASTNode*> geoNodes;
    for (const oscad::ASTNode* c : ev.expandChildStatements(node.children, blockCtx)) {
        if (c->kind() == oscad::NodeKind::Assignment) {
            assignNodes.push_back(c);
        } else if (c->kind() != oscad::NodeKind::ModuleDeclaration && c->kind() != oscad::NodeKind::FunctionDeclaration) {
            geoNodes.push_back(c);
        }
    }
    if (!assignNodes.empty()) ev.evalChildren(assignNodes, blockCtx);

    std::vector<Value> groupSizes;
    std::vector<Value> emptyIsAGroup;
    groupSizes.reserve(geoNodes.size());
    emptyIsAGroup.reserve(geoNodes.size());
    for (const oscad::ASTNode* geoNode : geoNodes) {
        const size_t before = ev.currentTreeFrameSize();
        ev.evalChildren(std::vector<const oscad::ASTNode*>{geoNode}, blockCtx);
        groupSizes.push_back(Value{static_cast<double>(ev.currentTreeFrameSize() - before)});
        // Whether a statement that produced NOTHING still counts as an
        // operand.
        //
        // The reference builds a node for a module instantiation and for a
        // loop whatever they contain, so an empty one is an empty operand
        // and annihilates an intersection. An `if` that takes no branch, a
        // bare block, and a `*`-disabled statement build no node at all, so
        // they are not operands and must be skipped.
        //
        // Every line of this was checked against OpenSCAD rather than
        // reasoned about -- `intersection(){ cube(10); X; }` keeps the cube
        // for `if(false) sphere(6)`, `{ }` and `*cube(1)`, and comes out
        // empty for `for(i=[1:0]) sphere(6)`, `union(){}`, `group(){}` and
        // a call to an empty user module.
        const oscad::NodeKind k = geoNode->kind();
        emptyIsAGroup.push_back(Value{
                                       k == oscad::NodeKind::ModularCall ||
                                       k == oscad::NodeKind::ModularFor ||
                                       k == oscad::NodeKind::ModularIntersectionFor ||
                                       k == oscad::NodeKind::ModularLet ||
                                       k == oscad::NodeKind::ModularModifierShowOnly ||
                                       k == oscad::NodeKind::ModularModifierHighlight ||
                                       k == oscad::NodeKind::ModularModifierBackground});
    }

    CSGParams params;
    params["op"] = Value{op};
    params["group_sizes"] = Value{std::make_shared<const ValueList>(ValueList{std::move(groupSizes)})};
    params["empty_is_a_group"] =
        Value{std::make_shared<const ValueList>(ValueList{std::move(emptyIsAGroup)})};
    return params;
}

namespace {

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
        // nullopt, NOT cb.color, when a run has no recorded colour of its
        // own. cb.color is the FIRST child's colour, so falling back to it
        // made an uncoloured child look identically coloured, `allSame`
        // came out true, and the whole merge kept one colour -- which is
        // the "union() { color("red") a; b; } is all red" bug. nullopt is
        // what an uncoloured body means everywhere else: follow the theme.
        perRunColor.push_back(it != ev.idToColor.end() ? it->second : std::nullopt);
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

// keepMinuendColor: the leaf parts of a minuend operand -- the operand
// itself, or what a union() merged it from, recursively.
void collectKeepParts(const ColoredBody& b, std::vector<ColoredBody>& out) {
    if (b.mergedFrom) {
        for (const ColoredBody& part : *b.mergedFrom) collectKeepParts(part, out);
    } else if (b.body) {
        out.push_back(b);
    }
}

// keepMinuendColor: one minuend part, differenced on its own.
struct KeepPart {
    ColoredBody body;
    std::vector<uint32_t> ownIds;   // run IDs the part was born with; anything else on it afterwards is a cut face
};

std::vector<uint32_t> runIdsOf(const ColoredBody& b) {
    const int original = b.body->OriginalID();
    if (original >= 0) return {static_cast<uint32_t>(original)};
    return b.body->GetMeshGL().runOriginalID;
}

// Union the per-part differences back into one body, first re-minting each
// part's cut-face runs (the subtrahend's IDs, shared by every part's result)
// under fresh IDs carrying THAT part's colour, so attachTriColors can tell
// one part's cut faces from another's. The source node stays the
// subtrahend's: clicking a cut face still finds the tool that made it.
manifold::Manifold finishKeepMinuend(Evaluator& ev, std::vector<KeepPart>& parts) {
    std::optional<manifold::Manifold> out;
    for (KeepPart& part : parts) {
        if (!part.body.body) continue;
        manifold::MeshGL mesh = part.body.body->GetMeshGL();
        if (mesh.triVerts.empty()) continue;
        std::unordered_map<uint32_t, uint32_t> remap;
        for (uint32_t& id : mesh.runOriginalID) {
            if (std::find(part.ownIds.begin(), part.ownIds.end(), id) != part.ownIds.end()) continue;
            auto found = remap.find(id);
            if (found == remap.end()) {
                const uint32_t fresh = manifold::Manifold::ReserveIDs(1);
                auto node = ev.idToNode.find(id);
                if (node != ev.idToNode.end()) ev.idToNode[fresh] = node->second;
                // ponytail: a part that is itself a multi-colour merge
                // gives its cut faces its first child's colour rather than
                // the colour of whichever child the cut passed through.
                ev.idToColor[fresh] = part.body.color;
                found = remap.emplace(id, fresh).first;
            }
            id = found->second;
        }
        manifold::Manifold rebuilt(mesh);
        out = out ? *out + rebuilt : rebuilt;
    }
    return out.value_or(manifold::Manifold());
}

// One colour's worth of 2D result.
//
// 3D recovers per-child colour AFTER the merge, from Manifold's own
// per-triangle provenance (attachTriColors above). A CrossSection has no
// such thing -- it is contours, not a mesh, and nothing in it remembers
// which child a given edge came from. So 2D has to keep colour
// GEOMETRICALLY instead: one part per colour, each notched by whatever is
// drawn over it later. That is the same painter's-order rule
// splitBodiesForExport applies to overlapping 3D solids, and it is what
// `union() { color("red") square(10); color("blue") ... }` needs to come
// out as two coloured shapes instead of one shape wearing the first
// child's colour.
struct Part2d {
    std::optional<std::array<float, 4>> color;
    manifold::CrossSection section;
};

// Appends `add` to `parts`, merging into an existing part of the same
// colour so `union() { color("red") a; color("red") b; }` stays one body.
void addPart(std::vector<Part2d>& parts, const std::optional<std::array<float, 4>>& color,
             const manifold::CrossSection& section) {
    for (Part2d& p : parts) {
        if (p.color == color) {
            p.section = p.section + section;
            return;
        }
    }
    parts.push_back({color, section});
}

// The 2D operands of one statement, grouped by colour.
std::vector<Part2d> partsOf(const std::vector<ColoredBody>& sections2d) {
    std::vector<Part2d> parts;
    for (const ColoredBody& c : sections2d) addPart(parts, c.color, *c.section);
    return parts;
}

// Every part's section unioned -- the shape the statement covers,
// whatever colours it is in.
manifold::CrossSection coveredBy(const std::vector<Part2d>& parts) {
    manifold::CrossSection all;
    for (const Part2d& p : parts) all = all + p.section;
    return all;
}

void dropEmptyParts(std::vector<Part2d>& parts) {
    parts.erase(std::remove_if(parts.begin(), parts.end(),
                                [](const Part2d& p) { return p.section.IsEmpty(); }),
                parts.end());
}

} // namespace

std::vector<ColoredBody> generateCsg(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>& children,
                                      const oscad::ASTNode&) {
    const std::string& op = std::get<std::string>(params.at("op"));
    const auto& groupSizes = std::get<ListPtr>(params.at("group_sizes"))->items;
    const auto emptyIsAGroupIt = params.find("empty_is_a_group");
    const std::vector<Value>* emptyIsAGroup =
        emptyIsAGroupIt == params.end()
            ? nullptr
            : &std::get<ListPtr>(emptyIsAGroupIt->second)->items;

    std::vector<ColoredBody> allBg, allHi, allSo, allDo;
    // Two accumulators, one per dimension. union() may legitimately carry
    // both -- `translate(...) { part(); text("label"); }` is how a docs
    // figure annotates a 3D part, and the reference draws both -- and a
    // single optional that switched between body and section is exactly
    // what used to be dereferenced empty when a group changed dimension
    // part-way. difference and intersection still see one dimension only:
    // applyDimensionRules drops the mismatch before it gets here, so for
    // them just one of these is ever populated.
    std::optional<ColoredBody> res3d;
    // 2D keeps one entry per colour rather than one merged section; see
    // Part2d. `have2d` distinguishes "no 2D operand yet" from "every 2D
    // part has been cut away", which matters to difference().
    std::vector<Part2d> res2d;
    bool have2d = false;
    size_t idx = 0;
    // Whether the 3D operands could possibly disagree on colour. Only then
    // is attachTriColors worth its GetMeshGL(), which evaluates the boolean
    // and copies the whole mesh out -- at EVERY level of a nested union
    // chain, when nothing but the top ever needed it. Operands that all
    // carry the same colour and no per-triangle colours of their own can
    // only produce a uniformly coloured result.
    std::optional<std::optional<std::array<float, 4>>> firstColor;
    bool mixedColors = false;
    // keepMinuendColor: the minuend's parts, each differenced on its own
    // (see Evaluator::keepMinuendColor). Not while measuring: a render()
    // expression only wants the volume, which is the same either way.
    const bool keeping = op == "difference" && ev.keepMinuendColor && !ev.measuring();
    std::vector<KeepPart> keepParts;
    // ... and a union() built under that mode remembers what it merged, so
    // a difference() it is the minuend of can cut each part on its own.
    const bool rememberParts = op == "union" && ev.keepMinuendColor && !ev.measuring();
    std::vector<ColoredBody> unionParts;

    size_t stmtIndex = 0;
    for (const Value& sizeVal : groupSizes) {
        const size_t size = static_cast<size_t>(std::get<double>(sizeVal));
        const size_t thisStmt = stmtIndex++;
        std::vector<ColoredBody> stmtBodies = flattenCsgTree(children, idx, size);
        idx += size;

        // A statement that built no CSG node at all is not an operand --
        // an `if` whose branch was not taken, or an empty block. Only a
        // for-loop still counts as one when it produced nothing, matching
        // the group node the reference builds for it regardless. Without
        // this, `intersection(){ X; if (crop) Y; }` -- BOSL2's hirth()
        // among others -- came out empty whenever the condition was false.
        if (size == 0) {
            const bool isGroup = emptyIsAGroup && thisStmt < emptyIsAGroup->size() &&
                                  std::holds_alternative<bool>((*emptyIsAGroup)[thisStmt]) &&
                                  std::get<bool>((*emptyIsAGroup)[thisStmt]);
            if (!isGroup) continue;
        }

        RoleSplit split = splitByRole(stmtBodies);
        allBg.insert(allBg.end(), split.background.begin(), split.background.end());
        allHi.insert(allHi.end(), split.highlight.begin(), split.highlight.end());
        allSo.insert(allSo.end(), split.showOnly.begin(), split.showOnly.end());
        allDo.insert(allDo.end(), split.displayOnly.begin(), split.displayOnly.end());

        std::vector<ColoredBody> bodies3d, sections2d;
        for (ColoredBody& c : split.foreground) {
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
            if (c.body && bodyStatus(c) == manifold::Manifold::Error::NoError) bodies3d.push_back(c);
        }
        for (const ColoredBody& c : bodies3d) {
            if (c.triColors) mixedColors = true;
            if (!firstColor) firstColor = c.color;
            else if (*firstColor != c.color) mixedColors = true;
        }
        // A subtrahend with no colour of its own paints the faces it
        // exposes the cut green, as the reference does, rather than the
        // default geometry colour -- otherwise a cut through an uncoloured
        // part is invisible as a cut. Recorded against its runs, which is
        // all attachTriColors will have left after the merge; and the
        // merge must then look, even when every operand's own colour
        // agrees.
        if (op == "difference" && res3d && !ev.measuring()) {
            for (ColoredBody& c : bodies3d) {
                if (c.color || c.triColors) continue;
                ev.recordRunColors(c, kCutFaceColor);
                mixedColors = true;
            }
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
                res3d.reset();
                res2d.clear();
                have2d = false;
                break;
            }
            if (op == "difference" && !res3d && !have2d) break;
            continue;
        }

        // Each dimension accumulates on its own, so a statement holding
        // both contributes to both rather than the 2D half falling off the
        // end of an if/else.
        if (!bodies3d.empty()) {
            manifold::Manifold grp = *bodies3d.front().body;
            for (size_t i = 1; i < bodies3d.size(); ++i) grp = grp + *bodies3d[i].body;
            // The result stays LAZY inside Manifold: its emptiness is derived
            // from the operands' flags here rather than asked, so a chain of
            // unions under transforms is evaluated once, at the top, instead
            // of once per level (see ColoredBody::knownEmpty).
            const std::optional<bool> grpEmpty =
                unionEmptyOf(bodies3d, [](const ColoredBody& b) -> const ColoredBody& { return b; });
            const auto known = [](const std::optional<bool>& e, bool v) { return e && *e == v; };
            if (!res3d) {
                ColoredBody cb;
                cb.body = std::move(grp);
                cb.color = bodies3d.front().color;
                cb.knownStatus = manifold::Manifold::Error::NoError; // every operand was
                cb.knownEmpty = grpEmpty;
                res3d = std::move(cb);
                if (keeping) {
                    std::vector<ColoredBody> leaves;
                    for (const ColoredBody& c : bodies3d) collectKeepParts(c, leaves);
                    for (ColoredBody& leaf : leaves) {
                        std::vector<uint32_t> ids = runIdsOf(leaf);
                        keepParts.push_back({std::move(leaf), std::move(ids)});
                    }
                }
                if (rememberParts) {
                    for (const ColoredBody& c : bodies3d) collectKeepParts(c, unionParts);
                }
            } else if (keeping) {
                // The whole-minuend result above is never evaluated (Manifold
                // is lazy); finishKeepMinuend replaces it after the loop.
                for (KeepPart& part : keepParts) part.body.body = *part.body.body - grp;
                res3d->knownEmpty.reset();
            } else if (op == "union") {
                res3d->body = *res3d->body + grp;
                if (rememberParts) {
                    for (const ColoredBody& c : bodies3d) collectKeepParts(c, unionParts);
                }
                if (known(res3d->knownEmpty, false) || known(grpEmpty, false)) res3d->knownEmpty = false;
                else if (known(res3d->knownEmpty, true) && known(grpEmpty, true)) res3d->knownEmpty = true;
                else res3d->knownEmpty.reset();
            } else if (op == "difference") {
                res3d->body = *res3d->body - grp;
                if (known(res3d->knownEmpty, true)) res3d->knownEmpty = true;
                else if (!known(grpEmpty, true)) res3d->knownEmpty.reset(); // subtracting nothing changes nothing
            } else if (op == "intersection") {
                res3d->body = *res3d->body ^ grp;
                if (known(res3d->knownEmpty, true) || known(grpEmpty, true)) res3d->knownEmpty = true;
                else res3d->knownEmpty.reset();
            }
        }
        if (!sections2d.empty()) {
            std::vector<Part2d> stmtParts = partsOf(sections2d);
            const manifold::CrossSection grp = coveredBy(stmtParts);
            if (!have2d) {
                res2d = std::move(stmtParts);
                have2d = true;
            } else if (op == "union") {
                // Later wins the overlap: notch what is already there by
                // everything this statement covers, then add this
                // statement's parts. Same rule as two overlapping 3D
                // solids of different colours, and the same thing a
                // painter does.
                for (Part2d& p : res2d) p.section = p.section - grp;
                for (Part2d& np : stmtParts) addPart(res2d, np.color, np.section);
            } else if (op == "difference") {
                for (Part2d& p : res2d) p.section = p.section - grp;
            } else if (op == "intersection") {
                for (Part2d& p : res2d) p.section = p.section ^ grp;
            }
            dropEmptyParts(res2d);
        }
    }

    if (res3d && res3d->body && rememberParts && unionParts.size() > 1) {
        res3d->mergedFrom = std::make_shared<const std::vector<ColoredBody>>(std::move(unionParts));
    }
    if (res3d && res3d->body && keeping && !keepParts.empty()) {
        res3d->body = finishKeepMinuend(ev, keepParts);
        res3d->knownEmpty.reset();
        mixedColors = true;   // attachTriColors still no-ops when every run agrees
    }
    if (res3d && res3d->body && mixedColors) attachTriColors(ev, *res3d);

    std::vector<ColoredBody> result;
    if (res3d) result.push_back(std::move(*res3d));
    for (Part2d& p : res2d) {
        ColoredBody cb;
        cb.color = p.color;
        cb.section = std::move(p.section);
        result.push_back(std::move(cb));
    }
    result.insert(result.end(), allBg.begin(), allBg.end());
    result.insert(result.end(), allHi.begin(), allHi.end());
    result.insert(result.end(), allSo.begin(), allSo.end());
    result.insert(result.end(), allDo.begin(), allDo.end());
    return result;
}

} // namespace oscadeval
