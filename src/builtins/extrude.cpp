#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <manifold/polygon.h>
#include <cmath>
#include <numbers>

namespace oscadeval {

// linear_extrude(height=1, center=false, twist=0, slices=0, scale=1) --
// mirrors _resolve_linear_extrude/_generate_linear_extrude. `scale` is
// either a single number (applied to both X/Y) or a [x,y] pair for the top
// face; the bottom face is always full-size.
//
// Split into computeLinearExtrudeParams (pure, no children-evaluation side
// effect) + resolveLinearExtrude (adds the evalChildren call) so Op::
// PushBuiltinWrap's own runtime handler (bytecode_vm.cpp) can call the pure
// half directly instead of going through the whole resolve function --
// exactly the same split computeTransformParams/computeColorParams already
// use, extended to this builtin. Params computation here has no observable
// side effect of its own (no warn()/echo()/rands() call), so moving it
// before evalChildren (unlike this function's own former single-body
// shape, which computed params AFTER children like every other builtin
// still not covered by PushBuiltinWrap) is behaviorally unobservable --
// see computeRoofParams's own doc comment (roof.cpp) for the one builtin
// in this group where that ISN'T true and a different split was needed.
namespace {

// Upstream's sgn_vdiff: which of two diagonals is shorter, treating lengths
// within 1 part in 1e5 as a tie.
int sgnVdiff(const manifold::vec2& v1, const manifold::vec2& v2) {
    const double l1 = manifold::la::length(v1), l2 = manifold::la::length(v2);
    return 2 * std::fabs(l1 - l2) * 1e5 > l1 + l2 ? (l1 < l2 ? -1 : 1) : 0;
}

// A twisted or non-uniformly scaled extrusion, built the way upstream's
// linear_extrude.cc builds it. Its side quads are not planar, so which
// diagonal splits each one changes the solid, and Manifold's own Extrude
// always picks the same one: the result had OpenSCAD's triangle count and a
// different volume (241 vs 254 for a 180-degree twist of square(5)).
// Upstream splits along the SHORTER diagonal, breaking exact ties by twist
// direction and whether the outline is a hole.
manifold::Manifold extrudeTwisted(const manifold::Polygons& polys, double height, int slices, double twist,
                                  double scaleX, double scaleY) {
    size_t stride = 0;
    for (const manifold::SimplePolygon& o : polys) stride += o.size();
    const auto ringPoint = [&](const manifold::vec2& v, int j) {
        const double t = static_cast<double>(j) / slices;
        const double a = -twist * t * std::numbers::pi / 180.0;
        return manifold::vec2((v.x * std::cos(a) - v.y * std::sin(a)) * (1 - (1 - scaleX) * t),
                              (v.x * std::sin(a) + v.y * std::cos(a)) * (1 - (1 - scaleY) * t));
    };

    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    for (int j = 0; j <= slices; ++j) {
        for (const manifold::SimplePolygon& o : polys) {
            for (const manifold::vec2& v : o) {
                const manifold::vec2 p = ringPoint(v, j);
                mesh.vertProperties.insert(mesh.vertProperties.end(),
                                           {p.x, p.y, height * j / slices});
            }
        }
    }
    const auto tri = [&](size_t a, size_t b, size_t c) { mesh.triVerts.insert(mesh.triVerts.end(), {a, b, c}); };

    const bool backTwist = twist <= 0; // rotation_slice_top <= rotation_slice_bottom, every slice
    // A top scaled to zero in either axis collapses the last ring to a line
    // or a point. Upstream reverses its shorter-diagonal choice on that last
    // slice (splitfirst xor any_zero), to avoid zero-thickness "ears", and
    // has no top cap to add.
    const bool anyZero = scaleX == 0.0 || scaleY == 0.0;
    for (int j = 1; j <= slices; ++j) {
        const size_t bot = (j - 1) * stride, top = j * stride;
        const bool flipForZero = anyZero && j == slices;
        size_t cur = 0;
        for (const manifold::SimplePolygon& o : polys) {
            const size_t n = o.size();
            double area2 = 0;
            for (size_t i = 0; i < n; ++i) area2 += o[i].x * o[(i + 1) % n].y - o[(i + 1) % n].x * o[i].y;
            const bool flip = (area2 < 0) != backTwist; // !positive xor back_twist
            manifold::vec2 prevBot = ringPoint(o[0], j - 1), prevTop = ringPoint(o[0], j);
            for (size_t i = 1; i <= n; ++i) {
                const manifold::vec2 vBot = ringPoint(o[i % n], j - 1), vTop = ringPoint(o[i % n], j);
                const size_t idx = cur + i % n, prev = cur + i - 1;
                const int diff = sgnVdiff(prevBot - vTop, vBot - prevTop);
                if ((diff == -1 || (diff == 0 && !flip)) != flipForZero) {
                    tri(bot + idx, top + idx, bot + prev);
                    tri(top + prev, bot + prev, top + idx);
                } else {
                    tri(bot + idx, top + prev, bot + prev);
                    tri(bot + idx, top + idx, top + prev);
                }
                prevBot = vBot;
                prevTop = vTop;
            }
            cur += n;
        }
    }

    // Caps: one triangulation of the outline, reused at both ends.
    manifold::PolygonsIdx indexed;
    size_t cur = 0;
    for (const manifold::SimplePolygon& o : polys) {
        manifold::SimplePolygonIdx ring;
        for (size_t i = 0; i < o.size(); ++i) ring.push_back({o[i], static_cast<int>(cur + i)});
        indexed.push_back(std::move(ring));
        cur += o.size();
    }
    const size_t topBase = static_cast<size_t>(slices) * stride;
    // A collapsed top keeps its cap: zero-area triangles, but they close the
    // mesh topologically, and Manifold collapses degenerate triangles itself.
    // (Upstream simply skips the cap; its PolySet need not be manifold.)
    for (const manifold::ivec3& t : manifold::TriangulateIdx(indexed)) {
        tri(t.x, t.z, t.y);                                    // bottom faces down
        tri(topBase + t.x, topBase + t.y, topBase + t.z);      // top faces up
    }
    return manifold::Manifold(mesh);
}

} // namespace

BuiltinWrapParams computeLinearExtrudeParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);

    // Positional order is upstream's: height, v, scale, center, twist,
    // slices, segments. Only height used to be read positionally.
    //
    // Upstream's LinearExtrudeNode: the extrusion runs along a vector, (0,0,1)
    // unless `v` gives one. Its length is `height` (`h` its alias, BOSL2's
    // override forwards it) when given -- `v` then only sets the direction --
    // else |v| when `v` is given, else 100. A vector pointing down (z <= 0)
    // extrudes nothing. Bad values warn and fall back as upstream's do.
    // `v` was ignored here, so an oblique extrusion came out straight.
    double hv[3] = {0.0, 0.0, 1.0};
    double length = 100.0;
    const Value vArg = getArg(args, 1, "v", Value{});
    if (!std::holds_alternative<std::monostate>(vArg)) {
        const ListPtr* l = std::get_if<ListPtr>(&vArg);
        bool ok = l && *l && (*l)->items.size() == 3;
        double t[3] = {0, 0, 0};
        for (int i = 0; ok && i < 3; ++i) {
            const double* d = std::get_if<double>(&(*l)->items[i]);
            ok = d && std::isfinite(*d);
            if (ok) t[i] = *d;
        }
        if (ok) std::copy(t, t + 3, hv);
        else ev.warn("v when specified should be a 3d vector", &node.position());
        length = 1.0;
    }
    const Value heightArg = getArgOrAlias(ev, &node.position(), args, 0, "height", "h", Value{});
    if (!std::holds_alternative<std::monostate>(heightArg)) {
        const double* d = std::get_if<double>(&heightArg);
        if (d && std::isfinite(*d)) {
            length = *d;
        } else {
            ev.warn("height when specified should be a number", &node.position());
            length = 100.0;
        }
        const double norm = std::sqrt(hv[0] * hv[0] + hv[1] * hv[1] + hv[2] * hv[2]);
        for (double& c : hv) c /= norm;
    }
    for (double& c : hv) c *= length;
    if (hv[2] <= 0) hv[2] = 0;
    const double height = hv[2];
    // Only a real boolean counts, as upstream: center="yes" is not true.
    const Value centerArg = getArg(args, 3, "center", Value{false});
    const bool* centerBool = std::get_if<bool>(&centerArg);
    const bool center = centerBool && *centerBool;
    const double twist = toDoubleLenient(getArg(args, 4, "twist", Value{0.0}));
    // Upstream's validate_integral: any finite number counts as given, and is
    // truncated and clamped (slices >= 1, segments >= 0). Not given, the
    // discretizer decides.
    const auto integral = [&](int pos, const char* name, double lo) -> std::optional<double> {
        const Value v = getArg(args, pos, name, Value{});
        const double* d = std::get_if<double>(&v);
        if (!d || !std::isfinite(*d)) return std::nullopt;
        return *d < lo ? lo : std::trunc(*d);
    };
    const std::optional<double> slices = integral(5, "slices", 1.0);
    const std::optional<double> segments = integral(6, "segments", 0.0);
    const Value scaleArg = getArg(args, 2, "scale", Value{});

    double scaleX = 1.0, scaleY = 1.0;
    bool scaleOk = true;
    if (const double* s = std::get_if<double>(&scaleArg)) {
        scaleX = scaleY = *s;
        scaleOk = std::isfinite(*s);
    } else if (const ListPtr* l = std::get_if<ListPtr>(&scaleArg); l && *l && (*l)->items.size() == 2) {
        // Exactly two, as Value::getVec2 takes them: a 3-vector is the
        // "could not be converted" case below, not its first two.
        scaleX = toDoubleLenient((*l)->items[0]);
        scaleY = toDoubleLenient((*l)->items[1]);
        scaleOk = std::isfinite(scaleX) && std::isfinite(scaleY);
    } else if (!std::holds_alternative<std::monostate>(scaleArg)) {
        scaleOk = false;
    }
    if (!scaleOk) {
        ev.warn("linear_extrude(..., scale=" + fmtValue(scaleArg) + ") could not be converted", &node.position());
        scaleX = scaleY = 1.0;
    }

    CSGParams params;
    params["height"] = Value{height};
    // How far the top slides sideways per unit of height (0 unless `v`).
    params["shear_x"] = Value{height > 0 ? hv[0] / height : 0.0};
    params["shear_y"] = Value{height > 0 ? hv[1] / height : 0.0};
    params["center"] = Value{center};
    params["twist"] = Value{twist};
    params["slices"] = slices ? Value{*slices} : Value{};
    params["segments"] = segments ? Value{*segments} : Value{};
    Discretizer::fromCtx(effCtx, [&](const std::string& m) { ev.warn(m, &node.position()); }).store(params);
    params["scale_x"] = Value{scaleX};
    params["scale_y"] = Value{scaleY};
    params["color"] = colorToValue(effCtx.color);
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveLinearExtrude(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeLinearExtrudeParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateLinearExtrude(Evaluator& ev, const CSGParams& params,
                                                const std::vector<std::unique_ptr<CSGNode>>& children,
                                                const oscad::ASTNode& node) {
    const std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs || cs->IsEmpty()) return {};

    const double height = std::get<double>(params.at("height"));
    const double twist = std::get<double>(params.at("twist"));
    const double scaleX = std::get<double>(params.at("scale_x"));
    const double scaleY = std::get<double>(params.at("scale_y"));
    const double* givenSlices = std::get_if<double>(&params.at("slices"));
    const double* givenSegments = std::get_if<double>(&params.at("segments"));
    const Discretizer disc = Discretizer::fromParams(params);
    manifold::Polygons polys = cs->ToPolygons();

    // Upstream's calc_num_slices (linear_extrude.cc).
    const auto maxDeltaSqr = [&] {
        double m = 0;
        for (const manifold::SimplePolygon& o : polys)
            for (const manifold::vec2& v : o) m = std::max(m, manifold::la::length2(v - manifold::vec2(v.x * scaleX, v.y * scaleY)));
        return m;
    };
    const int twistFallback = std::max(static_cast<int>(std::ceil(twist / 120.0)), 1);
    int slices = 1;
    if (givenSlices) {
        slices = static_cast<int>(*givenSlices);
    } else if (twist != 0.0) {
        double maxR1Sqr = 0;
        for (const manifold::SimplePolygon& o : polys)
            for (const manifold::vec2& v : o) maxR1Sqr = std::max(maxR1Sqr, manifold::la::length2(v));
        if (scaleX == 1.0 && scaleY == 1.0) {
            slices = disc.helixSlices(maxR1Sqr, height, twist).value_or(twistFallback);
        } else if (scaleX != scaleY) {
            slices = std::max(disc.diagonalSlices(maxDeltaSqr(), height).value_or(1),
                              disc.helixSlices(maxR1Sqr, height, twist).value_or(twistFallback));
        } else {
            slices = disc.conicalHelixSlices(maxR1Sqr, height, twist, scaleX).value_or(twistFallback);
        }
    } else if (scaleX != scaleY) {
        slices = disc.diagonalSlices(maxDeltaSqr(), height).value_or(1);
    }

    // Split outline edges where a straight one would lose the shape between
    // slices: twist or non-uniform scale, or `segments` asked for. segments=0
    // turns it off.
    const unsigned segments = givenSegments ? static_cast<unsigned>(*givenSegments) : 0;
    const bool nonLinear = twist != 0.0 || scaleX != scaleY;
    if (!(givenSegments && segments == 0) && (segments > 0 || nonLinear)) {
        for (manifold::SimplePolygon& o : polys)
            o = disc.splitOutline(o, twist, scaleX, scaleY, static_cast<unsigned>(slices), segments);
    }

    // Planar side quads (no twist, uniform scale) come out the same however
    // they are split, so Manifold's own Extrude does. So does a top scaled to
    // zero in either axis, where upstream's shorter-diagonal rule is
    // reversed to avoid zero-thickness ears; a cone tip is Extrude's to make.
    // Manifold's nDivisions is the copies BETWEEN the ends: slices - 1.
    manifold::Manifold body =
        nonLinear && scaleX >= 0 && scaleY >= 0 && height > 0
            ? extrudeTwisted(polys, height, slices, twist, scaleX, scaleY)
            : manifold::Manifold::Extrude(polys, height, slices - 1, -twist, manifold::vec2(scaleX, scaleY));
    if (std::get<bool>(params.at("center"))) body = body.Translate(manifold::vec3(0, 0, -height / 2));
    // `v`: slide each point sideways in proportion to its height -- upstream
    // places slice k at bottom + v*k/n, and centring subtracts v/2, so the
    // same shear is right either way.
    const double shearX = std::get<double>(params.at("shear_x"));
    const double shearY = std::get<double>(params.at("shear_y"));
    if (shearX != 0.0 || shearY != 0.0) {
        body = body.Transform(manifold::mat3x4({1, 0, 0}, {0, 1, 0}, {shearX, shearY, 1}, {0, 0, 0}));
    }
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// rotate_extrude(angle=360) -- mirrors _resolve_rotate_extrude/
// _generate_rotate_extrude. Segment count can't be derived until generate
// (it depends on the merged children's bounds, unknown at resolve time),
// so resolve only caches $fn/$fa/$fs for generate to call fnSegments()
// with once the real max-x bound exists.

// Split the same way as computeLinearExtrudeParams, above -- see its own
// doc comment.
BuiltinWrapParams computeRotateExtrudeParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);

    // Positional order is upstream's: angle, start.
    //
    // Upstream's RotateExtrudeNode: the sweep runs from `start` through
    // `start + angle`. With an angle, start defaults to 0 (the +X axis) and
    // an angle outside (-360, 360] becomes a full turn; with none, the turn
    // is 360 and starts at 180 (-X), which upstream says will change, and
    // says so for an odd $fn, where the start shows. An explicit start
    // always wins. It was ignored here: a quarter turn from 90 drew 0..90.
    const Value angleArg = getArgOrAlias(ev, &node.position(), args, 0, "angle", "a", Value{});
    const Value startArg = getArg(args, 1, "start", Value{});
    // Only a number is an angle (rotate_extrude(angle="x") is a full turn,
    // as upstream reads it; it came out as 0, a flat sheet).
    const bool hasAngle = std::holds_alternative<double>(angleArg) && std::isfinite(std::get<double>(angleArg));
    const bool hasStart = !std::holds_alternative<std::monostate>(startArg) && std::isfinite(toDoubleLenient(startArg));
    double angle = 360.0;
    double start = 180.0;
    if (hasAngle) {
        angle = toDoubleLenient(angleArg);
        start = 0.0;
        if (angle <= -360.0 || angle > 360.0) angle = 360.0;
    }
    if (hasStart) start = toDoubleLenient(startArg);
    const Discretizer disc = Discretizer::fromCtx(effCtx, [&](const std::string& m) { ev.warn(m, &node.position()); });
    if (!hasAngle && !hasStart && (static_cast<int>(disc.fn) & 1)) {
        ev.emitWarning("DEPRECATED: In future releases, rotational extrusion without \"angle\" will start at zero, "
                       "the +X axis.  Set start=180 to explicitly start on the -X axis.");
    }

    CSGParams params;
    params["angle"] = Value{angle};
    params["start"] = Value{start};
    disc.store(params);
    params["color"] = colorToValue(effCtx.color);
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveRotateExtrude(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeRotateExtrudeParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateRotateExtrude(Evaluator& ev, const CSGParams& params,
                                                const std::vector<std::unique_ptr<CSGNode>>& children,
                                                const oscad::ASTNode& node) {
    std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs || cs->IsEmpty()) return {};

    // Upstream rotatePolygon (rotate_extrude.cc): no sweep at all draws
    // nothing; a profile on both sides of the axis is an error (logged, not
    // fatal) and draws nothing; one entirely left of it is swept as it lies.
    // Manifold::Revolve only handles x >= 0, so a left-hand profile is
    // mirrored in and the result turned half a revolution back: (-r, z) at
    // angle a is (r, z) at a + 180. It used to draw nothing, or silently
    // only the right-hand part of a profile across the axis.
    const double rawAngle = std::get<double>(params.at("angle"));
    if (rawAngle == 0.0) return {};
    const manifold::Rect bounds = cs->Bounds();
    if (bounds.min.x < 0 && bounds.max.x > 0) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "[%.2f : %.2f]", bounds.min.x, bounds.max.x);
        ev.emitWarning(std::string("ERROR: Children of rotate_extrude() may not lie across the Y axis (Range of X "
                                   "coords for all children ") +
                       buf + ")");
        return {};
    }
    const bool leftOfAxis = bounds.max.x <= 0;
    if (leftOfAxis) cs = cs->Mirror(manifold::vec2(1.0, 0.0));
    // A negative angle is the same sweep run the other way: |angle| from
    // start + angle. Handing Revolve the negative angle built the solid
    // inside out, so subtracting it ADDED material.
    const double angle = std::fabs(rawAngle);

    // Upstream sizes the arc by the profile's extent in X, measured from the
    // axis: both ends start at 0.
    const manifold::Rect profileBounds = cs->Bounds();
    const double width = std::max(profileBounds.max.x, 0.0) - std::min(profileBounds.min.x, 0.0);
    const int sections = Discretizer::fromParams(params).circular(width, angle).value_or(
        std::max(1, static_cast<int>(std::fabs(angle) / 360 * 3)));
    // Revolve takes this as the section count for the arc it is given -- not
    // per full circle, which is what it used to be handed, packing a whole
    // circle's worth into a partial one.
    manifold::Manifold body;
    if (sections >= 3) {
        body = manifold::Manifold::Revolve(cs->ToPolygons(), std::max(sections, 3), angle);
    } else {
        // Revolve substitutes its own default below 3 sections, but a short
        // arc at coarse settings is 1 or 2 (upstream draws exactly that). So
        // extrude the profile into that many sections and wrap them round the
        // Z axis. The wrap turns the solid inside out for a positive angle, so
        // then the profile goes in mirrored in Y and comes back out unmirrored.
        const double sign = angle > 0 ? -1.0 : 1.0;
        manifold::Polygons profile = cs->ToPolygons();
        if (sign < 0) {
            for (manifold::SimplePolygon& o : profile) {
                for (manifold::vec2& v : o) v.y = -v.y;
                std::reverse(o.begin(), o.end());
            }
        }
        const double rad = angle * std::numbers::pi / 180.0;
        body = manifold::Manifold::Extrude(profile, 1.0, sections - 1).Warp([rad, sign](manifold::vec3& v) {
            const double a = v.z * rad;
            v = manifold::vec3(v.x * std::cos(a), v.x * std::sin(a), sign * v.y);
        });
    }
    // The arc above runs from +X; turn it to begin at `start` (or at
    // start + angle, for a negative sweep), and back round for a left-hand
    // profile.
    const double start = std::get<double>(params.at("start")) + (rawAngle < 0 ? rawAngle : 0.0) +
                         (leftOfAxis ? 180.0 : 0.0);
    if (start != 0.0) body = body.Rotate(0.0, 0.0, start);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// projection(cut=false) -- mirrors _resolve_projection/_generate_projection.
// `cut`: a Z=0 cross-section slice through the combined 3D children.
// Otherwise: an orthographic projection onto the XY plane, re-filled with
// FillRule::Positive to clean up self-intersections Manifold's own
// Project() can produce.

// Split the same way as computeLinearExtrudeParams, above -- see its own
// doc comment.
BuiltinWrapParams computeProjectionParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    // Positional, and only a real bool cuts (ProjectionNode.cc): cut=1 and
    // cut="yes" do not, projection(true) does.
    const Value cutArg = getArg(args, 0, "cut", Value{false});
    params["cut"] = Value{std::holds_alternative<bool>(cutArg) && std::get<bool>(cutArg)};
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveProjection(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeProjectionParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateProjection(Evaluator&, const CSGParams& params,
                                             const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode&) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    std::vector<ColoredBody> bodies3d;
    for (const ColoredBody& c : bodies) {
        if (c.body) bodies3d.push_back(c);
    }
    if (bodies3d.empty()) return {};

    const ColoredBody combined = combineBodies(bodies3d);
    manifold::CrossSection cs;
    if (std::get<bool>(params.at("cut"))) {
        cs = manifold::CrossSection(combined.body->Slice(0.0), manifold::CrossSection::FillRule::EvenOdd);
    } else {
        const manifold::Polygons polys = combined.body->Project();
        cs = polys.empty() ? manifold::CrossSection() : manifold::CrossSection(polys, manifold::CrossSection::FillRule::Positive);
    }

    ColoredBody result;
    result.section = std::move(cs);
    result.color = combined.color;
    return {result};
}

// offset(r=)/offset(delta=, chamfer=false) -- mirrors _resolve_offset/
// _generate_offset. `r` (rounded corners, JoinType::Round) and `delta`
// (JoinType::Square, or Miter if chamfer=true) are mutually exclusive;
// neither given passes the first child through unchanged.

// Split the same way as computeLinearExtrudeParams, above -- see its own
// doc comment.
BuiltinWrapParams computeOffsetParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);

    // Upstream builtin_offset (OffsetNode.cc), rule for rule. A NUMBER r is
    // a round offset; otherwise a number delta is a sharp (miter) one, or a
    // square-cut one when chamfer is a real true; otherwise r=1 -- so a bare
    // offset(), and offset(r="a"), are offset(r=1). The join types used to
    // be swapped: plain delta came out chamfered and chamfer=true sharp.
    const Value rArg = getArg(args, 0, "r");
    const Value deltaArg = getArg(args, 1, "delta");
    const Value chamferArg = getArg(args, 2, "chamfer");
    double delta = 1.0;
    std::string join = "round";
    if (const double* r = std::get_if<double>(&rArg)) {
        if (std::holds_alternative<double>(deltaArg))
            ev.warn("Ignoring \"delta\" argument as \"r\" is defined too.", &node.position());
        delta = *r;
    } else if (const double* d = std::get_if<double>(&deltaArg)) {
        delta = *d;
        join = std::holds_alternative<bool>(chamferArg) && std::get<bool>(chamferArg) ? "square" : "miter";
    }

    CSGParams params;
    params["delta"] = Value{delta};
    params["join"] = Value{join};
    // Silent clamping (upstream builds this node's discretizer without a
    // location), and segments for |delta|, as GeometryEvaluator asks.
    params["segs"] = Value{static_cast<double>(fnSegmentsFromCtx(effCtx, std::fabs(delta)))};
    params["color"] = colorToValue(effCtx.color);
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveOffset(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeOffsetParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateOffset(Evaluator&, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>& children,
                                         const oscad::ASTNode&) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    const std::optional<manifold::CrossSection> cs = toCrossSection(bodies);
    if (!cs) return {};

    const double delta = std::get<double>(params.at("delta"));
    const std::string& join = std::get<std::string>(params.at("join"));
    const int segs = static_cast<int>(std::get<double>(params.at("segs")));
    using JT = manifold::CrossSection::JoinType;
    const JT jt = join == "round" ? JT::Round : join == "square" ? JT::Square : JT::Miter;
    // upstream's miter_limit: "fixed high value to disable chamfers with
    // jtMiter". Manifold's default of 2 cut a sharp corner off at twice the
    // offset, so an acute delta offset came out blunted.
    constexpr double kMiterLimit = 1000000.0;
    ColoredBody result;
    result.section = cs->Offset(delta, jt, kMiterLimit, segs);
    result.color = valueToColor(params.at("color"));
    return {result};
}

} // namespace oscadeval
