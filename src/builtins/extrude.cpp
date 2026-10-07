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

// A twisted and/or non-uniformly scaled extrusion of `polys` (outlines
// already split as needed), `slices` slices over `height`: a mesh whose side
// quads are each split into two triangles. Manifold's own Extrude always
// splits them the same way, which gives a different solid.
// CLEAN-ROOM: reimplement from spec section C1.
manifold::Manifold extrudeTwisted(const manifold::Polygons&, double, int, double, double, double) {
    return manifold::Manifold();
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
        // Exactly two: a 3-vector is the "could not be converted" case
        // below, not its first two.
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
    // they are split, so Manifold's own Extrude does them.
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

    // angle/start reading (positional: angle, start).
    // CLEAN-ROOM: reimplement from spec section C2.
    const double angle = 360.0;
    const double start = 0.0;
    const Discretizer disc = Discretizer::fromCtx(effCtx, [&](const std::string& m) { ev.warn(m, &node.position()); });

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
    // The sweep itself: params "angle", "start" and the stored
    // discretization settings, over the children's 2D profile.
    // CLEAN-ROOM: reimplement from spec section C3.
    (void)ev;
    (void)params;
    (void)children;
    (void)node;
    return {};
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
    // Positional, and only a real bool cuts: cut=1 and cut="yes" do not,
    // projection(true) does.
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

    // r/delta/chamfer reading: the offset distance and the join ("round",
    // "miter" or "square").
    // CLEAN-ROOM: reimplement from spec section C4.
    const double delta = 0.0;
    const std::string join = "round";

    CSGParams params;
    params["delta"] = Value{delta};
    params["join"] = Value{join};
    // Segments for |delta|; $fa/$fs/$fn clamping is silent here.
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
    // The miter limit for "miter" joins.
    // CLEAN-ROOM: reimplement from spec section C4.
    constexpr double kMiterLimit = 2.0;
    ColoredBody result;
    result.section = cs->Offset(delta, jt, kMiterLimit, segs);
    result.color = valueToColor(params.at("color"));
    return {result};
}

} // namespace oscadeval
