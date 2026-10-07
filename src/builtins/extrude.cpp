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

    // Argument reading (positional: height, v, scale, center, twist, slices,
    // segments): the extrusion vector, center, twist, slices, segments and
    // scale, with their warnings.
    // CLEAN-ROOM: reimplement from spec section C5.
    const double hv[3] = {0.0, 0.0, 1.0};
    const double height = hv[2];
    const bool center = false;
    const double twist = 0.0;
    const std::optional<double> slices;
    const std::optional<double> segments;
    const double scaleX = 1.0, scaleY = 1.0;

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

    // The slice count, and the subdivision of outline edges (through
    // disc.splitOutline) that a twisted or scaled extrusion needs.
    // CLEAN-ROOM: reimplement from spec section C6.
    const int slices = givenSlices ? static_cast<int>(*givenSlices) : 1;
    (void)givenSegments;
    (void)disc;
    const bool nonLinear = twist != 0.0 || scaleX != scaleY;

    // Planar side quads (no twist, uniform scale) come out the same however
    // they are split, so Manifold's own Extrude does them.
    // Manifold's nDivisions is the copies BETWEEN the ends: slices - 1.
    manifold::Manifold body =
        nonLinear && scaleX >= 0 && scaleY >= 0 && height > 0
            ? extrudeTwisted(polys, height, slices, twist, scaleX, scaleY)
            : manifold::Manifold::Extrude(polys, height, slices - 1, -twist, manifold::vec2(scaleX, scaleY));
    if (std::get<bool>(params.at("center"))) body = body.Translate(manifold::vec3(0, 0, -height / 2));
    // `v`: slide each point sideways in proportion to its height; centring
    // subtracts v/2, so the same shear is right either way.
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
