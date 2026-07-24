#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <cmath>

namespace oscadeval {

// linear_extrude(height=1, center=false, twist=0, slices=0, scale=1) --
// mirrors _resolve_linear_extrude/_generate_linear_extrude. `scale` is
// either a single number (applied to both X/Y) or a [x,y] pair for the top
// face; the bottom face is always full-size.

CSGParams resolveLinearExtrude(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    ev.evalChildren(node.children, effCtx);

    const double height = toDoubleLenient(getArg(args, 0, "height", Value{1.0}));
    const bool center = truthy(getArg(args, std::nullopt, "center", Value{false}));
    const double twist = toDoubleLenient(getArg(args, std::nullopt, "twist", Value{0.0}));
    const int slices = static_cast<int>(toDoubleLenient(getArg(args, std::nullopt, "slices", Value{0.0})));
    const Value scaleArg = getArg(args, std::nullopt, "scale", Value{});

    double scaleX = 1.0, scaleY = 1.0;
    if (const double* s = std::get_if<double>(&scaleArg)) {
        scaleX = scaleY = *s;
    } else if (const ListPtr* l = std::get_if<ListPtr>(&scaleArg); l && *l && (*l)->items.size() >= 2) {
        scaleX = toDoubleLenient((*l)->items[0]);
        scaleY = toDoubleLenient((*l)->items[1]);
    }

    CSGParams params;
    params["height"] = Value{height};
    params["center"] = Value{center};
    params["twist"] = Value{twist};
    params["slices"] = Value{static_cast<double>(slices)};
    params["scale_x"] = Value{scaleX};
    params["scale_y"] = Value{scaleY};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generateLinearExtrude(Evaluator& ev, const CSGParams& params,
                                                const std::vector<std::unique_ptr<CSGNode>>& children,
                                                const oscad::ASTNode& node) {
    const std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs || cs->IsEmpty()) return {};

    const double height = std::get<double>(params.at("height"));
    const int slices = static_cast<int>(std::get<double>(params.at("slices")));
    const double twist = std::get<double>(params.at("twist"));
    const manifold::vec2 scaleTop(std::get<double>(params.at("scale_x")), std::get<double>(params.at("scale_y")));

    manifold::Manifold body = manifold::Manifold::Extrude(cs->ToPolygons(), height, slices, -twist, scaleTop);
    if (std::get<bool>(params.at("center"))) body = body.Translate(manifold::vec3(0, 0, -height / 2));
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// rotate_extrude(angle=360) -- mirrors _resolve_rotate_extrude/
// _generate_rotate_extrude. Segment count can't be derived until generate
// (it depends on the merged children's bounds, unknown at resolve time),
// so resolve only caches $fn/$fa/$fs for generate to call fnSegments()
// with once the real max-x bound exists.

CSGParams resolveRotateExtrude(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    ev.evalChildren(node.children, effCtx);

    const double angle = toDoubleLenient(getArg(args, 0, "angle", Value{360.0}));

    const auto dynOr = [&](const char* name, double fallback) {
        auto it = effCtx.dyn->find(name);
        if (it == effCtx.dyn->end()) return fallback;
        const double* d = std::get_if<double>(&it->second);
        return d ? *d : fallback;
    };

    CSGParams params;
    params["angle"] = Value{angle};
    params["fn"] = Value{dynOr("$fn", 0.0)};
    params["fa"] = Value{dynOr("$fa", 12.0)};
    params["fs"] = Value{dynOr("$fs", 2.0)};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generateRotateExtrude(Evaluator& ev, const CSGParams& params,
                                                const std::vector<std::unique_ptr<CSGNode>>& children,
                                                const oscad::ASTNode& node) {
    const std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs || cs->IsEmpty()) return {};

    const manifold::Rect bounds = cs->Bounds();
    const double maxX = std::max(std::fabs(bounds.min.x), std::fabs(bounds.max.x));
    const int segs = fnSegments(std::get<double>(params.at("fn")), std::get<double>(params.at("fa")),
                                 std::get<double>(params.at("fs")), maxX);

    manifold::Manifold body = manifold::Manifold::Revolve(cs->ToPolygons(), segs, std::get<double>(params.at("angle")));
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// projection(cut=false) -- mirrors _resolve_projection/_generate_projection.
// `cut`: a Z=0 cross-section slice through the combined 3D children.
// Otherwise: an orthographic projection onto the XY plane, re-filled with
// FillRule::Positive to clean up self-intersections Manifold's own
// Project() can produce.

CSGParams resolveProjection(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    ev.evalChildren(node.children, effCtx);
    CSGParams params;
    params["cut"] = Value{truthy(getArg(args, std::nullopt, "cut", Value{false}))};
    return params;
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

CSGParams resolveOffset(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    ev.evalChildren(node.children, effCtx);

    const Value rArg = getArg(args, std::nullopt, "r", Value{});
    const Value deltaArg = getArg(args, std::nullopt, "delta", Value{});
    const bool chamfer = truthy(getArg(args, std::nullopt, "chamfer", Value{false}));

    CSGParams params;
    params["r"] = rArg;
    params["delta"] = deltaArg;
    params["chamfer"] = Value{chamfer};
    if (!std::holds_alternative<std::monostate>(rArg)) {
        params["segs"] = Value{static_cast<double>(fnSegmentsFromCtx(effCtx, std::fabs(toDoubleLenient(rArg))))};
    }
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generateOffset(Evaluator&, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>& children,
                                         const oscad::ASTNode&) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    const std::optional<manifold::CrossSection> cs = toCrossSection(bodies);
    if (!cs) return {};

    const Value rArg = params.at("r");
    const Value deltaArg = params.at("delta");

    if (!std::holds_alternative<std::monostate>(rArg)) {
        const int segs = static_cast<int>(std::get<double>(params.at("segs")));
        ColoredBody result;
        result.section = cs->Offset(toDoubleLenient(rArg), manifold::CrossSection::JoinType::Round, 2.0, segs);
        result.color = valueToColor(params.at("color"));
        return {result};
    }
    if (!std::holds_alternative<std::monostate>(deltaArg)) {
        const manifold::CrossSection::JoinType jt =
            std::get<bool>(params.at("chamfer")) ? manifold::CrossSection::JoinType::Miter : manifold::CrossSection::JoinType::Square;
        ColoredBody result;
        result.section = cs->Offset(toDoubleLenient(deltaArg), jt);
        result.color = valueToColor(params.at("color"));
        return {result};
    }
    return bodies.empty() ? std::vector<ColoredBody>{} : std::vector<ColoredBody>{bodies.front()};
}

} // namespace oscadeval
