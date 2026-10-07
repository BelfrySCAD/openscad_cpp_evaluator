#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/surface_load.hpp"

#include <cstdint>

// surface(file, center, invert, convexity): a height map read by
// loadSurface (surface_load.cpp) built into a solid.

#include <manifold/polygon.h>

namespace oscadeval {

CSGParams resolveSurface(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    // (file, center, convexity) positional, invert named after them; center
    // and invert count only as real bools.
    const Value fileArg = getArg(args, 0, "file", Value{std::string("")});
    const Value centerArg = getArg(args, 1, "center", Value{false});
    const Value invertArg = getArg(args, 3, "invert", Value{false});
    const bool center = std::holds_alternative<bool>(centerArg) && std::get<bool>(centerArg);
    const bool invert = std::holds_alternative<bool>(invertArg) && std::get<bool>(invertArg);

    CSGParams params;
    params["center"] = Value{center};
    params["color"] = colorToValue(effCtx.color);
    const std::string path = resolveFilePath(fileArg, node);

    SurfaceData data = loadSurface(path, invert);
    // Logged without a location.
    for (const std::string& w : data.warnings) ev.warn(w, nullptr);
    params["rows"] = Value{static_cast<double>(data.rows)};
    params["cols"] = Value{static_cast<double>(data.cols)};
    params["min"] = Value{data.minVal};
    std::vector<Value> hs;
    hs.reserve(data.heights.size());
    for (double v : data.heights) hs.push_back(Value{v});
    params["heights"] = Value{makeList(std::move(hs))};
    return params;
}

std::vector<ColoredBody> generateSurface(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                          const oscad::ASTNode& node) {
    // The solid built from params "rows", "cols", "heights" (row-major),
    // "min" and "center".
    // CLEAN-ROOM: reimplement from spec section D3.
    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    manifold::Manifold body(mesh);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

} // namespace oscadeval
