#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/surface_load.hpp"

#include <cstdint>

// surface(file, center, invert, convexity) -- upstream's SurfaceNode,
// construction for construction: each cell is four triangles meeting at a
// centre point at the cell's mean height (it was two), the walls and a flat
// base sit at (minimum height - 1) (the base was always at z = 0, so any
// negative height turned the solid inside out), and a file that cannot be
// read warns and draws nothing instead of stopping the render.

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
    // Upstream logs these without a location.
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
    const int rows = static_cast<int>(std::get<double>(params.at("rows")));
    const int cols = static_cast<int>(std::get<double>(params.at("cols")));
    // A single row or column has no cells and no base: nothing, as upstream.
    if (rows < 2 || cols < 2) return {};
    const auto& hsItems = std::get<ListPtr>(params.at("heights"))->items;
    const auto h = [&](int i, int j) { return std::get<double>(hsItems[static_cast<size_t>(i) * cols + j]); };
    const double base = std::get<double>(params.at("min")) - 1;
    const bool center = std::get<bool>(params.at("center"));
    const double ox = center ? -(cols - 1) / 2.0 : 0, oy = center ? -(rows - 1) / 2.0 : 0;

    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    const auto vert = [&](double x, double y, double z) {
        mesh.vertProperties.insert(mesh.vertProperties.end(), {ox + x, oy + y, z});
        return static_cast<uint64_t>(mesh.vertProperties.size() / 3 - 1);
    };
    const auto tri = [&](uint64_t a, uint64_t b, uint64_t c) { mesh.triVerts.insert(mesh.triVerts.end(), {a, b, c}); };

    // Grid points (row i is y = i, column j is x = j), then the base ring.
    std::vector<uint64_t> top(static_cast<size_t>(rows) * cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) top[static_cast<size_t>(i) * cols + j] = vert(j, i, h(i, j));
    const auto T = [&](int i, int j) { return top[static_cast<size_t>(i) * cols + j]; };
    std::vector<uint64_t> bottom(static_cast<size_t>(rows) * cols, UINT64_MAX);
    const auto B = [&](int i, int j) -> uint64_t {
        uint64_t& b = bottom[static_cast<size_t>(i) * cols + j];
        if (b == UINT64_MAX) b = vert(j, i, base);
        return b;
    };

    // Each cell: four triangles about its centre, counter-clockwise from
    // above (p1 -> p2 -> p4 -> p3), so the top faces up.
    for (int i = 1; i < rows; ++i)
        for (int j = 1; j < cols; ++j) {
            const uint64_t p1 = T(i - 1, j - 1), p2 = T(i - 1, j), p3 = T(i, j - 1), p4 = T(i, j);
            const uint64_t c = vert(j - 0.5, i - 0.5, (h(i - 1, j - 1) + h(i - 1, j) + h(i, j - 1) + h(i, j)) / 4);
            tri(p1, p2, c);
            tri(p2, p4, c);
            tri(p4, p3, c);
            tri(p3, p1, c);
        }
    // Walls, each quad facing outward.
    const auto wall = [&](uint64_t a, uint64_t b, uint64_t bb, uint64_t ba) {  // a->b along the top edge
        tri(a, b, bb);
        tri(a, bb, ba);
    };
    for (int j = 1; j < cols; ++j) {
        wall(T(0, j), T(0, j - 1), B(0, j - 1), B(0, j));                                    // y = 0 face
        wall(T(rows - 1, j - 1), T(rows - 1, j), B(rows - 1, j), B(rows - 1, j - 1));        // y = max face
    }
    for (int i = 1; i < rows; ++i) {
        wall(T(i - 1, 0), T(i, 0), B(i, 0), B(i - 1, 0));                                    // x = 0 face
        wall(T(i, cols - 1), T(i - 1, cols - 1), B(i - 1, cols - 1), B(i, cols - 1));        // x = max face
    }
    // The base: the perimeter ring, triangulated, facing down.
    manifold::SimplePolygonIdx ring;
    const auto add = [&](int i, int j) { ring.push_back({manifold::vec2(ox + j, oy + i), static_cast<int>(B(i, j))}); };
    for (int j = 0; j < cols - 1; ++j) add(0, j);
    for (int i = 0; i < rows - 1; ++i) add(i, cols - 1);
    for (int j = cols - 1; j > 0; --j) add(rows - 1, j);
    for (int i = rows - 1; i > 0; --i) add(i, 0);
    for (const manifold::ivec3& t : manifold::TriangulateIdx({ring}))
        tri(static_cast<uint64_t>(t.x), static_cast<uint64_t>(t.z), static_cast<uint64_t>(t.y));

    manifold::Manifold body(mesh);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

} // namespace oscadeval
