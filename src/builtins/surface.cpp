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
    // No file at all is reported as '', not as the script's directory.
    const bool noFile = std::holds_alternative<std::string>(fileArg) && std::get<std::string>(fileArg).empty();
    const std::string path = noFile ? std::string() : resolveFilePath(fileArg, node);

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
    // "min" and "center": the height grid on top, each cell split into four
    // triangles at its centre, closed by walls and a flat base 1 below the
    // lowest height.
    const int rows = static_cast<int>(std::get<double>(params.at("rows")));
    const int cols = static_cast<int>(std::get<double>(params.at("cols")));
    if (rows < 2 || cols < 2) return {};
    const auto& heights = std::get<ListPtr>(params.at("heights"))->items;
    const double base = std::get<double>(params.at("min")) - 1.0;
    const bool center = std::get<bool>(params.at("center"));
    const double dx = center ? -(cols - 1) / 2.0 : 0.0;
    const double dy = center ? -(rows - 1) / 2.0 : 0.0;

    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    const auto addVert = [&](double x, double y, double z) {
        mesh.vertProperties.insert(mesh.vertProperties.end(), {x + dx, y + dy, z});
        return static_cast<uint64_t>(mesh.vertProperties.size() / 3 - 1);
    };
    const auto addTri = [&](uint64_t a, uint64_t b, uint64_t c) { mesh.triVerts.insert(mesh.triVerts.end(), {a, b, c}); };
    const auto h = [&](int i, int j) { return std::get<double>(heights[static_cast<size_t>(i * cols + j)]); };

    // Grid point (row i, column j) is vertex i * cols + j.
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) addVert(j, i, h(i, j));
    const auto grid = [&](int i, int j) { return static_cast<uint64_t>(i * cols + j); };

    for (int i = 1; i < rows; ++i)
        for (int j = 1; j < cols; ++j) {
            const double mid = (h(i - 1, j - 1) + h(i - 1, j) + h(i, j) + h(i, j - 1)) / 4.0;
            const uint64_t m = addVert(j - 0.5, i - 0.5, mid);
            const uint64_t a = grid(i - 1, j - 1), b = grid(i - 1, j), c = grid(i, j), d = grid(i, j - 1);
            addTri(a, b, m);
            addTri(b, c, m);
            addTri(c, d, m);
            addTri(d, a, m);
        }

    // The boundary, counter-clockwise seen from above, each point with a
    // copy dropped to the base.
    std::vector<std::pair<int, int>> ring;
    for (int j = 0; j < cols - 1; ++j) ring.emplace_back(0, j);
    for (int i = 0; i < rows - 1; ++i) ring.emplace_back(i, cols - 1);
    for (int j = cols - 1; j > 0; --j) ring.emplace_back(rows - 1, j);
    for (int i = rows - 1; i > 0; --i) ring.emplace_back(i, 0);

    manifold::SimplePolygonIdx floor;
    std::vector<uint64_t> top, bottom;
    for (const auto& [i, j] : ring) {
        top.push_back(grid(i, j));
        bottom.push_back(addVert(j, i, base));
        floor.push_back({manifold::vec2(j, i), static_cast<int>(bottom.back())});
    }
    for (size_t k = 0; k < ring.size(); ++k) {
        const size_t n = (k + 1) % ring.size();
        addTri(bottom[k], bottom[n], top[n]);
        addTri(bottom[k], top[n], top[k]);
    }
    // The base faces down: its counter-clockwise triangles reversed.
    for (const manifold::ivec3& t : manifold::TriangulateIdx({floor}))
        addTri(static_cast<uint64_t>(t[0]), static_cast<uint64_t>(t[2]), static_cast<uint64_t>(t[1]));

    manifold::Manifold body(mesh);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

} // namespace oscadeval
