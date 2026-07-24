#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/surface_load.hpp"

#include <cstdint>

// surface(file, center=false, invert=false) -- mirrors _resolve_surface/
// _generate_surface: loads a height grid (surface_load.hpp), builds a
// terrain solid (grid of top vertices at each cell's height + a flat
// bottom cap at z=0 + side walls), exactly matching the reference's own
// vertex/triangle indexing so cross-checked output triangle-counts match.

namespace oscadeval {

CSGParams resolveSurface(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    const Value fileArg = getArg(args, 0, "file", Value{});
    const bool center = truthy(getArg(args, std::nullopt, "center", Value{false}));
    const bool invert = truthy(getArg(args, std::nullopt, "invert", Value{false}));

    CSGParams params;
    params["center"] = Value{center};
    params["color"] = colorToValue(effCtx.color);
    if (std::holds_alternative<std::monostate>(fileArg)) {
        ev.error("surface: 'file' parameter is required", node);
    }
    const std::string path = resolveFilePath(fileArg, node);

    std::vector<std::vector<double>> heights;
    try {
        heights = loadSurfaceHeights(path, invert);
    } catch (const std::exception& e) {
        ev.error(std::string("surface: ") + e.what(), node);
    }
    if (heights.empty() || heights.front().empty()) {
        ev.error("surface: empty height data", node);
    }

    std::vector<Value> rows;
    rows.reserve(heights.size());
    for (const auto& row : heights) {
        std::vector<Value> rowVals;
        rowVals.reserve(row.size());
        for (double v : row) rowVals.push_back(Value{v});
        rows.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(rowVals)})});
    }
    params["heights"] = Value{std::make_shared<const ValueList>(ValueList{std::move(rows)})};
    return params;
}

std::vector<ColoredBody> generateSurface(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                          const oscad::ASTNode& node) {
    const auto& rowsList = std::get<ListPtr>(params.at("heights"))->items;
    const std::size_t rows = rowsList.size();
    const std::size_t cols = std::get<ListPtr>(rowsList[0])->items.size();
    const bool center = std::get<bool>(params.at("center"));

    std::vector<std::vector<double>> heights(rows);
    for (std::size_t r = 0; r < rows; ++r) {
        const auto& rowItems = std::get<ListPtr>(rowsList[r])->items;
        heights[r].resize(cols);
        for (std::size_t c = 0; c < cols; ++c) heights[r][c] = std::get<double>(rowItems[c]);
    }

    const double xOff = center ? -(static_cast<double>(cols) - 1) / 2.0 : 0.0;
    const double yOff = center ? -(static_cast<double>(rows) - 1) / 2.0 : 0.0;
    const std::size_t n = rows * cols;

    const auto top = [&](std::size_t r, std::size_t c) { return static_cast<uint32_t>(r * cols + c); };
    const auto bot = [&](std::size_t r, std::size_t c) { return static_cast<uint32_t>(n + r * cols + c); };

    manifold::MeshGL mesh;
    mesh.numProp = 3;
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            mesh.vertProperties.push_back(static_cast<float>(static_cast<double>(c) + xOff));
            mesh.vertProperties.push_back(static_cast<float>(static_cast<double>(r) + yOff));
            mesh.vertProperties.push_back(static_cast<float>(heights[r][c]));
        }
    }
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            mesh.vertProperties.push_back(static_cast<float>(static_cast<double>(c) + xOff));
            mesh.vertProperties.push_back(static_cast<float>(static_cast<double>(r) + yOff));
            mesh.vertProperties.push_back(0.0f);
        }
    }

    const auto addTri = [&](uint32_t a, uint32_t b, uint32_t c) {
        mesh.triVerts.push_back(a);
        mesh.triVerts.push_back(b);
        mesh.triVerts.push_back(c);
    };

    for (std::size_t r = 0; r + 1 < rows; ++r) { // top surface
        for (std::size_t c = 0; c + 1 < cols; ++c) {
            const uint32_t tl = top(r + 1, c), tr = top(r + 1, c + 1), bl = top(r, c), br = top(r, c + 1);
            addTri(tl, bl, br);
            addTri(tl, br, tr);
        }
    }
    for (std::size_t r = 0; r + 1 < rows; ++r) { // bottom cap (z=0)
        for (std::size_t c = 0; c + 1 < cols; ++c) {
            const uint32_t tl = bot(r + 1, c), tr = bot(r + 1, c + 1), bl = bot(r, c), br = bot(r, c + 1);
            addTri(tl, tr, br);
            addTri(tl, br, bl);
        }
    }
    for (std::size_t c = 0; c + 1 < cols; ++c) { // front wall (r=0, outward -Y)
        addTri(top(0, c), bot(0, c), bot(0, c + 1));
        addTri(top(0, c), bot(0, c + 1), top(0, c + 1));
    }
    for (std::size_t c = 0; c + 1 < cols; ++c) { // back wall (r=rows-1, outward +Y)
        addTri(top(rows - 1, c), top(rows - 1, c + 1), bot(rows - 1, c + 1));
        addTri(top(rows - 1, c), bot(rows - 1, c + 1), bot(rows - 1, c));
    }
    for (std::size_t r = 0; r + 1 < rows; ++r) { // left wall (c=0, outward -X)
        addTri(top(r, 0), top(r + 1, 0), bot(r + 1, 0));
        addTri(top(r, 0), bot(r + 1, 0), bot(r, 0));
    }
    for (std::size_t r = 0; r + 1 < rows; ++r) { // right wall (c=cols-1, outward +X)
        addTri(top(r, cols - 1), bot(r + 1, cols - 1), top(r + 1, cols - 1));
        addTri(top(r, cols - 1), bot(r, cols - 1), bot(r + 1, cols - 1));
    }

    manifold::Manifold body(mesh);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

} // namespace oscadeval
