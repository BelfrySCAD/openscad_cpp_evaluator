#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

namespace oscadeval {

// Reads one radius/diameter argument pair (r/d, r1/d1, r2/d2) for circle,
// sphere and cylinder. nullopt: neither gives a radius, and the caller keeps
// its default.
// CLEAN-ROOM: reimplement from spec section B1.
std::optional<double> lookupRadius(Evaluator&, const CallArgs&, std::optional<int>, std::optional<int>,
                                   const std::string&, const std::string&, const oscad::Position*) {
    return std::nullopt;
}

namespace {
// Validates polygon()'s points and paths and stores them in `params`:
// "pts" a list of [x, y] numbers, "paths" undef (the points are one outline)
// or a list of lists of indices, every one in range for "pts".
// CLEAN-ROOM: reimplement from spec section B3.
void readPolygonInput(Evaluator&, const Value&, const Value&, const oscad::Position*, CSGParams& params) {
    params["pts"] = Value{makeList({})};
    params["paths"] = Value{};
}
} // namespace

// circle/square/polygon share one dispatch entry (registered under all 3
// names), matching the reference's _resolve_2d/_generate_2d name-based
// if/elif structure exactly.

CSGParams resolve2d(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    const std::string& name = node.name->name;
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    params["name"] = Value{name};
    params["color"] = colorToValue(effCtx.color);

    if (name == "circle") {
        // circle(r, d): both positional, so circle(3, 5) is d=5.
        const double r = lookupRadius(ev, args, 0, 1, "r", "d", &node.position()).value_or(1.0);
        params["r"] = Value{r};
        params["segs"] = Value{static_cast<double>(
            fnSegmentsFromCtx(effCtx, r, [&](const std::string& m) { ev.warn(m, &node.position()); }))};
        return params;
    }

    if (name == "square") {
        // size/center reading.
        // CLEAN-ROOM: reimplement from spec section B2.
        const double sx = 1.0, sy = 1.0;
        const bool center = false;
        params["size_x"] = Value{sx};
        params["size_y"] = Value{sy};
        params["center"] = Value{center};
        return params;
    }

    // polygon
    Value pointsArg = getArg(args, 0, "points", Value{});
    Value pathsArg = getArg(args, 1, "paths", Value{});

    // polygon(obj) -- the 2D counterpart of polyhedron(obj): an object()
    // with `vertices` (and optionally `paths`) stands in for the two lists.
    if (isObject(pointsArg)) {
        const Value* verts = objectFieldOrNull(pointsArg, "vertices");
        if (!verts) verts = objectFieldOrNull(pointsArg, "points");
        if (!verts) ev.error("polygon: object has no 'vertices' (or 'points') key", node);
        const Value* paths = objectFieldOrNull(pointsArg, "paths");
        // Copy before assigning -- both borrow into pointsArg's ObjectPtr.
        Value newPoints = *verts;
        Value newPaths = paths ? *paths : Value{};
        pointsArg = std::move(newPoints);
        // A missing `paths` stays undef, which already means "pts is one
        // single contour" below -- no error, unlike polyhedron's `faces`.
        pathsArg = std::move(newPaths);
    }

    readPolygonInput(ev, pointsArg, pathsArg, &node.position(), params);
    return params;
}

std::vector<ColoredBody> generate2d(Evaluator&, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                     const oscad::ASTNode&) {
    const std::string& name = std::get<std::string>(params.at("name"));
    manifold::CrossSection cs;

    if (name == "circle") {
        const double r = std::get<double>(params.at("r"));
        const int segs = static_cast<int>(std::get<double>(params.at("segs")));
        // Nothing for r <= 0 or a non-finite r.
        if (r > 0 && std::isfinite(r)) cs = manifold::CrossSection::Circle(r, segs);
    } else if (name == "square") {
        const double sx = std::get<double>(params.at("size_x"));
        const double sy = std::get<double>(params.at("size_y"));
        const bool center = std::get<bool>(params.at("center"));
        if (sx > 0 && sy > 0 && std::isfinite(sx) && std::isfinite(sy))
            cs = manifold::CrossSection::Square(manifold::vec2{sx, sy}, center);
    } else { // polygon
        const auto& ptsItems = std::get<ListPtr>(params.at("pts"))->items;
        manifold::SimplePolygon allPts;
        allPts.reserve(ptsItems.size());
        for (const Value& p : ptsItems) {
            const auto& xy = std::get<ListPtr>(p)->items;
            allPts.push_back(manifold::vec2{std::get<double>(xy[0]), std::get<double>(xy[1])});
        }

        manifold::Polygons contours;
        const Value& pathsVal = params.at("paths");
        if (std::holds_alternative<std::monostate>(pathsVal)) {
            contours.push_back(allPts);
        } else {
            for (const Value& path : std::get<ListPtr>(pathsVal)->items) {
                manifold::SimplePolygon contour;
                for (const Value& idx : std::get<ListPtr>(path)->items) {
                    contour.push_back(allPts[static_cast<size_t>(std::get<double>(idx))]);
                }
                contours.push_back(std::move(contour));
            }
        }
        // EvenOdd (not the CrossSection default, Positive) matches
        // OpenSCAD: fills the interior regardless of contour winding
        // direction. The default would silently produce an empty
        // CrossSection for a clockwise-wound polygon.
        cs = manifold::CrossSection(contours, manifold::CrossSection::FillRule::EvenOdd);
    }

    ColoredBody body;
    body.section = std::move(cs);
    body.color = valueToColor(params.at("color"));
    return {body};
}

} // namespace oscadeval
