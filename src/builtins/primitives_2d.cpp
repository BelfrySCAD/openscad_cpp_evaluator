#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <optional>

namespace oscadeval {

// Reads one radius/diameter argument pair (r/d, r1/d1, r2/d2) for circle,
// sphere and cylinder. A numeric diameter wins (halved), warning if a
// numeric radius was given too; otherwise a numeric radius. Anything that
// is not a number is ignored, never coerced. nullopt: neither gives a
// radius, and the caller keeps its default.
std::optional<double> lookupRadius(Evaluator& ev, const CallArgs& args, std::optional<int> rPos,
                                   std::optional<int> dPos, const std::string& rName, const std::string& dName,
                                   const oscad::Position* where) {
    const Value r = getArg(args, rPos, rName);
    const Value d = getArg(args, dPos, dName);
    const double* rNum = std::get_if<double>(&r);
    if (const double* dNum = std::get_if<double>(&d)) {
        if (rNum) {
            ev.warn("Ignoring radius variable \"" + rName + "\" as diameter \"" + dName + "\" is defined too.",
                    where);
        }
        return *dNum / 2.0;
    }
    if (rNum) return *rNum;
    return std::nullopt;
}

void emitInputError(Evaluator& ev, const std::string& text, const oscad::Position* where) {
    ev.emitWarning("ERROR: " + text + locSuffix(where));
}

std::vector<std::vector<size_t>> readIndexLists(Evaluator& ev, const ValueList& lists, const std::string& listName,
                                                size_t pointCount, const oscad::Position* where) {
    constexpr double kTwoTo64 = 18446744073709551616.0;
    std::vector<std::vector<size_t>> result;
    for (size_t i = 0; i < lists.items.size(); ++i) {
        const Value& entry = lists.items[i];
        const std::string entryName = listName + "[" + std::to_string(i) + "]";
        const ListPtr* indices = std::get_if<ListPtr>(&entry);
        if (!indices) {
            emitInputError(ev, "Unable to convert " + entryName + " = " + fmtValue(entry) + " to a vector of numbers",
                           where);
            continue;
        }
        std::vector<size_t> kept;
        for (size_t k = 0; k < (*indices)->items.size(); ++k) {
            const Value& v = (*indices)->items[k];
            const std::string indexName = entryName + "[" + std::to_string(k) + "]";
            const double* num = std::get_if<double>(&v);
            if (!num) {
                emitInputError(ev, "Unable to convert " + indexName + " = " + fmtValue(v) + " to a number", where);
                continue;
            }
            // Truncated toward zero; negative and NaN read as 0, anything
            // past the largest 64-bit index as that largest index.
            uint64_t index = 0;
            if (*num >= kTwoTo64) index = std::numeric_limits<uint64_t>::max();
            else if (*num > 0) index = static_cast<uint64_t>(*num);
            if (index >= pointCount) {
                ev.warn("Point index " + std::to_string(index) + " is out of bounds (from " + indexName + ")", where);
                continue;
            }
            kept.push_back(static_cast<size_t>(index));
        }
        result.push_back(std::move(kept));
    }
    return result;
}

namespace {
// Validates polygon()'s points and paths and stores them in `params`:
// "pts" a list of [x, y] numbers, "paths" undef (the points are one outline)
// or a list of lists of indices, every one in range for "pts".
void readPolygonInput(Evaluator& ev, const Value& points, const Value& paths, const oscad::Position* where,
                      CSGParams& params) {
    params["pts"] = Value{makeList({})};
    params["paths"] = Value{};

    const ListPtr* pointList = std::get_if<ListPtr>(&points);
    if (!pointList) {
        emitInputError(ev, "Unable to convert points = " + fmtValue(points) + " to a vector of coordinates", where);
        return;
    }

    // A bad point becomes the origin rather than being dropped, so the
    // indices of the points after it still mean the same thing.
    std::vector<Value> pts;
    const auto& items = (*pointList)->items;
    for (size_t i = 0; i < items.size(); ++i) {
        double xy[2] = {0.0, 0.0};
        const ListPtr* coords = std::get_if<ListPtr>(&items[i]);
        bool ok = coords && (*coords)->items.size() == 2;
        for (size_t c = 0; ok && c < 2; ++c) {
            const double* n = std::get_if<double>(&(*coords)->items[c]);
            ok = n && std::isfinite(*n);
            if (ok) xy[c] = *n;
        }
        if (!ok) {
            emitInputError(ev,
                           "Unable to convert points[" + std::to_string(i) + "] = " + fmtValue(items[i]) +
                               " to a vec2 of numbers",
                           where);
            xy[0] = xy[1] = 0.0;
        }
        pts.push_back(Value{makeList({Value{xy[0]}, Value{xy[1]}})});
    }
    params["pts"] = Value{makeList(std::move(pts))};

    if (std::holds_alternative<std::monostate>(paths)) return;
    const ListPtr* pathList = std::get_if<ListPtr>(&paths);
    if (!pathList) {
        emitInputError(ev, "Unable to convert paths = " + fmtValue(paths) + " to a vector of vector of point indices",
                       where);
        return;
    }
    const auto contours = readIndexLists(ev, **pathList, "paths", items.size(), where);
    // No usable path at all: the points are one outline after all.
    if (contours.empty()) return;
    std::vector<Value> pathValues;
    for (const auto& contour : contours) {
        std::vector<Value> indices;
        for (size_t index : contour) indices.push_back(Value{static_cast<double>(index)});
        pathValues.push_back(Value{makeList(std::move(indices))});
    }
    params["paths"] = Value{makeList(std::move(pathValues))};
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
        // A number is a square, a list of exactly two numbers its sides;
        // anything else is a unit square, with a warning (undef silently).
        double sx = 1.0, sy = 1.0;
        const Value size = getArg(args, 0, "size");
        if (const double* n = std::get_if<double>(&size)) {
            sx = sy = *n;
        } else if (!std::holds_alternative<std::monostate>(size)) {
            const ListPtr* list = std::get_if<ListPtr>(&size);
            const double* x = nullptr;
            const double* y = nullptr;
            if (list && (*list)->items.size() == 2) {
                x = std::get_if<double>(&(*list)->items[0]);
                y = std::get_if<double>(&(*list)->items[1]);
            }
            if (x && y) {
                sx = *x;
                sy = *y;
            } else {
                ev.warn("Unable to convert square(size=" + fmtValue(size) +
                            ", ...) parameter to a number or a vec2 of numbers",
                        &node.position());
            }
        }
        const Value centerArg = getArg(args, 1, "center");
        const bool center = std::holds_alternative<bool>(centerArg) && std::get<bool>(centerArg);
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
