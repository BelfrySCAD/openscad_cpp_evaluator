#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

namespace oscadeval {

namespace {
// Value::getDouble: a number and nothing else -- no coercion.
const double* asNumber(const Value& v) { return std::get_if<double>(&v); }

// Value::getVec2: exactly two numbers.
bool asVec2(const Value& v, double& x, double& y) {
    const ListPtr* l = std::get_if<ListPtr>(&v);
    if (!l || !*l || (*l)->items.size() != 2) return false;
    const double* a = asNumber((*l)->items[0]);
    const double* b = asNumber((*l)->items[1]);
    if (!a || !b) return false;
    x = *a;
    y = *b;
    return true;
}
} // namespace

// OpenSCAD's lookup_radius(): `d` wins when it is a number, with a warning
// if `r` is one too; otherwise `r` if it is a number; otherwise nothing,
// and the caller keeps its default. Anything that is not a number is not a
// radius -- it does not become 0.
std::optional<double> lookupRadius(Evaluator& ev, const CallArgs& args, std::optional<int> rPos,
                                   std::optional<int> dPos, const std::string& rName, const std::string& dName,
                                   const oscad::Position* where) {
    const Value r = getArg(args, rPos, rName);
    const Value d = getArg(args, dPos, dName);
    if (const double* dv = asNumber(d)) {
        if (asNumber(r)) {
            ev.warn("Ignoring radius variable \"" + rName + "\" as diameter \"" + dName + "\" is defined too.",
                    where);
        }
        return *dv / 2.0;
    }
    if (const double* rv = asNumber(r)) return *rv;
    return std::nullopt;
}

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
        // circle(r, d): both positional, as upstream's Parameters::parse
        // reads them, so circle(3, 5) is d=5.
        const double r = lookupRadius(ev, args, 0, 1, "r", "d", &node.position()).value_or(1.0);
        params["r"] = Value{r};
        params["segs"] = Value{static_cast<double>(fnSegmentsFromCtx(effCtx, r))};
        return params;
    }

    if (name == "square") {
        // Upstream builtin_square: a number or exactly two numbers; anything
        // else warns and keeps the 1x1 default. center counts only as a
        // real bool.
        const Value sizeArg = getArg(args, 0, "size");
        const Value centerArg = getArg(args, 1, "center");
        const bool* centerBool = std::get_if<bool>(&centerArg);
        const bool center = centerBool && *centerBool;
        double sx = 1.0, sy = 1.0;
        if (!std::holds_alternative<std::monostate>(sizeArg)) {
            if (const double* s = asNumber(sizeArg)) {
                sx = sy = *s;
            } else if (!asVec2(sizeArg, sx, sy)) {
                ev.warn("Unable to convert square(size=" + fmtValue(sizeArg) +
                            ", ...) parameter to a number or a vec2 of numbers",
                        &node.position());
            }
        }
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

    // Upstream builtin_polygon, warning for warning: bad input is reported
    // and repaired or dropped, never fatal to the rest of the script, and
    // every index that reaches generate2d is known to be in range.
    const oscad::Position* where = &node.position();
    std::vector<Value> pts;
    const ListPtr* pointsList = std::get_if<ListPtr>(&pointsArg);
    if (!pointsList || !*pointsList) {
        ev.warn("Unable to convert points = " + fmtValue(pointsArg) + " to a vector of coordinates", where);
        params["pts"] = Value{makeList({})};
        params["paths"] = Value{};
        return params;
    }
    for (const Value& p : (*pointsList)->items) {
        double x = 0.0, y = 0.0;
        if (!asVec2(p, x, y) || !std::isfinite(x) || !std::isfinite(y)) {
            ev.warn("Unable to convert points[" + std::to_string(pts.size()) + "] = " + fmtValue(p) +
                        " to a vec2 of numbers",
                    where);
            x = y = 0.0;
        }
        pts.push_back(Value{makeList({Value{x}, Value{y}})});
    }
    const size_t numPts = pts.size();
    params["pts"] = Value{makeList(std::move(pts))};

    if (const ListPtr* pathsList = std::get_if<ListPtr>(&pathsArg); pathsList && *pathsList) {
        std::vector<Value> paths;
        size_t pathIndex = 0;
        for (const Value& path : (*pathsList)->items) {
            const ListPtr* pathIdxList = std::get_if<ListPtr>(&path);
            if (!pathIdxList || !*pathIdxList) {
                ev.warn("Unable to convert paths[" + std::to_string(pathIndex) + "] = " + fmtValue(path) +
                            " to a vector of numbers",
                        where);
            } else {
                std::vector<Value> idxVals;
                size_t k = 0;
                for (const Value& idx : (*pathIdxList)->items) {
                    const double* n = asNumber(idx);
                    if (!n) {
                        ev.warn("Unable to convert paths[" + std::to_string(pathIndex) + "][" + std::to_string(k) +
                                    "] = " + fmtValue(idx) + " to a number",
                                where);
                    } else {
                        // Upstream casts with (size_t), undefined below 0;
                        // its builds saturate a negative index to 0, so
                        // that is what scripts see. NaN is out of range.
                        const size_t i = *n < 0 ? 0 : (*n < 1.8e19 ? static_cast<size_t>(*n) : SIZE_MAX);
                        if (i < numPts) {
                            idxVals.push_back(Value{static_cast<double>(i)});
                        } else {
                            ev.warn("Point index " + std::to_string(i) + " is out of bounds (from paths[" +
                                        std::to_string(pathIndex) + "][" + std::to_string(k) + "])",
                                    where);
                        }
                    }
                    ++k;
                }
                paths.push_back(Value{makeList(std::move(idxVals))});
            }
            ++pathIndex;
        }
        // No path survived: upstream's PolygonNode draws the points as one
        // outline, exactly as if paths had not been given.
        params["paths"] = paths.empty() ? Value{} : Value{makeList(std::move(paths))};
    } else if (!std::holds_alternative<std::monostate>(pathsArg)) {
        // Upstream returns here with the points already read and no paths,
        // so the points still draw as one outline.
        ev.warn("Unable to convert paths = " + fmtValue(pathsArg) + " to a vector of vector of point indices", where);
        params["paths"] = Value{};
    } else {
        params["paths"] = Value{}; // undef == "no paths given" -- pts is one single contour
    }
    return params;
}

std::vector<ColoredBody> generate2d(Evaluator&, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                     const oscad::ASTNode&) {
    const std::string& name = std::get<std::string>(params.at("name"));
    manifold::CrossSection cs;

    if (name == "circle") {
        const double r = std::get<double>(params.at("r"));
        const int segs = static_cast<int>(std::get<double>(params.at("segs")));
        // Upstream draws nothing for r <= 0 or a non-finite r.
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
