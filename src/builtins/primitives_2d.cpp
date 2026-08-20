#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

namespace oscadeval {

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
        Value dArg = getArg(args, std::nullopt, "d", Value{});
        double r;
        if (!std::holds_alternative<std::monostate>(dArg)) {
            r = toDoubleLenient(dArg) / 2.0;
        } else {
            Value rArg = getArg(args, 0, "r", Value{});
            r = std::holds_alternative<std::monostate>(rArg) ? 1.0 : toDoubleLenient(rArg);
        }
        params["r"] = Value{r};
        params["segs"] = Value{static_cast<double>(fnSegmentsFromCtx(effCtx, r))};
        return params;
    }

    if (name == "square") {
        Value sizeArg = getArg(args, 0, "size", Value{1.0});
        const bool center = truthy(getArg(args, 1, "center", Value{false}));
        double sx, sy;
        if (const double* s = std::get_if<double>(&sizeArg)) {
            sx = sy = *s;
        } else if (const ListPtr* l = std::get_if<ListPtr>(&sizeArg); l && *l && (*l)->items.size() >= 2) {
            sx = toDoubleLenient((*l)->items[0]);
            sy = toDoubleLenient((*l)->items[1]);
        } else {
            sx = sy = 0.0;
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

    const ListPtr* pointsList = std::get_if<ListPtr>(&pointsArg);
    if (!pointsList || !*pointsList) {
        ev.error("polygon: 'points' is required", node);
    }
    std::vector<Value> pts;
    for (const Value& p : (*pointsList)->items) {
        const ListPtr* pl = std::get_if<ListPtr>(&p);
        const double x = (pl && *pl && !(*pl)->items.empty()) ? toDoubleLenient((*pl)->items[0]) : 0.0;
        const double y = (pl && *pl && (*pl)->items.size() > 1) ? toDoubleLenient((*pl)->items[1]) : 0.0;
        std::vector<Value> xy = {Value{x}, Value{y}};
        pts.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(xy)})});
    }
    params["pts"] = Value{std::make_shared<const ValueList>(ValueList{std::move(pts)})};

    if (const ListPtr* pathsList = std::get_if<ListPtr>(&pathsArg); pathsList && *pathsList) {
        std::vector<Value> paths;
        for (const Value& path : (*pathsList)->items) {
            const ListPtr* pathIdxList = std::get_if<ListPtr>(&path);
            std::vector<Value> idxVals;
            if (pathIdxList && *pathIdxList) {
                for (const Value& idx : (*pathIdxList)->items) idxVals.push_back(Value{toDoubleLenient(idx)});
            }
            paths.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(idxVals)})});
        }
        params["paths"] = Value{std::make_shared<const ValueList>(ValueList{std::move(paths)})};
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
        cs = manifold::CrossSection::Circle(r, segs);
    } else if (name == "square") {
        const double sx = std::get<double>(params.at("size_x"));
        const double sy = std::get<double>(params.at("size_y"));
        const bool center = std::get<bool>(params.at("center"));
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
