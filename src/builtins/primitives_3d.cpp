#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <numbers>

namespace oscadeval {

namespace {
bool isUndef(const Value& v) { return std::holds_alternative<std::monostate>(v); }
} // namespace

// cube(size = 1, center = false) -- mirrors _resolve_cube/_generate_cube.

CSGParams resolveCube(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    Value sizeArg = getArg(args, 0, "size", Value{1.0});
    const bool center = truthy(getArg(args, 1, "center", Value{false}));

    std::vector<Value> sizeVec;
    if (const double* s = std::get_if<double>(&sizeArg)) {
        sizeVec = {Value{*s}, Value{*s}, Value{*s}};
    } else if (const ListPtr* l = std::get_if<ListPtr>(&sizeArg); l && *l) {
        for (size_t i = 0; i < 3; ++i) {
            sizeVec.push_back(Value{i < (*l)->items.size() ? toDoubleLenient((*l)->items[i]) : 0.0});
        }
    } else {
        sizeVec = {Value{0.0}, Value{0.0}, Value{0.0}};
    }

    CSGParams params;
    params["size"] = Value{std::make_shared<const ValueList>(ValueList{std::move(sizeVec)})};
    params["center"] = Value{center};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generateCube(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                       const oscad::ASTNode& node) {
    const auto& sizeItems = std::get<ListPtr>(params.at("size"))->items;
    const manifold::vec3 size{std::get<double>(sizeItems[0]), std::get<double>(sizeItems[1]), std::get<double>(sizeItems[2])};
    const bool center = std::get<bool>(params.at("center"));
    manifold::Manifold body = manifold::Manifold::Cube(size, center);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// sphere(r|d, $fn/$fa/$fs) -- a custom lat-long mesh (polygon caps at the
// poles, quad belts between rings, no triangulated pole point), NOT
// manifold::Manifold::Sphere() -- that uses a different tessellation that
// doesn't match real OpenSCAD's vertex/triangle layout. Mirrors
// _resolve_sphere/_generate_sphere exactly, including the "stacks =
// max(2, ceil(n/2))" ring count and the fan-cap winding.

CSGParams resolveSphere(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    Value dArg = getArg(args, std::nullopt, "d", Value{});
    double r;
    if (!isUndef(dArg)) {
        r = toDoubleLenient(dArg) / 2.0;
    } else {
        Value rArg = getArg(args, 0, "r", Value{});
        r = isUndef(rArg) ? 1.0 : toDoubleLenient(rArg);
    }

    const int n = fnSegmentsFromCtx(effCtx, r); // longitude segments
    const int stacks = std::max(2, static_cast<int>(std::ceil(n / 2.0)));
    const double step = std::numbers::pi / stacks;

    std::vector<double> verts; // flat x,y,z,...
    std::vector<std::vector<int>> rings(static_cast<size_t>(stacks));
    for (int s = 0; s < stacks; ++s) {
        const double lat = -std::numbers::pi / 2.0 + (s + 0.5) * step;
        const double ringR = r * std::cos(lat);
        const double z = r * std::sin(lat);
        for (int seg = 0; seg < n; ++seg) {
            const double angle = 2.0 * std::numbers::pi * seg / n;
            rings[static_cast<size_t>(s)].push_back(static_cast<int>(verts.size() / 3));
            verts.push_back(ringR * std::cos(angle));
            verts.push_back(ringR * std::sin(angle));
            verts.push_back(z);
        }
    }

    std::vector<int> tris;
    const auto& bot = rings.front();
    for (int i = 1; i < n - 1; ++i) {
        tris.insert(tris.end(), {bot[0], bot[static_cast<size_t>(i) + 1], bot[static_cast<size_t>(i)]});
    }
    for (int s = 0; s < stacks - 1; ++s) {
        const auto& lo = rings[static_cast<size_t>(s)];
        const auto& hi = rings[static_cast<size_t>(s) + 1];
        for (int seg = 0; seg < n; ++seg) {
            const int a = lo[static_cast<size_t>(seg)], b = lo[static_cast<size_t>((seg + 1) % n)];
            const int c = hi[static_cast<size_t>(seg)], d = hi[static_cast<size_t>((seg + 1) % n)];
            tris.insert(tris.end(), {a, b, d});
            tris.insert(tris.end(), {a, d, c});
        }
    }
    const auto& top = rings.back();
    for (int i = 1; i < n - 1; ++i) {
        tris.insert(tris.end(), {top[0], top[static_cast<size_t>(i)], top[static_cast<size_t>(i) + 1]});
    }

    std::vector<Value> vertsValues;
    vertsValues.reserve(verts.size());
    for (double v : verts) vertsValues.push_back(Value{v});
    std::vector<Value> trisValues;
    trisValues.reserve(tris.size());
    for (int t : tris) trisValues.push_back(Value{static_cast<double>(t)});

    CSGParams params;
    params["verts"] = Value{std::make_shared<const ValueList>(ValueList{std::move(vertsValues)})};
    params["tris"] = Value{std::make_shared<const ValueList>(ValueList{std::move(trisValues)})};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generateSphere(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                         const oscad::ASTNode& node) {
    manifold::MeshGL mesh;
    mesh.numProp = 3;
    for (const Value& v : std::get<ListPtr>(params.at("verts"))->items) {
        mesh.vertProperties.push_back(static_cast<float>(std::get<double>(v)));
    }
    for (const Value& t : std::get<ListPtr>(params.at("tris"))->items) {
        mesh.triVerts.push_back(static_cast<uint32_t>(std::get<double>(t)));
    }
    manifold::Manifold body(mesh);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// cylinder(h, r1, r2, center) -- via manifold::Manifold::Cylinder directly
// (unlike sphere, its tessellation already matches real OpenSCAD). Mirrors
// _resolve_cylinder/_generate_cylinder, including the r/r1/r2/d/d1/d2
// precedence order (order-dependent -- see the reference source before
// reordering any of these ifs).

CSGParams resolveCylinder(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    const double h = toDoubleLenient(getArg(args, 0, "h", Value{1.0}));
    Value rV = getArg(args, 1, "r", Value{});
    Value r1V = getArg(args, std::nullopt, "r1", Value{});
    Value r2V = getArg(args, std::nullopt, "r2", Value{});
    Value dV = getArg(args, std::nullopt, "d", Value{});
    Value d1V = getArg(args, std::nullopt, "d1", Value{});
    Value d2V = getArg(args, std::nullopt, "d2", Value{});
    const bool center = truthy(getArg(args, std::nullopt, "center", Value{false}));

    std::optional<double> r = isUndef(rV) ? std::nullopt : std::optional<double>(toDoubleLenient(rV));
    std::optional<double> r1 = isUndef(r1V) ? std::nullopt : std::optional<double>(toDoubleLenient(r1V));
    std::optional<double> r2 = isUndef(r2V) ? std::nullopt : std::optional<double>(toDoubleLenient(r2V));

    if (!isUndef(dV) && !r) r = toDoubleLenient(dV) / 2.0;
    if (!isUndef(d1V) && !r1) r1 = toDoubleLenient(d1V) / 2.0;
    if (!isUndef(d2V) && !r2) r2 = toDoubleLenient(d2V) / 2.0;
    if (r) {
        r1 = *r;
        r2 = *r;
    }
    if (!r1) r1 = 1.0;
    if (!r2) r2 = *r1;

    const int segs = fnSegmentsFromCtx(effCtx, std::max(*r1, *r2));

    CSGParams params;
    params["h"] = Value{h};
    params["r1"] = Value{*r1};
    params["r2"] = Value{*r2};
    params["center"] = Value{center};
    params["segs"] = Value{static_cast<double>(segs)};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generateCylinder(Evaluator& ev, const CSGParams& params,
                                           const std::vector<std::unique_ptr<CSGNode>>&, const oscad::ASTNode& node) {
    const double h = std::get<double>(params.at("h"));
    const double r1 = std::get<double>(params.at("r1"));
    const double r2 = std::get<double>(params.at("r2"));
    const bool center = std::get<bool>(params.at("center"));
    const int segs = static_cast<int>(std::get<double>(params.at("segs")));
    manifold::Manifold body = manifold::Manifold::Cylinder(h, r1, r2, segs, center);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// polyhedron(points, faces) -- fan-triangulates each face (reversing
// winding: OpenSCAD's CW-from-outside -> Manifold's CCW-from-outside), and
// deduplicates vertices at 1e-6 precision first (VNF meshes, e.g. from
// BOSL2, often have coincident vertices at seams/poles that would
// otherwise produce a NotManifold body). Mirrors
// _resolve_polyhedron/_generate_polyhedron.

CSGParams resolvePolyhedron(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    Value pointsArg = getArg(args, 0, "points", Value{});
    Value facesArg = getArg(args, 1, "faces", Value{});
    if (isUndef(facesArg)) facesArg = getArg(args, 1, "triangles", Value{}); // legacy alias

    const ListPtr* pointsList = std::get_if<ListPtr>(&pointsArg);
    const ListPtr* facesList = std::get_if<ListPtr>(&facesArg);
    if (!pointsList || !*pointsList || !facesList || !*facesList) {
        ev.error("polyhedron: 'points' and 'faces' are required", node);
    }

    std::vector<std::array<double, 3>> rawVerts;
    rawVerts.reserve((*pointsList)->items.size());
    for (size_t i = 0; i < (*pointsList)->items.size(); ++i) {
        const ListPtr* pl = std::get_if<ListPtr>(&(*pointsList)->items[i]);
        if (!pl || !*pl || (*pl)->items.size() != 3) {
            ev.error("polyhedron: point[" + std::to_string(i) + "] is not a valid [x,y,z] coordinate", node);
        }
        rawVerts.push_back(
            {toDoubleLenient((*pl)->items[0]), toDoubleLenient((*pl)->items[1]), toDoubleLenient((*pl)->items[2])});
    }

    std::vector<std::array<double, 3>> uniqueVerts;
    std::vector<size_t> remap(rawVerts.size());
    std::map<std::array<int64_t, 3>, size_t> seen;
    for (size_t i = 0; i < rawVerts.size(); ++i) {
        const std::array<int64_t, 3> key = {
            static_cast<int64_t>(std::llround(rawVerts[i][0] * 1e6)),
            static_cast<int64_t>(std::llround(rawVerts[i][1] * 1e6)),
            static_cast<int64_t>(std::llround(rawVerts[i][2] * 1e6)),
        };
        auto it = seen.find(key);
        if (it != seen.end()) {
            remap[i] = it->second;
        } else {
            const size_t newIdx = uniqueVerts.size();
            uniqueVerts.push_back(rawVerts[i]);
            seen[key] = newIdx;
            remap[i] = newIdx;
        }
    }

    std::vector<uint32_t> tris;
    for (const Value& faceVal : (*facesList)->items) {
        const ListPtr* faceList = std::get_if<ListPtr>(&faceVal);
        if (!faceList || !*faceList) continue;
        std::vector<size_t> remapped;
        remapped.reserve((*faceList)->items.size());
        for (const Value& idxVal : (*faceList)->items) {
            const size_t idx = static_cast<size_t>(toDoubleLenient(idxVal));
            remapped.push_back(idx < remap.size() ? remap[idx] : 0);
        }
        for (size_t i = 1; i + 1 < remapped.size(); ++i) {
            const size_t a = remapped[0], b = remapped[i + 1], c = remapped[i];
            if (a != b && b != c && a != c) {
                tris.push_back(static_cast<uint32_t>(a));
                tris.push_back(static_cast<uint32_t>(b));
                tris.push_back(static_cast<uint32_t>(c));
            }
        }
    }

    std::vector<Value> vertsValues;
    vertsValues.reserve(uniqueVerts.size() * 3);
    for (const auto& v : uniqueVerts) {
        vertsValues.push_back(Value{v[0]});
        vertsValues.push_back(Value{v[1]});
        vertsValues.push_back(Value{v[2]});
    }
    std::vector<Value> trisValues;
    trisValues.reserve(tris.size());
    for (uint32_t t : tris) trisValues.push_back(Value{static_cast<double>(t)});

    CSGParams params;
    params["verts"] = Value{std::make_shared<const ValueList>(ValueList{std::move(vertsValues)})};
    params["tris"] = Value{std::make_shared<const ValueList>(ValueList{std::move(trisValues)})};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generatePolyhedron(Evaluator& ev, const CSGParams& params,
                                             const std::vector<std::unique_ptr<CSGNode>>&, const oscad::ASTNode& node) {
    manifold::MeshGL mesh;
    mesh.numProp = 3;
    for (const Value& v : std::get<ListPtr>(params.at("verts"))->items) {
        mesh.vertProperties.push_back(static_cast<float>(std::get<double>(v)));
    }
    for (const Value& t : std::get<ListPtr>(params.at("tris"))->items) {
        mesh.triVerts.push_back(static_cast<uint32_t>(std::get<double>(t)));
    }
    manifold::Manifold body(mesh);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

} // namespace oscadeval
