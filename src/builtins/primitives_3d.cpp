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

// Edges used by an odd number of triangles -- i.e. the mesh's open border.
// A closed solid has none; every edge is shared by exactly two faces.
//
// Counted on the UNDIRECTED edge (sorted endpoints) rather than the
// directed half-edge, so a hole reads as a hole regardless of winding:
// counting directed edges would also flag a merely back-to-front face,
// which builds into a perfectly valid solid and is not what this reports.
// Whether a mesh Manifold rejected is nonetheless safe to hand to a
// renderer. Only NotManifold qualifies: it means "the triangles are fine,
// they just don't close a solid", which is exactly the open-surface case
// worth drawing. Every other status (NonFiniteVertex above all, but also
// VertexOutOfBounds and friends) means the vertex data itself is broken --
// drawing NaN coordinates would poison the scene bounding box and send the
// camera auto-fit to infinity, which is worse than showing nothing. The
// explicit finite check is belt-and-braces, since nothing documents that a
// NotManifold mesh can't ALSO contain a NaN.
bool isDrawableFailure(const manifold::Manifold& body, const manifold::MeshGL& mesh) {
    if (body.Status() != manifold::Manifold::Error::NotManifold) return false;
    if (mesh.triVerts.empty()) return false;
    return std::all_of(mesh.vertProperties.begin(), mesh.vertProperties.end(),
                        [](float v) { return std::isfinite(v); });
}

size_t countBoundaryEdges(const manifold::MeshGL& mesh) {
    std::map<std::pair<uint32_t, uint32_t>, int> uses;
    for (size_t i = 0; i + 2 < mesh.triVerts.size(); i += 3) {
        const uint32_t v[3] = {mesh.triVerts[i], mesh.triVerts[i + 1], mesh.triVerts[i + 2]};
        for (int e = 0; e < 3; ++e) {
            const uint32_t a = v[e], b = v[(e + 1) % 3];
            ++uses[{std::min(a, b), std::max(a, b)}];
        }
    }
    size_t open = 0;
    for (const auto& [edge, count] : uses) {
        if (count % 2 != 0) ++open;
    }
    return open;
}
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
    // Positional order is (h, r1, r2, center); r/d/d1/d2 are named-only.
    // NOTE r1/r2 -- not r -- own positions 1 and 2, so `cylinder(10, 5, 2)`
    // is a cone, and `cylinder(10, 5)` is a cone tapering to the r2 default
    // of 1 rather than a straight r=5 cylinder. Both match the reference.
    Value rV = getArg(args, std::nullopt, "r", Value{});
    Value dV = getArg(args, std::nullopt, "d", Value{});
    Value r1V = getArg(args, 1, "r1", Value{});
    Value r2V = getArg(args, 2, "r2", Value{});
    Value d1V = getArg(args, std::nullopt, "d1", Value{});
    Value d2V = getArg(args, std::nullopt, "d2", Value{});
    const bool center = truthy(getArg(args, 3, "center", Value{false}));

    // Application order is significant and mirrors the reference's own
    // sequence of independent `if (x.isNumber()) ...` assignments: r, then
    // d (which overrides r outright rather than deferring to it), then the
    // per-end r1/r2, then the per-end d1/d2. So `cylinder(h=10, d=8, r1=5)`
    // is r1=5/r2=4, and `cylinder(h=10, r=5, r2=2)` is r1=5/r2=2. Both ends
    // default to 1 independently -- r2 does NOT fall back to r1.
    double r1 = 1.0, r2 = 1.0;
    if (!isUndef(rV)) r1 = r2 = toDoubleLenient(rV);
    if (!isUndef(dV)) r1 = r2 = toDoubleLenient(dV) / 2.0;
    if (!isUndef(r1V)) r1 = toDoubleLenient(r1V);
    if (!isUndef(r2V)) r2 = toDoubleLenient(r2V);
    if (!isUndef(d1V)) r1 = toDoubleLenient(d1V) / 2.0;
    if (!isUndef(d2V)) r2 = toDoubleLenient(d2V) / 2.0;

    const int segs = fnSegmentsFromCtx(effCtx, std::max(r1, r2));

    CSGParams params;
    params["h"] = Value{h};
    params["r1"] = Value{r1};
    params["r2"] = Value{r2};
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


namespace {

// Triangulate one polyhedron face.
//
// A fan -- (v0, vi, vi+1) for every i -- is only correct for a polygon
// that is both convex and planar, and BOSL2's vnf_polyhedron() routinely
// hands over neither. The end caps of a nurbs_sheet() are 34-gons that
// are concave and 3.5 units out of plane; fanning one produced 32
// triangles covering 281% of the cap's true area with 15 of them wound
// inside out, which inflated the finished solid by 12% of its surface
// area and 5% of its volume.
//
// Ear clipping in the face's own best-fit plane instead. Newell's method
// gives a normal that stays meaningful when the points are not coplanar,
// which is what makes projecting them usable at all here.
//
// ponytail: O(n^2). Faces are a handful of points in nearly every model
// and 34 in the one that prompted this; revisit if a model ever arrives
// with thousand-sided faces.
void triangulateFace(const std::vector<std::array<double, 3>>& verts, const std::vector<size_t>& loop,
                      std::vector<uint32_t>& out) {
    const size_t n = loop.size();
    if (n < 3) return;

    // Winding is reversed on the way out throughout: OpenSCAD's faces are
    // clockwise seen from outside, Manifold wants counter-clockwise.
    auto emit = [&out](size_t a, size_t b, size_t c) {
        if (a == b || b == c || a == c) return;
        out.push_back(static_cast<uint32_t>(a));
        out.push_back(static_cast<uint32_t>(c));
        out.push_back(static_cast<uint32_t>(b));
    };
    if (n == 3) {
        emit(loop[0], loop[1], loop[2]);
        return;
    }

    std::array<double, 3> nrm = {0.0, 0.0, 0.0};
    for (size_t i = 0; i < n; ++i) {
        const std::array<double, 3>& a = verts[loop[i]];
        const std::array<double, 3>& b = verts[loop[(i + 1) % n]];
        nrm[0] += (a[1] - b[1]) * (a[2] + b[2]);
        nrm[1] += (a[2] - b[2]) * (a[0] + b[0]);
        nrm[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    const double len = std::sqrt(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]);
    if (!(len > 1e-12)) {
        // Every point collinear, or the loop encloses no area: a fan is as
        // good as anything and cannot make it worse.
        for (size_t i = 1; i + 1 < n; ++i) emit(loop[0], loop[i], loop[i + 1]);
        return;
    }
    for (double& c : nrm) c /= len;

    // Any two axes spanning the plane will do; take the world axis least
    // aligned with the normal so the projection never collapses.
    const size_t drop = (std::abs(nrm[0]) > std::abs(nrm[1]))
                            ? ((std::abs(nrm[0]) > std::abs(nrm[2])) ? 0 : 2)
                            : ((std::abs(nrm[1]) > std::abs(nrm[2])) ? 1 : 2);
    std::array<double, 3> axis = {0.0, 0.0, 0.0};
    axis[(drop + 1) % 3] = 1.0;
    std::array<double, 3> u = {axis[1] * nrm[2] - axis[2] * nrm[1], axis[2] * nrm[0] - axis[0] * nrm[2],
                                axis[0] * nrm[1] - axis[1] * nrm[0]};
    const double ulen = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (!(ulen > 1e-12)) {
        for (size_t i = 1; i + 1 < n; ++i) emit(loop[0], loop[i], loop[i + 1]);
        return;
    }
    for (double& c : u) c /= ulen;
    const std::array<double, 3> w = {nrm[1] * u[2] - nrm[2] * u[1], nrm[2] * u[0] - nrm[0] * u[2],
                                      nrm[0] * u[1] - nrm[1] * u[0]};

    std::vector<std::array<double, 2>> flat(n);
    for (size_t i = 0; i < n; ++i) {
        const std::array<double, 3>& p = verts[loop[i]];
        flat[i] = {p[0] * u[0] + p[1] * u[1] + p[2] * u[2], p[0] * w[0] + p[1] * w[1] + p[2] * w[2]};
    }

    auto cross2 = [](const std::array<double, 2>& a, const std::array<double, 2>& b,
                     const std::array<double, 2>& c) {
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    };
    double twiceArea = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const std::array<double, 2>& a = flat[i];
        const std::array<double, 2>& b = flat[(i + 1) % n];
        twiceArea += a[0] * b[1] - b[0] * a[1];
    }

    std::vector<size_t> idx(n);
    for (size_t i = 0; i < n; ++i) idx[i] = i;
    if (twiceArea < 0.0) std::reverse(idx.begin(), idx.end());   // work counter-clockwise

    auto inside = [&](const std::array<double, 2>& a, const std::array<double, 2>& b,
                      const std::array<double, 2>& c, const std::array<double, 2>& p) {
        // Strictly inside, so a vertex sitting exactly on an edge does not
        // veto an otherwise good ear.
        const double d1 = cross2(a, b, p), d2 = cross2(b, c, p), d3 = cross2(c, a, p);
        return d1 > 1e-12 && d2 > 1e-12 && d3 > 1e-12;
    };

    // Take the best-shaped ear available rather than the first one found.
    // Any valid ear gives a correct triangulation, but on a face that is
    // not flat the choice decides how the surface folds: first-found
    // clipping strung long thin triangles across the curved end caps and
    // came out 29% larger in area than the reference's tessellation of the
    // same polygon. Preferring fat ears tracks the surface instead.
    auto squareness = [&](size_t a, size_t b, size_t c) {
        // Twice the area over the sum of the squared sides -- highest for
        // an equilateral triangle, near zero for a sliver.
        const std::array<double, 3>& p = verts[loop[a]];
        const std::array<double, 3>& q = verts[loop[b]];
        const std::array<double, 3>& r = verts[loop[c]];
        const std::array<double, 3> e1 = {q[0] - p[0], q[1] - p[1], q[2] - p[2]};
        const std::array<double, 3> e2 = {r[0] - p[0], r[1] - p[1], r[2] - p[2]};
        const std::array<double, 3> e3 = {r[0] - q[0], r[1] - q[1], r[2] - q[2]};
        const std::array<double, 3> x = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                                          e1[0] * e2[1] - e1[1] * e2[0]};
        const double area = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
        const double sides = e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2] +
                              e2[0] * e2[0] + e2[1] * e2[1] + e2[2] * e2[2] +
                              e3[0] * e3[0] + e3[1] * e3[1] + e3[2] * e3[2];
        return sides > 1e-18 ? area / sides : 0.0;
    };

    size_t guard = 0;
    while (idx.size() > 3 && guard++ < n * n) {
        size_t bestAt = idx.size();
        double bestScore = -1.0;
        for (size_t i = 0; i < idx.size(); ++i) {
            const size_t pi = idx[(i + idx.size() - 1) % idx.size()];
            const size_t ci = idx[i];
            const size_t ni = idx[(i + 1) % idx.size()];
            if (cross2(flat[pi], flat[ci], flat[ni]) <= 1e-12) continue;   // reflex, not an ear
            bool empty = true;
            for (size_t other : idx) {
                if (other == pi || other == ci || other == ni) continue;
                if (inside(flat[pi], flat[ci], flat[ni], flat[other])) { empty = false; break; }
            }
            if (!empty) continue;
            const double score = squareness(pi, ci, ni);
            if (score > bestScore) { bestScore = score; bestAt = i; }
        }
        if (bestAt == idx.size()) break;   // self-intersecting or otherwise unclippable
        const size_t pi = idx[(bestAt + idx.size() - 1) % idx.size()];
        const size_t ci = idx[bestAt];
        const size_t ni = idx[(bestAt + 1) % idx.size()];
        emit(loop[pi], loop[ci], loop[ni]);
        idx.erase(idx.begin() + static_cast<long>(bestAt));
    }
    // Whatever is left: three points, or a remainder no ear could be found
    // in. A fan over the remainder is the best available answer.
    for (size_t i = 1; i + 1 < idx.size(); ++i) emit(loop[idx[0]], loop[idx[i]], loop[idx[i + 1]]);
}

} // namespace

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
        triangulateFace(uniqueVerts, remapped, tris);
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
    if (isDrawableFailure(body, mesh)) {
        // An open surface -- faces that don't close the solid. Manifold
        // signals that by returning an EMPTY body rather than throwing, so
        // without this branch the polyhedron simply vanishes from the
        // render with nothing said, leaving the author to guess which face
        // they forgot.
        //
        // Report the boundary-edge count rather than Manifold's own
        // "NotManifold" status string, which is actively misleading here: a
        // closed mesh with reversed winding, non-manifold vertices or
        // self-intersections builds perfectly well, and only an OPEN one
        // reaches this branch. The count points straight at the problem.
        ev.warn("polyhedron: mesh is not closed -- " + std::to_string(countBoundaryEdges(mesh)) +
                    " boundary edge(s); drawing it as a surface, but it cannot "
                    "take part in any CSG operation",
                &node.position());
        return {ev.tagDisplayOnly(std::move(mesh), node, params.at("color"))};
    }
    if (body.Status() != manifold::Manifold::Error::NoError) {
        // Broken vertex data rather than merely-open topology (a NaN
        // coordinate, an out-of-range index). Not drawable, so this keeps
        // the old drop-it behaviour -- but says so, which it never used to.
        ev.warn(std::string("polyhedron: ") + manifoldErrorName(body.Status()) + "; geometry discarded",
                &node.position());
    }
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

} // namespace oscadeval
