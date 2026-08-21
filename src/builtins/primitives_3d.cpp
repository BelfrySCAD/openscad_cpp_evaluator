#include "openscad_cpp_evaluator/mesh_check.hpp"
#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <numbers>
#include <set>

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
template <typename M>
bool isDrawableFailure(const manifold::Manifold& body, const M& mesh) {
    if (body.Status() != manifold::Manifold::Error::NotManifold) return false;
    if (mesh.triVerts.empty()) return false;
    return std::all_of(mesh.vertProperties.begin(), mesh.vertProperties.end(),
                        [](float v) { return std::isfinite(v); });
}

template <typename M>
size_t countBoundaryEdges(const M& mesh) {
    std::map<std::pair<uint32_t, uint32_t>, int> uses;
    for (size_t i = 0; i + 2 < mesh.triVerts.size(); i += 3) {
        const uint32_t v[3] = {static_cast<uint32_t>(mesh.triVerts[i]),
                                static_cast<uint32_t>(mesh.triVerts[i + 1]),
                                static_cast<uint32_t>(mesh.triVerts[i + 2])};
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

// sphere(r|d, style) -- `style` is a BelfrySCAD extension naming the
// tessellation, with the same five names and the same constructions as
// BOSL2's spheroid():
//
//   "orig"    stacked rings offset half a step from the poles, no pole
//             vertex. What OpenSCAD's own sphere() builds, hence the name,
//             and still the default here.
//   "aligned" a vertex at each pole and rings on the latitudes between, so
//             an $fn divisible by 4 puts vertices exactly on +-X and +-Y.
//   "stagger" "aligned" with alternate rings rotated half a face, giving
//             triangles that alternate direction rather than stacking.
//   "octa"    a subdivided octahedron projected onto the sphere.
//   "icosa"   a subdivided icosahedron projected onto the sphere -- the most
//             uniform of the five.
//
// Only "orig" matches real OpenSCAD; the rest are additions, so a script
// using them will not render the same shape elsewhere.

namespace {

// BOSL2's spherical_to_xyz(r, theta, phi): theta around Z from +X, phi down
// from +Z.
std::array<double, 3> sphericalToXyz(double r, double thetaDeg, double phiDeg) {
    const double th = thetaDeg * std::numbers::pi / 180.0;
    const double ph = phiDeg * std::numbers::pi / 180.0;
    return {r * std::sin(ph) * std::cos(th), r * std::sin(ph) * std::sin(th), r * std::cos(ph)};
}

struct SphereMesh {
    std::vector<double> verts;   // flat x,y,z
    std::vector<int> tris;
};

void pushVert(SphereMesh& m, const std::array<double, 3>& p) {
    m.verts.insert(m.verts.end(), {p[0], p[1], p[2]});
}

// "orig": rings at (i+0.5) steps, no pole vertices, capped by a fan at each
// end. Unchanged from the original implementation.
SphereMesh buildOrig(double r, int n, int stacks) {
    SphereMesh m;
    const double step = std::numbers::pi / stacks;
    std::vector<std::vector<int>> rings(static_cast<size_t>(stacks));
    for (int s = 0; s < stacks; ++s) {
        const double lat = -std::numbers::pi / 2.0 + (s + 0.5) * step;
        const double ringR = r * std::cos(lat);
        const double z = r * std::sin(lat);
        for (int seg = 0; seg < n; ++seg) {
            const double angle = 2.0 * std::numbers::pi * seg / n;
            rings[static_cast<size_t>(s)].push_back(static_cast<int>(m.verts.size() / 3));
            pushVert(m, {ringR * std::cos(angle), ringR * std::sin(angle), z});
        }
    }
    const auto& bot = rings.front();
    for (int i = 1; i < n - 1; ++i) m.tris.insert(m.tris.end(), {bot[0], bot[size_t(i) + 1], bot[size_t(i)]});
    for (int s = 0; s < stacks - 1; ++s) {
        const auto& lo = rings[size_t(s)];
        const auto& hi = rings[size_t(s) + 1];
        for (int seg = 0; seg < n; ++seg) {
            const int a = lo[size_t(seg)], b = lo[size_t((seg + 1) % n)];
            const int c = hi[size_t(seg)], d = hi[size_t((seg + 1) % n)];
            m.tris.insert(m.tris.end(), {a, b, d});
            m.tris.insert(m.tris.end(), {a, d, c});
        }
    }
    const auto& top = rings.back();
    for (int i = 1; i < n - 1; ++i) m.tris.insert(m.tris.end(), {top[0], top[size_t(i)], top[size_t(i) + 1]});
    return m;
}

// "aligned"/"stagger": pole, vsides-1 rings, pole. Vertex order and face
// indices follow BOSL2's spheroid() exactly so the two agree triangle for
// triangle, not merely in shape.
SphereMesh buildAligned(double r, int hsides, int vsides, bool stagger) {
    SphereMesh m;
    pushVert(m, sphericalToXyz(r, 0, 0));                      // north pole, index 0
    for (int i = 1; i <= vsides - 1; ++i) {
        const double phi = i * 180.0 / vsides;
        for (int j = 0; j < hsides; ++j) {
            const double theta = (j + ((stagger && i % 2 != 0) ? 0.5 : 0.0)) * 360.0 / hsides;
            pushVert(m, sphericalToXyz(r, theta, phi));
        }
    }
    pushVert(m, sphericalToXyz(r, 0, 180));                    // south pole, last index
    const int lv = static_cast<int>(m.verts.size() / 3);

    // BOSL2's VNF winding is the opposite of what Manifold wants, so every
    // triangle below is emitted with its last two indices swapped. Without
    // that the solid comes out inside-out -- and invisibly so if you only
    // ever check |volume|, which is how this first went unnoticed.
    const auto tri = [&m](int a, int b, int c) { m.tris.insert(m.tris.end(), {a, c, b}); };

    for (int i = 0; i < hsides; ++i) {
        const int b2 = lv - 2 - hsides;
        tri(i + 1, 0, ((i + 1) % hsides) + 1);
        tri(lv - 1, b2 + i + 1, b2 + ((i + 1) % hsides) + 1);
    }
    for (int i = 0; i <= vsides - 3; ++i) {
        const int base = 1 + hsides * i;
        for (int j = 0; j < hsides; ++j) {
            if (stagger && i % 2 != 0) {
                tri(base + j, base + hsides + j % hsides, base + hsides + (j + hsides - 1) % hsides);
                tri(base + j, base + (j + 1) % hsides, base + hsides + j);
            } else {
                tri(base + j, base + (j + 1) % hsides, base + hsides + (j + 1) % hsides);
                tri(base + j, base + hsides + (j + 1) % hsides, base + hsides + j);
            }
        }
    }
    return m;
}

// "icosa": subdivide every icosahedral face into a triangular grid and push
// each sample out to the sphere. BOSL2 subsamples one face and rotates
// copies onto the rest; sampling each face against its own corners is the
// same points (the sampling is affine in those corners) with far less
// machinery. Coincident vertices along shared edges are welded here rather
// than left for Manifold, so the mesh arrives already manifold.
SphereMesh buildIcosa(double r, int hsides) {
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    std::vector<std::array<double, 3>> ico;
    for (int i : {-1, 1}) {
        for (int j : {-1, 1}) {
            ico.push_back({0.0, double(i), double(j) * phi});
            ico.push_back({double(i), double(j) * phi, 0.0});
            ico.push_back({double(j) * phi, 0.0, double(i)});
        }
    }
    // Hull faces by brute force: a triple is a face when every other vertex
    // lies on one side of its plane. 12 vertices, so 220 triples.
    std::vector<std::array<int, 3>> faces;
    const int nv = static_cast<int>(ico.size());
    for (int a = 0; a < nv; ++a)
        for (int b = a + 1; b < nv; ++b)
            for (int c = b + 1; c < nv; ++c) {
                const std::array<double, 3> u{ico[b][0] - ico[a][0], ico[b][1] - ico[a][1], ico[b][2] - ico[a][2]};
                const std::array<double, 3> v{ico[c][0] - ico[a][0], ico[c][1] - ico[a][1], ico[c][2] - ico[a][2]};
                const std::array<double, 3> nrm{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                                                 u[0] * v[1] - u[1] * v[0]};
                int pos = 0, neg = 0;
                for (int k = 0; k < nv; ++k) {
                    if (k == a || k == b || k == c) continue;
                    const double d = nrm[0] * (ico[k][0] - ico[a][0]) + nrm[1] * (ico[k][1] - ico[a][1]) +
                                     nrm[2] * (ico[k][2] - ico[a][2]);
                    if (d > 1e-9) ++pos;
                    if (d < -1e-9) ++neg;
                }
                if (pos && neg) continue;
                // Orient outward: the normal must point away from the centre.
                if (neg == 0) faces.push_back({a, c, b});
                else faces.push_back({a, b, c});
            }

    const int steps = std::max(1, static_cast<int>(std::lround(std::max(5, hsides) / 5.0)));
    const int N = steps - 1;   // BOSL2's N; the grid has N+2 rows

    SphereMesh m;
    std::map<std::array<long long, 3>, int> weld;
    const auto add = [&](const std::array<double, 3>& p) {
        const double len = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        const std::array<double, 3> u{r * p[0] / len, r * p[1] / len, r * p[2] / len};
        const std::array<long long, 3> key{std::llround(u[0] * 1e9), std::llround(u[1] * 1e9),
                                            std::llround(u[2] * 1e9)};
        auto it = weld.find(key);
        if (it != weld.end()) return it->second;
        const int idx = static_cast<int>(m.verts.size() / 3);
        pushVert(m, u);
        weld.emplace(key, idx);
        return idx;
    };

    for (const std::array<int, 3>& f : faces) {
        const std::array<double, 3>& p0 = ico[f[0]];
        const std::array<double, 3>& p1 = ico[f[1]];
        const std::array<double, 3>& p2 = ico[f[2]];
        // Row i has N+2-i samples, mirroring _subsample_triangle.
        std::vector<std::vector<int>> grid;
        for (int i = 0; i <= N + 1; ++i) {
            std::vector<int> row;
            for (int j = 0; j <= N + 1 - i; ++j) {
                const double a = double(i) / (N + 1), b = double(j) / (N + 1);
                row.push_back(add({p0[0] + (p1[0] - p0[0]) * a + (p2[0] - p0[0]) * b,
                                    p0[1] + (p1[1] - p0[1]) * a + (p2[1] - p0[1]) * b,
                                    p0[2] + (p1[2] - p0[2]) * a + (p2[2] - p0[2]) * b}));
            }
            grid.push_back(std::move(row));
        }
        for (int i = 0; i <= N; ++i) {
            for (int j = 0; j <= N - i; ++j) {
                m.tris.insert(m.tris.end(), {grid[i][j], grid[i + 1][j], grid[i][j + 1]});
                if (j < N - i) {
                    m.tris.insert(m.tris.end(), {grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]});
                }
            }
        }
    }
    return m;
}

} // namespace

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

    const Value styleArg = getArg(args, std::nullopt, "style", Value{});
    std::string style = "orig";
    if (const std::string* sv = std::get_if<std::string>(&styleArg)) {
        static const std::set<std::string> known{"orig", "aligned", "stagger", "octa", "icosa"};
        if (known.count(*sv)) {
            style = *sv;
        } else {
            ev.warn("sphere: unknown style \"" + *sv + "\"; expected one of orig, aligned, stagger, octa, icosa",
                    &node.position());
        }
    } else if (!isUndef(styleArg)) {
        ev.warn("sphere: style must be a string", &node.position());
    }

    const int hsides = fnSegmentsFromCtx(effCtx, r);
    const int vsides = std::max(2, static_cast<int>(std::ceil(hsides / 2.0)));

    CSGParams params;
    params["style"] = Value{style};
    params["r"] = Value{r};
    params["segs"] = Value{static_cast<double>(hsides)};
    params["color"] = colorToValue(effCtx.color);

    if (style == "octa") {
        // Manifold::Sphere IS a subdivided octahedron (Shape::Octahedron,
        // then Subdivide), so this needs no mesh of our own -- generate
        // calls it directly.
        params["verts"] = Value{std::make_shared<const ValueList>(ValueList{})};
        params["tris"] = Value{std::make_shared<const ValueList>(ValueList{})};
        return params;
    }

    const SphereMesh m = style == "aligned"   ? buildAligned(r, hsides, vsides, false)
                         : style == "stagger" ? buildAligned(r, hsides, vsides, true)
                         : style == "icosa"   ? buildIcosa(r, hsides)
                                              : buildOrig(r, hsides, vsides);

    std::vector<Value> vertsValues;
    vertsValues.reserve(m.verts.size());
    for (double v : m.verts) vertsValues.push_back(Value{v});
    std::vector<Value> trisValues;
    trisValues.reserve(m.tris.size());
    for (int t : m.tris) trisValues.push_back(Value{static_cast<double>(t)});
    params["verts"] = Value{std::make_shared<const ValueList>(ValueList{std::move(vertsValues)})};
    params["tris"] = Value{std::make_shared<const ValueList>(ValueList{std::move(trisValues)})};
    return params;
}

std::vector<ColoredBody> generateSphere(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                         const oscad::ASTNode& node) {
    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    for (const Value& v : std::get<ListPtr>(params.at("verts"))->items) {
        mesh.vertProperties.push_back(std::get<double>(v));
    }
    for (const Value& t : std::get<ListPtr>(params.at("tris"))->items) {
        mesh.triVerts.push_back(static_cast<uint64_t>(std::get<double>(t)));
    }
    if (std::get<std::string>(params.at("style")) == "octa") {
        manifold::Manifold octa = manifold::Manifold::Sphere(
            std::get<double>(params.at("r")), static_cast<int>(std::get<double>(params.at("segs"))));
        return {ev.tagGenerated(std::move(octa), node, params.at("color"))};
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

    // polyhedron(obj) -- an object() with `vertices` and `faces` stands in
    // for the two lists, so a render() expression round-trips in one call.
    // Works for ANY such object, not just one this evaluator produced, so a
    // script can build or transform its own. `points` is accepted as an
    // alias for `vertices` since that is this argument's own name.
    if (isObject(pointsArg)) {
        const Value* verts = objectFieldOrNull(pointsArg, "vertices");
        if (!verts) verts = objectFieldOrNull(pointsArg, "points");
        const Value* faces = objectFieldOrNull(pointsArg, "faces");
        if (!verts) ev.error("polyhedron: object has no 'vertices' (or 'points') key", node);
        if (!faces) ev.error("polyhedron: object has no 'faces' key", node);
        // Copy BEFORE assigning: both pointers borrow into pointsArg's own
        // ObjectPtr, which the first assignment would release.
        Value newPoints = *verts;
        Value newFaces = *faces;
        pointsArg = std::move(newPoints);
        facesArg = std::move(newFaces);
    }

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

    // Triangulate against the RAW indices first. Welding only renames
    // vertices, never moves them, so the ear clipping is identical either
    // way -- which means the welded triangles can be derived afterwards by
    // remapping, rather than triangulating twice.
    std::vector<uint32_t> rawTris;
    for (const Value& faceVal : (*facesList)->items) {
        const ListPtr* faceList = std::get_if<ListPtr>(&faceVal);
        if (!faceList || !*faceList) continue;
        std::vector<size_t> face;
        face.reserve((*faceList)->items.size());
        for (const Value& idxVal : (*faceList)->items) {
            const size_t idx = static_cast<size_t>(toDoubleLenient(idxVal));
            face.push_back(idx < rawVerts.size() ? idx : 0);
        }
        triangulateFace(rawVerts, face, rawTris);
    }

    std::vector<uint32_t> weldedTris;
    weldedTris.reserve(rawTris.size());
    for (uint32_t t : rawTris) weldedTris.push_back(static_cast<uint32_t>(remap[t]));

    // Welding is a repair for meshes whose seams and poles carry duplicate
    // vertices -- BOSL2 VNFs routinely do, and without it they come out as
    // NotManifold. But it is only ever a repair, and applied blindly it
    // BREAKS a mesh that was already sound: a solid with two shells that
    // touch (the two halves of an XOR meeting along a shared surface) has
    // genuinely coincident vertices belonging to different shells, and
    // merging those fuses the shells into edges with four faces. Measured
    // on exactly such a case: 248 raw vertices, 168 distinct positions,
    // welding turned a watertight manifold mesh into one with 76
    // non-manifold edges and silently lost 500 units of volume.
    //
    // So: weld only when the raw mesh actually needs it. checkMesh is a
    // cheap combinatorial pass (no Manifold construction), and its own doc
    // comment names this exact hazard -- "two boxes fused along a face are
    // watertight but have edges with four faces".
    const bool weldChangesAnything = uniqueVerts.size() != rawVerts.size();
    bool useWelded = weldChangesAnything;
    if (weldChangesAnything) {
        manifold::MeshGL64 probe;
        probe.numProp = 3;
        probe.vertProperties.reserve(rawVerts.size() * 3);
        for (const auto& v : rawVerts) {
            probe.vertProperties.push_back(v[0]);
            probe.vertProperties.push_back(v[1]);
            probe.vertProperties.push_back(v[2]);
        }
        probe.triVerts.assign(rawTris.begin(), rawTris.end());
        // Already sound without the repair -> leave it alone.
        if (checkMesh(probe).manifold()) useWelded = false;
    }

    const std::vector<std::array<double, 3>>& outVerts = useWelded ? uniqueVerts : rawVerts;
    const std::vector<uint32_t>& tris = useWelded ? weldedTris : rawTris;

    std::vector<Value> vertsValues;
    vertsValues.reserve(outVerts.size() * 3);
    for (const auto& v : outVerts) {
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
    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    for (const Value& v : std::get<ListPtr>(params.at("verts"))->items) {
        mesh.vertProperties.push_back(std::get<double>(v));
    }
    for (const Value& t : std::get<ListPtr>(params.at("tris"))->items) {
        mesh.triVerts.push_back(static_cast<uint64_t>(std::get<double>(t)));
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
        manifold::MeshGL soup;
        soup.numProp = 3;
        soup.vertProperties.assign(mesh.vertProperties.begin(), mesh.vertProperties.end());
        soup.triVerts.assign(mesh.triVerts.begin(), mesh.triVerts.end());
        return {ev.tagDisplayOnly(std::move(soup), node, params.at("color"))};
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
