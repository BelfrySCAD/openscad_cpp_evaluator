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
#include <optional>
#include <set>
#include <string>
#include <utility>

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

struct BoundaryEdges {
    size_t count = 0;
    // Vertex indices of one representative open edge, or nullopt when the
    // mesh is closed. A count alone is enough on a hand-written face list,
    // where you can just read it; it is useless on generated geometry --
    // three bad edges in a few thousand triangles cannot be found by eye,
    // which is what happened in BelfrySCAD #521 / issue #187.
    std::optional<std::pair<uint32_t, uint32_t>> first;
    // An edge both of whose faces run it the SAME way: the mesh is closed
    // but a face is wound backwards. Named so the author knows which face
    // to flip (BelfrySCAD #566, where "not closed -- 0 boundary edges"
    // described a closed tetrahedron with two reversed faces).
    size_t misoriented = 0;
    std::optional<std::pair<uint32_t, uint32_t>> firstMisoriented;
};

template <typename M>
BoundaryEdges findBoundaryEdges(const M& mesh) {
    std::map<std::pair<uint32_t, uint32_t>, int> uses;
    std::map<std::pair<uint32_t, uint32_t>, int> forward;  // uses running low -> high
    for (size_t i = 0; i + 2 < mesh.triVerts.size(); i += 3) {
        const uint32_t v[3] = {static_cast<uint32_t>(mesh.triVerts[i]),
                                static_cast<uint32_t>(mesh.triVerts[i + 1]),
                                static_cast<uint32_t>(mesh.triVerts[i + 2])};
        for (int e = 0; e < 3; ++e) {
            const uint32_t a = v[e], b = v[(e + 1) % 3];
            ++uses[{std::min(a, b), std::max(a, b)}];
            if (a < b) ++forward[{a, b}];
        }
    }
    BoundaryEdges out;
    // std::map iterates in vertex-index order, which is arbitrary from the
    // author's point of view but DETERMINISTIC: the same mesh names the same
    // edge on every run. That matters more than which edge gets picked --
    // an identifier that moves between renders is worse than no identifier.
    // A cluster of open edges is nearly always one hole, so the first is
    // usually representative of the rest.
    for (const auto& [edge, count] : uses) {
        if (count % 2 == 0) {
            // Closed here; consistently wound only if the uses split evenly
            // between the two directions.
            const auto f = forward.find(edge);
            if ((f == forward.end() ? 0 : f->second) * 2 != count) {
                ++out.misoriented;
                if (!out.firstMisoriented) out.firstMisoriented = edge;
            }
            continue;
        }
        ++out.count;
        if (!out.first) out.first = edge;
    }
    return out;
}

// "[1, 2.5, 0] - [1, 3, 0]" for one edge, using the same number formatting
// echo() uses, so a coordinate can be pasted straight back into a script.
template <typename M>
std::string describeEdge(const M& mesh, std::pair<uint32_t, uint32_t> edge) {
    const size_t stride = mesh.numProp > 0 ? static_cast<size_t>(mesh.numProp) : 3;
    auto point = [&](uint32_t v) {
        const size_t base = static_cast<size_t>(v) * stride;
        if (base + 2 >= mesh.vertProperties.size()) return std::string("?");
        return "[" + formatNumber(static_cast<double>(mesh.vertProperties[base])) + ", " +
               formatNumber(static_cast<double>(mesh.vertProperties[base + 1])) + ", " +
               formatNumber(static_cast<double>(mesh.vertProperties[base + 2])) + "]";
    };
    return point(edge.first) + " - " + point(edge.second);
}
} // namespace

// cube(size = 1, center = false) -- mirrors _resolve_cube/_generate_cube.

CSGParams resolveCube(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    // A number is a cube. A list of exactly three is read axis by axis up
    // to its first non-number, which (with the axes after it) stays at 1;
    // that, and any other kind of size, warns. undef is a unit cube.
    std::vector<Value> sizeVec(3, Value{1.0});
    const Value size = getArg(args, 0, "size");
    if (std::holds_alternative<double>(size)) {
        sizeVec.assign(3, size);
    } else if (!std::holds_alternative<std::monostate>(size)) {
        bool ok = false;
        if (const ListPtr* list = std::get_if<ListPtr>(&size); list && (*list)->items.size() == 3) {
            size_t axis = 0;
            while (axis < 3 && std::holds_alternative<double>((*list)->items[axis])) {
                sizeVec[axis] = (*list)->items[axis];
                ++axis;
            }
            ok = axis == 3;
        }
        if (!ok) {
            ev.warn("Unable to convert cube(size=" + fmtValue(size) + ", ...) parameter to a number or a vec3 of numbers",
                    &node.position());
        }
    }
    const Value centerArg = getArg(args, 1, "center");
    const bool center = std::holds_alternative<bool>(centerArg) && std::get<bool>(centerArg);

    CSGParams params;
    params["size"] = Value{makeList(std::move(sizeVec))};
    params["center"] = Value{center};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generateCube(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                       const oscad::ASTNode& node) {
    const auto& sizeItems = std::get<ListPtr>(params.at("size"))->items;
    const manifold::vec3 size{std::get<double>(sizeItems[0]), std::get<double>(sizeItems[1]), std::get<double>(sizeItems[2])};
    const bool center = std::get<bool>(params.at("center"));
    // Any side <= 0 or non-finite is nothing at
    // all -- not a zero-volume sheet.
    const bool valid = size.x > 0 && size.y > 0 && size.z > 0 && std::isfinite(size.x) && std::isfinite(size.y) &&
                       std::isfinite(size.z);
    manifold::Manifold body = valid ? manifold::Manifold::Cube(size, center) : manifold::Manifold();
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

// "octa": an octahedron with each face cut into an n x n triangular grid,
// n = ceil(segments / 4), pushed out to the sphere -- 4n^2 + 2 vertices,
// one on each pole of all three axes. Edge vertices are equally spaced on
// the octahedron's great circles; interior vertex placement is spec section
// B8.
//
// The mesh is EXACTLY symmetric under all 48 symmetries of the octahedron,
// not merely to rounding: every vertex is computed once, for its
// barycentric weights sorted largest first, and the other copies are that
// one point with its coordinates permuted and negated, which is exact.
// Within a rounding error is not good enough. Manifold::Sphere, which this
// used to be, is symmetric only to ~1e-15, so a copy rotated 90 degrees
// landed a hair off the original and difference() of the two left a mass of
// sliver shards where it should leave nothing.
//
// ponytail: the great-circle vertices are not bit-identical to circle()'s
// (Manifold's Circle reduces angles by quadrant, not octant, so it is not
// exactly symmetric itself); a sphere and a coaxial circle of the same $fn
// agree to rounding only.
namespace {
// (cos, sin) of (90 * m / n) degrees, always computed from the angle at or
// below 45 degrees, so the points at m and n - m are exact mirrors.
std::array<double, 2> quarterArc(int m, int n) {
    if (2 * m == n) return {std::sqrt(0.5), std::sqrt(0.5)};
    if (2 * m > n) {
        const std::array<double, 2> p = quarterArc(n - m, n);
        return {p[1], p[0]};
    }
    const double t = (std::numbers::pi / 2.0) * m / n;
    return {std::cos(t), std::sin(t)};
}

using V3 = std::array<double, 3>;

// The unit-sphere point with barycentric weights a >= b >= c (a + b + c = n)
// toward three orthogonal axes, as coordinates along those axes.
V3 octaCanonical(int a, int b, int c, int n) {
    if (b == 0) return {1.0, 0.0, 0.0};
    if (c == 0) {
        const std::array<double, 2> e = quarterArc(b, n);
        return {e[0], e[1], 0.0};
    }
    // An interior vertex (all three weights non-zero): each weight picks a
    // great circle crossing the octant at that weight's level, and the
    // vertex is where the three meet -- the normalised mean of their
    // pairwise crossings, as three great circles do not quite share a point.
    const auto cross = [](const V3& u, const V3& v) -> V3 {
        return {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    };
    const auto normalised = [](V3 v) {
        const double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        return V3{v[0] / len, v[1] / len, v[2] / len};
    };
    const auto xy = [&](int m) { const auto e = quarterArc(m, n); return V3{e[0], e[1], 0.0}; };
    const auto xz = [&](int m) { const auto e = quarterArc(m, n); return V3{e[0], 0.0, e[1]}; };
    const auto yz = [&](int m) { const auto e = quarterArc(m, n); return V3{0.0, e[0], e[1]}; };

    // Each great circle as its plane's normal.
    const V3 cLevel = cross(xz(c), yz(c));
    const V3 bLevel = cross(xy(b), yz(n - b));
    const V3 aLevel = cross(xy(n - a), xz(n - a));
    const auto meet = [&](const V3& u, const V3& v) {
        V3 p = normalised(cross(u, v));
        if (p[0] + p[1] + p[2] < 0) p = {-p[0], -p[1], -p[2]};
        return p;
    };
    const V3 p = meet(cLevel, bLevel), q = meet(bLevel, aLevel), s = meet(aLevel, cLevel);
    V3 v = normalised({p[0] + q[0] + s[0], p[1] + q[1] + s[1], p[2] + q[2] + s[2]});
    if (a == b) v[1] = v[0];
    if (b == c) v[2] = v[1];
    return v;
}
} // namespace

SphereMesh buildOcta(double r, int segments) {
    const int n = std::max(1, (segments + 3) / 4);
    SphereMesh m;
    std::map<std::array<int, 3>, int> index;  // signed barycentric weights -> vertex
    const auto vertex = [&](int i, int j, int k, int sx, int sy, int sz) {
        const std::array<int, 3> key{sx * i, sy * j, sz * k};
        auto [it, fresh] = index.try_emplace(key, static_cast<int>(m.verts.size() / 3));
        if (fresh) {
            const std::array<int, 3> w{i, j, k};
            std::array<int, 3> axis{0, 1, 2};
            std::stable_sort(axis.begin(), axis.end(), [&](int p, int q) { return w[p] > w[q]; });
            const V3 c = octaCanonical(w[axis[0]], w[axis[1]], w[axis[2]], n);
            V3 p{};
            for (int rank = 0; rank < 3; ++rank) p[axis[rank]] = c[rank];
            pushVert(m, {r * sx * p[0], r * sy * p[1], r * sz * p[2]});
        }
        return it->second;
    };
    for (int sx : {1, -1})
        for (int sy : {1, -1})
            for (int sz : {1, -1}) {
                const bool flip = sx * sy * sz < 0;
                const auto tri = [&](int a, int b, int c) {
                    if (flip) std::swap(b, c);
                    m.tris.insert(m.tris.end(), {a, b, c});
                };
                // "Up" triangles toward +x, +y, +z, then the "down" ones
                // between them; both wind outward in the (+,+,+) octant.
                for (int i = 0; i < n; ++i)
                    for (int j = 0; i + j < n; ++j) {
                        const int k = n - 1 - i - j;
                        tri(vertex(i + 1, j, k, sx, sy, sz), vertex(i, j + 1, k, sx, sy, sz),
                            vertex(i, j, k + 1, sx, sy, sz));
                        if (k >= 1)
                            tri(vertex(i, j + 1, k, sx, sy, sz), vertex(i + 1, j, k, sx, sy, sz),
                                vertex(i + 1, j + 1, k - 1, sx, sy, sz));
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
    // r/d: see lookupRadius (spec section B1); r=1 by default.
    const double r = lookupRadius(ev, args, 0, 1, "r", "d", &node.position()).value_or(1.0);

    const Value styleArg = getArg(args, 2, "style", Value{});
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

    const int hsides = fnSegmentsFromCtx(effCtx, r, [&](const std::string& m) { ev.warn(m, &node.position()); });
    const int vsides = std::max(2, static_cast<int>(std::ceil(hsides / 2.0)));

    CSGParams params;
    params["style"] = Value{style};
    params["r"] = Value{r};
    params["segs"] = Value{static_cast<double>(hsides)};
    params["color"] = colorToValue(effCtx.color);

    // Nothing for r <= 0 or a non-finite r. A
    // negative r built an inside-out sphere that corrupted any boolean.
    const bool valid = r > 0 && std::isfinite(r);
    const SphereMesh m = !valid               ? SphereMesh{}
                         : style == "aligned" ? buildAligned(r, hsides, vsides, false)
                         : style == "stagger" ? buildAligned(r, hsides, vsides, true)
                         : style == "icosa"   ? buildIcosa(r, hsides)
                         : style == "octa"    ? buildOcta(r, hsides)
                                              : buildOrig(r, hsides, vsides);

    std::vector<Value> vertsValues;
    vertsValues.reserve(m.verts.size());
    for (double v : m.verts) vertsValues.push_back(Value{v});
    std::vector<Value> trisValues;
    trisValues.reserve(m.tris.size());
    for (int t : m.tris) trisValues.push_back(Value{static_cast<double>(t)});
    params["verts"] = Value{makeList(std::move(vertsValues))};
    params["tris"] = Value{makeList(std::move(trisValues))};
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
    manifold::Manifold body(mesh);
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// cylinder(h, r1, r2, center) -- via manifold::Manifold::Cylinder directly
// (unlike sphere, its tessellation already matches real OpenSCAD).

CSGParams resolveCylinder(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    // cylinder(h, r1, r2, center, r, d, d1, d2). r (or d) sets both ends,
    // then r1/d1 and r2/d2 override their own end; mixing r with either is
    // ambiguous and warned about.
    const Value hArg = getArg(args, 0, "h");
    const double h = std::holds_alternative<double>(hArg) ? std::get<double>(hArg) : 1.0;
    const Value centerArg = getArg(args, 3, "center");
    const bool center = std::holds_alternative<bool>(centerArg) && std::get<bool>(centerArg);

    const oscad::Position* where = &node.position();
    const std::optional<double> r = lookupRadius(ev, args, 4, 5, "r", "d", where);
    const std::optional<double> bottom = lookupRadius(ev, args, 1, 6, "r1", "d1", where);
    const std::optional<double> top = lookupRadius(ev, args, 2, 7, "r2", "d2", where);
    if (r && (bottom || top)) ev.warn("Cylinder parameters ambiguous", where);
    const double r1 = bottom.value_or(r.value_or(1.0));
    const double r2 = top.value_or(r.value_or(1.0));

    const int segs =
        fnSegmentsFromCtx(effCtx, std::max(r1, r2), [&](const std::string& m) { ev.warn(m, &node.position()); });

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
    // Nothing for h <= 0, a negative radius,
    // both radii zero, or anything non-finite. A negative r2 drew a prism.
    const bool valid = h > 0 && std::isfinite(h) && r1 >= 0 && r2 >= 0 && std::isfinite(r1) && std::isfinite(r2) &&
                       (r1 > 0 || r2 > 0);
    manifold::Manifold body = valid ? manifold::Manifold::Cylinder(h, r1, r2, segs, center) : manifold::Manifold();
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

namespace {
// Validates polyhedron()'s points and faces (after the object()/VNF forms
// have been unpacked): `points` gets one [x, y, z] per input point, `faces`
// the faces to draw, each a list of indices in range for `points`. False
// when nothing at all can be drawn.
bool readPolyhedronInput(Evaluator& ev, const Value& points, const Value& faces, const oscad::Position* where,
                         std::vector<std::array<double, 3>>& outPoints, std::vector<std::vector<size_t>>& outFaces) {
    const ListPtr* pointList = std::get_if<ListPtr>(&points);
    if (!pointList) {
        emitInputError(ev, "Unable to convert points = " + fmtValue(points) + " to a vector of coordinates", where);
        return false;
    }
    // A point is two or three finite numbers (z defaults to 0). A bad one
    // becomes the origin rather than being dropped, so later indices hold.
    const auto& items = (*pointList)->items;
    outPoints.clear();
    for (size_t i = 0; i < items.size(); ++i) {
        std::array<double, 3> xyz{0.0, 0.0, 0.0};
        const ListPtr* coords = std::get_if<ListPtr>(&items[i]);
        const size_t dims = coords ? (*coords)->items.size() : 0;
        bool ok = dims == 2 || dims == 3;
        for (size_t c = 0; ok && c < dims; ++c) {
            const double* n = std::get_if<double>(&(*coords)->items[c]);
            ok = n && std::isfinite(*n);
            if (ok) xyz[c] = *n;
        }
        if (!ok) {
            emitInputError(ev,
                           "Unable to convert points[" + std::to_string(i) + "] = " + fmtValue(items[i]) +
                               " to a vec3 of numbers",
                           where);
            xyz = {0.0, 0.0, 0.0};
        }
        outPoints.push_back(xyz);
    }

    const ListPtr* faceList = std::get_if<ListPtr>(&faces);
    if (!faceList) {
        emitInputError(ev, "Unable to convert faces = " + fmtValue(faces) + " to a vector of vector of point indices",
                       where);
        return false;
    }
    outFaces.clear();
    for (auto& face : readIndexLists(ev, **faceList, "faces", outPoints.size(), where)) {
        if (face.size() >= 3) outFaces.push_back(std::move(face));
    }
    return true;
}
} // namespace

CSGParams resolvePolyhedron(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    Value pointsArg = getArg(args, 0, "points", Value{});
    Value facesArg = getArg(args, 1, "faces", Value{});
    if (isUndef(facesArg)) {
        // The legacy alias, with OpenSCAD 2026.02.01's own notice.
        facesArg = getArg(args, std::nullopt, "triangles", Value{});
        if (!isUndef(facesArg))
            ev.emitWarning("DEPRECATED: polyhedron(triangles=[]) will be removed in future releases. Use "
                           "polyhedron(faces=[]) instead." +
                           locSuffix(&node.position()));
    }

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

    // polyhedron(vnf) -- BOSL2's [vertices, faces] 2-list, which is what
    // every BOSL2 function passes around and what obj.vnf holds. Only
    // considered when `faces` was not given separately, so the two-argument
    // form always wins and can never be reinterpreted.
    //
    // The discriminator is BOSL2's own is_vnf test: a VNF's second element
    // is a list of FACES, i.e. a list of lists. A plain points list of
    // length 2 has a point there instead -- bare numbers, not lists -- and
    // is left alone. (Two points cannot describe a polyhedron anyway, so
    // nothing legitimate is being taken over.)
    if (isUndef(facesArg)) {
        if (const ListPtr* outer = std::get_if<ListPtr>(&pointsArg); outer && *outer &&
            (*outer)->items.size() == 2) {
            const ListPtr* maybeVerts = std::get_if<ListPtr>(&(*outer)->items[0]);
            const ListPtr* maybeFaces = std::get_if<ListPtr>(&(*outer)->items[1]);
            const bool facesAreLists =
                maybeFaces && *maybeFaces && !(*maybeFaces)->items.empty() &&
                std::holds_alternative<ListPtr>((*maybeFaces)->items[0]);
            const bool vertsArePoints =
                maybeVerts && *maybeVerts && !(*maybeVerts)->items.empty() &&
                std::holds_alternative<ListPtr>((*maybeVerts)->items[0]);
            if (facesAreLists && vertsArePoints) {
                // Copy before assigning: both borrow into pointsArg's list.
                Value newPoints = (*outer)->items[0];
                Value newFaces = (*outer)->items[1];
                pointsArg = std::move(newPoints);
                facesArg = std::move(newFaces);
            }
        }
    }

    std::vector<std::array<double, 3>> rawVerts;
    std::vector<std::vector<size_t>> faces;
    if (!readPolyhedronInput(ev, pointsArg, facesArg, &node.position(), rawVerts, faces)) {
        CSGParams params;
        params["verts"] = Value{makeList({})};
        params["tris"] = Value{makeList({})};
        params["color"] = colorToValue(effCtx.color);
        return params;
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
    for (const std::vector<size_t>& face : faces)
        if (face.size() >= 3) triangulateFace(rawVerts, face, rawTris);

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
    params["verts"] = Value{makeList(std::move(vertsValues))};
    params["tris"] = Value{makeList(std::move(trisValues))};
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
    // No faces at all is empty geometry, not a failure. BOSL2's debug_vnf()
    // passes [verts, []] to draw vertex labels with no surface, and real
    // OpenSCAD accepts `polyhedron(points=..., faces=[])` silently. Checked
    // before the status branches below, which would otherwise report the
    // empty mesh as NotManifold and discard it with a warning.
    if (mesh.triVerts.empty()) {
        return {ev.tagGenerated(manifold::Manifold(), node, params.at("color"))};
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
        if (!ev.insideHull) {
            const BoundaryEdges open = findBoundaryEdges(mesh);
            // Name one of them. Where the hole IS is the only part of this
            // message an author can act on; the count alone just says to go
            // looking, across the whole mesh.
            std::string what;
            if (open.count) {
                what = "mesh is not closed -- " + std::to_string(open.count) + " boundary edge(s)" +
                       (open.first ? ", first at " + describeEdge(mesh, *open.first) : std::string());
            } else if (open.misoriented) {
                // Closed, but some faces wound backwards: "not closed -- 0
                // boundary edge(s)" contradicted itself (BelfrySCAD #566).
                what = "faces are not consistently wound -- " + std::to_string(open.misoriented) +
                       " edge(s) run the same way by both faces sharing them" +
                       (open.firstMisoriented ? ", first at " + describeEdge(mesh, *open.firstMisoriented)
                                              : std::string()) +
                       "; reverse the point order of the faces on one side of it";
            } else {
                what = "mesh is not a valid solid (" + checkMesh(mesh).summary() + ")";
            }
            ev.warn("polyhedron: " + what +
                        // "drawing it as a surface" was read by a reporter as
                        // the HOLE being surfaced over -- i.e. as automatic
                        // repair hiding their defect. Nothing is repaired: the
                        // mesh is drawn as the open surface it is. Name the
                        // object as the thing being drawn, and say what it is
                        // NOT, since "surface" alone does not imply "not solid"
                        // to someone who did not know it could be either.
                        (open.count ? "; drawing the object as an open surface rather than a "
                                    : "; drawing the object as a surface rather than a ") +
                        "solid -- nothing is patched. hull() can still "
                        "use its points, but it cannot take part in union/difference/intersection",
                    &node.position());
        }
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
