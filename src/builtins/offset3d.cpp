// offset() in 3D -- r=, delta= and chamfer=, outward and inward.
//
// One construction for all three modes. Outward:
//     offset(A, d) = A  U  slabs(faces)  U  wedges(convex edges)  U  caps(vertices)
// where a slab is hull(tri, tri + d*n) and the modes differ only in the
// wedge/cap kernel:
//     chamfer: hull of the feature and its faces' offset points (flat cut)
//     delta:   chamfer + the miter points (planes extended until they meet),
//              clipped to the shifted half-spaces that bound them
//     r:       the triangle swept by an inscribed faceted ball
// Inward is the same pieces built on A with every triangle flipped (concave
// and convex swap), subtracted from A:
//     offset(A, -d) = A - pieces(flip(A), d)
// Features thinner than 2d vanish because the pieces from both sides meet.
//
// Every piece is convex, so its own geometry is exact; what goes wrong is the
// union of thousands of them. Almost everything below that is not the
// construction itself exists to make that union trustworthy, and each guard
// names the failure it was added for. Ported function for function from the
// Python prototype the design was worked out and fuzzed in; see CLAUDE.md
// "offset() in 3D".

#include "openscad_cpp_evaluator/offset3d.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

namespace oscadeval {

namespace {

using manifold::Manifold;
using manifold::MeshGL64;
using manifold::OpType;
using manifold::vec3;
namespace la = manifold::la;

constexpr double kFlat = 1 - 1e-12;   // two normals this close: the same plane

// OFFSET3D_TIMING=1 prints each phase's wall time to stderr: the test hook
// for tuning, costing one getenv per offset otherwise.
struct PhaseTimer {
    bool on = std::getenv("OFFSET3D_TIMING") != nullptr;
    std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
    void lap(const char* what) {
        if (!on) return;
        const auto now = std::chrono::steady_clock::now();
        std::fprintf(stderr, "offset3d %-10s %8.3f s\n", what, std::chrono::duration<double>(now - t).count());
        t = now;
    }
};

struct Mesh {
    std::vector<vec3> V;
    std::vector<std::array<int, 3>> T;
};

Mesh meshOf(const Manifold& m) {
    const MeshGL64 g = m.GetMeshGL64();
    Mesh out;
    const size_t np = g.numProp, nv = g.NumVert(), nt = g.NumTri();
    out.V.resize(nv);
    for (size_t i = 0; i < nv; ++i)
        out.V[i] = {g.vertProperties[i * np], g.vertProperties[i * np + 1], g.vertProperties[i * np + 2]};
    out.T.resize(nt);
    for (size_t i = 0; i < nt; ++i)
        out.T[i] = {static_cast<int>(g.triVerts[3 * i]), static_cast<int>(g.triVerts[3 * i + 1]),
                    static_cast<int>(g.triVerts[3 * i + 2])};
    return out;
}

Manifold fromMesh(const std::vector<vec3>& V, const std::vector<std::array<int, 3>>& T) {
    MeshGL64 g;
    g.numProp = 3;
    g.vertProperties.reserve(V.size() * 3);
    for (const vec3& v : V) g.vertProperties.insert(g.vertProperties.end(), {v.x, v.y, v.z});
    g.triVerts.reserve(T.size() * 3);
    for (const auto& t : T) g.triVerts.insert(g.triVerts.end(), {uint64_t(t[0]), uint64_t(t[1]), uint64_t(t[2])});
    return Manifold(g);
}

// Just the vertices and triangles of x, as a new leaf (see whitenedHull).
Manifold bare(const Manifold& x) {
    const Mesh mm = meshOf(x);
    return fromMesh(mm.V, mm.T);
}

double maxAbs(const std::vector<vec3>& pts) {
    double s = 0;
    for (const vec3& p : pts) s = std::max({s, std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)});
    return s;
}

struct Planes {
    std::vector<vec3> n;
    std::vector<double> c;
};

Planes planesOf(const Manifold& m) {
    const Mesh mesh = meshOf(m);
    Planes out;
    for (const auto& t : mesh.T) {
        const vec3 a = mesh.V[t[0]];
        vec3 n = la::cross(mesh.V[t[1]] - a, mesh.V[t[2]] - a);
        const double l = la::length(n);
        if (l <= 0) continue;
        n /= l;
        out.n.push_back(n);
        out.c.push_back(la::dot(n, a));
    }
    return out;
}

std::optional<vec3> solve3(const vec3& r0, const vec3& r1, const vec3& r2, double b0, double b1, double b2) {
    const vec3 c12 = la::cross(r1, r2), c20 = la::cross(r2, r0), c01 = la::cross(r0, r1);
    const double det = la::dot(r0, c12);
    if (det == 0 || !std::isfinite(det)) return std::nullopt;
    const vec3 x = (b0 * c12 + b1 * c20 + b2 * c01) / det;
    if (!std::isfinite(x.x) || !std::isfinite(x.y) || !std::isfinite(x.z)) return std::nullopt;
    return x;
}

// Eigen-decomposition of a symmetric 3x3 (cyclic Jacobi): Q's columns are the
// eigenvectors of A, lam the eigenvalues.
void eigenSym3(double A[3][3], double Q[3][3], double lam[3]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Q[i][j] = i == j;
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = A[0][1] * A[0][1] + A[0][2] * A[0][2] + A[1][2] * A[1][2];
        if (off <= 1e-300) break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (A[p][q] == 0) continue;
                const double th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
                const double t = (th >= 0 ? 1 : -1) / (std::fabs(th) + std::sqrt(th * th + 1));
                const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; ++k) {
                    const double akp = A[k][p], akq = A[k][q];
                    A[k][p] = c * akp - s * akq;
                    A[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = A[p][k], aqk = A[q][k];
                    A[p][k] = c * apk - s * aqk;
                    A[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double qkp = Q[k][p], qkq = Q[k][q];
                    Q[k][p] = c * qkp - s * qkq;
                    Q[k][q] = s * qkp + c * qkq;
                }
            }
    }
    for (int i = 0; i < 3; ++i) lam[i] = A[i][i];
}

// Manifold::Hull in a frame where the points spread equally on every axis,
// mapped back. A hull is affine-invariant, and Manifold's lost whole corners
// of a long thin set (a chamfer wedge 3000 by 0.1 sealed a void in a rod's
// offset).
Manifold whitenedHull(const std::vector<vec3>& U) {
    vec3 c(0.0);
    for (const vec3& p : U) c += p;
    c /= double(U.size());
    double A[3][3] = {};
    for (const vec3& p : U) {
        const vec3 q = p - c;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) A[i][j] += q[i] * q[j];
    }
    for (auto& row : A)
        for (double& x : row) x /= double(U.size());
    double Q[3][3], lam[3];
    eigenSym3(A, Q, lam);
    double s[3];
    for (int i = 0; i < 3; ++i) s[i] = std::sqrt(std::max(lam[i], 0.0));
    const double smax = std::max({s[0], s[1], s[2]}), smin = std::min({s[0], s[1], s[2]});
    if (smin <= 1e-12 * smax) return Manifold::Hull(U);   // flat: no volume either way
    std::vector<vec3> Y;
    Y.reserve(U.size());
    for (const vec3& p : U) {
        const vec3 q = p - c;
        vec3 y;
        for (int j = 0; j < 3; ++j) y[j] = (Q[0][j] * q.x + Q[1][j] * q.y + Q[2][j] * q.z) / s[j];
        Y.push_back(y);
    }
    const Manifold h = Manifold::Hull(Y);
    if (h.IsEmpty()) return h;
    manifold::mat3x4 M;
    for (int j = 0; j < 3; ++j) M[j] = vec3(Q[0][j], Q[1][j], Q[2][j]) * s[j];   // x = c + M y
    M[3] = c;
    // Rebuilt from its mesh, not returned transformed: a Transform()ed
    // Manifold keeps state from the whitened frame, and Manifold's unions of
    // such pieces lost whole wedges (seed 13: a union that ADDED six pieces
    // came out 0.006 smaller) where the same meshes reloaded unioned exactly.
    return bare(h.Transform(M));
}

// Convex hull, deduplicated and verified. Manifold's hull came out SMALLER
// when exact-duplicate points were added (a 3.5 corner cube, 42.9 -> 35.9):
// every piece must contain the points it was built from, so check that, and
// retry with points merged on a coarser grid.
Manifold hullChecked(const std::vector<vec3>& P) {
    if (P.size() < 4) return Manifold();
    const double scale = std::max(1.0, maxAbs(P));
    Manifold h;
    for (const double grid : {1e-12, 1e-9, 1e-7}) {
        const double q = scale * grid;
        std::set<std::array<long long, 3>> keys;
        for (const vec3& p : P)
            keys.insert({std::llround(p.x / q), std::llround(p.y / q), std::llround(p.z / q)});
        if (keys.size() < 4) return Manifold();
        std::vector<vec3> U;
        U.reserve(keys.size());
        for (const auto& k : keys) U.push_back(vec3(double(k[0]), double(k[1]), double(k[2])) * q);
        h = whitenedHull(U);
        if (h.IsEmpty()) return h;
        // Manifold's hull drops points within ~1e-8 of a facet on purpose;
        // only a gross miss (the corner cube was off by whole units) is a
        // failure worth a retry.
        const Planes pl = planesOf(h);
        bool ok = true;
        for (size_t i = 0; ok && i < U.size(); ++i)
            for (size_t j = 0; j < pl.n.size(); ++j)
                if (la::dot(U[i], pl.n[j]) > pl.c[j] + 1e-6 * scale) { ok = false; break; }
        if (ok) return h;
    }
    return h;
}

// Inscribed: every vertex on the true sphere, so the faceted offset lies
// within the exact one, as OpenSCAD's own circles and 2D offset(r=) do.
std::vector<vec3> sphereDirs(int segs) {
    const int rings = std::max(2, segs / 2);
    std::vector<vec3> out;
    for (int i = 0; i <= rings; ++i) {
        const double th = M_PI * i / rings;
        for (int j = 0; j < segs; ++j) {
            const double ph = 2 * M_PI * j / segs;
            out.push_back({std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)});
        }
    }
    return out;
}

// The normals, one of each direction, to the same 1e-12 that calls an edge
// flat -- at 1e-7 a knife's two faces 4e-4 apart merged, and its apex got no
// cap at all.
std::vector<vec3> distinct(const std::vector<vec3>& ns, double tol = 1e-12) {
    std::vector<char> keep(ns.size(), 0);
    for (size_t k = 0; k < ns.size(); ++k) {
        size_t first = k;
        for (size_t j = 0; j < ns.size(); ++j)
            if (la::dot(ns[k], ns[j]) >= 1 - tol) { first = j; break; }
        keep[first] = 1;
    }
    std::vector<vec3> out;
    for (size_t k = 0; k < ns.size(); ++k)
        if (keep[k]) out.push_back(ns[k]);
    return out;
}

// The point t along p->q, at most halfway. An absolute sliver, not a fraction
// of the edge: a fraction moved when refine() split the edge, and the
// unclipped caps at saddles changed the result by 1e-3 thick sheets.
vec3 toward(const vec3& p, const vec3& q, double t) {
    const vec3 u = q - p;
    const double L = la::length(u);
    return L > 0 ? p + std::min(t, 0.5 * L) / L * u : p;
}

vec3 miter(const vec3& n1, const vec3& n2) { return (n1 + n2) / (1 + la::dot(n1, n2)); }

// A convex vertex's mitered corner is Q = {x : n.(x - v) <= d}, and its
// corners are the facets of the polar dual hull(0, n/d) that miss the origin
// -- O(k log k), where every triple of planes was O(k^3) (3 s at a 64-face
// cone apex in the prototype). Triples are still cheaper for a few faces.
std::vector<vec3> cornerPoints(const vec3& pv, const std::vector<vec3>& ns, double d) {
    std::vector<vec3> out;
    const size_t k = ns.size();
    if (k <= 6) {
        for (size_t i = 0; i < k; ++i)
            for (size_t j = i + 1; j < k; ++j)
                for (size_t l = j + 1; l < k; ++l) {
                    const auto x = solve3(ns[i], ns[j], ns[l], la::dot(ns[i], pv) + d, la::dot(ns[j], pv) + d,
                                          la::dot(ns[l], pv) + d);
                    if (!x) continue;
                    bool in = true;
                    for (const vec3& n : ns)
                        if (la::dot(n, *x - pv) > d * (1 + 1e-9)) { in = false; break; }
                    if (in) out.push_back(*x);
                }
        return out;
    }
    std::vector<vec3> dualPts{vec3(0.0)};
    for (const vec3& n : ns) dualPts.push_back(n / d);
    const Mesh dual = meshOf(hullChecked(dualPts));
    for (const auto& t : dual.T) {
        const vec3 a = dual.V[t[0]], b = dual.V[t[1]], c = dual.V[t[2]];
        if (std::min({la::length(a), la::length(b), la::length(c)}) < 1e-12) continue;   // unbounded
        if (const auto x = solve3(a, b, c, 1, 1, 1)) out.push_back(pv + *x);
    }
    return out;
}

// w cut to every n.x <= o, but only by planes that still cut it: the one it
// violates most each time, rechecked after every cut. Cutting by each plane in
// turn made a 256-face cone apex 51 s; most stop cutting once the first few
// have.
Manifold trimAll(Manifold w, std::vector<vec3> normals, std::vector<double> offsets, double tol) {
    while (!w.IsEmpty() && !normals.empty()) {
        const Mesh mesh = meshOf(w);
        size_t best = 0;
        double worst = -std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < normals.size(); ++j) {
            double over = -std::numeric_limits<double>::infinity();
            for (const vec3& v : mesh.V) over = std::max(over, la::dot(v, normals[j]) - offsets[j]);
            if (over > worst) { worst = over; best = j; }
        }
        if (worst <= tol) break;
        w = w.TrimByPlane(-normals[best], -offsets[best]);
        normals.erase(normals.begin() + best);
        offsets.erase(offsets.begin() + best);
    }
    return w;
}

// Keep n.(x - pv) <= d: exact plane cuts, at any scale.
Manifold clipPlanes(const Manifold& w, const std::vector<vec3>& normals, const vec3& pv, double d) {
    const std::vector<vec3> ns = distinct(normals);
    std::vector<double> offs;
    for (const vec3& n : ns) offs.push_back(la::dot(n, pv) + d);
    const double s = std::max({1.0, std::fabs(pv.x), std::fabs(pv.y), std::fabs(pv.z)});
    return trimAll(w, ns, offs, 1e-12 * s);
}

struct Edge {
    int a, b;
    std::vector<int> ts;
};

// fn(0..n-1) on every core. ponytail: one thread per core over an atomic
// counter; Manifold's own TBB work nests inside, which TBB handles.
void parallelFor(size_t n, const std::function<void(size_t)>& fn) {
    std::atomic<size_t> next{0};
    auto worker = [&] {
        for (size_t i; (i = next++) < n;) fn(i);
    };
    const unsigned k = unsigned(std::min<size_t>(std::max(1u, std::thread::hardware_concurrency()), n));
    std::vector<std::thread> pool;
    for (unsigned t = 1; t < k; ++t) pool.emplace_back(worker);
    worker();
    for (std::thread& t : pool) t.join();
}

class Builder {
    // Each piece is a job, run in parallel by build(): the planning that
    // decides what to build (and fills the wedge bookkeeping the caps read)
    // stays serial, the hulls and trims run on every core.
    using Jobs = std::vector<std::function<Manifold()>>;

public:
    Builder(const std::vector<vec3>& V, std::vector<std::array<int, 3>> T, double d, Offset3DMode mode, int segs, bool flip)
        : V_(V), T_(std::move(T)), d_(d), mode_(mode), ball_(sphereDirs(segs)) {
        if (flip)
            for (auto& t : T_) std::swap(t[0], t[2]);
        const size_t nt = T_.size();
        N_.resize(nt);
        off_.resize(nt);
        for (size_t t = 0; t < nt; ++t) {
            const vec3 a = V_[T_[t][0]];
            vec3 n = la::cross(V_[T_[t][1]] - a, V_[T_[t][2]] - a);
            const double l = la::length(n);
            N_[t] = l > 0 ? n / l : vec3(0.0);
            off_[t] = la::dot(N_[t], a);
        }
        scale_ = std::max(1.0, maxAbs(V_));
        // Every piece starts a hair behind the surface (inside A outward,
        // outside it inward), so no piece face coincides with A's own: a
        // coincident face leaves a rounding-thin skin when the pieces are
        // subtracted. Relative to the mesh too: 1e-5*d was below Manifold's
        // precision on a 3e4-long rod, which then sealed a sliver void. Any
        // e <= d is safe: a piece pushed through a thin wall stays within d.
        e_ = std::min(d, std::max(1e-5 * d, 1e-9 * scale_));
    }

    std::vector<Manifold> build() {
        Jobs out;
        plan(out);
        std::vector<Manifold> made(out.size());
        parallelFor(out.size(), [&](size_t i) {
            const Manifold m = out[i]();
            if (!m.IsEmpty()) made[i] = bare(m);
        });
        std::vector<Manifold> res;
        for (Manifold& m : made)
            if (!m.IsEmpty()) res.push_back(std::move(m));
        return res;
    }

private:
    void plan(Jobs& out) {
        if (mode_ == Offset3DMode::Round) {
            // Each triangle swept by the one faceted ball: exactly the
            // Minkowski sum with it.
            for (size_t t = 0; t < T_.size(); ++t) {
                std::vector<vec3> pts;
                for (int k = 0; k < 3; ++k) pts.push_back(V_[T_[t][k]] - e_ * N_[t]);
                for (int k = 0; k < 3; ++k)
                    for (const vec3& b : ball_) pts.push_back(V_[T_[t][k]] + d_ * b);
                push(out, [pts] { return hullChecked(pts); });
            }
            return;
        }
        indexEdges();
        classify();
        slabs(out);
        wedges(out);
        caps(out);
    }

    const std::vector<vec3>& V_;
    std::vector<std::array<int, 3>> T_;
    double d_;
    Offset3DMode mode_;
    std::vector<vec3> ball_;
    std::vector<vec3> N_;
    std::vector<double> off_;
    double scale_ = 1, e_ = 0;
    static constexpr double kLam = 1e-3;

    std::vector<Edge> edges_;
    std::unordered_map<uint64_t, size_t> edgeIndex_;
    std::vector<std::vector<int>> vertFaces_;
    std::vector<char> concaveAt_, convexAt_;
    std::vector<std::vector<std::array<vec3, 2>>> convEdgeNormals_;   // per vertex: (n1, n2)
    std::vector<std::vector<int>> convEdgeOther_;                      // per vertex: the other end
    std::vector<std::array<std::array<double, 4>, 3>> sides_;
    std::vector<std::vector<int>> cnbr_;
    std::vector<int> comp_, flat_;
    std::set<int> concaveComp_;
    std::map<int, std::vector<int>> members_;
    std::map<int, bool> gone_;

    static void push(Jobs& out, std::function<Manifold()> job) { out.push_back(std::move(job)); }

    uint64_t key(int a, int b) const {
        return (uint64_t(std::min(a, b)) << 32) | uint64_t(std::max(a, b));
    }

    int otherVertex(int t, int a, int b) const {
        for (int v : T_[t])
            if (v != a && v != b) return v;
        return T_[t][0];
    }

    int across(int t, int a, int b) const {
        const Edge& e = edges_[edgeIndex_.at(key(a, b))];
        return e.ts[0] == t ? e.ts[1] : e.ts[0];
    }

    bool isFlat(int t, int u) const { return la::dot(N_[t], N_[u]) > kFlat; }

    // Convex: not flat, and the far triangle's third vertex behind t's plane.
    bool isConvex(int t, int u, int a, int b) const {
        return !isFlat(t, u) && la::dot(N_[t], V_[otherVertex(u, a, b)] - V_[a]) < 0;
    }

    void indexEdges() {
        for (size_t t = 0; t < T_.size(); ++t)
            for (int i = 0; i < 3; ++i) {
                const int a = T_[t][i], b = T_[t][(i + 1) % 3];
                auto [it, fresh] = edgeIndex_.emplace(key(a, b), edges_.size());
                if (fresh) edges_.push_back({std::min(a, b), std::max(a, b), {}});
                edges_[it->second].ts.push_back(int(t));
            }
        vertFaces_.assign(V_.size(), {});
        for (size_t t = 0; t < T_.size(); ++t)
            for (int v : T_[t]) vertFaces_[v].push_back(int(t));
    }

    // Slabs, each widened by e past its concave and flat edges. Otherwise a
    // slab's side wall at a right-angled edge lies exactly in the next face's
    // plane, and subtracting it from A left a zero-thickness fin there
    // (inward delta: vertices 0.16 from the surface at d=0.5, genus -87).
    // Past a concave edge the sliver lands inside A outward, outside A
    // inward, and past a flat one inside the neighbour's slab: within d of the
    // surface every time. Convex edges have their wedges' strips.
    // The boundary of a convex flat face, or nothing if the face is not convex
    // (or has a hole): every face vertex on the inner side of every boundary
    // edge. q gets the face's vertices, plus each non-convex boundary edge's
    // ends pushed e outward, as the triangle slabs below widen them.
    bool convexFace(int g, const std::vector<int>& ts, std::vector<vec3>& q) const {
        const vec3 n = N_[ts[0]];
        std::set<int> vs;
        for (int t : ts)
            for (int v : T_[t]) vs.insert(v);
        std::vector<vec3> wide;
        for (int t : ts)
            for (int i = 0; i < 3; ++i) {
                const int a = T_[t][i], b = T_[t][(i + 1) % 3];
                const int u = across(t, a, b);
                if (flat_[u] == g) continue;
                const vec3 in = la::cross(n, V_[b] - V_[a]);   // left of a->b: the face's side
                const double tol = 1e-9 * scale_ * la::length(in);
                for (int v : vs)
                    if (la::dot(in, V_[v] - V_[a]) < -tol) return false;
                if (isConvex(t, u, a, b)) continue;
                const vec3 k = -la::normalize(in);
                wide.push_back(V_[a] + e_ * k);
                wide.push_back(V_[b] + e_ * k);
            }
        std::vector<vec3> all;
        for (int v : vs) all.push_back(V_[v]);
        all.insert(all.end(), wide.begin(), wide.end());
        q = corners2D(all, n);
        return q.size() >= 3;
    }

    // The strict corners of coplanar points (normal n): a 2D hull in the
    // plane, collinear points dropped. refine() leaves midpoints along every
    // edge; Manifold's hull kept them as zero-area slivers (a 422-triangle
    // slab), and the union with them lost 0.27 of t41 seed 18 once moved back.
    std::vector<vec3> corners2D(const std::vector<vec3>& P, const vec3& n) const {
        const vec3 u = la::normalize(std::fabs(n.x) < 0.9 ? la::cross(n, vec3(1, 0, 0)) : la::cross(n, vec3(0, 1, 0)));
        const vec3 w = la::cross(n, u);
        std::vector<std::pair<std::array<double, 2>, size_t>> pts;
        for (size_t i = 0; i < P.size(); ++i) pts.push_back({{la::dot(P[i], u), la::dot(P[i], w)}, i});
        std::sort(pts.begin(), pts.end());
        const double tol = 1e-12 * scale_ * scale_;
        auto cross = [](const std::array<double, 2>& o, const std::array<double, 2>& a, const std::array<double, 2>& b) {
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
        };
        std::vector<size_t> h(2 * pts.size());
        size_t k = 0;
        for (size_t i = 0; i < pts.size(); ++i) {   // Andrew's monotone chain
            while (k >= 2 && cross(pts[h[k - 2]].first, pts[h[k - 1]].first, pts[i].first) <= tol) --k;
            h[k++] = i;
        }
        for (size_t i = pts.size() - 1, t = k + 1; i-- > 0;) {
            while (k >= t && cross(pts[h[k - 2]].first, pts[h[k - 1]].first, pts[i].first) <= tol) --k;
            h[k++] = i;
        }
        std::vector<vec3> out;
        for (size_t i = 0; i + 1 < k; ++i) out.push_back(P[pts[h[i]].second]);
        return out;
    }

    void slabs(Jobs& out) {
        // One slab per convex flat face: a fan of 254 slabs meeting at one
        // vertex (a cylinder's end) made the union several times slower.
        std::vector<char> done(T_.size(), 0);
        for (const auto& [g, ts] : members_) {
            std::vector<vec3> q;
            if (ts.size() < 2 || !convexFace(g, ts, q)) continue;
            for (int t : ts) done[t] = 1;
            const vec3 n = N_[ts[0]];
            std::vector<vec3> pts;
            for (const vec3& p : q) pts.push_back(p - e_ * n);
            for (const vec3& p : q) pts.push_back(p + d_ * n);
            push(out, [pts] { return hullChecked(pts); });
        }
        for (size_t t = 0; t < T_.size(); ++t) {
            if (done[t]) continue;
            const vec3 n = N_[t];
            std::vector<vec3> q;
            for (int k = 0; k < 3; ++k) q.push_back(V_[T_[t][k]]);
            for (int i = 0; i < 3; ++i) {
                const int a = T_[t][i], b = T_[t][(i + 1) % 3];
                const int u = across(int(t), a, b);
                if (isConvex(int(t), u, a, b)) continue;
                vec3 k = la::normalize(la::cross(n, V_[b] - V_[a]));
                if (la::dot(k, V_[T_[t][(i + 2) % 3]] - V_[a]) > 0) k = -k;
                q.push_back(V_[a] + e_ * k);
                q.push_back(V_[b] + e_ * k);
            }
            std::vector<vec3> pts;
            for (const vec3& p : q) pts.push_back(p - e_ * n);
            for (const vec3& p : q) pts.push_back(p + d_ * n);
            push(out, [pts] { return hullChecked(pts); });
        }
    }

    static int find(std::vector<int>& root, int i) {
        while (root[i] != i) {
            root[i] = root[root[i]];
            i = root[i];
        }
        return i;
    }

    void classify() {
        const size_t nv = V_.size(), nt = T_.size();
        // Convex means every neighbour on or behind every adjacent face's
        // plane. "No concave edge" is NOT the same: a vertex with only convex
        // edges can still have a neighbour in front of a face it doesn't
        // share an edge with, and clipping it as convex cut real material.
        concaveAt_.assign(nv, 0);
        convexAt_.assign(nv, 0);
        convEdgeNormals_.assign(nv, {});
        convEdgeOther_.assign(nv, {});
        std::vector<std::set<int>> nbr(nv);
        for (const Edge& e : edges_) {
            nbr[e.a].insert(e.b);
            nbr[e.b].insert(e.a);
        }
        for (size_t v = 0; v < nv; ++v) {
            double worst = -std::numeric_limits<double>::infinity();
            for (int t : vertFaces_[v])
                for (int u : nbr[v]) worst = std::max(worst, la::dot(N_[t], V_[u] - V_[v]));
            concaveAt_[v] = worst > 1e-9 * scale_;
        }

        // Each face's offset facet: its shifted plane, bounded at every edge
        // by the bisector with the neighbour (convex) or the side wall
        // (otherwise). Used only to tell whether the facet survives at d.
        sides_.assign(nt, {});
        for (size_t t = 0; t < nt; ++t)
            for (int i = 0; i < 3; ++i) {
                const int a = T_[t][i], b = T_[t][(i + 1) % 3];
                const int g = across(int(t), a, b);
                vec3 k;
                if (isConvex(int(t), g, a, b)) {
                    k = N_[g] - N_[t];
                } else {
                    k = la::cross(N_[t], V_[b] - V_[a]);
                    if (la::dot(k, V_[T_[t][(i + 2) % 3]] - V_[a]) > 0) k = -k;
                }
                k = la::normalize(k);
                sides_[t][i] = {k.x, k.y, k.z, la::dot(k, V_[a])};
            }
        // A face VANISHES when its facet -- its shifted plane inside its three
        // bisectors -- is inside out at d: a short face between sharp turns.
        // Only there does the offset surface invert and cancel another miter
        // (the winding rule Clipper2 applies in 2D); a spike that merely
        // crosses over a face's facet stays.
        std::vector<char> vanished(nt, 0);
        for (size_t t = 0; t < nt; ++t) {
            std::array<vec3, 3> q;
            bool ok = true;
            for (int k = 0; k < 3 && ok; ++k) {
                const auto& s0 = sides_[t][(k + 2) % 3];
                const auto& s1 = sides_[t][k];
                const auto x = solve3(N_[t], vec3(s0[0], s0[1], s0[2]), vec3(s1[0], s1[1], s1[2]), off_[t] + d_, s0[3],
                                      s1[3]);
                if (!x) ok = false;
                else q[k] = *x;
            }
            vanished[t] = !ok || la::dot(la::cross(q[1] - q[0], q[2] - q[0]), N_[t]) <= 0;
        }
        cnbr_.assign(nt, {});
        std::vector<int> root(nt), rootFlat(nt);
        std::iota(root.begin(), root.end(), 0);
        std::iota(rootFlat.begin(), rootFlat.end(), 0);
        for (const Edge& e : edges_) {
            const int t1 = e.ts[0], t2 = e.ts[1];
            if (isConvex(t1, t2, e.a, e.b)) {
                cnbr_[t1].push_back(t2);
                cnbr_[t2].push_back(t1);
            }
            root[find(root, t1)] = find(root, t2);
            if (isFlat(t1, t2)) rootFlat[find(rootFlat, t1)] = find(rootFlat, t2);
        }
        // A component with no concave edge is convex: its mitered offset is
        // the intersection of every shifted plane, chain or no chain (a
        // needle's long face 22 away cut its tip's miter).
        comp_.resize(nt);
        flat_.resize(nt);
        for (size_t t = 0; t < nt; ++t) {
            comp_[t] = find(root, int(t));
            flat_[t] = find(rootFlat, int(t));
        }
        for (const Edge& e : edges_) {
            const int t1 = e.ts[0], t2 = e.ts[1];
            if (!isFlat(t1, t2) && !isConvex(t1, t2, e.a, e.b)) concaveComp_.insert(comp_[t1]);
        }
        // Per planar FACE, not per triangle: a small triangle that refine()
        // cut off a face vanishes long before the face does.
        for (size_t t = 0; t < nt; ++t) members_[flat_[t]].push_back(int(t));
        for (const auto& [g, ts] : members_) {
            bool all = true;
            for (int t : ts) all = all && vanished[t];
            gone_[g] = all;
        }
    }

    // Faces whose shifted planes may clip a piece built on `start`.
    std::vector<char> reach(const std::vector<int>& start) const {
        std::vector<char> mask(T_.size(), 0);
        if (!concaveComp_.count(comp_[start[0]])) {
            for (size_t t = 0; t < T_.size(); ++t) mask[t] = comp_[t] == comp_[start[0]];
            return mask;
        }
        // From the piece's own faces, step only into vanished faces; from a
        // vanished face, every neighbour is in reach (the face beyond it whose
        // plane now bounds the miter). A plain neighbour of the piece's face
        // never clips it -- in 2D that cut a spike Clipper keeps.
        std::set<int> seen;
        std::vector<int> todo;
        for (int t : start)
            if (seen.insert(flat_[t]).second)
                for (int f : members_.at(flat_[t])) todo.push_back(f);
        while (!todo.empty()) {
            const int f = todo.back();
            todo.pop_back();
            for (int g : cnbr_[f]) {
                const int fg = flat_[g];
                if (seen.count(fg)) continue;
                if (gone_.at(fg)) {
                    seen.insert(fg);
                    for (int x : members_.at(fg)) todo.push_back(x);
                } else if (gone_.at(flat_[f])) {
                    seen.insert(fg);
                }
            }
        }
        for (int g : seen)
            for (int t : members_.at(g)) mask[t] = 1;
        return mask;
    }

    // Clip a mitered piece by the shifted planes of faces that do not meet at
    // its own vertex but bound the offset there anyway: a short face between
    // sharp turns drops out of a mitered offset, and the planes beside it
    // meet sooner than its neighbours' miters say. Clipper2 does this in 2D
    // (the half-plane intersection on a convex polygon whose short edge
    // vanishes); without it delta= stuck out 12*d past the true miter.
    Manifold clipFar(const Manifold& w, const std::vector<vec3>& base, const std::vector<vec3>& pts,
                     const std::vector<char>& mask) const {
        if (w.IsEmpty()) return w;
        std::vector<vec3> ns;
        std::vector<double> os;
        const double tol = 1e-9 * scale_;
        for (size_t j = 0; j < T_.size(); ++j) {
            if (!mask[j]) continue;
            bool behind = true;
            for (const vec3& b : base)
                if (la::dot(N_[j], b) - off_[j] > tol) { behind = false; break; }
            if (!behind) continue;
            double over = -std::numeric_limits<double>::infinity();
            for (const vec3& p : pts) over = std::max(over, la::dot(N_[j], p) - off_[j]);
            if (over > d_ + tol) {
                ns.push_back(N_[j]);
                os.push_back(off_[j] + d_);
            }
        }
        return trimAll(w, ns, os, tol);
    }

    // The ends of the straight edge that e is part of: the run of collinear
    // edges between the same two flat faces. An edge refine() split in two
    // is still one edge here, so its wedges are clipped as the whole one was
    // (by the midpoint's planes alone, a third face tilting over the edge at
    // the far end no longer cut the near half's miter: a 0.6 spike at d=0.7).
    std::pair<int, int> chainEnds(size_t ei) const {
        const Edge& e = edges_[ei];
        const auto pairOf = [&](const Edge& x) {
            const int g1 = flat_[x.ts[0]], g2 = flat_[x.ts[1]];
            return std::pair{std::min(g1, g2), std::max(g1, g2)};
        };
        const auto want = pairOf(e);
        auto walk = [&](int from, int v) {   // follow the run past v, away from `from`
            for (size_t guard = 0; guard < edges_.size(); ++guard) {
                int next = -1;
                for (int t : vertFaces_[v])
                    for (int i = 0; i < 3 && next < 0; ++i) {
                        const int a = T_[t][i], b = T_[t][(i + 1) % 3];
                        if (a != v && b != v) continue;
                        const int o = a == v ? b : a;
                        if (o == from) continue;
                        const Edge& x = edges_[edgeIndex_.at(key(a, b))];
                        if (x.ts.size() == 2 && pairOf(x) == want) next = o;
                    }
                if (next < 0 || next == e.a || next == e.b) return v;
                from = v;
                v = next;
            }
            return v;
        };
        return {walk(e.b, e.a), walk(e.a, e.b)};
    }

    // One wedge per straight convex edge, however many triangle edges it is
    // made of: refine() split an edge in two, and the halves' wedges met face
    // to face at the midpoint -- a zero-width crack (genus 1 for 0), which
    // Manifold later collapsed and took 0.27 of t41 seed 18's volume with it.
    void wedges(Jobs& out) {
        std::map<std::pair<int, int>, std::vector<size_t>> chains;
        for (size_t ei = 0; ei < edges_.size(); ++ei) {
            const Edge& e = edges_[ei];
            const int a = e.a, b = e.b, t1 = e.ts[0], t2 = e.ts[1];
            const vec3 n1 = N_[t1], n2 = N_[t2];
            if (!isConvex(t1, t2, a, b)) continue;   // flat or concave: the slabs cover it
            convexAt_[a] = convexAt_[b] = 1;
            convEdgeNormals_[a].push_back({n1, n2});
            convEdgeOther_[a].push_back(b);
            convEdgeNormals_[b].push_back({n1, n2});
            convEdgeOther_[b].push_back(a);
            const auto [ea, eb] = chainEnds(ei);
            chains[{std::min(ea, eb), std::max(ea, eb)}].push_back(ei);
        }
        for (const auto& [ends, members] : chains) {
            const auto [ea, eb] = ends;
            const Edge& e0 = edges_[members[0]];
            const vec3 n1 = N_[e0.ts[0]], n2 = N_[e0.ts[1]];
            const vec3 pa = V_[ea], pb = V_[eb];
            std::vector<vec3> pts{pa - e_ * (n1 + n2), pb - e_ * (n1 + n2), pa + d_ * n1, pb + d_ * n1,
                                  pa + d_ * n2, pb + d_ * n2};
            std::vector<int> tris;
            // A sliver of each adjacent slab, so the wedge overlaps both:
            // walls that touch exactly leave rounding-thin tunnels.
            for (size_t ei : members) {
                const Edge& e = edges_[ei];
                for (int t : e.ts) {
                    tris.push_back(t);
                    const vec3 c = V_[otherVertex(t, e.a, e.b)], n = N_[t];
                    for (const vec3& p : {V_[e.a], V_[e.b]})
                        for (const vec3& h : {-e_ * n, d_ * n}) pts.push_back(toward(p, c, kLam * d_) + h);
                }
            }
            if (mode_ != Offset3DMode::Delta) {
                push(out, [pts] { return hullChecked(pts); });
                continue;
            }
            const vec3 m = miter(n1, n2);
            pts.push_back(pa + d_ * m);
            pts.push_back(pb + d_ * m);
            // The miter must not run past any plane meeting at either end (a
            // sharp edge's miter line would otherwise overshoot a third face
            // that tilts over it).
            push(out, [this, pts, ea, eb, tris] {
                Manifold w = hullChecked(pts);
                for (const int v : {ea, eb})
                    if (!concaveAt_[v]) {
                        std::vector<vec3> ns;
                        for (int t : vertFaces_[v]) ns.push_back(N_[t]);
                        w = clipPlanes(w, ns, V_[v], d_);
                    }
                return clipFar(w, {V_[ea], V_[eb]}, pts, reach(tris));
            });
        }
    }

    void caps(Jobs& out) {
        for (size_t v = 0; v < V_.size(); ++v) {
            const std::vector<int>& faces = vertFaces_[v];
            if (faces.empty()) continue;
            std::vector<vec3> fn;
            for (int t : faces) fn.push_back(N_[t]);
            const std::vector<vec3> ns = distinct(fn);
            const vec3 pv = V_[v];
            vec3 sum(0.0);
            for (const vec3& n : ns) sum += n;
            if (!convexAt_[v]) {
                // A concave vertex: the point above it can project outside
                // every adjacent triangle, so no slab holds it. This hull is
                // inside every adjacent face's shifted half-space, so inside the
                // offset in every mode -- and it covers that region.
                if (ns.size() >= 2) {
                    std::vector<vec3> pts{pv - e_ * sum};
                    for (const vec3& n : ns) pts.push_back(pv + d_ * n);
                    for (int t : faces)
                        for (int c : T_[t])
                            if (c != int(v)) pts.push_back(toward(pv, V_[c], kLam * d_) + d_ * N_[t]);
                    push(out, [pts] { return hullChecked(pts); });
                }
                continue;
            }
            if (mode_ == Offset3DMode::Delta && concaveAt_[v] && ns.size() >= 2) {
                // A saddle: the flat fill below lies inside the rounded offset
                // there, so delta= came out smaller than r=. Every point within
                // d of v is in any true mitered offset, so the ball is safe.
                std::vector<vec3> pts;
                for (const vec3& b : ball_) pts.push_back(pv + d_ * b);
                push(out, [pts] { return hullChecked(pts); });
            }
            if (ns.size() < 3) continue;   // the edge wedges already meet here
            std::vector<vec3> pts{pv - e_ * sum};
            for (const vec3& n : ns) pts.push_back(pv + d_ * n);
            // A sliver of each convex edge's wedge, so the cap overlaps them.
            for (size_t i = 0; i < convEdgeOther_[v].size(); ++i) {
                const vec3 q = toward(pv, V_[convEdgeOther_[v][i]], kLam * d_);
                const vec3 n1 = convEdgeNormals_[v][i][0], n2 = convEdgeNormals_[v][i][1];
                pts.push_back(q + d_ * n1);
                pts.push_back(q + d_ * n2);
                if (mode_ == Offset3DMode::Delta) pts.push_back(q + d_ * miter(n1, n2));
            }
            if (mode_ != Offset3DMode::Delta || concaveAt_[v]) {
                push(out, [pts] { return hullChecked(pts); });
                continue;
            }
            for (const vec3& x : cornerPoints(pv, ns, d_)) pts.push_back(x);
            // Every pair's miter point as well: with only the adjacent pairs'
            // the cap no longer overlapped its wedges and a tunnel came back.
            // k^2/2 points made a 256-face apex cap a 32,640-point hull, so
            // past 16 faces its own edges' miters plus the exact corners do.
            if (ns.size() <= 16) {
                for (size_t i = 0; i < ns.size(); ++i)
                    for (size_t j = i + 1; j < ns.size(); ++j)
                        if (la::dot(ns[i], ns[j]) > -1 + 1e-9) pts.push_back(pv + d_ * miter(ns[i], ns[j]));
            } else {
                for (const auto& nn : convEdgeNormals_[v]) pts.push_back(pv + d_ * miter(nn[0], nn[1]));
            }
            push(out, [this, pts, pv, fn, faces] {
            Manifold cap = hullChecked(pts);
            // The cap is the hull of a few points of the mitered corner, which
            // need not hold the ball of radius d round v -- though the corner
            // always does: on a knife (every normal near +-y) the hull was a
            // sliver in x and r= stuck out of delta= above the apex.
            if (!cap.IsEmpty()) {
                const Planes pl = planesOf(cap);
                bool outside = false;
                for (size_t i = 0; !outside && i < ball_.size(); ++i) {
                    const vec3 bp = pv + d_ * ball_[i];
                    for (size_t j = 0; j < pl.n.size(); ++j)
                        if (la::dot(bp, pl.n[j]) > pl.c[j] + 1e-9 * scale_) { outside = true; break; }
                }
                if (outside) {
                    std::vector<vec3> all = pts;
                    for (const vec3& b : ball_) all.push_back(pv + d_ * b);
                    cap = hullChecked(all);
                }
            }
            cap = clipPlanes(cap, fn, pv, d_);
            return clipFar(cap, {pv}, pts, reach(faces));
            });
        }
    }
};

// Pieces in Morton order of their box centres, so the tree's early unions
// join neighbours (which overlap and simplify) instead of pieces from opposite
// ends of the part (built slabs-then-wedges-then-caps).
std::vector<Manifold> spatialOrder(const std::vector<Manifold>& ps) {
    if (ps.size() < 3) return ps;
    manifold::Box all;
    std::vector<vec3> cen(ps.size());
    for (size_t i = 0; i < ps.size(); ++i) {
        const manifold::Box b = ps[i].BoundingBox();
        cen[i] = (b.min + b.max) / 2.0;
        all.Union(cen[i]);
    }
    const vec3 size = la::max(all.max - all.min, vec3(1e-300));
    auto spread = [](uint64_t x) {   // 21 bits -> every third of 63
        x &= 0x1fffff;
        x = (x | x << 32) & 0x1f00000000ffffULL;
        x = (x | x << 16) & 0x1f0000ff0000ffULL;
        x = (x | x << 8) & 0x100f00f00f00f00fULL;
        x = (x | x << 4) & 0x10c30c30c30c30c3ULL;
        x = (x | x << 2) & 0x1249249249249249ULL;
        return x;
    };
    std::vector<std::pair<uint64_t, size_t>> keys(ps.size());
    for (size_t i = 0; i < ps.size(); ++i) {
        const vec3 f = (cen[i] - all.min) / size * double((1 << 21) - 1);
        keys[i] = {spread(uint64_t(f.x)) | spread(uint64_t(f.y)) << 1 | spread(uint64_t(f.z)) << 2, i};
    }
    std::sort(keys.begin(), keys.end());
    std::vector<Manifold> out;
    out.reserve(ps.size());
    for (const auto& k : keys) out.push_back(ps[k.second]);
    return out;
}

// The union of the pieces: BatchBoolean, in Morton order. A pairwise tree of
// the same pieces is what lost material -- 5e-3 on an organic mesh, and once
// 8.8e-4 of one wedge in the Python prototype -- where BatchBoolean agreed
// with every other order tried. Merging the two as a cross-check unioned two
// near-coincident surfaces and left tunnels; a whole piece the union drops is
// still caught afterwards by lostPieces().
Manifold unionPieces(const std::vector<Manifold>& input) {
    if (input.empty()) return Manifold();
    if (input.size() == 1) return input[0];
    return Manifold::BatchBoolean(spatialOrder(input), OpType::Add);
}

// Indices of pieces whose material the union lost, whole or in part. Manifold
// unions these near-coincident pieces differently by input order and thread
// count (sphere(10,32) chamfer=1 lost 0.045 of one piece under TBB, and
// another 0.11 in manifold3d's own BatchBoolean), so every piece is checked
// against the result: outward P - u, inward P ^ u, must be empty. Pieces are
// coloured so that no two of one colour have overlapping boxes, which makes
// Compose() of a colour exact -- about 20 booleans in all, not one per piece.
// A remnant counts only if it is thicker (2*volume/area) than 10*e: the
// construction's own widening leaves slivers e thick, which are not losses.
std::vector<size_t> lostPieces(const std::vector<Manifold>& ps, const Manifold& result, bool wantInside, double e) {
    // A leaf of the result's mesh, not the result itself: Manifold's booleans
    // are lazy, and G - (A + pieces) is flattened to G - A - pieces, which
    // subtracts the very pieces being checked -- seed 13's lost 0.064 of a
    // wedge passed unseen.
    const Manifold u = bare(result);
    const size_t n = ps.size();
    std::vector<manifold::Box> box(n);
    double cell = 0;
    for (size_t i = 0; i < n; ++i) {
        box[i] = ps[i].BoundingBox();
        const vec3 s = box[i].Size();
        cell = std::max(cell, std::max({s.x, s.y, s.z}));
    }
    if (n == 0 || !(cell > 0)) return {};
    // Greedy colouring over a grid of cells the size of the biggest piece, so
    // a piece meets only the pieces in its own and neighbouring cells.
    auto key = [&](const vec3& p) {
        return std::array<long long, 3>{(long long)std::floor(p.x / cell), (long long)std::floor(p.y / cell),
                                        (long long)std::floor(p.z / cell)};
    };
    std::map<std::array<long long, 3>, std::vector<size_t>> grid;
    std::vector<int> colour(n, -1);
    int ncol = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto k = key(box[i].min);
        std::set<int> used;
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy)
                for (long long dz = -1; dz <= 1; ++dz) {
                    auto it = grid.find({k[0] + dx, k[1] + dy, k[2] + dz});
                    if (it == grid.end()) continue;
                    for (size_t j : it->second)
                        if (la::all(la::lequal(box[j].min, box[i].max)) && la::all(la::lequal(box[i].min, box[j].max)))
                            used.insert(colour[j]);
                }
        int c = 0;
        while (used.count(c)) ++c;
        colour[i] = c;
        ncol = std::max(ncol, c + 1);
        grid[k].push_back(i);
    }
    // Each colour is its own check: all of them at once.
    std::vector<std::vector<size_t>> lostBy(ncol);
    parallelFor(size_t(ncol), [&](size_t c) {
        std::vector<Manifold> group;
        std::vector<size_t> who;
        for (size_t i = 0; i < n; ++i)
            if (colour[i] == int(c)) { group.push_back(ps[i]); who.push_back(i); }
        const Manifold G = Manifold::Compose(group);
        const Manifold R = wantInside ? G - u : G ^ u;
        if (R.IsEmpty()) return;
        for (const Manifold& comp : R.Decompose()) {
            const double v = std::fabs(comp.Volume()), a = comp.SurfaceArea();
            if (!(a > 0) || 2 * v / a <= 10 * e) continue;
            const manifold::Box cb = comp.BoundingBox();
            for (size_t i : who)
                if (la::all(la::lequal(box[i].min, cb.max)) && la::all(la::lequal(cb.min, box[i].max)))
                    lostBy[c].push_back(i);
        }
    });
    std::set<size_t> lost;
    for (const auto& l : lostBy) lost.insert(l.begin(), l.end());
    return {lost.begin(), lost.end()};
}

// Generalized winding number (van Oosterom-Strackee solid angles) of x in a
// closed mesh: scale-free, unlike a probe solid of any fixed size.
double winding(const vec3& x, const Mesh& m) {
    double sum = 0;
    for (const auto& t : m.T) {
        const vec3 a = m.V[t[0]] - x, b = m.V[t[1]] - x, c = m.V[t[2]] - x;
        const double la_ = la::length(a), lb = la::length(b), lc = la::length(c);
        const double num = la::dot(a, la::cross(b, c));
        const double den = la_ * lb * lc + la::dot(a, b) * lc + la::dot(b, c) * la_ + la::dot(c, a) * lb;
        sum += std::atan2(num, den);
    }
    return sum / (2 * M_PI);
}

// A point inside the region a decomposed shell bounds: off its largest
// triangle, a little way against its normal for a solid, and along it for a
// void, whose triangles face into the void.
vec3 probe(const Manifold& shell) {
    const Mesh m = meshOf(shell);
    size_t best = 0;
    double area = -1;
    vec3 bn(0.0);
    for (size_t i = 0; i < m.T.size(); ++i) {
        const vec3 a = m.V[m.T[i][0]];
        const vec3 n = la::cross(m.V[m.T[i][1]] - a, m.V[m.T[i][2]] - a);
        if (la::length(n) > area) { area = la::length(n); best = i; bn = n; }
    }
    const vec3 n = area > 0 ? bn / area : vec3(0.0);
    const manifold::Box bb = shell.BoundingBox();
    const vec3 size = bb.max - bb.min;
    const double step = 1e-3 * (std::min({size.x, size.y, size.z}) + 1e-12);
    const vec3 c = (m.V[m.T[best][0]] + m.V[m.T[best][1]] + m.V[m.T[best][2]]) / 3.0;
    return c + (shell.Volume() < 0 ? step : -step) * n;
}

Manifold rightSideOut(const Manifold& v) {
    Mesh m = meshOf(v);
    for (auto& t : m.T) std::swap(t[0], t[2]);
    return fromMesh(m.V, m.T);
}

// Each solid and void the union came out with, checked against the definition
// at one point inside it -- against the convex pieces themselves, which are
// exact, not their union, which is where rounding went wrong.
Manifold cleanShells(const Manifold& out, const Manifold& A, const std::function<std::vector<Manifold>()>& makePieces,
                     double d) {
    const double tiny = 1e-9 * std::max(std::fabs(A.Volume()), std::fabs(d * d * d));
    std::vector<Manifold> parts;
    for (Manifold& p : out.Decompose())
        if (std::fabs(p.Volume()) > tiny) parts.push_back(std::move(p));
    bool allSolid = true;
    for (const Manifold& p : parts) allSolid = allSolid && p.Volume() > 0;
    if (parts.size() <= 1 && allSolid) return parts.empty() ? out : parts[0];

    const std::vector<Manifold> ps = makePieces();
    std::vector<std::optional<Planes>> planeCache(ps.size());
    auto inPieces = [&](const vec3& x) {
        for (size_t i = 0; i < ps.size(); ++i) {
            const manifold::Box bb = ps[i].BoundingBox();
            if (x.x < bb.min.x || x.y < bb.min.y || x.z < bb.min.z || x.x > bb.max.x || x.y > bb.max.y ||
                x.z > bb.max.z)
                continue;
            if (!planeCache[i]) planeCache[i] = planesOf(ps[i]);
            const Planes& pl = *planeCache[i];
            double cm = 0;
            for (double c : pl.c) cm = std::max(cm, std::fabs(c));
            // A hull's facet normals carry rounding: an exact test called
            // points of a valid cap outside it.
            const double tol = 1e-9 * std::max({1.0, std::fabs(x.x), std::fabs(x.y), std::fabs(x.z), cm});
            bool in = true;
            for (size_t j = 0; j < pl.n.size(); ++j)
                if (la::dot(pl.n[j], x) > pl.c[j] + tol) { in = false; break; }
            if (in) return true;
        }
        return false;
    };
    const Mesh am = meshOf(A);
    auto inA = [&](const vec3& x) { return winding(x, am) > 0.5; };

    std::vector<Manifold> solids, holes;
    for (const Manifold& p : parts) {
        const vec3 x = probe(p);
        const bool isVoid = p.Volume() < 0;
        bool ok;
        if (d > 0) ok = isVoid ? (!inA(x) && !inPieces(x)) : !(p ^ A).IsEmpty();
        else ok = isVoid ? (!inA(x) || inPieces(x)) : (inA(x) && !inPieces(x));
        if (!ok) continue;
        if (isVoid) holes.push_back(rightSideOut(p));
        else solids.push_back(p);
    }
    // Compose() does not read an inside-out shell as a hole, so rebuild: the
    // solids kept, minus each kept void turned right side out.
    Manifold res = solids.empty() ? Manifold() : unionPieces(solids);
    return holes.empty() ? res : res - unionPieces(holes);
}

// One closed, connected surface with no concave edge: a convex solid (a
// locally convex closed surface bounds a convex body).
bool convexSolid(const Mesh& mesh) {
    const size_t nt = mesh.T.size();
    if (nt < 4) return false;
    std::vector<vec3> N(nt);
    for (size_t t = 0; t < nt; ++t) {
        const auto& f = mesh.T[t];
        N[t] = la::normalize(la::cross(mesh.V[f[1]] - mesh.V[f[0]], mesh.V[f[2]] - mesh.V[f[0]]));
    }
    std::unordered_map<uint64_t, std::vector<int>> edges;
    for (size_t t = 0; t < nt; ++t)
        for (int i = 0; i < 3; ++i) {
            const int a = mesh.T[t][i], b = mesh.T[t][(i + 1) % 3];
            edges[(uint64_t(std::min(a, b)) << 32) | uint64_t(std::max(a, b))].push_back(int(t));
        }
    std::vector<int> root(nt);
    std::iota(root.begin(), root.end(), 0);
    std::function<int(int)> find = [&](int i) { return root[i] == i ? i : root[i] = find(root[i]); };
    const double tol = 1e-9 * std::max(1.0, maxAbs(mesh.V));
    for (const auto& [k, ts] : edges) {
        if (ts.size() != 2) return false;
        const int a = int(k >> 32), t1 = ts[0], t2 = ts[1];
        root[find(t1)] = find(t2);
        if (la::dot(N[t1], N[t2]) > kFlat) continue;
        for (int v : mesh.T[t2])
            if (la::dot(N[t1], mesh.V[v] - mesh.V[a]) > tol) return false;   // concave
    }
    for (size_t t = 0; t < nt; ++t)
        if (find(int(t)) != find(0)) return false;
    return true;
}

Manifold offsetCentered(const Manifold& m, double d, Offset3DMode mode, int segs) {
    const manifold::Box bb = m.BoundingBox();
    const vec3 size = bb.max - bb.min;
    if (std::fabs(d) < 1e-7 * std::max({1.0, size.x, size.y, size.z})) {
        // Below what double precision can place against a part this size
        // (d=1e-6 on a 20-unit part came out 17-22% of the right volume
        // change, genus up to 9). The input is the best answer there is.
        return m;
    }
    PhaseTimer timer;
    const Mesh mesh = meshOf(m);
    Manifold out, tools;
    std::vector<Manifold> ps;
    std::function<std::vector<Manifold>()> makePieces;
    const bool sum = mode == Offset3DMode::Round && d > 0;
    if (sum) {
        // Manifold's own Minkowski sum is this construction (one hull of each
        // triangle swept by the ball), and faster than building it here.
        std::vector<vec3> ball;
        for (const vec3& b : sphereDirs(segs)) ball.push_back(d * b);
        out = m.MinkowskiSum(hullChecked(ball));
        makePieces = [&] { return Builder(mesh.V, mesh.T, d, mode, segs, false).build(); };
    } else {
        // Not MinkowskiDifference for r < 0: it removed 0.002 of material more
        // than d inside the surface, where these pieces are exact.
        // Touching counts as joined, as in Minkowski: grown by a hair, two
        // offsets that meet exactly overlap instead of touching, where the
        // union decided by rounding (chamfer genus 1 against 0 at a gap 2d).
        const double mu = 1e-9 * std::max(1.0, maxAbs(mesh.V));
        ps = Builder(mesh.V, mesh.T, std::fabs(d) + mu, mode, segs, d < 0).build();
        timer.lap("pieces");
        if (d > 0 && mode == Offset3DMode::Delta && convexSolid(mesh)) {
            // The mitered offset of a convex solid is convex, so it is the
            // hull of the pieces that fill it: one hull, not a union of
            // hundreds of near-coplanar wedges (a 256-sided cone: 4.4 s).
            // Not chamfer=: its offset of a convex solid need not be convex.
            const double tol = 1e-9 * std::max(1.0, maxAbs(mesh.V)), D = d + mu;
            std::vector<vec3> fn;
            std::vector<double> fc;
            for (const auto& f : mesh.T) {
                const vec3 n = la::normalize(la::cross(mesh.V[f[1]] - mesh.V[f[0]], mesh.V[f[2]] - mesh.V[f[0]]));
                fn.push_back(n);
                fc.push_back(la::dot(n, mesh.V[f[0]]) + D);
            }
            // The pieces' vertices, near-duplicates (shared corners, ~1e-9 apart)
            // merged to one exact point each rather than snapped to a grid.
            std::map<std::array<long long, 3>, vec3> near;
            const double q = 10 * tol;
            for (const Manifold& p : ps)
                for (const vec3& v : meshOf(p).V)
                    near.emplace(std::array<long long, 3>{std::llround(v.x / q), std::llround(v.y / q), std::llround(v.z / q)}, v);
            // Only the offset's corners go in: a vertex of an intersection of
            // half-spaces lies on three or more planes whose normals span 3D.
            // Manifold's hull of every piece vertex (11k-24k of them, most on a
            // face's plane) came out with two shells, or points 3e-4 outside.
            std::vector<vec3> all;
            for (const auto& kv : near) all.push_back(kv.second);
            std::vector<char> corner(all.size(), 0);
            parallelFor(all.size(), [&](size_t a) {
                const vec3 v = all[a];
                std::vector<vec3> on;
                bool inside = true;
                for (size_t j = 0; j < fn.size() && inside; ++j) {
                    const double w = la::dot(fn[j], v) - fc[j];
                    if (w > 10 * tol) inside = false;
                    else if (w > -10 * tol) on.push_back(fn[j]);
                }
                if (!inside || on.size() < 3) return;
                bool span = false;
                for (size_t i = 1; i < on.size() && !span; ++i) {
                    const vec3 c = la::cross(on[0], on[i]);
                    if (la::length(c) < 1e-9) continue;
                    for (size_t j = i + 1; j < on.size() && !span; ++j)
                        span = std::fabs(la::dot(c, on[j])) > 1e-9;
                }
                corner[a] = span;
            });
            std::vector<vec3> pts;
            for (size_t a = 0; a < all.size(); ++a)
                if (corner[a]) pts.push_back(all[a]);
            // Trusted only if provably the offset: one shell, inside every
            // shifted plane (so within the offset), and holding every piece
            // vertex (so every piece, and their union). Otherwise the union
            // runs as for any other input.
            Manifold h = pts.size() >= 4 ? whitenedHull(pts) : Manifold();
            bool good = !h.IsEmpty() && h.Status() == Manifold::Error::NoError && h.Genus() == 0 &&
                        h.Decompose().size() == 1;
            if (good) {
                const Planes pl = planesOf(h);
                std::atomic<bool> ok{true};
                parallelFor(all.size(), [&](size_t a) {
                    for (size_t j = 0; ok && j < pl.n.size(); ++j)
                        if (la::dot(all[a], pl.n[j]) > pl.c[j] + 10 * tol) ok = false;
                });
                const std::vector<vec3> hv = meshOf(h).V;
                parallelFor(hv.size(), [&](size_t a) {
                    for (size_t j = 0; ok && j < fn.size(); ++j)
                        if (la::dot(fn[j], hv[a]) > fc[j] + 10 * tol) ok = false;
                });
                good = ok;
            }
            if (good) {
                ps.clear();   // nothing for lostPieces() to check: no union ran
                tools = h;
            } else {
                tools = unionPieces(ps);
            }
        } else {
            tools = unionPieces(ps);
        }
        timer.lap("union");
        out = d > 0 ? m + tools : m - tools;
        out.NumTri();
        timer.lap("A+-tools");
        makePieces = [&] { return ps; };
    }
    // Merge what the pieces split: every hull is its own source object, and
    // Simplify() never merges across sources, so reset them first (a box
    // whose faces were 768 triangles came out as 1,900; this makes it 12).
    // Simplify() is not topology-safe, so it runs before the shell check, and
    // not always shape-preserving either (it moved 0.063 of a part's volume
    // at a 9e-12 tolerance): keep it only when it changed nothing but the
    // triangulation -- the same volume, and the same number and genus of
    // shells with any volume (zero-volume ones are dropped later anyway).
    const double tiny = 1e-9 * std::max(std::fabs(m.Volume()), std::fabs(d * d * d));
    auto shape = [&](const Manifold& x) {
        int n = 0, g = 0;
        for (const Manifold& c : x.Decompose())
            if (std::fabs(c.Volume()) > tiny) { ++n; g += c.Genus(); }
        return std::pair{n, g};
    };
    auto simplified = [&](const Manifold& x) {
        const Manifold s = x.AsOriginal().Simplify(0);
        return std::fabs(s.Volume() - x.Volume()) <= tiny && shape(s) == shape(x) ? s : x;
    };
    Manifold simple = simplified(out);
    timer.lap("simplify");
    if (!sum) {
        // Manifold's union loses material of pieces: every piece must be in
        // the result outward, and out of it inward. Checked on the simplified
        // result, far smaller than the raw union of the pieces; a piece found
        // short is added again, and the result checked again.
        const double D = std::fabs(d) + 1e-9 * std::max(1.0, maxAbs(mesh.V));
        const double e = std::min(D, std::max(1e-5 * D, 1e-9 * std::max(1.0, maxAbs(mesh.V))));
        for (int round = 0; round < 3; ++round) {
            const std::vector<size_t> lost = lostPieces(ps, simple, d > 0, e);
            if (lost.empty()) break;
            std::vector<Manifold> again;
            for (size_t i : lost) again.push_back(ps[i]);
            // Onto the finished result, as a leaf: re-running the whole
            // union lost other pieces each round (6, 5, 5 on seed 13).
            const Manifold leaf = bare(simple);
            const Manifold raw = d > 0 ? leaf + unionPieces(again) : leaf - unionPieces(again);
            simple = simplified(raw);
        }
    }
    timer.lap("lost");
    Manifold res = cleanShells(simple, m, makePieces, d);
    timer.lap("clean");
    // Manifold's booleans leave slivers (vertices 4.5e-12 apart, zero-area
    // triangles) that Simplify(0) keeps. Moved back off the origin, Manifold's
    // own precision grows past them and collapses them for us -- once losing
    // 0.114 of a part's volume. Collapse them here, where it is safe, and keep
    // the result only if it changed nothing else.
    const double sc = std::max(1.0, maxAbs(mesh.V));
    const Manifold tidy = bare(res).Simplify(1e-9 * sc);
    if (std::fabs(tidy.Volume() - res.Volume()) <= tiny && shape(tidy) == shape(res)) res = tidy;
    return res;
}

} // namespace

Manifold offset3d(const Manifold& m, double d, Offset3DMode mode, int segs) {
    if (d == 0 || m.IsEmpty() || !std::isfinite(d)) return m;
    segs = std::max(segs, 3);
    // Work at the origin: Manifold's union of the pieces lost 0.35 of a
    // chamfered 10-cube's volume at 1e5 from it.
    const manifold::Box bb = m.BoundingBox();
    const vec3 c = (bb.min + bb.max) / 2.0;
    // A leaf first: Translate() of a lazy CSG result pushes the move into its
    // operands and redoes the booleans out there (t41 seed 2 lost 0.114).
    Mesh mm = meshOf(offsetCentered(bare(m.Translate(-c)), d, mode, segs));
    for (vec3& v : mm.V) v += c;
    return fromMesh(mm.V, mm.T);
}

} // namespace oscadeval
