// alpha_wrap(alpha, offset) -- shrink-wrap the children in a closed,
// manifold skin. A BelfrySCAD extension (BelfrySCAD #941).
//
// The skin follows any concavity a ball of radius `alpha` can enter and
// bridges any gap or hole that ball cannot pass (about 2*alpha wide), and every point of it lies
// `offset` outside the input. So it is also a robust rounded outward offset,
// and a way to wrap geometry too broken for Manifold to accept: an open
// polyhedron()'s raw triangles are wrapped like any other.
//
// Written from Portaneri, Rouxel-Labbe, Hemmer, Cohen-Steiner and Alliez,
// "Alpha Wrapping with an Offset" (ACM TOG 41(4), SIGGRAPH 2022) and CGAL's
// user manual for its Alpha_wrap_3 package -- NOT from CGAL's source, which
// is GPL and could not be carried into this MIT project. The predicates are
// Shewchuk's, which are public domain (third_party/predicates).
//
// The algorithm, as the paper gives it:
//  - A 3D Delaunay triangulation of a loose box around the input. Its cells
//    start "inside"; the region beyond the box is "outside".
//  - A gate is a face between an outside cell and an inside one. It is
//    traversable when the smallest empty sphere through it -- whose centre
//    lies on the Voronoi edge dual to the face -- is wider than alpha.
//    Gates are taken widest first.
//  - Crossing a gate into an inside cell either inserts a Steiner point on
//    the offset surface (rule 1: where the Voronoi edge first meets it;
//    rule 2: when the cell overlaps the input, its circumcentre projected
//    onto it), whose new cells are inside, or marks the cell outside.
//  - When no gate is left, the faces between inside and outside are the skin.
//
// Four far "super" vertices stand in for the triangulation's infinite
// vertex: a cell using one is "infinite", always outside, never entered.

#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "../../third_party/predicates/predicates.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>

namespace oscadeval {

namespace {

// CGAL's suggested defaults: alpha 1/20 of the bounding-box diagonal, and
// the offset 1/30 of alpha -- small, so sharp features survive.
constexpr double kDefaultAlphaFraction = 1.0 / 20.0;
constexpr double kDefaultOffsetFraction = 1.0 / 30.0;
// A runaway guard, not a quality knob: a tiny alpha on a big model asks for
// tens of millions of points, which would exhaust memory long before ending.
constexpr size_t kMaxSteinerPoints = 5'000'000;
// The skin's flat sides are cut into many small coplanar triangles, its
// points all lying on the offset surface. Merging at a tolerance this small
// next to the offset leaves the rounded edges' shape and the volume alone
// while cutting the triangle count by up to ~3x (#941).
constexpr double kMergeFraction = 1.0 / 100.0;

using V3 = std::array<double, 3>;
V3 operator+(V3 a, V3 b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 operator-(V3 a, V3 b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 operator*(V3 a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double dot(V3 a, V3 b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V3 cross(V3 a, V3 b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double len(V3 a) { return std::sqrt(dot(a, a)); }

struct Tri { V3 a, b, c; };

// Closest point on a triangle (Ericson, Real-Time Collision Detection 5.1.5).
V3 closestOnTri(V3 p, const Tri& t) {
    V3 ab = t.b - t.a, ac = t.c - t.a, ap = p - t.a;
    double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return t.a;
    V3 bp = p - t.b;
    double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return t.b;
    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return t.a + ab * (d1 / (d1 - d3));
    V3 cp = p - t.c;
    double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return t.c;
    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return t.a + ac * (d2 / (d2 - d6));
    double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return t.b + (t.c - t.b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    double den = 1.0 / (va + vb + vc);
    return t.a + ab * (vb * den) + ac * (vc * den);
}

// Separating-axis test: does the tetrahedron overlap the triangle?
bool tetTriOverlap(const V3 q[4], const Tri& t) {
    const V3 tp[3] = {t.a, t.b, t.c};
    auto separated = [&](V3 axis) {
        if (dot(axis, axis) < 1e-30) return false;
        double a0 = 1e300, a1 = -1e300, b0 = 1e300, b1 = -1e300;
        for (int i = 0; i < 4; ++i) { double d = dot(axis, q[i]); a0 = std::min(a0, d); a1 = std::max(a1, d); }
        for (int i = 0; i < 3; ++i) { double d = dot(axis, tp[i]); b0 = std::min(b0, d); b1 = std::max(b1, d); }
        return a1 < b0 || b1 < a0;
    };
    static const int F[4][3] = {{1, 2, 3}, {0, 2, 3}, {0, 1, 3}, {0, 1, 2}};
    for (const auto& f : F)
        if (separated(cross(q[f[1]] - q[f[0]], q[f[2]] - q[f[0]]))) return false;
    if (separated(cross(t.b - t.a, t.c - t.a))) return false;
    static const int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    const V3 te[3] = {t.b - t.a, t.c - t.b, t.a - t.c};
    for (const auto& e : E)
        for (const auto& d : te)
            if (separated(cross(q[e[1]] - q[e[0]], d))) return false;
    return true;
}

// Bounding-volume hierarchy over the input triangles: nearest point, and
// whether a tetrahedron overlaps any of them.
class Oracle {
public:
    explicit Oracle(std::vector<Tri> t) : tris_(std::move(t)) { build(0, static_cast<int>(tris_.size())); }

    V3 lo() const { return nodes_[0].lo; }
    V3 hi() const { return nodes_[0].hi; }

    V3 closest(V3 p) const {
        double best = 1e300;
        V3 bp{};
        std::vector<int> st{0};
        while (!st.empty()) {
            const Node& n = nodes_[st.back()];
            st.pop_back();
            if (boxDist2(n, p) >= best) continue;
            if (n.left < 0) {
                for (int k = n.first; k < n.first + n.count; ++k) {
                    V3 c = closestOnTri(p, tris_[k]);
                    double d = dot(c - p, c - p);
                    if (d < best) { best = d; bp = c; }
                }
            } else {
                st.push_back(n.left);
                st.push_back(n.right);
            }
        }
        return bp;
    }
    double dist(V3 p) const { return len(closest(p) - p); }

    bool overlapsTet(const V3 q[4]) const {
        V3 lo = q[0], hi = q[0];
        for (int i = 1; i < 4; ++i)
            for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], q[i][k]); hi[k] = std::max(hi[k], q[i][k]); }
        std::vector<int> st{0};
        while (!st.empty()) {
            const Node& n = nodes_[st.back()];
            st.pop_back();
            bool apart = false;
            for (int k = 0; k < 3; ++k)
                if (n.hi[k] < lo[k] || hi[k] < n.lo[k]) apart = true;
            if (apart) continue;
            if (n.left < 0) {
                for (int k = n.first; k < n.first + n.count; ++k)
                    if (tetTriOverlap(q, tris_[k])) return true;
            } else {
                st.push_back(n.left);
                st.push_back(n.right);
            }
        }
        return false;
    }

private:
    struct Node { V3 lo, hi; int left = -1, right = -1, first = 0, count = 0; };
    std::vector<Tri> tris_;
    std::vector<Node> nodes_;

    int build(int first, int count) {
        Node n;
        n.lo = {1e300, 1e300, 1e300};
        n.hi = {-1e300, -1e300, -1e300};
        for (int i = first; i < first + count; ++i)
            for (const V3& p : {tris_[i].a, tris_[i].b, tris_[i].c})
                for (int k = 0; k < 3; ++k) { n.lo[k] = std::min(n.lo[k], p[k]); n.hi[k] = std::max(n.hi[k], p[k]); }
        const int id = static_cast<int>(nodes_.size());
        nodes_.push_back(n);
        if (count <= 4) {
            nodes_[id].first = first;
            nodes_[id].count = count;
            return id;
        }
        int axis = 0;
        V3 ext = n.hi - n.lo;
        if (ext[1] > ext[axis]) axis = 1;
        if (ext[2] > ext[axis]) axis = 2;
        const int mid = first + count / 2;
        std::nth_element(tris_.begin() + first, tris_.begin() + mid, tris_.begin() + first + count,
                         [axis](const Tri& x, const Tri& y) {
                             return x.a[axis] + x.b[axis] + x.c[axis] < y.a[axis] + y.b[axis] + y.c[axis];
                         });
        const int l = build(first, mid - first);
        const int r = build(mid, first + count - mid);
        nodes_[id].left = l;
        nodes_[id].right = r;
        return id;
    }
    static double boxDist2(const Node& n, V3 p) {
        double d = 0;
        for (int k = 0; k < 3; ++k) {
            double e = std::max({n.lo[k] - p[k], 0.0, p[k] - n.hi[k]});
            d += e * e;
        }
        return d;
    }
};

struct Cell {
    int v[4];
    int n[4];                // neighbour across the face opposite v[i]; -1: none
    bool alive = true, outside = false, infinite = false;
    unsigned gen = 0;        // bumped when the slot is reused, so a stale gate is recognised
    V3 cc{};                 // circumcentre
    double r2 = 0;           // squared circumradius
};

class Wrapper {
public:
    Wrapper(const Oracle& oracle, double alpha, double offset, Evaluator& ev)
        : oracle_(oracle), alpha2_(alpha * alpha), offset_(offset), ev_(ev) {}

    // False if the runaway guard stopped it; the skin is then incomplete.
    bool run(V3 lo, V3 hi);
    manifold::MeshGL64 skin() const;
    size_t steinerCount() const { return steiner_; }

private:
    struct Gate {
        double prio; int cell, face; unsigned gen;
        bool operator<(const Gate& o) const { return prio < o.prio; }
    };

    const Oracle& oracle_;
    double alpha2_, offset_;
    Evaluator& ev_;
    std::vector<V3> pts_;
    std::vector<Cell> cells_;
    std::vector<int> free_;
    std::priority_queue<Gate> queue_;
    int hint_ = 0;
    size_t steiner_ = 0;

    const double* P(int i) const { return pts_[i].data(); }

    void setGeometry(Cell& c) {
        c.infinite = c.v[0] < 4 || c.v[1] < 4 || c.v[2] < 4 || c.v[3] < 4;
        V3 a = pts_[c.v[0]], b = pts_[c.v[1]] - a, d = pts_[c.v[2]] - a, e = pts_[c.v[3]] - a;
        const double den = 2 * dot(b, cross(d, e));
        V3 o = (cross(d, e) * dot(b, b) + cross(e, b) * dot(d, d) + cross(b, d) * dot(e, e)) * (1.0 / den);
        c.cc = a + o;
        c.r2 = dot(o, o);
    }

    int newCell(const int v[4]) {
        int id;
        if (!free_.empty()) { id = free_.back(); free_.pop_back(); }
        else { id = static_cast<int>(cells_.size()); cells_.emplace_back(); }
        Cell& c = cells_[id];
        const unsigned g = c.gen + 1;
        c = Cell();
        c.gen = g;
        std::memcpy(c.v, v, sizeof c.v);
        std::fill(std::begin(c.n), std::end(c.n), -1);
        setGeometry(c);
        return id;
    }

    // The cell containing p, by walking from the last insertion.
    int locate(V3 p) const {
        int t = hint_;
        while (!cells_[t].alive) ++t;
        for (size_t steps = 0; steps < cells_.size() + 16; ++steps) {
            const Cell& c = cells_[t];
            int next = -1;
            for (int f = 0; f < 4 && next < 0; ++f) {
                const double* q[4] = {P(c.v[0]), P(c.v[1]), P(c.v[2]), P(c.v[3])};
                q[f] = p.data();
                if (orient3d(q[0], q[1], q[2], q[3]) < 0 && c.n[f] >= 0) next = c.n[f];
            }
            if (next < 0) return t;
            t = next;
        }
        return t;
    }

    // Bowyer-Watson insertion of p; returns the cells made.
    std::vector<int> insert(V3 p) {
        const int c0 = locate(p);
        const int pi = static_cast<int>(pts_.size());
        pts_.push_back(p);
        std::vector<int> cavity{c0};
        std::vector<char> inCavity(cells_.size(), 0);
        inCavity[c0] = 1;
        for (size_t k = 0; k < cavity.size(); ++k) {
            const Cell& c = cells_[cavity[k]];
            for (int f = 0; f < 4; ++f) {
                const int nb = c.n[f];
                if (nb < 0 || inCavity[nb]) continue;
                const Cell& N = cells_[nb];
                if (insphere(P(N.v[0]), P(N.v[1]), P(N.v[2]), P(N.v[3]), p.data()) > 0) {
                    inCavity[nb] = 1;
                    cavity.push_back(nb);
                }
            }
        }
        struct Boundary { int cell, face, outer; std::array<int, 4> v; };
        std::vector<Boundary> bound;
        for (int c : cavity)
            for (int f = 0; f < 4; ++f) {
                const int nb = cells_[c].n[f];
                if (nb >= 0 && inCavity[nb]) continue;
                Boundary b{c, f, nb, {}};
                std::memcpy(b.v.data(), cells_[c].v, sizeof cells_[c].v);
                b.v[f] = pi;                     // keeps the cell's orientation
                bound.push_back(b);
            }
        for (int c : cavity) { cells_[c].alive = false; free_.push_back(c); }

        std::vector<int> made;
        std::unordered_map<uint64_t, std::pair<int, int>> open;   // edge -> (cell, face)
        for (const Boundary& b : bound) {
            const int id = newCell(b.v.data());
            made.push_back(id);
            cells_[id].n[b.face] = b.outer;
            if (b.outer >= 0) {
                Cell& O = cells_[b.outer];
                for (int f = 0; f < 4; ++f)
                    if (O.n[f] == b.cell && !std::count(b.v.begin(), b.v.end(), O.v[f])) { O.n[f] = id; break; }
            }
            for (int f = 0; f < 4; ++f) {
                if (f == b.face) continue;
                int a = -1, e = -1;
                for (int i = 0; i < 4; ++i)
                    if (i != f && i != b.face) (a < 0 ? a : e) = cells_[id].v[i];
                if (a > e) std::swap(a, e);
                const uint64_t key = static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32 | static_cast<uint32_t>(e);
                auto it = open.find(key);
                if (it == open.end()) {
                    open[key] = {id, f};
                } else {
                    cells_[id].n[f] = it->second.first;
                    cells_[it->second.first].n[it->second.second] = id;
                    open.erase(it);
                }
            }
        }
        if (!made.empty()) hint_ = made[0];
        return made;
    }

    // Squared radius of the smallest empty sphere through face f of cell c.
    double minEmptyR2(int c, int f) const {
        const Cell& C = cells_[c];
        int idx[3], k = 0;
        for (int i = 0; i < 4; ++i)
            if (i != f) idx[k++] = C.v[i];
        V3 a = pts_[idx[0]], b = pts_[idx[1]] - a, d = pts_[idx[2]] - a;
        V3 nrm = cross(b, d);
        V3 o = (cross(nrm, b) * dot(d, d) + cross(d, nrm) * dot(b, b)) * (1.0 / (2 * dot(nrm, nrm)));
        const V3 fc = a + o;
        const double fr2 = dot(o, o);
        const int nb = C.n[f];
        if (nb < 0) return fr2;
        V3 c1 = C.cc, dir = cells_[nb].cc - c1;
        const double s = dot(fc - c1, dir) / std::max(dot(dir, dir), 1e-300);
        if (s >= 0 && s <= 1) return fr2;
        return s < 0 ? C.r2 : cells_[nb].r2;
    }

    void pushGatesOf(int c) {
        const Cell& C = cells_[c];
        if (!C.outside) return;
        for (int f = 0; f < 4; ++f) {
            const int nb = C.n[f];
            if (nb < 0 || cells_[nb].outside || cells_[nb].infinite) continue;
            const double r2 = minEmptyR2(c, f);
            if (r2 > alpha2_) queue_.push({r2, c, f, C.gen});
        }
    }

    // Rule 1: the first point from a towards b at the offset distance. The
    // distance is 1-Lipschitz, so stepping by (distance - offset) cannot
    // overshoot the surface.
    bool firstOffsetHit(V3 a, V3 b, V3& out) const {
        V3 d = b - a;
        const double L = len(d);
        if (L <= 0) return false;
        d = d * (1.0 / L);
        const double eps = offset_ * 1e-6;
        double s = 0;
        for (int it = 0; it < 100000 && s <= L; ++it) {
            const V3 p = a + d * s;
            const double g = oracle_.dist(p) - offset_;
            if (g <= eps) {
                if (s == 0) return false;        // a is itself within the offset
                out = p;
                return true;
            }
            s += std::max(g, eps);
        }
        return false;
    }

    bool steinerPoint(int from, int into, V3& sp) const {
        const Cell& N = cells_[into];
        if (oracle_.dist(N.cc) < offset_ && firstOffsetHit(cells_[from].cc, N.cc, sp)) return true;
        V3 q[4];
        for (int i = 0; i < 4; ++i) q[i] = pts_[N.v[i]];
        if (oracle_.overlapsTet(q)) {
            const V3 c = oracle_.closest(N.cc), dir = N.cc - c;
            const double l = len(dir);
            if (l < 1e-12) return false;
            sp = c + dir * (offset_ / l);
            return true;
        }
        return false;
    }
};

bool Wrapper::run(V3 lo, V3 hi) {
    const V3 ctr = (lo + hi) * 0.5;
    const double R = len(hi - lo) * 1000;
    pts_ = {ctr + V3{R, R, R}, ctr + V3{-R, -R, R}, ctr + V3{-R, R, -R}, ctr + V3{R, -R, -R}};
    int v0[4] = {0, 1, 2, 3};
    if (orient3d(P(0), P(1), P(2), P(3)) < 0) std::swap(v0[2], v0[3]);
    newCell(v0);
    // The loose box's corners, jittered a hair so no four are cospherical.
    unsigned seed = 12345;
    auto jitter = [&] { seed = seed * 1103515245u + 12345u; return ((seed >> 8) % 1000) * 1e-9; };
    const double diag = len(hi - lo);
    for (int i = 0; i < 8; ++i) {
        V3 p{i & 1 ? hi[0] : lo[0], i & 2 ? hi[1] : lo[1], i & 4 ? hi[2] : lo[2]};
        insert(p + V3{jitter(), jitter(), jitter()} * diag);
    }
    for (Cell& c : cells_)
        if (c.alive) c.outside = c.infinite;
    for (size_t c = 0; c < cells_.size(); ++c)
        if (cells_[c].alive) pushGatesOf(static_cast<int>(c));

    size_t gates = 0;
    while (!queue_.empty()) {
        if ((++gates & 0xFF) == 0) ev_.checkCancel();
        const Gate g = queue_.top();
        queue_.pop();
        const Cell& C = cells_[g.cell];
        if (!C.alive || C.gen != g.gen || !C.outside) continue;
        const int into = C.n[g.face];
        if (into < 0 || cells_[into].outside || cells_[into].infinite) continue;
        V3 sp;
        if (steinerPoint(g.cell, into, sp)) {
            if (++steiner_ > kMaxSteinerPoints) return false;
            const std::vector<int> made = insert(sp);
            for (int c : made) cells_[c].outside = cells_[c].infinite;   // new cells are inside
            // New gates can only touch the new cells: theirs, and their
            // outside neighbours' faces onto them.
            for (int c : made) {
                pushGatesOf(c);
                for (int f = 0; f < 4; ++f) {
                    const int nb = cells_[c].n[f];
                    if (nb >= 0 && cells_[nb].outside) pushGatesOf(nb);
                }
            }
        } else {
            cells_[into].outside = true;
            pushGatesOf(into);
        }
    }
    return true;
}

// MeshGL64: the Steiner points are placed in double precision, and rounding
// them to float could fold the thin triangles a small offset produces.
manifold::MeshGL64 Wrapper::skin() const {
    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    std::unordered_map<int, uint64_t> remap;
    auto vert = [&](int v) {
        auto [it, fresh] = remap.try_emplace(v, static_cast<uint64_t>(remap.size()));
        if (fresh)
            for (int k = 0; k < 3; ++k) mesh.vertProperties.push_back(pts_[v][k]);
        return it->second;
    };
    for (size_t c = 0; c < cells_.size(); ++c) {
        const Cell& C = cells_[c];
        if (!C.alive || C.outside) continue;
        for (int f = 0; f < 4; ++f) {
            const int nb = C.n[f];
            if (nb >= 0 && !cells_[nb].outside) continue;
            // Wound so the normal points away from v[f], out of the inside cell.
            int r[3], k = 0;
            for (int i = 0; i < 4; ++i)
                if (i != f) r[k++] = C.v[i];
            if (orient3d(P(r[0]), P(r[1]), P(r[2]), P(C.v[f])) < 0) std::swap(r[1], r[2]);
            for (int v : r) mesh.triVerts.push_back(vert(v));
        }
    }
    return mesh;
}

void appendTriangles(const manifold::MeshGL& m, std::vector<Tri>& out) {
    const size_t np = m.numProp;
    auto at = [&](uint32_t v) {
        return V3{m.vertProperties[v * np], m.vertProperties[v * np + 1], m.vertProperties[v * np + 2]};
    };
    for (size_t i = 0; i + 2 < m.triVerts.size(); i += 3)
        out.push_back({at(m.triVerts[i]), at(m.triVerts[i + 1]), at(m.triVerts[i + 2])});
}

} // namespace

CSGParams resolveAlphaWrap(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    params["alpha"] = getArg(args, 0, "alpha", Value{});
    params["offset"] = getArg(args, 1, "offset", Value{});
    EvalContext blockCtx = ev.blockScope(effCtx);
    ev.evalChildren(node.children, blockCtx);
    return params;
}

std::vector<ColoredBody> generateAlphaWrap(Evaluator& ev, const CSGParams& params,
                                           const std::vector<std::unique_ptr<CSGNode>>& children,
                                           const oscad::ASTNode& node) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    if (bodies.empty()) return {};

    // Background/highlight geometry and 2D sections pass through untouched;
    // every other solid, including an open polyhedron()'s raw triangles, is
    // wrapped together.
    std::vector<Tri> tris;
    std::vector<ColoredBody> out;
    std::optional<std::array<float, 4>> color;
    bool colorTaken = false, sawSection = false;
    for (const ColoredBody& b : bodies) {
        if (b.role != BodyRole::Normal || (!b.body && !b.rawMesh)) {
            sawSection = sawSection || (b.role == BodyRole::Normal && b.section);
            out.push_back(b);
            continue;
        }
        const size_t before = tris.size();
        if (b.rawMesh) appendTriangles(*b.rawMesh, tris);
        else if (!b.body->IsEmpty()) appendTriangles(b.body->GetMeshGL(), tris);
        if (!colorTaken && tris.size() > before) { color = b.color; colorTaken = true; }
    }
    if (sawSection) ev.warn("alpha_wrap: 2D children are not wrapped", &node.position());
    if (tris.empty()) return out;

    Oracle oracle(std::move(tris));
    const V3 d = oracle.hi() - oracle.lo();
    const double diag = len(d);

    auto number = [&](const char* name, double fallback) -> std::optional<double> {
        const Value& v = params.at(name);
        if (std::holds_alternative<std::monostate>(v)) return fallback;
        if (std::holds_alternative<double>(v) && std::get<double>(v) > 0 && std::isfinite(std::get<double>(v)))
            return std::get<double>(v);
        ev.warn(std::string("alpha_wrap: ") + name + " must be a positive number", &node.position());
        return std::nullopt;
    };
    const std::optional<double> alpha = number("alpha", kDefaultAlphaFraction * diag);
    if (!alpha) return bodies;
    const std::optional<double> offset = number("offset", kDefaultOffsetFraction * *alpha);
    if (!offset) return bodies;

    static std::once_flag predicatesReady;
    std::call_once(predicatesReady, [] { exactinit(); });

    // The box must leave room for the skin, which sits `offset` out, and for
    // a ball of radius alpha to pass all round it.
    const double margin = 2 * (*alpha + *offset);
    Wrapper wrap(oracle, *alpha, *offset, ev);
    if (!wrap.run(oracle.lo() - V3{margin, margin, margin}, oracle.hi() + V3{margin, margin, margin})) {
        ev.warn("alpha_wrap: stopped after " + std::to_string(kMaxSteinerPoints) +
                    " points; alpha is too small for this model",
                &node.position());
        return bodies;
    }
    manifold::Manifold body(wrap.skin());
    if (body.Status() != manifold::Manifold::Error::NoError) {
        ev.warn("alpha_wrap: the skin is not manifold", &node.position());
    } else {
        body = body.Simplify(kMergeFraction * *offset);
    }
    ColoredBody cb = ev.tagGenerated(std::move(body), node, Value{});
    cb.color = color;
    out.insert(out.begin(), std::move(cb));
    return out;
}

} // namespace oscadeval
