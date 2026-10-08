#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <manifold/polygon.h>

#include <clipper2/clipper.h>

#include <boost/polygon/point_data.hpp>
#include <boost/polygon/segment_data.hpp>
#include <boost/polygon/voronoi.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>

// roof(): a roof over the children's 2D outline.
namespace oscadeval {

namespace {

// The roof is the surface z = (distance from (x, y) to the outline's nearest
// boundary point). It is built from the Voronoi diagram of the outline's edges:
// within the cell of an edge the surface is the plane rising at 45 degrees from
// that edge, and within the cell of a reflex corner it is a cone around the
// corner. So the part of an edge's cell inside the outline becomes one planar
// facet, bounded by the edge and by the Voronoi edges above it; the part of a
// corner's cell becomes a fan of triangles from the corner; and the outline
// closes the floor.
//
// Boost's Voronoi builder needs integer input, so the outline is snapped to a
// power-of-two grid fine enough (2^30 steps across the largest coordinate) for
// the snapping to be far below anything visible, and exact on simple shapes.

namespace la = manifold::la;
using Vec2 = manifold::vec2;
using VoronoiDiagram = boost::polygon::voronoi_diagram<double>;
using VoronoiEdge = VoronoiDiagram::edge_type;
using VoronoiVertex = VoronoiDiagram::vertex_type;
using VoronoiCell = VoronoiDiagram::cell_type;

double cross2(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

struct OutlineEdge {
    Vec2 a, b; // interior on the left of a -> b
};

// Builds the mesh, sharing one vertex per distinct (x, y): every point is
// computed exactly once and reused by each face that meets it, so equal
// positions are bit-identical and the mesh closes without tolerances.
class RoofMesh {
  public:
    explicit RoofMesh(double toWorld) : toWorld_(toWorld) {}

    int vertex(Vec2 p, double z) {
        auto [it, inserted] = index_.try_emplace({p.x, p.y}, static_cast<int>(index_.size()));
        if (inserted) {
            for (double c : {p.x, p.y, z}) mesh_.vertProperties.push_back(c * toWorld_);
        }
        return it->second;
    }

    // A vertex of its own, even where another sits at the same (x, y).
    int addVertex(Vec2 p, double z) {
        for (double c : {p.x, p.y, z}) mesh_.vertProperties.push_back(c * toWorld_);
        return static_cast<int>(mesh_.vertProperties.size() / 3) - 1;
    }

    double height(int i) const { return mesh_.vertProperties[3 * i + 2] / toWorld_; }

    Vec2 position(int i) const { return Vec2{mesh_.vertProperties[3 * i], mesh_.vertProperties[3 * i + 1]} / toWorld_; }

    // A triangle meeting its own reverse cancels it: where a cell has been
    // pinched down to two coincident sides, the facets there enclose nothing.
    void triangle(int a, int b, int c) {
        if (a == b || b == c || c == a) return;
        const auto key = [](int x, int y, int z) {
            if (y < x && y < z) return std::array<int, 3>{y, z, x};
            if (z < x && z < y) return std::array<int, 3>{z, x, y};
            return std::array<int, 3>{x, y, z};
        };
        if (auto reverse = triangles_.find(key(a, c, b)); reverse != triangles_.end()) {
            triangles_.erase(reverse);
            return;
        }
        triangles_.insert(key(a, b, c));
    }

    // Triangulates the polygons (counter-clockwise outlines, clockwise holes)
    // through the given vertices, facing up or down.
    void fill(const std::vector<std::vector<int>>& loops, bool up) {
        manifold::PolygonsIdx polys;
        for (const auto& loop : loops) {
            manifold::SimplePolygonIdx poly;
            for (int i : loop) {
                if (poly.empty() || poly.back().idx != i) poly.push_back({position(i), i});
            }
            while (poly.size() > 1 && poly.front().idx == poly.back().idx) poly.pop_back();
            if (poly.size() >= 3) polys.push_back(std::move(poly));
        }
        if (polys.empty()) return;
        for (const manifold::ivec3& t : manifold::TriangulateIdx(polys)) {
            if (up) triangle(t.x, t.y, t.z);
            else triangle(t.x, t.z, t.y);
        }
    }

    // A vertex no triangle uses -- the far end of an unbounded Voronoi edge
    // outside the outline can lie 10^12 away -- would stretch the bounding
    // box Manifold sizes its tolerance by, and its cleanup would then
    // collapse the whole roof; such vertices are left out. (Only when one
    // lies well beyond the rest: renumbering reorders the output, and one
    // nearer changes the tolerance by no more than a few times.)
    manifold::Manifold build() {
        mesh_.numProp = 3;
        const size_t n = mesh_.vertProperties.size() / 3;
        std::vector<bool> used(n, false);
        for (const auto& t : triangles_) {
            for (int i : t) used[i] = true;
        }
        manifold::Box box;
        for (size_t i = 0; i < n; ++i) {
            if (used[i]) box.Union(manifold::vec3(mesh_.vertProperties[3 * i], mesh_.vertProperties[3 * i + 1], mesh_.vertProperties[3 * i + 2]));
        }
        std::vector<uint64_t> to(n);
        std::vector<double> kept;
        bool stray = false;
        for (size_t i = 0; i < n; ++i) {
            const manifold::vec3 p(mesh_.vertProperties[3 * i], mesh_.vertProperties[3 * i + 1], mesh_.vertProperties[3 * i + 2]);
            stray = stray || (!used[i] && la::maxelem(la::abs(p)) > 16 * box.Scale());
            to[i] = kept.size() / 3;
            if (used[i]) kept.insert(kept.end(), {p.x, p.y, p.z});
        }
        if (stray) mesh_.vertProperties = std::move(kept);
        for (const auto& t : triangles_) {
            for (int i : t) mesh_.triVerts.push_back(stray ? to[i] : static_cast<uint64_t>(i));
        }
        manifold::Manifold body(mesh_);
        if (body.Status() == manifold::Manifold::Error::NoError) return body;
        mesh_.Merge();
        return manifold::Manifold(mesh_);
    }

  private:
    double toWorld_;
    std::map<std::pair<double, double>, int> index_;
    std::set<std::array<int, 3>> triangles_;
    manifold::MeshGL64 mesh_;
};

class RoofBuilder {
  public:
    RoofBuilder(std::vector<std::vector<Vec2>> contours, double toWorld, double fa, double fs)
        : contours_(std::move(contours)), toWorld_(toWorld), fa_(fa), fs_(fs), mesh_(toWorld) {
        for (const auto& c : contours_) {
            for (size_t i = 0; i < c.size(); ++i) {
                const int id = static_cast<int>(edges_.size());
                edges_.push_back({c[i], c[(i + 1) % c.size()]});
                cornerEdges_[{c[i].x, c[i].y}].push_back(id);
                cornerEdges_[{edges_.back().b.x, edges_.back().b.y}].push_back(id);
            }
        }
    }

    manifold::Manifold build() {
        std::vector<boost::polygon::segment_data<int>> segments;
        segments.reserve(edges_.size());
        for (const auto& e : edges_) {
            segments.emplace_back(boost::polygon::point_data<int>(static_cast<int>(e.a.x), static_cast<int>(e.a.y)),
                                  boost::polygon::point_data<int>(static_cast<int>(e.b.x), static_cast<int>(e.b.y)));
        }
        VoronoiDiagram vd;
        boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);
        placeVertices(vd);

        for (const VoronoiCell& cell : vd.cells()) {
            if (cell.is_degenerate()) continue;
            if (cell.contains_point()) cornerCone(cell);
            else edgeSlope(cell);
        }

        std::vector<std::vector<int>> floor;
        for (const auto& c : contours_) {
            floor.emplace_back();
            for (Vec2 p : c) floor.back().push_back(mesh_.vertex(p, 0));
        }
        mesh_.fill(floor, false);
        return mesh_.build();
    }

  private:
    Vec2 pointSite(const VoronoiCell& cell) const {
        const OutlineEdge& edge = edges_[cell.source_index()];
        return cell.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT ? edge.a : edge.b;
    }

    // Distance from p to a cell's site.
    double siteDistance(const VoronoiCell& cell, Vec2 p) const {
        if (cell.contains_point()) return la::length(p - pointSite(cell));
        const OutlineEdge& edge = edges_[cell.source_index()];
        const Vec2 d = edge.b - edge.a;
        const double t = std::clamp(la::dot(p - edge.a, d) / la::dot(d, d), 0.0, 1.0);
        return la::length(p - (edge.a + d * t));
    }

    // Each Voronoi vertex's position and roof height. A vertex at an outline
    // corner sits exactly on it. Voronoi edges shorter than kTol are
    // contracted, their ends taking one shared position: where many bisectors
    // should meet in one point, as at the apex over a regular polygon, the
    // snapping of the outline to the grid scatters them into a knot of
    // hairline edges that would otherwise become needle facets.
    struct VertexInfo {
        Vec2 p;
        double z;
    };
    void placeVertices(const VoronoiDiagram& vd) {
        const auto& vs = vd.vertices();
        vertexBase_ = vs.data();
        std::vector<VertexInfo> raw;
        raw.reserve(vs.size());
        for (const VoronoiVertex& v : vs) {
            Vec2 p{v.x(), v.y()};
            double z = std::numeric_limits<double>::infinity();
            const VoronoiEdge* e = v.incident_edge();
            do {
                const VoronoiCell& cell = *e->cell();
                if (cell.contains_point() && la::length(p - pointSite(cell)) < kTol) {
                    p = pointSite(cell);
                    z = 0;
                    break;
                }
                z = std::min(z, siteDistance(cell, p));
                e = e->rot_next();
            } while (e != v.incident_edge());
            raw.push_back({p, z});
        }

        std::vector<size_t> parent(vs.size());
        for (size_t i = 0; i < parent.size(); ++i) parent[i] = i;
        const auto root = [&](size_t i) {
            while (parent[i] != i) i = parent[i] = parent[parent[i]];
            return i;
        };
        for (const VoronoiEdge& e : vd.edges()) {
            if (!e.is_finite() || &e > e.twin()) continue;
            const size_t i0 = indexOf(e.vertex0()), i1 = indexOf(e.vertex1());
            const size_t a = root(i0), b = root(i1);
            if (a == b || la::length(raw[i0].p - raw[i1].p) >= kTol) continue;
            // Keep a corner if there is one, else the highest point: the
            // knot under an apex sits just below the true apex.
            if (raw[a].z == 0 || (raw[b].z != 0 && raw[a].z > raw[b].z)) parent[b] = a;
            else parent[a] = b;
        }
        vertices_.resize(vs.size());
        for (size_t i = 0; i < vs.size(); ++i) vertices_[i] = raw[root(i)];
    }
    size_t indexOf(const VoronoiVertex* v) const { return static_cast<size_t>(v - vertexBase_); }
    const VertexInfo& info(const VoronoiVertex* v) const { return vertices_[indexOf(v)]; }
    int roofVertex(const VoronoiVertex* v) { return mesh_.vertex(info(v).p, info(v).z); }

    // The roof vertices along a finite Voronoi half-edge, from vertex0 to
    // vertex1. A curved edge (between a corner and an edge) is a parabola; it
    // is split into pieces that, seen from the corner, turn by at most twice
    // $fa degrees and are at most $fs long, so either setting made smaller
    // refines the cone around the corner. (Twice $fa matches the reference's
    // facet density: its default roofs agree with these in volume to a few
    // parts per million.)
    std::vector<int> edgePath(const VoronoiEdge& e) {
        const bool forward = &e < e.twin();
        const VoronoiEdge& key = forward ? e : *e.twin();
        auto [it, inserted] = paths_.try_emplace(&key);
        std::vector<int>& path = it->second;
        if (inserted) {
            path.push_back(roofVertex(key.vertex0()));
            if (key.is_curved() && info(key.vertex0()).p != info(key.vertex1()).p) parabola(key, path);
            path.push_back(roofVertex(key.vertex1()));
        }
        return forward ? path : std::vector<int>(path.rbegin(), path.rend());
    }

    void parabola(const VoronoiEdge& e, std::vector<int>& path) {
        const VoronoiCell& pc = e.cell()->contains_point() ? *e.cell() : *e.twin()->cell();
        const VoronoiCell& sc = e.cell()->contains_point() ? *e.twin()->cell() : *e.cell();
        const Vec2 focus = pointSite(pc);
        const OutlineEdge& line = edges_[sc.source_index()];
        Vec2 n = Vec2{-(line.b - line.a).y, (line.b - line.a).x} / la::length(line.b - line.a);
        double h = la::dot(focus - line.a, n);
        if (h < 0) {
            n = -n;
            h = -h;
        }
        if (h < kTol) return;
        const Vec2 a = info(e.vertex0()).p - focus, b = info(e.vertex1()).p - focus;
        const double sweep = std::atan2(cross2(a, b), la::dot(a, b));
        const double byAngle = std::abs(sweep) * 180 / kPi / (2 * fa_);
        const double byLength = la::length(b - a) * toWorld_ / fs_;
        const int pieces = std::max(1, static_cast<int>(std::ceil(std::max(byAngle, byLength))));
        const double start = std::atan2(a.y, a.x);
        for (int i = 1; i < pieces; ++i) {
            const double angle = start + sweep * i / pieces;
            const Vec2 u{std::cos(angle), std::sin(angle)};
            // The point along u from the focus that is as far from it as
            // from the edge's line: t = h + t (u . n).
            const double t = h / (1 - la::dot(u, n));
            path.push_back(mesh_.vertex(focus + u * t, t));
        }
    }

    // A point on one of the cell's Voronoi edges, away from its ends (or the
    // end itself for an edge contracted to a point).
    Vec2 sample(const VoronoiEdge& e) {
        const std::vector<int> path = edgePath(e);
        if (path.size() > 2) return mesh_.position(path[path.size() / 2]);
        return (mesh_.position(path.front()) + mesh_.position(path.back())) * 0.5;
    }

    // Is `p`, which lies nearer to the corner `site` than to any edge, inside
    // the outline? Find the outline edge bounding, clockwise, the angular
    // sector around the corner that p falls in: the sector is inside exactly
    // when that edge leaves the corner (interior is on the left of every edge).
    bool isInsideAtCorner(Vec2 site, Vec2 p) const {
        const Vec2 w = p - site;
        double best = std::numeric_limits<double>::infinity();
        bool inside = false;
        for (int id : cornerEdges_.at({site.x, site.y})) {
            const OutlineEdge& edge = edges_[id];
            const bool leaves = edge.a == site;
            const Vec2 u = leaves ? edge.b - edge.a : edge.a - edge.b;
            double ccw = std::atan2(cross2(u, w), la::dot(u, w));
            if (ccw < 0) ccw += 2 * kPi;
            if (ccw < best) {
                best = ccw;
                inside = leaves;
            }
        }
        return inside;
    }

    // Around a reflex corner: the cone, as a fan from the corner to each
    // Voronoi edge of the cell inside the outline.
    void cornerCone(const VoronoiCell& cell) {
        const Vec2 site = pointSite(cell);
        const int apex = mesh_.vertex(site, 0);
        const VoronoiEdge* e = cell.incident_edge();
        do {
            if (e->is_finite() && isInsideAtCorner(site, sample(*e))) {
                const std::vector<int> path = edgePath(*e);
                for (size_t i = 0; i + 1 < path.size(); ++i) mesh_.triangle(apex, path[i], path[i + 1]);
            }
            e = e->next();
        } while (e != cell.incident_edge());
    }

    // Over an outline edge: one planar facet, between the edge and the run of
    // the cell's Voronoi edges on its inner side. The cell is walked
    // counter-clockwise, so that run goes from the edge's far end back to its
    // near one, and closing it along the edge itself gives a
    // counter-clockwise polygon.
    void edgeSlope(const VoronoiCell& cell) {
        const OutlineEdge& edge = edges_[cell.source_index()];
        std::vector<const VoronoiEdge*> ring;
        std::vector<bool> inner;
        const VoronoiEdge* e = cell.incident_edge();
        do {
            ring.push_back(e);
            inner.push_back(e->is_finite() && cross2(edge.b - edge.a, sample(*e) - edge.a) > 0);
            e = e->next();
        } while (e != cell.incident_edge());

        std::vector<std::vector<int>> facets;
        const size_t n = ring.size();
        for (size_t i = 0; i < n; ++i) {
            if (!inner[i] || inner[(i + n - 1) % n]) continue; // the start of a run
            std::vector<int> loop;
            for (size_t j = i; inner[j % n] && j < i + n; ++j) {
                const std::vector<int> path = edgePath(*ring[j % n]);
                loop.insert(loop.end(), path.begin(), path.end());
            }
            facets.push_back(std::move(loop));
        }
        for (const auto& loop : facets) mesh_.fill({loop}, true);
    }

    static constexpr double kTol = 1 << 10; // grid units: 2^-20 of the outline's extent
    static constexpr double kPi = 3.14159265358979323846;

    std::vector<std::vector<Vec2>> contours_;
    double toWorld_, fa_, fs_;
    RoofMesh mesh_;
    std::vector<OutlineEdge> edges_;
    std::map<std::pair<double, double>, std::vector<int>> cornerEdges_;
    const VoronoiVertex* vertexBase_ = nullptr;
    std::vector<VertexInfo> vertices_;
    std::map<const VoronoiEdge*, std::vector<int>> paths_;
};

// method="straight": the straight-skeleton roof (Aichholzer, Alberts,
// Aurenhammer & Gaertner, "A novel type of skeleton for polygons", 1995).
// Every outline edge moves inward at unit speed, parallel to itself; the
// roof's height over a point is the time at which this shrinking wavefront
// sweeps it, so each edge's face is a plane rising at 45 degrees. Unlike the
// Voronoi roof there are no cones: at a reflex corner the two neighbouring
// faces simply meet along the corner's bisector.
//
// The wavefront is simulated event by event, after Felkel & Obdrzalek
// ("Straight Skeleton Implementation", 1998). Each wavefront vertex sits where
// its two edges' moving lines cross and so moves at a fixed velocity until an
// event replaces it:
//   - edge event: an edge shrinks to nothing; its two ends merge.
//   - split event: a reflex vertex runs into another edge and cuts the
//     wavefront loop in two.
// A vertex never changes once made -- an event kills the vertices it involves
// and makes new ones -- so a queued event stays valid as long as the vertices
// it names are alive and still neighbours. Each reflex vertex keeps only its
// earliest split candidate queued; if that candidate's edge goes away first,
// the vertex is recomputed when the stale entry is popped, and each new edge
// is offered to every reflex vertex as it appears. That is O(n) per event
// and O(n^2 log n) in all.
//
// Degenerate cases (worked out against OpenSCAD 2026.02.01's output, which
// agrees on every shape tried that it does not crash on):
//   - simultaneous events are processed one at a time, an edge left with no
//     length collapsing at once;
//   - a reflex vertex arriving at another vertex rather than an edge (a
//     "vertex event", as where a frame's hole corner meets its outline's)
//     reconnects every wavefront edge at that point afresh -- see
//     vertexEvent -- unless the two only touch there and part again;
//   - two edges of one loop facing each other on the same line (the ridge of
//     a rectangle, a corridor of constant width) have closed to nothing
//     between them: where they meet at a vertex, their overlap is folded away
//     at once and becomes a ridge segment on both faces;
//   - a loop of two vertices has no area left and is retired whole.
// Roof vertices within kEps of each other are merged at the end, so the many
// coincident events of a symmetric shape meet in one point.
class StraightSkeletonRoof {
  public:
    StraightSkeletonRoof(const std::vector<std::vector<Vec2>>& contours, double toWorld) : mesh_(toWorld) {
        for (const auto& c : contours) {
            const int first = static_cast<int>(lines_.size()), m = static_cast<int>(c.size());
            std::vector<int> corners;
            for (int i = 0; i < m; ++i) {
                const Vec2 a = c[i], b = c[(i + 1) % m];
                const Vec2 d = la::normalize(b - a), n{-d.y, d.x};
                lines_.push_back({n, d, la::dot(n, a)});
                corners.push_back(node(a, 0));
            }
            for (int i = 0; i < m; ++i) floor_.push_back({corners[i], corners[(i + 1) % m]});
            segs_.resize(lines_.size());
            for (int i = 0; i < m; ++i) {
                addSeg(first + i, corners[i], corners[(i + 1) % m]);
                makeVertex(first + (i + m - 1) % m, first + i, c[i], corners[i]);
            }
            for (int i = 0; i < m; ++i) link(first + (i + m - 1) % m, first + i, first + (i + 1) % m);
        }
    }

    manifold::Manifold build() {
        // Any corner where the outline still doubles back folds away before
        // anything moves.
        std::vector<int> folds;
        for (int v = 0; v < static_cast<int>(verts_.size()); ++v) {
            if (verts_[v].fold) folds.push_back(v);
        }
        settle(folds);
        for (int v = 0; v < static_cast<int>(verts_.size()); ++v) {
            if (!verts_[v].alive) continue;
            scheduleEdge(v);
            if (verts_[v].reflex) {
                reflex_.push_back(v);
                recompute(v);
            }
        }
        size_t budget = 64 * verts_.size() + 1024;
        while (!queue_.empty()) {
            const Event e = queue_.top();
            queue_.pop();
            if (!stillValid(e)) continue;
            if (--budget == 0) throw std::runtime_error("straight skeleton did not converge");
            now_ = std::max(now_, e.t);
            if (e.kind == kEdge) edgeEvent(e.a, e.b);
            else splitEvent(e.a, e.b);
        }
        for (const Vert& v : verts_) {
            if (v.alive) throw std::runtime_error("straight skeleton did not converge");
        }

        // Events a hair apart -- the many near-simultaneous ones at the apex
        // of a fine circle, say -- leave a knot of roof vertices too close to
        // tell apart, and needle faces between them. Each vertex is merged
        // into the highest one within kEps of it; the faces' boundaries are
        // then cleaned of what that collapses (see cleanLoop), which keeps
        // every boundary edge paired with its neighbour's.
        std::vector<int> order(nodes_.size());
        for (size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
        std::stable_sort(order.begin(), order.end(), [&](int x, int y) { return nodes_[x].second > nodes_[y].second; });
        std::vector<int> vertex(nodes_.size(), -1);
        std::map<std::pair<int64_t, int64_t>, std::vector<int>> cells; // vertex ids by kEps cell
        for (int i : order) {
            const auto& [p, z] = nodes_[i];
            const int64_t cx = static_cast<int64_t>(std::floor(p.x / kEps)), cy = static_cast<int64_t>(std::floor(p.y / kEps));
            for (int64_t dx = -1; dx <= 1 && vertex[i] < 0; ++dx) {
                for (int64_t dy = -1; dy <= 1 && vertex[i] < 0; ++dy) {
                    auto it = cells.find({cx + dx, cy + dy});
                    if (it == cells.end()) continue;
                    for (int v : it->second) {
                        if (la::length(mesh_.position(v) - p) <= kEps && std::abs(mesh_.height(v) - z) <= kEps) {
                            vertex[i] = v;
                            break;
                        }
                    }
                }
            }
            if (vertex[i] < 0) {
                vertex[i] = mesh_.addVertex(p, z);
                cells[{cx, cy}].push_back(vertex[i]);
            }
        }
        const auto loopsOf = [&](const std::vector<std::pair<int, int>>& segs) {
            std::vector<std::vector<int>> loops;
            for (const auto& loop : faceLoops(segs)) {
                std::vector<int> ids;
                for (int i : loop) ids.push_back(vertex[i]);
                cleanLoop(ids, loops);
            }
            return loops;
        };
        for (const auto& segs : segs_) mesh_.fill(loopsOf(segs), true);
        mesh_.fill(loopsOf(floor_), false);
        return mesh_.build();
    }

  private:
    struct Line {
        Vec2 n, d; // inward normal, direction
        double c;  // at time t the line is n.x = c + t
    };
    struct Vert {
        int prev = -1, next = -1, lin, lout; // lines of the edges arriving and leaving
        Vec2 p0;                              // position at birth
        double t0;                            // birth time
        Vec2 w;                               // velocity
        int born;                             // its roof vertex
        bool alive = true, reflex = false, fold = false;
        unsigned stamp = 0;
    };
    enum Kind { kEdge = 0, kSplit = 1 };
    struct Event {
        double t;
        int kind, a, b, c; // edge a->b; or vertex a against edge b->c
        unsigned stamp;
        bool operator>(const Event& o) const { return t != o.t ? t > o.t : kind > o.kind; }
    };

    static constexpr double kEps = 256; // grid units: 2^-21 of the outline's extent
    static constexpr double kInf = std::numeric_limits<double>::infinity();

    Vec2 pos(int v, double t) const { return verts_[v].p0 + verts_[v].w * (t - verts_[v].t0); }
    Vec2 pos(int v) const { return pos(v, now_); }

    // A new roof vertex at (p, z).
    int node(Vec2 p, double z) {
        nodes_.push_back({p, z});
        return static_cast<int>(nodes_.size()) - 1;
    }
    Vec2 where(int node) const { return nodes_[node].first; }

    void addSeg(int line, int from, int to) {
        if (from != to) segs_[line].push_back({from, to});
    }

    int makeVertex(int lin, int lout, Vec2 p, int born) {
        Vert v;
        v.lin = lin;
        v.lout = lout;
        v.p0 = p;
        v.t0 = now_;
        v.born = born;
        const Line &a = lines_[lin], &b = lines_[lout];
        const double cosine = la::dot(a.n, b.n);
        if (cosine < -1 + 1e-12) {
            v.fold = true;
            v.w = Vec2{0, 0};
        } else {
            v.w = (a.n + b.n) / (1 + cosine); // n_in.w = n_out.w = 1
            v.reflex = cross2(a.d, b.d) < -1e-12;
            // Start exactly where the two lines cross, not at the event's
            // point, which is only as good as the vertices that met there:
            // left off its lines, a vertex's error would carry into every
            // later event (unless the lines are too near parallel to say).
            const double det = cross2(a.n, b.n);
            if (std::abs(det) > 1e-6) {
                const double ca = a.c + now_, cb = b.c + now_;
                const Vec2 meet{(ca * b.n.y - cb * a.n.y) / det, (a.n.x * cb - b.n.x * ca) / det};
                if (la::length(meet - p) <= 16 * kEps) v.p0 = meet;
            }
        }
        verts_.push_back(v);
        best_.push_back(kInf);
        return static_cast<int>(verts_.size()) - 1;
    }

    void link(int a, int z, int b) {
        verts_[a].next = z;
        verts_[z].prev = a;
        verts_[z].next = b;
        verts_[b].prev = z;
    }

    // The faces either side of a vertex's path gain it as a boundary.
    void kill(int v, int at) {
        Vert& x = verts_[v];
        x.alive = false;
        if (x.reflex) ++dead_;
        addSeg(x.lin, x.born, at);
        addSeg(x.lout, at, x.born);
    }

    bool stillValid(const Event& e) {
        if (e.kind == kEdge) return verts_[e.a].alive && verts_[e.b].alive && verts_[e.a].next == e.b;
        const Vert& v = verts_[e.a];
        if (!v.alive || v.stamp != e.stamp) return false;
        if (verts_[e.b].alive && verts_[e.c].alive && verts_[e.b].next == e.c) return true;
        recompute(e.a); // its target edge is gone: find its next one
        return false;
    }

    void scheduleEdge(int u) {
        const int v = verts_[u].next;
        const Vec2 d = lines_[verts_[u].lout].d;
        const double s = la::dot(d, pos(v) - pos(u)), rate = la::dot(d, verts_[v].w - verts_[u].w);
        if (s <= kEps) queue_.push({now_, kEdge, u, v, 0, 0});
        else if (rate < 0) queue_.push({now_ + s / -rate, kEdge, u, v, 0, 0});
    }

    // When reflex vertex r would hit the edge x -> next(x), or kInf.
    double splitTime(int r, int x) const {
        const int y = verts_[x].next;
        if (r == x || r == y) return kInf;
        const Line& l = lines_[verts_[x].lout];
        const Vec2 p = pos(r);
        const double dist = la::dot(l.n, p) - (l.c + now_), rate = la::dot(l.n, verts_[r].w) - 1;
        if (dist < -kEps || rate >= -1e-12) return kInf;
        const double t = now_ + std::max(dist, 0.0) / -rate;
        const Vec2 px = pos(x, t), hit = pos(r, t);
        const double len = la::dot(l.d, pos(y, t) - px), along = la::dot(l.d, hit - px);
        if (len < -kEps || along < -kEps || along > len + kEps) return kInf;
        // Arriving at an end of the edge, r meets the vertex there: only a
        // collision if the wavefront's interiors at the two meet or overlap,
        // not where the two merely touch at a point and part again.
        if (along < kEps && !overlaps(r, x)) return kInf;
        if (along > len - kEps && !overlaps(r, y)) return kInf;
        return t;
    }

    // Do the wavefront's interiors at vertices a and b, seen from one point,
    // overlap or share a side? Each is the wedge counter-clockwise from the
    // vertex's leaving edge round to its arriving one. (Sharing a side, as
    // where a hole's corner meets the outline's, is two edges colliding.)
    bool overlaps(int a, int b) const {
        const auto angle = [](Vec2 u, Vec2 v) { // counter-clockwise, in [0, 2 pi)
            const double a = std::atan2(cross2(u, v), la::dot(u, v));
            return a < 0 ? a + 2 * kPi : a;
        };
        const Vec2 ao = lines_[verts_[a].lout].d, bo = lines_[verts_[b].lout].d;
        const double aSpan = angle(ao, -lines_[verts_[a].lin].d), bSpan = angle(bo, -lines_[verts_[b].lin].d);
        constexpr double kSlack = 1e-9;
        return angle(ao, bo) < aSpan + kSlack || angle(bo, ao) < bSpan + kSlack;
    }

    void offer(int r, int x, double t) {
        if (!(t < best_[r])) return;
        best_[r] = t;
        queue_.push({t, kSplit, r, x, verts_[x].next, ++verts_[r].stamp});
    }

    void recompute(int r) {
        best_[r] = kInf;
        int bestX = -1;
        double bestT = kInf;
        for (int x = 0; x < static_cast<int>(verts_.size()); ++x) {
            if (!verts_[x].alive) continue;
            const double t = splitTime(r, x);
            if (t < bestT) {
                bestT = t;
                bestX = x;
            }
        }
        if (bestX >= 0) offer(r, bestX, bestT);
        else ++verts_[r].stamp;
    }

    void edgeEvent(int u, int v) {
        const int at = node((pos(u) + pos(v)) * 0.5, now_);
        const Vec2 p = where(at);
        const int a = verts_[u].prev, b = verts_[v].next;
        kill(u, at);
        kill(v, at);
        if (a == v) return; // a loop of two, already retired
        const int z = makeVertex(verts_[u].lin, verts_[v].lout, p, at);
        link(a, z, b);
        settle({z});
    }

    void splitEvent(int v, int x) {
        const int y = verts_[x].next;
        if (la::length(pos(v) - pos(x)) <= kEps || la::length(pos(v) - pos(y)) <= kEps) {
            vertexEvent(pos(v));
            return;
        }
        const int at = node(pos(v), now_);
        const Vec2 p = where(at);
        const int a = verts_[v].prev, b = verts_[v].next, l = verts_[x].lout;
        kill(v, at);
        const int v1 = makeVertex(verts_[v].lin, l, p, at);
        const int v2 = makeVertex(l, verts_[v].lout, p, at);
        link(a, v1, y);
        link(x, v2, b);
        settle({v1, v2});
    }

    // Vertices meeting at one point (two reflex corners head on, or more):
    // the wavefront edges at the point are reconnected afresh. Going
    // counter-clockwise round it, each edge leaving the point is joined to
    // the next edge arriving, so that the wedges of wavefront interior they
    // bound no longer overlap. Edges between two of the vertices have no
    // length left and go.
    void vertexEvent(Vec2 p) {
        std::vector<int> meet;
        for (int q = 0; q < static_cast<int>(verts_.size()); ++q) {
            if (verts_[q].alive && la::length(pos(q) - p) <= kEps) meet.push_back(q);
        }
        const auto in = [&](int q) { return std::find(meet.begin(), meet.end(), q) != meet.end(); };
        struct Spoke {
            double angle;
            bool leaving;
            int line, far; // the edge's line; the vertex at its other end
        };
        std::vector<Spoke> spokes;
        for (int q : meet) {
            const Vert& v = verts_[q];
            if (!in(v.next)) spokes.push_back({std::atan2(lines_[v.lout].d.y, lines_[v.lout].d.x), true, v.lout, v.next});
            if (!in(v.prev)) spokes.push_back({std::atan2(-lines_[v.lin].d.y, -lines_[v.lin].d.x), false, v.lin, v.prev});
        }
        const int at = node(p, now_);
        for (int q : meet) kill(q, at);
        // Edges along one ray -- one leaving, one arriving -- lie on top of
        // each other: leaving first, they are joined, and fold away.
        std::sort(spokes.begin(), spokes.end(), [](const Spoke& a, const Spoke& b) { return a.angle < b.angle; });
        for (size_t i = 0; i + 1 < spokes.size(); ++i) {
            if (!spokes[i].leaving && spokes[i + 1].leaving && spokes[i + 1].angle - spokes[i].angle <= 1e-9) std::swap(spokes[i], spokes[i + 1]);
        }
        // Match leaving to arriving edges like brackets, counter-clockwise
        // and round twice so that a match may wrap past the start.
        std::vector<int> open, fresh;
        std::vector<bool> used(spokes.size(), false);
        for (size_t k = 0; k < 2 * spokes.size(); ++k) {
            const size_t i = k % spokes.size();
            if (used[i]) continue;
            if (spokes[i].leaving) {
                if (k < spokes.size()) open.push_back(static_cast<int>(i));
            } else if (!open.empty()) {
                const Spoke &o = spokes[open.back()], &a = spokes[i];
                used[open.back()] = used[i] = true;
                open.pop_back();
                const int z = makeVertex(a.line, o.line, p, at);
                link(a.far, z, o.far);
                fresh.push_back(z);
            }
        }
        settle(fresh);
    }

    // x's two edges lie on one line, facing each other: the wavefront has
    // closed between them. Their overlap is folded away into a ridge.
    void fold(int x, std::vector<int>& work) {
        const int p = verts_[x].prev, n = verts_[x].next;
        const int li = verts_[x].lin, lo = verts_[x].lout;
        const Vec2 d = lines_[li].d, px = pos(x);
        const double a = la::dot(d, px - pos(p)), b = la::dot(d, px - pos(n));
        const int at = node(px, now_);
        if (std::abs(a - b) <= kEps) { // both edges fold away entirely
            const int pAt = node(pos(p), now_), nAt = pAt; // p and n meet
            const int pp = verts_[p].prev, nn = verts_[n].next;
            kill(x, at);
            kill(p, pAt);
            kill(n, nAt);
            addSeg(li, at, pAt);
            addSeg(lo, nAt, at);
            if (pp == n) { // the loop was just these three
                addSeg(verts_[n].lout, pAt, nAt);
                return;
            }
            const int y = makeVertex(verts_[p].lin, verts_[n].lout, where(pAt), pAt);
            link(pp, y, nn);
            work.push_back(y);
        } else if (a > b) { // n folds onto the edge p -> x
            const int nAt = node(pos(n), now_), nn = verts_[n].next;
            kill(x, at);
            kill(n, nAt);
            addSeg(li, at, nAt);
            addSeg(lo, nAt, at);
            const int y = makeVertex(li, verts_[n].lout, where(nAt), nAt);
            link(p, y, nn);
            work.push_back(y);
        } else { // p folds onto the edge x -> n
            const int pAt = node(pos(p), now_), pp = verts_[p].prev;
            kill(x, at);
            kill(p, pAt);
            addSeg(li, at, pAt);
            addSeg(lo, pAt, at);
            const int y = makeVertex(verts_[p].lin, lo, where(pAt), pAt);
            link(pp, y, n);
            work.push_back(y);
        }
    }

    // Resolves what the new vertices `work` leave degenerate, then queues the
    // events of the vertices and edges that remain.
    void settle(std::vector<int> work) {
        std::vector<int> fresh;
        while (!work.empty()) {
            const int z = work.back();
            work.pop_back();
            if (!verts_[z].alive) continue;
            const int o = verts_[z].next;
            if (o == z) {
                kill(z, node(pos(z), now_));
            } else if (verts_[o].next == z) { // a loop of two: no area left
                const int zAt = node(pos(z), now_), oAt = node(pos(o), now_);
                kill(z, zAt);
                kill(o, oAt);
                addSeg(verts_[z].lout, oAt, zAt);
                addSeg(verts_[o].lout, zAt, oAt);
            } else if (verts_[z].fold) {
                fold(z, work);
            } else {
                fresh.push_back(z);
            }
        }
        std::vector<int> edges; // by their first vertex
        for (int z : fresh) {
            if (!verts_[z].alive) continue;
            edges.push_back(verts_[z].prev);
            edges.push_back(z);
        }
        std::sort(edges.begin(), edges.end());
        edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
        if (reflex_.size() > 64 && dead_ * 2 > reflex_.size()) {
            std::erase_if(reflex_, [&](int r) { return !verts_[r].alive; });
            dead_ = 0;
        }
        for (int x : edges) {
            scheduleEdge(x);
            for (int r : reflex_) {
                if (verts_[r].alive) offer(r, x, splitTime(r, x));
            }
        }
        for (int z : fresh) {
            if (verts_[z].alive && verts_[z].reflex) {
                reflex_.push_back(z);
                recompute(z);
            }
        }
    }

    // Appends to `out` what remains of a boundary loop once merged vertices
    // have collapsed parts of it: repeats and out-and-back spikes go, and a
    // loop that now touches itself is split there. Each spike's two edges
    // were paired with edges of other faces, which now pair with each other.
    static void cleanLoop(std::vector<int> loop, std::vector<std::vector<int>>& out) {
        std::vector<int> kept;
        for (int v : loop) {
            if (!kept.empty() && kept.back() == v) continue;
            if (kept.size() >= 2 && kept[kept.size() - 2] == v) { // ... v, x, v: drop the spike to x
                kept.pop_back();
                continue;
            }
            kept.push_back(v);
        }
        // The same, across the loop's seam.
        bool changed = true;
        while (changed && kept.size() >= 2) {
            changed = false;
            if (kept.front() == kept.back()) {
                kept.pop_back();
                changed = true;
            } else if (kept.size() >= 3 && kept[kept.size() - 2] == kept.front()) {
                kept.pop_back();
                changed = true;
            } else if (kept.size() >= 3 && kept[1] == kept.back()) {
                kept.erase(kept.begin());
                changed = true;
            }
        }
        if (kept.size() < 3) return;
        // Split where the loop passes through a vertex twice.
        std::map<int, size_t> seen;
        for (size_t i = 0; i < kept.size(); ++i) {
            auto [it, fresh] = seen.try_emplace(kept[i], i);
            if (fresh) continue;
            std::vector<int> inner(kept.begin() + static_cast<std::ptrdiff_t>(it->second), kept.begin() + static_cast<std::ptrdiff_t>(i));
            std::vector<int> rest(kept.begin(), kept.begin() + static_cast<std::ptrdiff_t>(it->second));
            rest.insert(rest.end(), kept.begin() + static_cast<std::ptrdiff_t>(i), kept.end());
            cleanLoop(std::move(inner), out);
            cleanLoop(std::move(rest), out);
            return;
        }
        out.push_back(std::move(kept));
    }

    // A face's boundary segments chained into loops. Where the boundary
    // touches itself, the walk takes the sharpest turn clockwise, keeping to
    // the region it came along.
    std::vector<std::vector<int>> faceLoops(const std::vector<std::pair<int, int>>& segs) const {
        std::multimap<int, size_t> from;
        for (size_t i = 0; i < segs.size(); ++i) from.emplace(segs[i].first, i);
        std::vector<bool> used(segs.size(), false);
        std::vector<std::vector<int>> loops;
        for (size_t s = 0; s < segs.size(); ++s) {
            if (used[s]) continue;
            std::vector<int> loop;
            size_t cur = s;
            while (true) {
                used[cur] = true;
                loop.push_back(segs[cur].first);
                const int at = segs[cur].second;
                if (at == segs[s].first) break;
                const Vec2 back = where(segs[cur].first) - where(at);
                size_t pick = segs.size();
                double pickAngle = kInf;
                auto [lo, hi] = from.equal_range(at);
                for (auto it = lo; it != hi; ++it) {
                    if (used[it->second]) continue;
                    const Vec2 out = where(segs[it->second].second) - where(at);
                    double cw = -std::atan2(cross2(back, out), la::dot(back, out));
                    if (cw <= 0) cw += 2 * kPi;
                    if (cw < pickAngle) {
                        pickAngle = cw;
                        pick = it->second;
                    }
                }
                if (pick == segs.size()) break;
                cur = pick;
            }
            loops.push_back(std::move(loop));
        }
        return loops;
    }

    static constexpr double kPi = 3.14159265358979323846;

    RoofMesh mesh_;
    std::vector<Line> lines_;
    std::vector<Vert> verts_;
    std::vector<double> best_;
    std::vector<int> reflex_;
    size_t dead_ = 0;
    std::vector<std::vector<std::pair<int, int>>> segs_; // per line: its face's boundary
    std::vector<std::pair<int, int>> floor_; // the outline, as boundary segments
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> queue_;
    double now_ = 0;
    std::vector<std::pair<Vec2, double>> nodes_; // roof vertices: (x, y), z
};

// Removes the corners where an outline runs straight on (or doubles back):
// they are no corners of the roof, and the Voronoi diagram passes straight
// through them, which would leave them without a vertex to build from.
void dropStraightCorners(std::vector<Vec2>& c) {
    bool changed = true;
    while (changed && c.size() >= 3) {
        changed = false;
        for (size_t i = 0; i < c.size() && c.size() >= 3; ++i) {
            const Vec2 prev = c[(i + c.size() - 1) % c.size()], next = c[(i + 1) % c.size()];
            // Grid coordinates stay below 2^30, so this is exact in 64 bits.
            const auto x = [](double v) { return static_cast<int64_t>(v); };
            if ((x(c[i].x) - x(prev.x)) * (x(next.y) - x(prev.y)) == (x(c[i].y) - x(prev.y)) * (x(next.x) - x(prev.x))) {
                c.erase(c.begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
            }
        }
    }
}

// Removes the spikes where an outline doubles back on itself -- a slit or
// hairline a union can leave, often not exactly straight, so that
// dropStraightCorners misses it. Cut back to where the two sides part, a
// spike encloses nothing, and roofed it would be a face folded over itself.
void dropSpikes(std::vector<Vec2>& c) {
    bool changed = true;
    while (changed && c.size() >= 3) {
        changed = false;
        for (size_t i = 0; i < c.size() && c.size() >= 3; ++i) {
            const size_t m = c.size(), p = (i + m - 1) % m, n = (i + 1) % m;
            const Vec2 in = c[i] - c[p], out = c[n] - c[i];
            const double a = la::length(in), b = la::length(out);
            if (!(la::dot(in, out) < (-1 + 1e-12) * a * b)) continue;
            // The shorter side ends on the longer one: the tip goes, and if
            // the sides are as long as each other, so does the far end.
            if (std::abs(a - b) <= 1e-9 * std::max(a, b)) {
                c.erase(c.begin() + static_cast<std::ptrdiff_t>(std::max(i, n)));
                c.erase(c.begin() + static_cast<std::ptrdiff_t>(std::min(i, n)));
            } else {
                c.erase(c.begin() + static_cast<std::ptrdiff_t>(i));
            }
            changed = true;
        }
    }
}

// The outline of `cs` snapped to a grid of 2^(bits+1) steps across its
// largest coordinate, with repeated and straight corners dropped. Sets
// `toGrid`.
std::vector<std::vector<Vec2>> gridContours(const manifold::CrossSection& cs, double& toGrid, int bits = 29) {
    const manifold::Polygons polys = cs.ToPolygons();
    double extent = 0;
    for (const auto& poly : polys) {
        for (Vec2 p : poly) extent = std::max({extent, std::abs(p.x), std::abs(p.y)});
    }
    if (!(extent > 0) || !std::isfinite(extent)) return {};
    toGrid = std::ldexp(1.0, bits - std::ilogb(extent));

    std::vector<std::vector<Vec2>> contours;
    for (const auto& poly : polys) {
        std::vector<Vec2> c;
        for (Vec2 p : poly) {
            const Vec2 q{std::round(p.x * toGrid), std::round(p.y * toGrid)};
            if (c.empty() || c.back() != q) c.push_back(q);
        }
        while (c.size() > 1 && c.front() == c.back()) c.pop_back();
        dropStraightCorners(c);
        if (c.size() >= 3) contours.push_back(std::move(c));
    }
    return contours;
}

// The outline welded where it touches itself: a union can leave pieces
// that share an edge, or a hole against the outline, as outlines a hairline
// apart, crossing at nothing Clipper can see. Each corner within kWeld of
// another is moved onto it, and each corner within kWeld of an edge is
// added to the edge, so that touching boundaries share their corners
// exactly; a union then merges them. (The wavefront would otherwise roof the
// pieces apart, and their boundaries would run along each other.)
std::vector<std::vector<Vec2>> weldContours(std::vector<std::vector<Vec2>> contours) {
    constexpr double kWeld = 1 << 10; // grid units: 2^-19 of the outline's extent
    const auto cell = [](Vec2 p) { return std::pair<int64_t, int64_t>{static_cast<int64_t>(std::floor(p.x / kWeld)), static_cast<int64_t>(std::floor(p.y / kWeld))}; };
    std::map<std::pair<int64_t, int64_t>, std::vector<Vec2>> seen;
    std::vector<Vec2> corners;
    for (auto& c : contours) {
        for (Vec2& p : c) {
            const auto [cx, cy] = cell(p);
            bool moved = false;
            for (int64_t dx = -1; dx <= 1 && !moved; ++dx) {
                for (int64_t dy = -1; dy <= 1 && !moved; ++dy) {
                    auto it = seen.find({cx + dx, cy + dy});
                    if (it == seen.end()) continue;
                    for (Vec2 q : it->second) {
                        if (la::length(q - p) <= kWeld) {
                            p = q;
                            moved = true;
                            break;
                        }
                    }
                }
            }
            if (!moved) {
                seen[{cx, cy}].push_back(p);
                corners.push_back(p);
            }
        }
    }
    // Each edge looks only at the corners in its x range.
    // ponytail: still O(n^2) for an outline of long edges all spanning one
    // x range; a grid of edges if that ever matters.
    const auto byX = [](Vec2 p, Vec2 q) { return p.x != q.x ? p.x < q.x : p.y < q.y; };
    std::sort(corners.begin(), corners.end(), byX);
    Clipper2Lib::Paths64 paths;
    for (const auto& c : contours) {
        Clipper2Lib::Path64 path;
        for (size_t i = 0; i < c.size(); ++i) {
            const Vec2 a = c[i], b = c[(i + 1) % c.size()], d = b - a;
            const double len = la::length(d);
            path.push_back({static_cast<int64_t>(a.x), static_cast<int64_t>(a.y)});
            if (len == 0) continue;
            const Vec2 lo = la::min(a, b) - kWeld, hi = la::max(a, b) + kWeld;
            std::vector<std::pair<double, Vec2>> on;
            for (auto it = std::lower_bound(corners.begin(), corners.end(), Vec2{lo.x, -std::numeric_limits<double>::infinity()}, byX);
                 it != corners.end() && it->x <= hi.x; ++it) {
                const Vec2 q = *it;
                if (q.y < lo.y || q.y > hi.y || q == a || q == b) continue;
                const double t = la::dot(q - a, d) / len;
                if (t > 0 && t < len && std::abs(cross2(d, q - a)) / len <= kWeld) on.push_back({t, q});
            }
            std::sort(on.begin(), on.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
            for (const auto& [t, q] : on) path.push_back({static_cast<int64_t>(q.x), static_cast<int64_t>(q.y)});
        }
        paths.push_back(std::move(path));
    }
    std::vector<std::vector<Vec2>> welded;
    for (const auto& path : Clipper2Lib::Union(paths, Clipper2Lib::FillRule::Positive)) {
        std::vector<Vec2> c;
        for (const auto& p : path) c.push_back(Vec2{static_cast<double>(p.x), static_cast<double>(p.y)});
        dropStraightCorners(c);
        dropSpikes(c);
        if (c.size() >= 3) welded.push_back(std::move(c));
    }
    return welded;
}

// Each contour rotated to start at its least corner, and the contours sorted:
// equal for two outlines that differ only in where Clipper starts them.
std::vector<std::vector<Vec2>> canonical(std::vector<std::vector<Vec2>> contours) {
    const auto less = [](Vec2 a, Vec2 b) { return a.x != b.x ? a.x < b.x : a.y < b.y; };
    for (auto& c : contours) std::rotate(c.begin(), std::min_element(c.begin(), c.end(), less), c.end());
    std::sort(contours.begin(), contours.end(), [&](const auto& a, const auto& b) { return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(), less); });
    return contours;
}

// The roof solid over `cs` (its floor at z = 0), discretizing curved parts
// by `fa`/`fs`.
//
// Boost's Voronoi builder needs segments that meet only at their ends. A
// union's pieces sharing an edge come back as outlines a hairline apart,
// which on the grid overlap or cross: the diagram comes out wrong, or the
// builder never finishes, allocating without bound. So the outline is
// welded first, as for the straight skeleton.
//
// Even on valid input the builder occasionally gets a near-degenerate
// configuration wrong -- a cell left with no finite edges, where four sites
// are all but equidistant once the outline is rounded to the grid -- and the
// facets built from it do not close. Rounding to a coarser grid perturbs
// the outline differently, so the roof is retried there.
manifold::Manifold voronoiRoof(const manifold::CrossSection& cs, double fa, double fs) {
    manifold::Manifold body;
    for (int bits = 29; bits >= 25; --bits) {
        double toGrid = 1;
        std::vector<std::vector<Vec2>> contours = gridContours(cs, toGrid, bits);
        std::vector<std::vector<Vec2>> welded = weldContours(contours);
        // An outline welding leaves as it was keeps its own order, so its
        // roof is the same, vertex for vertex, as without welding.
        if (canonical(welded) != canonical(contours)) contours = std::move(welded);
        if (contours.empty()) return manifold::Manifold();
        // $fa/$fs below 0.01 would ask for unbounded detail
        // (computeRoofParams has already clamped them, warning).
        body = RoofBuilder(std::move(contours), 1 / toGrid, std::max(fa, 0.01), std::max(fs, 0.01)).build();
        if (body.Status() == manifold::Manifold::Error::NoError && !body.IsEmpty()) break;
    }
    return body;
}

manifold::Manifold straightRoof(const manifold::CrossSection& cs) {
    double toGrid = 1;
    std::vector<std::vector<Vec2>> contours = weldContours(gridContours(cs, toGrid));
    if (contours.empty()) return manifold::Manifold();
    return StraightSkeletonRoof(contours, 1 / toGrid).build();
}

} // namespace

// roof(method="voronoi"|"straight") -- unknown values warn and fall back to
// "voronoi" (spec section E1).

// Computed before the children are evaluated, as OpenSCAD 2026.02.01 does:
// its $fs/$fa clamping and unknown-method warnings come ahead of anything a
// child echoes.
CSGParams computeRoofParams(Evaluator& ev, const CallArgs& args, EvalContext& effCtx, const oscad::Position* where) {
    const Discretizer disc = Discretizer::fromCtx(effCtx, [&](const std::string& m) { ev.warn(m, where); });

    // Anything but the two names warns, a non-string shown the way str()
    // would show it; undef means the default.
    const Value methodArg = getArg(args, std::nullopt, "method", Value{});
    std::string method = "voronoi";
    if (const std::string* name = std::get_if<std::string>(&methodArg)) method = *name;
    else if (!std::holds_alternative<std::monostate>(methodArg)) method = fmtValue(methodArg);
    if (method != "voronoi" && method != "straight") {
        ev.warn("Unknown roof method '" + method + "'. Using 'voronoi'.", where);
        method = "voronoi";
    }

    CSGParams params;
    params["method"] = Value{method};
    params["fa"] = Value{disc.fa};
    params["fs"] = Value{disc.fs};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

CSGParams resolveRoof(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params = computeRoofParams(ev, args, effCtx, &node.position());
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(effCtx);
    ev.evalChildren(node.children, blockCtx);
    return params;
}

std::vector<ColoredBody> generateRoof(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>& children,
                                       const oscad::ASTNode& node) {
    const std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs) return {};
    if (cs->ToPolygons().empty()) return {};

    try {
        manifold::Manifold body = std::get<std::string>(params.at("method")) == "straight"
                                      ? straightRoof(*cs)
                                      : voronoiRoof(*cs, std::get<double>(params.at("fa")), std::get<double>(params.at("fs")));
        if (body.IsEmpty()) return {};
        return {ev.tagGenerated(std::move(body), node, params.at("color"))};
    } catch (const std::exception& e) {
        ev.error(std::string("roof: ") + e.what(), node);
        return {};
    }
}

} // namespace oscadeval
