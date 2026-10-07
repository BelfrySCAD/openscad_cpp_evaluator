#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <manifold/polygon.h>

#include <boost/polygon/point_data.hpp>
#include <boost/polygon/segment_data.hpp>
#include <boost/polygon/voronoi.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
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

    manifold::Manifold build() {
        mesh_.numProp = 3;
        for (const auto& t : triangles_) {
            for (int i : t) mesh_.triVerts.push_back(static_cast<uint64_t>(i));
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

// The roof solid over `cs` (its floor at z = 0), discretizing curved parts
// by `fa`/`fs`.
manifold::Manifold voronoiRoof(const manifold::CrossSection& cs, double fa, double fs) {
    const manifold::Polygons polys = cs.ToPolygons();
    double extent = 0;
    for (const auto& poly : polys) {
        for (Vec2 p : poly) extent = std::max({extent, std::abs(p.x), std::abs(p.y)});
    }
    if (!(extent > 0) || !std::isfinite(extent)) return manifold::Manifold();
    const double toGrid = std::ldexp(1.0, 29 - std::ilogb(extent));

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
    if (contours.empty()) return manifold::Manifold();

    // $fa/$fs below 0.01 would ask for unbounded detail.
    return RoofBuilder(std::move(contours), 1 / toGrid, std::max(fa, 0.01), std::max(fs, 0.01)).build();
}

} // namespace

// roof(method="voronoi") -- `method` is accepted and validated (unknown
// values warn and fall back to "voronoi") though both values build the same
// roof here (spec section E1).

// Split like computeLinearExtrudeParams (extrude.cpp) -- but UNLIKE that
// group, this one genuinely can't move before evalChildren: the "Unknown
// roof method" warning below is an observable side effect (an echo/warn
// message), and native resolveRoof always evaluates children FIRST, so
// this warning fires AFTER any echo()/warn() a child produces. Op::
// PushBuiltinWrap's own runtime handler (bytecode_vm.cpp) therefore calls
// this at POP time (after children finish -- VmFrame::builtinWrapStack
// retains `args` for exactly this) rather than at PUSH time the way
// computeLinearExtrudeParams/computeTransformParams/computeColorParams are
// called -- see Op::PushBuiltinWrap's own Roof-kind doc comment
// (bytecode.hpp) for the full contract. Takes the already-resolved
// `CallArgs`/`EvalContext` directly (not the raw node+ctx) since by POP
// time the argument expressions have already run once and must not
// re-run (double rands()/side effects).
CSGParams computeRoofParams(Evaluator& ev, const CallArgs& args, EvalContext& effCtx) {
    Value methodArg = getArg(args, std::nullopt, "method", Value{std::string("voronoi")});
    std::string method = std::holds_alternative<std::string>(methodArg) ? std::get<std::string>(methodArg) : "voronoi";
    if (method != "voronoi" && method != "straight") {
        // No location suffix here -- mirrors the reference's own bare
        // echo_fn call for this particular warning (unlike most others).
        ev.warn("Unknown roof method '" + method + "'. Using 'voronoi'.", nullptr);
        method = "voronoi";
    }

    const auto dynOr = [&](const char* name, double fallback) {
        const Value* v = effCtx.dyn->find(name);
        if (!v) return fallback;
        const double* d = std::get_if<double>(v);
        return d ? *d : fallback;
    };

    CSGParams params;
    params["method"] = Value{method};
    params["fa"] = Value{dynOr("$fa", 12.0)};
    params["fs"] = Value{dynOr("$fs", 2.0)};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

CSGParams resolveRoof(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(effCtx);
    ev.evalChildren(node.children, blockCtx);
    return computeRoofParams(ev, args, effCtx);
}

std::vector<ColoredBody> generateRoof(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>& children,
                                       const oscad::ASTNode& node) {
    const std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs) return {};
    if (cs->ToPolygons().empty()) return {};

    try {
        manifold::Manifold body = voronoiRoof(*cs, std::get<double>(params.at("fa")), std::get<double>(params.at("fs")));
        if (body.IsEmpty()) return {};
        return {ev.tagGenerated(std::move(body), node, params.at("color"))};
    } catch (const std::exception& e) {
        ev.error(std::string("roof: ") + e.what(), node);
        return {};
    }
}

} // namespace oscadeval
