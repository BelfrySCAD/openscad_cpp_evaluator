#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <manifold/polygon.h>

#include <boost/polygon/voronoi.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

// roof() -- built on Boost.Polygon's segment Voronoi diagram (Fortune's
// sweep line, with a documented multi-level numerical robustness system),
// the same construction real OpenSCAD's own default `method="voronoi"`
// uses (see openscad/src/geometry/roof_vd.cc). Deliberately NOT CGAL's
// straight skeleton (real OpenSCAD's non-default `method="straight"`): a
// general (multi-contour/hole) straight skeleton is a genuinely hard
// algorithm to implement correctly by hand -- the original Felkel &
// Obdrzalek paper CGAL's own implementation is based on was itself flawed
// and needed years of fixes by CGAL's maintainers to become correct (see
// CLAUDE.md's roof() note) -- where the Voronoi construction gets its
// robustness from a mature, purpose-built library instead of a hand-rolled
// geometric sweep. This file is a close port of roof_vd.cc's
// voronoi_diagram_roof/vd_inner_faces/discretize_arc (names kept close to
// the original for cross-reference), replacing its CGAL::Vector2d/Eigen
// math with plain manifold::vec2 arithmetic and its Clipper2/PolySetBuilder
// plumbing with manifold::Triangulate() and a MeshGL builder, since this
// project already has CrossSection/Manifold equivalents for all of that.

namespace oscadeval {

namespace {

using VDInt = std::int32_t;

struct VDPoint {
    VDInt a, b;
    VDPoint(VDInt x, VDInt y) : a(x), b(y) {}
};
struct VDSegment {
    VDPoint p0, p1;
    VDSegment(VDInt x1, VDInt y1, VDInt x2, VDInt y2) : p0(x1, y1), p1(x2, y2) {}
};
bool operator==(const VDPoint& lhs, const VDPoint& rhs) { return lhs.a == rhs.a && lhs.b == rhs.b; }
bool segmentHasEndpoint(const VDSegment& s, const VDPoint& p) { return s.p0 == p || s.p1 == p; }

} // namespace
} // namespace oscadeval

// boost::polygon's point/segment trait specializations must live in
// namespace boost::polygon (the primary templates' own namespace) --
// mirrors roof_vd.cc's identical structure exactly.
namespace boost::polygon {
template <>
struct geometry_concept<oscadeval::VDPoint> {
    using type = point_concept;
};
template <>
struct point_traits<oscadeval::VDPoint> {
    using coordinate_type = oscadeval::VDInt;
    static coordinate_type get(const oscadeval::VDPoint& point, orientation_2d orient) {
        return orient == HORIZONTAL ? point.a : point.b;
    }
};
template <>
struct geometry_concept<oscadeval::VDSegment> {
    using type = segment_concept;
};
template <>
struct segment_traits<oscadeval::VDSegment> {
    using coordinate_type = oscadeval::VDInt;
    using point_type = oscadeval::VDPoint;
    static point_type get(const oscadeval::VDSegment& segment, direction_1d dir) {
        return dir.to_int() ? segment.p1 : segment.p0;
    }
};
} // namespace boost::polygon

namespace oscadeval {

namespace {

using VoronoiDiagram = boost::polygon::voronoi_diagram<double>;
using Point2 = manifold::vec2;

double dot2(const Point2& a, const Point2& b) { return a.x * b.x + a.y * b.y; }
double len2(const Point2& a) { return std::sqrt(dot2(a, a)); }
Point2 sub2(const Point2& a, const Point2& b) { return Point2(a.x - b.x, a.y - b.y); }
Point2 norm2(const Point2& a) {
    const double l = len2(a);
    if (!(l > 0)) throw std::runtime_error("roof: degenerate direction vector");
    return Point2(a.x / l, a.y / l);
}

double distanceToSegment(const Point2& vertex, const VDSegment& segment) {
    Point2 segNormal(-(segment.p1.b - segment.p0.b), segment.p1.a - segment.p0.a);
    segNormal = norm2(segNormal);
    const Point2 p0ToVertex(vertex.x - segment.p0.a, vertex.y - segment.p0.b);
    return std::fabs(dot2(segNormal, p0ToVertex));
}

double distanceToPoint(const Point2& vertex, const VDPoint& point) {
    return len2(Point2(vertex.x - point.a, vertex.y - point.b));
}

// Discretizes a parabolic Voronoi edge (equidistant from `point` and
// `segment`) between `v0` and `v1` into a polyline within `fa`/`fs`
// tolerance of the true parabola, via an adaptive-bisection walk in the
// parabola's own canonical (rotated so the axis of symmetry is vertical)
// coordinate frame. Mirrors discretize_arc exactly; the affine transform
// there is a plain rotation (point_direction is a unit vector, so its
// matrix is orthogonal with determinant 1), so this uses scalar rotation
// math instead of a general 2x2 solve/Eigen.
std::vector<Point2> discretizeArc(const VDPoint& point, const VDSegment& segment, const Point2& v0, const Point2& v1,
                                   double fa, double fs) {
    const double maxAngleDeviation = M_PI / 180.0 * fa / 2.0;
    const double maxSegmentSqrLength = fs * fs;

    const Point2 p(point.a, point.b);
    const Point2 p0(segment.p0.a, segment.p0.b);
    const Point2 p1(segment.p1.a, segment.p1.b);
    const Point2 p0ToP1Norm = norm2(sub2(p1, p0));

    const Point2 projectedPoint(p0.x + p0ToP1Norm.x * dot2(p0ToP1Norm, sub2(p, p0)), p0.y + p0ToP1Norm.y * dot2(p0ToP1Norm, sub2(p, p0)));
    const double pointDistance = len2(sub2(p, projectedPoint));
    if (!(pointDistance > 0)) throw std::runtime_error("roof: error in parabolic arc discretization");

    const Point2 dir = norm2(sub2(p, projectedPoint)); // (dx, dy) below
    const double dx = dir.x, dy = dir.y;

    // transformX(v) == (A * (v - p))[0], where A is the rotation taking
    // `dir` to the +y axis -- see the class comment above.
    const auto transformX = [&](const Point2& v) { return dy * (v.x - p.x) - dx * (v.y - p.y); };
    const auto untransform = [&](double x, double y) { return Point2(p.x + dy * x + dx * y, p.y - dx * x + dy * y); };

    const double transformedV0X = transformX(v0);
    const double transformedV1X = transformX(v1);
    if (!(transformedV0X < transformedV1X)) throw std::runtime_error("roof: error in parabolic arc discretization");

    const auto y = [pointDistance](double x) { return (x * x - pointDistance * pointDistance) / (2 * pointDistance); };
    const auto yPrime = [pointDistance](double x) { return x / pointDistance; };
    const auto segmentAngle = [&](double x1, double x2) {
        const double dxs = x2 - x1, dys = y(x2) - y(x1);
        const double tx = 1, ty = (std::fabs(x1) < std::fabs(x2)) ? yPrime(x1) : yPrime(x2);
        return std::fabs(std::atan2(dxs * ty - dys * tx, dxs * tx + dys * ty));
    };
    const auto segmentSqrLength = [&](double x1, double x2) {
        const double dxs = x2 - x1, dys = y(x2) - y(x1);
        return dxs * dxs + dys * dys;
    };

    std::vector<double> xs = {transformedV0X, transformedV1X};
    for (;;) {
        const double x1 = xs[xs.size() - 2];
        const double x2 = xs[xs.size() - 1];
        if (segmentAngle(x1, x2) > maxAngleDeviation || (maxSegmentSqrLength > 0 && segmentSqrLength(x1, x2) > maxSegmentSqrLength)) {
            xs.back() = 0.5 * x1 + 0.5 * x2;
        } else if (x2 == transformedV1X) {
            break;
        } else {
            xs.push_back(transformedV1X);
        }
    }

    std::vector<Point2> ret;
    ret.reserve(xs.size());
    for (double x : xs) {
        if (x == transformedV0X) {
            ret.push_back(v0);
        } else if (x == transformedV1X) {
            ret.push_back(v1);
        } else {
            ret.push_back(untransform(x, y(x)));
        }
    }
    return ret;
}

struct Point2Less {
    bool operator()(const Point2& a, const Point2& b) const { return (a.x < b.x) || (a.x == b.x && a.y < b.y); }
};

struct InnerFaces {
    std::vector<std::vector<Point2>> faces;
    std::map<Point2, double, Point2Less> heights;
};

// Walks every Voronoi cell (one per input segment -- boost::polygon
// implicitly also creates a "point cell" per segment endpoint) and
// extracts the roof surface as a set of small polygon faces plus a height
// (offset distance from the boundary) per vertex. Mirrors vd_inner_faces
// exactly.
InnerFaces vdInnerFaces(const VoronoiDiagram& vd, const std::vector<VDSegment>& segments, double fa, double fs) {
    InnerFaces ret;

    const auto cellContainsBoundaryPoint = [&](const VoronoiDiagram::cell_type* cell, const VDPoint& point) {
        const VDSegment& segment = segments[cell->source_index()];
        return (cell->contains_segment() && segmentHasEndpoint(segment, point)) ||
               (cell->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT && segment.p0 == point) ||
               (cell->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT && segment.p1 == point);
    };

    for (const auto& cell : vd.cells()) {
        const std::size_t cellIndex = cell.source_index();
        if (cell.is_degenerate()) throw std::runtime_error("roof: degenerate Voronoi cell");
        const VDSegment& segment = segments[cellIndex];

        if (cell.contains_segment()) {
            const VoronoiDiagram::edge_type* edge = cell.incident_edge();
            for (;;) {
                if (cellContainsBoundaryPoint(edge->twin()->cell(), segment.p1) &&
                    !cellContainsBoundaryPoint(edge->next()->twin()->cell(), segment.p1)) {
                    break;
                }
                edge = edge->next();
                if (edge == cell.incident_edge()) throw std::runtime_error("roof: Voronoi cell walk did not terminate");
            }
            ret.faces.emplace_back();
            {
                const Point2 p(segment.p1.a, segment.p1.b);
                ret.faces.back().push_back(p);
                ret.heights[p] = 0.0;
            }
            do {
                if (edge->is_linear()) {
                    const Point2 p(edge->vertex1()->x(), edge->vertex1()->y());
                    ret.faces.back().push_back(p);
                    ret.heights[p] = distanceToSegment(p, segment);
                } else {
                    const VoronoiDiagram::cell_type* twinCell = edge->twin()->cell();
                    if (!twinCell->contains_point()) throw std::runtime_error("roof: expected a point cell across a curved edge");
                    const VDSegment& twinSegment = segments[twinCell->source_index()];
                    const VDPoint twinPoint = twinCell->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT
                                                   ? twinSegment.p0
                                                   : twinSegment.p1;
                    const Point2 v0(edge->vertex0()->x(), edge->vertex0()->y());
                    const Point2 v1(edge->vertex1()->x(), edge->vertex1()->y());
                    std::vector<Point2> discr = discretizeArc(twinPoint, segment, v1, v0, fa, fs);
                    std::reverse(discr.begin(), discr.end());
                    for (std::size_t k = 1; k < discr.size(); ++k) {
                        ret.faces.back().push_back(discr[k]);
                        ret.heights[discr[k]] = distanceToSegment(discr[k], segment);
                    }
                }
                edge = edge->next();
            } while (!cellContainsBoundaryPoint(edge->twin()->cell(), segment.p0));
            {
                const Point2 p(segment.p0.a, segment.p0.b);
                ret.faces.back().push_back(p);
                ret.heights[p] = 0.0;
            }
        } else {
            const VoronoiDiagram::edge_type* edge = cell.incident_edge();
            const VDPoint point = cell.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT ? segment.p0 : segment.p1;
            while (!(edge->is_secondary() && edge->prev()->is_secondary())) {
                edge = edge->next();
                if (edge == cell.incident_edge()) throw std::runtime_error("roof: Voronoi point-cell walk did not terminate");
            }

            const auto addTriangle = [&](const Point2& v0, const Point2& v1) {
                ret.faces.emplace_back();
                const Point2 p(point.a, point.b);
                ret.faces.back().push_back(p);
                ret.heights[p] = 0.0;
                ret.faces.back().push_back(v0);
                ret.heights[v0] = distanceToPoint(v0, point);
                ret.faces.back().push_back(v1);
                ret.heights[v1] = distanceToPoint(v1, point);
            };

            if (edge->next()->next() != edge &&
                segments[edge->twin()->cell()->source_index()].p0 == segments[edge->prev()->twin()->cell()->source_index()].p1) {
                for (;;) {
                    edge = edge->next();
                    if (edge->is_secondary()) break;
                    const Point2 v0(edge->vertex0()->x(), edge->vertex0()->y());
                    const Point2 v1(edge->vertex1()->x(), edge->vertex1()->y());
                    if (edge->is_curved()) {
                        const VDSegment& twinSegment = segments[edge->twin()->cell()->source_index()];
                        std::vector<Point2> discr = discretizeArc(point, twinSegment, v0, v1, fa, fs);
                        for (std::size_t k = 1; k < discr.size(); ++k) addTriangle(discr[k - 1], discr[k]);
                    } else {
                        addTriangle(v0, v1);
                    }
                }
            }
        }
    }
    return ret;
}

// Chooses a power-of-two scale mapping `poly`'s bounding box into
// comfortably-signed-32-bit integer coordinates (Boost.Polygon's Voronoi
// builder takes integral input), leaving headroom below 2^31 for its own
// internal arithmetic margin.
double chooseScale(const manifold::Rect& bounds) {
    const double maxExtent =
        std::max({std::fabs(bounds.min.x), std::fabs(bounds.max.x), std::fabs(bounds.min.y), std::fabs(bounds.max.y), 1e-9});
    const int scaleBits = std::clamp(static_cast<int>(std::floor(std::log2(static_cast<double>(1 << 30) / maxExtent))), 0, 30);
    return std::ldexp(1.0, scaleBits);
}

manifold::Manifold voronoiRoof(const manifold::CrossSection& cs, double fa, double fs) {
    const manifold::Polygons polys = cs.ToPolygons();
    if (polys.empty()) throw std::runtime_error("roof: empty cross section");

    const double scale = chooseScale(cs.Bounds());

    // Every path is re-expressed in the same scaled-then-divided-back-by-
    // `scale` double coordinates for BOTH the Voronoi input and the floor
    // triangulation below, so their shared boundary vertices land on
    // bit-identical doubles (mirrors the reference's own "poly has to go
    // through clipper just as it does for the roof, because this may
    // change coordinates" comment) -- without this, the floor and roof
    // surfaces could disagree by a rounding ULP along their shared edge
    // and leave the final mesh non-manifold.
    std::vector<VDSegment> segments;
    manifold::Polygons scaledBackPaths;
    for (const auto& path : polys) {
        std::vector<std::array<VDInt, 2>> ipts;
        ipts.reserve(path.size());
        manifold::SimplePolygon scaledBackPath;
        scaledBackPath.reserve(path.size());
        for (const auto& p : path) {
            const VDInt ix = static_cast<VDInt>(std::llround(p.x * scale));
            const VDInt iy = static_cast<VDInt>(std::llround(p.y * scale));
            ipts.push_back({ix, iy});
            scaledBackPath.push_back(manifold::vec2(ix / scale, iy / scale));
        }
        const std::size_t n = ipts.size();
        for (std::size_t i = 0; i < n; ++i) {
            const auto& prev = ipts[(i + n - 1) % n];
            const auto& cur = ipts[i];
            segments.emplace_back(prev[0], prev[1], cur[0], cur[1]);
        }
        scaledBackPaths.push_back(std::move(scaledBackPath));
    }

    VoronoiDiagram vd;
    boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);
    const InnerFaces inner = vdInnerFaces(vd, segments, fa, scale * fs);

    // Exact-match weld: the roof/floor share boundary vertices at height 0
    // by construction (same scaled-back doubles), everything else is
    // unique to whichever face produced it.
    std::map<std::array<double, 3>, uint32_t> vertMap;
    std::vector<std::array<double, 3>> verts;
    const auto vertexIndex = [&](double x, double y, double z) {
        const std::array<double, 3> key{x, y, z};
        auto it = vertMap.find(key);
        if (it != vertMap.end()) return it->second;
        const uint32_t idx = static_cast<uint32_t>(verts.size());
        verts.push_back(key);
        vertMap.emplace(key, idx);
        return idx;
    };

    std::vector<std::array<uint32_t, 3>> tris;

    // Roof faces.
    for (const std::vector<Point2>& face : inner.faces) {
        if (face.size() < 3) throw std::runtime_error("roof: degenerate Voronoi face");
        manifold::Polygons single = {manifold::SimplePolygon(face.begin(), face.end())};
        const std::vector<manifold::ivec3> faceTris = manifold::Triangulate(single);
        for (const manifold::ivec3& t : faceTris) {
            const auto vertAt = [&](int i) {
                const Point2& p = face[static_cast<std::size_t>(i)];
                auto hIt = inner.heights.find(p);
                const double h = hIt != inner.heights.end() ? hIt->second / scale : 0.0;
                return vertexIndex(p.x / scale, p.y / scale, h);
            };
            tris.push_back({vertAt(t.x), vertAt(t.y), vertAt(t.z)});
        }
    }

    // Floor, triangulated from the exact same rescaled boundary
    // coordinates as the roof faces above, wound downward (facing -z).
    {
        const std::vector<manifold::ivec3> floorTris = manifold::Triangulate(scaledBackPaths);
        std::vector<Point2> flat;
        for (const auto& path : scaledBackPaths) flat.insert(flat.end(), path.begin(), path.end());
        for (const manifold::ivec3& t : floorTris) {
            const uint32_t a = vertexIndex(flat[static_cast<std::size_t>(t.x)].x, flat[static_cast<std::size_t>(t.x)].y, 0.0);
            const uint32_t b = vertexIndex(flat[static_cast<std::size_t>(t.y)].x, flat[static_cast<std::size_t>(t.y)].y, 0.0);
            const uint32_t c = vertexIndex(flat[static_cast<std::size_t>(t.z)].x, flat[static_cast<std::size_t>(t.z)].y, 0.0);
            tris.push_back({c, b, a}); // reversed: floor faces -z
        }
    }

    // MeshGL64, not MeshGL: MeshGL is MeshGLP<float>, so building through
    // it truncates these coordinates to ~7 significant digits on the way
    // into Manifold. That does not merely lose precision -- it snaps
    // nearly-distinct coordinates onto exactly-equal ones, manufacturing
    // the degenerate coincidences that make a later boolean leave a
    // zero-thickness membrane behind. See generatePolyhedron.
    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    for (const auto& v : verts) {
        mesh.vertProperties.push_back(v[0]);
        mesh.vertProperties.push_back(v[1]);
        mesh.vertProperties.push_back(v[2]);
    }
    for (const auto& t : tris) {
        mesh.triVerts.push_back(t[0]);
        mesh.triVerts.push_back(t[1]);
        mesh.triVerts.push_back(t[2]);
    }
    return manifold::Manifold(mesh);
}

} // namespace

// roof(method="voronoi") -- `method` is still accepted/validated (unknown
// values warn and fall back to "voronoi", matching the reference) even
// though both values currently dispatch to the same Voronoi-based
// construction here -- this port has no CGAL-based "straight" method (see
// the file header comment and CLAUDE.md). Mirrors _resolve_roof.

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
    ev.evalChildren(node.children, effCtx);
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
