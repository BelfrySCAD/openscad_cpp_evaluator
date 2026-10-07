#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <manifold/polygon.h>
#include <cmath>
#include <numbers>

namespace oscadeval {

// linear_extrude(height=100, v, scale=1, center=false, twist=0, slices, segments) --
// mirrors _resolve_linear_extrude/_generate_linear_extrude. `scale` is
// either a single number (applied to both X/Y) or a [x,y] pair for the top
// face; the bottom face is always full-size.
//
// Split into computeLinearExtrudeParams (pure, no children-evaluation side
// effect) + resolveLinearExtrude (adds the evalChildren call) so Op::
// PushBuiltinWrap's own runtime handler (bytecode_vm.cpp) can call the pure
// half directly instead of going through the whole resolve function --
// exactly the same split computeTransformParams/computeColorParams already
// use, extended to this builtin. Params computation here has no observable
// side effect of its own (no warn()/echo()/rands() call), so moving it
// before evalChildren (unlike this function's own former single-body
// shape, which computed params AFTER children like every other builtin
// still not covered by PushBuiltinWrap) is behaviorally unobservable --
// see computeRoofParams's own doc comment (roof.cpp) for the one builtin
// in this group where that ISN'T true and a different split was needed.
namespace {

// Picks the diagonal that splits one side quad: true for A0-B1, false for
// B0-A1, where A->B is an outline edge and 0/1 are the lower/upper ring.
using DiagonalChoice = std::function<bool(int slice, size_t contour, const manifold::vec3& a0, const manifold::vec3& b0,
                                          const manifold::vec3& a1, const manifold::vec3& b1)>;

// A solid swept from a 2D profile: `rings` copies of every profile vertex,
// placed by `place(ring, vertex)`, consecutive rings joined by side quads
// (two triangles each, split as `diagonal` says). An open sweep is capped at
// both ends with the triangulated profile; a closed one joins its last ring
// back to its first. `inverted` is for placements that mirror the profile
// (the triangles are then wound the other way round).
manifold::Manifold sweepProfile(const manifold::Polygons& profile, int rings, bool closed, bool inverted,
                                const std::function<manifold::vec3(int, const manifold::vec2&)>& place,
                                const DiagonalChoice& diagonal) {
    size_t n = 0;
    for (const manifold::SimplePolygon& contour : profile) n += contour.size();

    manifold::MeshGL64 mesh;
    mesh.numProp = 3;
    std::vector<manifold::vec3> points;
    points.reserve(n * rings);
    for (int r = 0; r < rings; ++r) {
        for (const manifold::SimplePolygon& contour : profile) {
            for (const manifold::vec2& p : contour) points.push_back(place(r, p));
        }
    }
    for (const manifold::vec3& p : points) mesh.vertProperties.insert(mesh.vertProperties.end(), {p.x, p.y, p.z});

    const auto addTri = [&](size_t a, size_t b, size_t c) {
        if (inverted) std::swap(b, c);
        mesh.triVerts.insert(mesh.triVerts.end(), {a, b, c});
    };

    if (!closed) {
        const size_t top = n * (rings - 1);
        for (const manifold::ivec3& t : manifold::Triangulate(profile)) {
            addTri(t.x, t.z, t.y);
            addTri(top + t.x, top + t.y, top + t.z);
        }
    }

    const int slices = closed ? rings : rings - 1;
    size_t first = 0;
    for (size_t c = 0; c < profile.size(); ++c) {
        const size_t size = profile[c].size();
        for (int s = 0; s < slices; ++s) {
            const size_t lower = s * n + first;
            const size_t upper = ((s + 1) % rings) * n + first;
            for (size_t i = 0; i < size; ++i) {
                const size_t j = (i + 1) % size;
                const size_t a0 = lower + i, b0 = lower + j, a1 = upper + i, b1 = upper + j;
                if (diagonal(s, c, points[a0], points[b0], points[a1], points[b1])) {
                    addTri(a0, b0, b1);
                    addTri(a0, b1, a1);
                } else {
                    addTri(a0, b0, a1);
                    addTri(b0, b1, a1);
                }
            }
        }
        first += size;
    }
    return manifold::Manifold(mesh);
}

double signedArea(const manifold::SimplePolygon& contour) {
    double twice = 0.0;
    for (size_t i = 0; i < contour.size(); ++i) {
        const manifold::vec2& a = contour[i];
        const manifold::vec2& b = contour[(i + 1) % contour.size()];
        twice += a.x * b.y - b.x * a.y;
    }
    return twice / 2.0;
}

// A twisted and/or non-uniformly scaled extrusion of `polys` (outlines
// already split as needed), `slices` slices over `height`. Ring j sits at
// t = j/slices of the height, its outline turned clockwise by twist*t and
// then scaled by 1 + (scale - 1)*t. Each side quad is split along its
// shorter diagonal (a near-tie goes by the twist's direction), except that
// a top shrunk to nothing flips the last slice's choice.
manifold::Manifold extrudeTwisted(const manifold::Polygons& polys, double height, int slices, double twist,
                                  double scaleX, double scaleY) {
    const auto place = [&](int ring, const manifold::vec2& p) {
        const double t = static_cast<double>(ring) / slices;
        const double angle = -twist * t * std::numbers::pi / 180.0;
        const double c = std::cos(angle), s = std::sin(angle);
        return manifold::vec3((c * p.x - s * p.y) * (1 + (scaleX - 1) * t),
                              (s * p.x + c * p.y) * (1 + (scaleY - 1) * t), height * t);
    };
    std::vector<bool> counterClockwise;
    for (const manifold::SimplePolygon& contour : polys) counterClockwise.push_back(signedArea(contour) > 0);
    const bool pinchedTop = scaleX == 0 || scaleY == 0;

    const auto diagonal = [&](int slice, size_t contour, const manifold::vec3& a0, const manifold::vec3& b0,
                              const manifold::vec3& a1, const manifold::vec3& b1) {
        const double l1 = manifold::la::length(b1 - a0), l2 = manifold::la::length(a1 - b0);
        bool useA0B1 = 2.0 * std::fabs(l1 - l2) * 1e5 <= l1 + l2 ? counterClockwise[contour] == (twist > 0) : l1 < l2;
        if (pinchedTop && slice == slices - 1) useA0B1 = !useA0B1;
        return useA0B1;
    };
    return sweepProfile(polys, slices + 1, false, false, place, diagonal);
}

double sinDegrees(double degrees) {
    const double turn = std::fmod(degrees, 360.0);
    if (turn == 0.0 || std::fabs(turn) == 180.0) return 0.0;
    if (turn == 90.0 || turn == -270.0) return 1.0;
    if (turn == 270.0 || turn == -90.0) return -1.0;
    return std::sin(turn * std::numbers::pi / 180.0);
}

double cosDegrees(double degrees) { return sinDegrees(degrees + 90.0); }

} // namespace

BuiltinWrapParams computeLinearExtrudeParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);

    // Argument reading (positional: height, v, scale, center, twist, slices,
    // segments): the extrusion vector, center, twist, slices, segments and
    // scale, with their warnings.
    const oscad::Position* where = &node.position();
    const auto isUndef = [](const Value& v) { return std::holds_alternative<std::monostate>(v); };
    const auto finiteNumber = [](const Value& v) -> std::optional<double> {
        const double* d = std::get_if<double>(&v);
        return d && std::isfinite(*d) ? std::optional<double>(*d) : std::nullopt;
    };
    // A list of exactly `size` finite numbers.
    const auto finiteNumbers = [&](const Value& v, size_t size) -> std::optional<std::vector<double>> {
        const ListPtr* list = std::get_if<ListPtr>(&v);
        if (!list || !*list || (*list)->items.size() != size) return std::nullopt;
        std::vector<double> out;
        for (size_t i = 0; i < size; ++i) {
            const std::optional<double> d = finiteNumber((*list)->items[i]);
            if (!d) return std::nullopt;
            out.push_back(*d);
        }
        return out;
    };

    // The extrusion vector: `v` alone is the whole vector; a height, when
    // given, keeps only v's direction.
    double hv[3] = {0.0, 0.0, 1.0};
    double length = 100.0;
    const Value vArg = getArg(args, 1, "v");
    if (!isUndef(vArg)) {
        if (const auto v = finiteNumbers(vArg, 3)) {
            std::copy(v->begin(), v->end(), hv);
        } else {
            ev.emitWarning("ERROR: v when specified should be a 3d vector.");
        }
        length = 1.0;
    }
    const Value heightArg = getArgOrAlias(ev, where, args, 0, "height", "h", Value{});
    if (!isUndef(heightArg)) {
        if (const auto h = finiteNumber(heightArg)) {
            length = *h;
        } else {
            ev.emitWarning("ERROR: height when specified should be a number.");
            length = 100.0;
        }
        const double norm = std::sqrt(hv[0] * hv[0] + hv[1] * hv[1] + hv[2] * hv[2]);
        if (norm > 0) {
            for (double& c : hv) c /= norm;
        }
    }
    for (double& c : hv) c *= length;
    if (!(hv[2] > 0)) hv[0] = hv[1] = hv[2] = 0.0;
    const double height = hv[2];

    const Value centerArg = getArg(args, 3, "center");
    const bool center = std::holds_alternative<bool>(centerArg) && std::get<bool>(centerArg);
    const double twist = finiteNumber(getArg(args, 4, "twist")).value_or(0.0);

    // slices/segments: whole numbers, at least `least`; absent (with a
    // warning, for a bad value) leaves the count to the discretizer.
    const auto count = [&](int pos, const std::string& name, double least) -> std::optional<double> {
        const Value v = getArg(args, pos, name);
        if (isUndef(v)) return std::nullopt;
        const std::string prefix = "linear_extrude(..., " + name + "=" + fmtValue(v) + ") ";
        const double* d = std::get_if<double>(&v);
        if (!d) {
            ev.warn(prefix + "Invalid type: expected number, found " + oscTypeName(v), where);
            return std::nullopt;
        }
        if (!std::isfinite(*d)) {
            ev.warn(prefix + "argument cannot be infinite or nan", where);
            return std::nullopt;
        }
        return std::max(std::trunc(*d), least);
    };
    const std::optional<double> slices = count(5, "slices", 1.0);
    const std::optional<double> segments = count(6, "segments", 0.0);

    double scaleX = 1.0, scaleY = 1.0;
    const Value scaleArg = getArg(args, 2, "scale");
    if (!isUndef(scaleArg)) {
        if (const auto s = finiteNumber(scaleArg)) {
            scaleX = scaleY = *s;
        } else if (const auto xy = finiteNumbers(scaleArg, 2)) {
            scaleX = (*xy)[0];
            scaleY = (*xy)[1];
        } else {
            ev.warn("linear_extrude(..., scale=" + fmtValue(scaleArg) + ") could not be converted", where);
        }
    }

    CSGParams params;
    params["height"] = Value{height};
    // How far the top slides sideways per unit of height (0 unless `v`).
    params["shear_x"] = Value{height > 0 ? hv[0] / height : 0.0};
    params["shear_y"] = Value{height > 0 ? hv[1] / height : 0.0};
    params["center"] = Value{center};
    params["twist"] = Value{twist};
    params["slices"] = slices ? Value{*slices} : Value{};
    params["segments"] = segments ? Value{*segments} : Value{};
    Discretizer::fromCtx(effCtx, [&](const std::string& m) { ev.warn(m, &node.position()); }).store(params);
    params["scale_x"] = Value{scaleX};
    params["scale_y"] = Value{scaleY};
    params["color"] = colorToValue(effCtx.color);
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveLinearExtrude(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeLinearExtrudeParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateLinearExtrude(Evaluator& ev, const CSGParams& params,
                                                const std::vector<std::unique_ptr<CSGNode>>& children,
                                                const oscad::ASTNode& node) {
    const std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs || cs->IsEmpty()) return {};

    const double height = std::get<double>(params.at("height"));
    const double twist = std::get<double>(params.at("twist"));
    const double scaleX = std::get<double>(params.at("scale_x"));
    const double scaleY = std::get<double>(params.at("scale_y"));
    const double* givenSlices = std::get_if<double>(&params.at("slices"));
    const double* givenSegments = std::get_if<double>(&params.at("segments"));
    const Discretizer disc = Discretizer::fromParams(params);
    manifold::Polygons polys = cs->ToPolygons();

    // The slice count, and the subdivision of outline edges (through
    // disc.splitOutline) that a twisted or scaled extrusion needs.
    const bool nonLinear = twist != 0.0 || scaleX != scaleY;
    double rSqr = 0.0, deltaSqr = 0.0;
    for (const manifold::SimplePolygon& contour : polys) {
        for (const manifold::vec2& p : contour) {
            rSqr = std::max(rSqr, manifold::la::dot(p, p));
            const manifold::vec2 delta(p.x - scaleX * p.x, p.y - scaleY * p.y);
            deltaSqr = std::max(deltaSqr, manifold::la::dot(delta, delta));
        }
    }
    const double byTwist = std::max(std::ceil(twist / 120.0), 1.0);
    int slices = 1;
    if (givenSlices) {
        slices = static_cast<int>(*givenSlices);
    } else if (twist != 0.0 && scaleX == 1.0 && scaleY == 1.0) {
        slices = disc.helixSlices(rSqr, height, twist).value_or(byTwist);
    } else if (twist != 0.0 && scaleX != scaleY) {
        slices = std::max(disc.diagonalSlices(deltaSqr, height).value_or(1),
                          disc.helixSlices(rSqr, height, twist).value_or(byTwist));
    } else if (twist != 0.0) {
        slices = disc.conicalHelixSlices(rSqr, height, twist, scaleX).value_or(byTwist);
    } else if (scaleX != scaleY) {
        slices = disc.diagonalSlices(deltaSqr, height).value_or(1);
    }

    // Long outline edges are split so the twisted/scaled sides follow the
    // true surface; segments=0 turns that off.
    const bool split = givenSegments ? *givenSegments > 0 : nonLinear;
    if (split) {
        const unsigned segments = givenSegments ? static_cast<unsigned>(*givenSegments) : 0;
        for (manifold::SimplePolygon& contour : polys) {
            contour = disc.splitOutline(contour, twist, scaleX, scaleY, static_cast<unsigned>(slices), segments);
        }
    }

    // Planar side quads (no twist, uniform scale) come out the same however
    // they are split, so Manifold's own Extrude does them.
    // Manifold's nDivisions is the copies BETWEEN the ends: slices - 1.
    manifold::Manifold body =
        nonLinear && scaleX >= 0 && scaleY >= 0 && height > 0
            ? extrudeTwisted(polys, height, slices, twist, scaleX, scaleY)
            : manifold::Manifold::Extrude(polys, height, slices - 1, -twist, manifold::vec2(scaleX, scaleY));
    if (std::get<bool>(params.at("center"))) body = body.Translate(manifold::vec3(0, 0, -height / 2));
    // `v`: slide each point sideways in proportion to its height; centring
    // subtracts v/2, so the same shear is right either way.
    const double shearX = std::get<double>(params.at("shear_x"));
    const double shearY = std::get<double>(params.at("shear_y"));
    if (shearX != 0.0 || shearY != 0.0) {
        body = body.Transform(manifold::mat3x4({1, 0, 0}, {0, 1, 0}, {shearX, shearY, 1}, {0, 0, 0}));
    }
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// rotate_extrude(angle=360) -- mirrors _resolve_rotate_extrude/
// _generate_rotate_extrude. Segment count can't be derived until generate
// (it depends on the merged children's bounds, unknown at resolve time),
// so resolve only caches $fn/$fa/$fs for generate to call fnSegments()
// with once the real max-x bound exists.

// Split the same way as computeLinearExtrudeParams, above -- see its own
// doc comment.
BuiltinWrapParams computeRotateExtrudeParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);

    // angle/start reading (positional: angle, start).
    const Discretizer disc = Discretizer::fromCtx(effCtx, [&](const std::string& m) { ev.warn(m, &node.position()); });
    const auto finiteNumber = [](const Value& v) -> const double* {
        const double* d = std::get_if<double>(&v);
        return d && std::isfinite(*d) ? d : nullptr;
    };
    // With an angle the sweep starts on +X; without one it is a full turn
    // starting on -X.
    const Value angleArg = getArgOrAlias(ev, &node.position(), args, 0, "angle", "a", Value{});
    const double* givenAngle = finiteNumber(angleArg);
    double angle = 360.0;
    double start = 180.0;
    if (givenAngle) {
        angle = *givenAngle <= -360.0 || *givenAngle > 360.0 ? 360.0 : *givenAngle;
        start = 0.0;
    }
    const double* givenStart = finiteNumber(getArg(args, 1, "start"));
    if (givenStart) start = *givenStart;
    if (!givenAngle && !givenStart && std::fmod(std::trunc(disc.fn), 2.0) == 1.0) {
        ev.emitWarning("DEPRECATED: In future releases, rotational extrusion without \"angle\" will start at zero, "
                       "the +X axis.  Set start=180 to explicitly start on the -X axis.");
    }

    CSGParams params;
    params["angle"] = Value{angle};
    params["start"] = Value{start};
    disc.store(params);
    params["color"] = colorToValue(effCtx.color);
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveRotateExtrude(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeRotateExtrudeParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateRotateExtrude(Evaluator& ev, const CSGParams& params,
                                                const std::vector<std::unique_ptr<CSGNode>>& children,
                                                const oscad::ASTNode& node) {
    // The sweep itself: params "angle", "start" and the stored
    // discretization settings, over the children's 2D profile.
    const std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs || cs->IsEmpty()) return {};
    double angle = std::get<double>(params.at("angle"));
    double start = std::get<double>(params.at("start"));
    if (angle == 0.0) return {};

    const manifold::Rect bounds = cs->Bounds();
    const double xmin = std::min(bounds.min.x, 0.0), xmax = std::max(bounds.max.x, 0.0);
    if (xmin < 0 && xmax > 0) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "[%.2f : %.2f])", xmin, xmax);
        ev.emitWarning(std::string("ERROR: Children of rotate_extrude() may not lie across the Y axis "
                                   "(Range of X coords for all children ") + buf);
        return {};
    }

    const Discretizer disc = Discretizer::fromParams(params);
    const int sections = disc.circular(xmax - xmin, std::fabs(angle))
                             .value_or(std::max(1, static_cast<int>(std::floor(std::fabs(angle) / 360.0 * 3))));

    // Sweep counter-clockwise from the lower end of the angle range; a
    // profile left of the axis is the mirror image of one right of it,
    // half a turn round.
    if (angle < 0) {
        start += angle;
        angle = -angle;
    }
    manifold::Polygons profile = cs->ToPolygons();
    const bool leftOfAxis = xmin < 0;
    if (leftOfAxis) {
        start += 180.0;
        for (manifold::SimplePolygon& contour : profile) {
            for (manifold::vec2& p : contour) p.x = -p.x;
            std::reverse(contour.begin(), contour.end());
        }
    }

    const bool fullTurn = angle >= 360.0;
    const auto place = [&](int ring, const manifold::vec2& p) {
        const double theta = start + angle * ring / sections;
        return manifold::vec3(p.x * cosDegrees(theta), p.x * sinDegrees(theta), p.y);
    };
    const auto shorterDiagonal = [](int, size_t, const manifold::vec3& a0, const manifold::vec3& b0,
                                    const manifold::vec3& a1, const manifold::vec3& b1) {
        return manifold::la::length(b1 - a0) <= manifold::la::length(a1 - b0);
    };
    // The profile's (x, y) become (radius, z): a left-handed frame against
    // the sweep direction, so the sweep's triangles wind the other way.
    manifold::Manifold body = sweepProfile(profile, fullTurn ? sections : sections + 1, fullTurn, true, place,
                                           shorterDiagonal);
    if (body.IsEmpty()) return {};
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

// projection(cut=false) -- mirrors _resolve_projection/_generate_projection.
// `cut`: a Z=0 cross-section slice through the combined 3D children.
// Otherwise: an orthographic projection onto the XY plane, re-filled with
// FillRule::Positive to clean up self-intersections Manifold's own
// Project() can produce.

// Split the same way as computeLinearExtrudeParams, above -- see its own
// doc comment.
BuiltinWrapParams computeProjectionParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    // Positional, and only a real bool cuts: cut=1 and cut="yes" do not,
    // projection(true) does.
    const Value cutArg = getArg(args, 0, "cut", Value{false});
    params["cut"] = Value{std::holds_alternative<bool>(cutArg) && std::get<bool>(cutArg)};
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveProjection(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeProjectionParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateProjection(Evaluator&, const CSGParams& params,
                                             const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode&) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    std::vector<ColoredBody> bodies3d;
    for (const ColoredBody& c : bodies) {
        if (c.body) bodies3d.push_back(c);
    }
    if (bodies3d.empty()) return {};

    const ColoredBody combined = combineBodies(bodies3d);
    manifold::CrossSection cs;
    if (std::get<bool>(params.at("cut"))) {
        cs = manifold::CrossSection(combined.body->Slice(0.0), manifold::CrossSection::FillRule::EvenOdd);
    } else {
        const manifold::Polygons polys = combined.body->Project();
        cs = polys.empty() ? manifold::CrossSection() : manifold::CrossSection(polys, manifold::CrossSection::FillRule::Positive);
    }

    ColoredBody result;
    result.section = std::move(cs);
    result.color = combined.color;
    return {result};
}

// offset(r=)/offset(delta=, chamfer=false) -- mirrors _resolve_offset/
// _generate_offset. `r` (rounded corners, JoinType::Round) wins over
// `delta` (JoinType::Miter, or Square if chamfer=true); neither given is
// r=1.

// Split the same way as computeLinearExtrudeParams, above -- see its own
// doc comment.
BuiltinWrapParams computeOffsetParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);

    // r/delta/chamfer reading: the offset distance and the join ("round",
    // "miter" or "square").
    // r rounds; delta keeps corners sharp, or cuts them with chamfer=true;
    // neither is r=1.
    const Value r = getArg(args, 0, "r");
    const Value deltaArg = getArg(args, 1, "delta");
    double delta = 1.0;
    std::string join = "round";
    if (const double* d = std::get_if<double>(&r)) {
        delta = *d;
        if (std::holds_alternative<double>(deltaArg)) {
            ev.warn("Ignoring \"delta\" argument as \"r\" is defined too.", &node.position());
        }
    } else if (const double* d = std::get_if<double>(&deltaArg)) {
        delta = *d;
        const Value chamfer = getArg(args, 2, "chamfer");
        join = std::holds_alternative<bool>(chamfer) && std::get<bool>(chamfer) ? "square" : "miter";
    }

    CSGParams params;
    params["delta"] = Value{delta};
    params["join"] = Value{join};
    // Segments for |delta|; $fa/$fs/$fn clamping is silent here.
    params["segs"] = Value{static_cast<double>(fnSegmentsFromCtx(effCtx, std::fabs(delta)))};
    params["color"] = colorToValue(effCtx.color);
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveOffset(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeOffsetParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateOffset(Evaluator&, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>& children,
                                         const oscad::ASTNode&) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    const std::optional<manifold::CrossSection> cs = toCrossSection(bodies);
    if (!cs) return {};

    const double delta = std::get<double>(params.at("delta"));
    // An infinite (or NaN) distance has no shape to give.
    if (!std::isfinite(delta)) return {};
    const std::string& join = std::get<std::string>(params.at("join"));
    const int segs = static_cast<int>(std::get<double>(params.at("segs")));
    using JT = manifold::CrossSection::JoinType;
    const JT jt = join == "round" ? JT::Round : join == "square" ? JT::Square : JT::Miter;
    // "miter" joins stay sharp however acute the corner: no realistic
    // corner reaches this limit.
    constexpr double kMiterLimit = 1e6;
    ColoredBody result;
    result.section = cs->Offset(delta, jt, kMiterLimit, segs);
    result.color = valueToColor(params.at("color"));
    return {result};
}

} // namespace oscadeval
