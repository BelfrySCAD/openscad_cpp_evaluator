#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <manifold/polygon.h>

namespace oscadeval {

// simplify()'s default tolerance, as a fraction of the body's bounding-box
// diagonal. 0.1% measured out at roughly a quarter of the triangles gone for
// under 0.1% volume error on a representative model -- a good trade, and
// scale-independent in a way no absolute default can be.
static constexpr double kDefaultSimplifyFraction = 0.001;

// hull()/minkowski() -- like union/difference/intersection, these splice
// their children transparently in the CSG tree (resolve just evaluates
// them for the side effect); the actual hull/minkowski-sum only happens
// once every child's real geometry exists, in generate. Mirrors
// _resolve_hull/_resolve_minkowski (both bodies of the same one-line
// "evaluate children, return {}" shape as _resolve_transform).

CSGParams resolveHull(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    (void)args;
    ev.evalChildren(node.children, effCtx);
    return CSGParams{};
}

// hull() of the foreground children: an all-3D hull if any child has a
// body, else an all-2D hull over sections -- mirrors _generate_hull exactly
// (a 2D/3D mix only ever hulls the 3D bodies, dropping any 2D sibling,
// matching the reference's own `if bodies_3d: ... else: sections...`
// either/or dispatch). background/highlight/show_only bodies pass through
// untouched, same as union/difference/intersection.
std::vector<ColoredBody> generateHull(Evaluator&, const CSGParams&, const std::vector<std::unique_ptr<CSGNode>>& children,
                                       const oscad::ASTNode&) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    if (bodies.empty()) return {};
    const RoleSplit split = splitByRole(bodies);

    std::optional<ColoredBody> hullResult;
    if (!split.foreground.empty()) {
        std::vector<manifold::Manifold> bodies3d;
        for (const ColoredBody& c : split.foreground) {
            if (c.body) bodies3d.push_back(*c.body);
        }
        if (!bodies3d.empty()) {
            ColoredBody cb;
            cb.body = manifold::Manifold::Hull(bodies3d);
            cb.color = split.foreground.front().color;
            hullResult = std::move(cb);
        } else {
            std::vector<manifold::CrossSection> sections;
            for (const ColoredBody& c : split.foreground) {
                if (c.section) sections.push_back(*c.section);
            }
            if (!sections.empty()) {
                ColoredBody cb;
                cb.section = manifold::CrossSection::Hull(sections);
                cb.color = split.foreground.front().color;
                hullResult = std::move(cb);
            }
        }
    }

    std::vector<ColoredBody> result;
    if (hullResult) result.push_back(std::move(*hullResult));
    result.insert(result.end(), split.background.begin(), split.background.end());
    result.insert(result.end(), split.highlight.begin(), split.highlight.end());
    result.insert(result.end(), split.showOnly.begin(), split.showOnly.end());
    result.insert(result.end(), split.displayOnly.begin(), split.displayOnly.end());
    return result;
}

// fill() -- 2D only: union the children, then discard every hole, keeping
// just the outer boundaries. Ported from the reference's applyFill2D: union
// the children, keep only the *positive* outlines, then re-union those (two
// outers can overlap once the holes between them are gone, and the result
// has to come back as one sanitized region rather than a pile of loops).
//
// A 3D child gets the reference's own "not yet implemented for 3D" warning
// and contributes nothing.

CSGParams resolveFill(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    (void)args;
    ev.evalChildren(node.children, effCtx);
    return CSGParams{};
}

namespace {

// Shoelace. Manifold orients an outer contour counter-clockwise (positive
// area) and a hole clockwise, so the sign is the whole test.
double signedArea(const manifold::SimplePolygon& poly) {
    double a = 0.0;
    for (size_t i = 0, n = poly.size(); i < n; ++i) {
        const auto& p = poly[i];
        const auto& q = poly[(i + 1) % n];
        a += static_cast<double>(p.x) * q.y - static_cast<double>(q.x) * p.y;
    }
    return a / 2.0;
}

} // namespace

std::vector<ColoredBody> generateFill(Evaluator& ev, const CSGParams&,
                                       const std::vector<std::unique_ptr<CSGNode>>& children,
                                       const oscad::ASTNode& node) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    if (bodies.empty()) return {};
    const RoleSplit split = splitByRole(bodies);

    std::optional<ColoredBody> filled;
    if (!split.foreground.empty()) {
        for (const ColoredBody& c : split.foreground) {
            if (c.body) {
                ev.warn("fill() not yet implemented for 3D", &node.position());
                break;
            }
        }
        if (const std::optional<manifold::CrossSection> cs = toCrossSection(split.foreground)) {
            manifold::Polygons outers;
            for (const manifold::SimplePolygon& loop : cs->ToPolygons()) {
                if (signedArea(loop) > 0.0) outers.push_back(loop);
            }
            if (!outers.empty()) {
                ColoredBody cb;
                cb.section = manifold::CrossSection(outers, manifold::CrossSection::FillRule::Positive);
                cb.color = split.foreground.front().color;
                filled = std::move(cb);
            }
        }
    }

    std::vector<ColoredBody> result;
    if (filled) result.push_back(std::move(*filled));
    result.insert(result.end(), split.background.begin(), split.background.end());
    result.insert(result.end(), split.highlight.begin(), split.highlight.end());
    result.insert(result.end(), split.showOnly.begin(), split.showOnly.end());
    result.insert(result.end(), split.displayOnly.begin(), split.displayOnly.end());
    return result;
}

CSGParams resolveMinkowski(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    (void)args;
    ev.evalChildren(node.children, effCtx);
    return CSGParams{};
}


namespace {

// The convex pieces of a 2D shape, as point lists.
//
// A Minkowski sum only has a closed form for convex operands -- there it
// is the convex hull of every pairwise sum -- so a shape that is not
// convex has to be cut into pieces that are. A convex outline with no
// holes is already one piece, which is the common case (a circle being
// swept over something) and much cheaper than triangulating it.
std::vector<manifold::SimplePolygon> convexPieces(const manifold::CrossSection& section) {
    const manifold::Polygons polys = section.ToPolygons();
    std::vector<manifold::SimplePolygon> out;

    if (polys.size() == 1) {
        const manifold::SimplePolygon& ring = polys[0];
        bool convex = ring.size() >= 3;
        int sign = 0;
        for (size_t i = 0; convex && i < ring.size(); ++i) {
            const manifold::vec2& a = ring[i];
            const manifold::vec2& b = ring[(i + 1) % ring.size()];
            const manifold::vec2& c = ring[(i + 2) % ring.size()];
            const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if (std::abs(cross) < 1e-12) continue;      // collinear, no turn
            const int s = cross > 0 ? 1 : -1;
            if (sign == 0) sign = s;
            else if (s != sign) convex = false;
        }
        if (convex) {
            out.push_back(ring);
            return out;
        }
    }

    for (const manifold::ivec3& tri : manifold::Triangulate(polys)) {
        // Triangulate indexes the contours end to end.
        std::vector<manifold::vec2> flat;
        for (const manifold::SimplePolygon& ring : polys)
            flat.insert(flat.end(), ring.begin(), ring.end());
        if (static_cast<size_t>(tri.x) >= flat.size() || static_cast<size_t>(tri.y) >= flat.size() ||
            static_cast<size_t>(tri.z) >= flat.size()) {
            continue;
        }
        out.push_back({flat[tri.x], flat[tri.y], flat[tri.z]});
    }
    return out;
}

// The 2D Minkowski sum of two shapes.
//
// Manifold has no 2D Minkowski, so this is the boundary-sweep identity:
// for B convex and containing the origin,
//
//     A (+) B  =  A  union  (boundary of A (+) B)
//
// and the boundary is a chain of segments, each of which sums with a
// convex B to the hull of B at its two ends. So sweeping B along every
// edge of A and unioning A itself is the whole answer -- A is never cut
// up, however concave it is or however many holes it has. Only B is,
// and only because the per-segment hull needs it convex.
//
// The two conditions on B are met rather than assumed: it is decomposed
// into convex pieces (Minkowski distributes over union, so the pieces'
// sums are unioned), and each piece is shifted onto the origin with the
// shift undone afterwards, since A (+) B = ((A (+) (B - c)) + c).
//
// ponytail: one hull per edge of A per convex piece of B. A piece count
// of one is the case that turns up -- a circle swept over something --
// and then it is simply one hull per edge.
manifold::CrossSection minkowski2d(const manifold::CrossSection& a, const manifold::CrossSection& b) {
    const manifold::Polygons outline = a.ToPolygons();
    std::vector<manifold::CrossSection> parts;

    for (const manifold::SimplePolygon& piece : convexPieces(b)) {
        if (piece.size() < 3) continue;
        // Bring the piece onto the origin; the sum is shifted back after.
        manifold::vec2 shift = piece[0];
        manifold::SimplePolygon centred;
        centred.reserve(piece.size());
        for (const manifold::vec2& q : piece) centred.push_back({q.x - shift.x, q.y - shift.y});

        std::vector<manifold::CrossSection> swept;
        // A itself: only sound because `centred` contains the origin.
        swept.push_back(a);
        for (const manifold::SimplePolygon& ring : outline) {
            for (size_t i = 0; i < ring.size(); ++i) {
                const manifold::vec2& v0 = ring[i];
                const manifold::vec2& v1 = ring[(i + 1) % ring.size()];
                manifold::SimplePolygon ends;
                ends.reserve(centred.size() * 2);
                for (const manifold::vec2& q : centred) {
                    ends.push_back({v0.x + q.x, v0.y + q.y});
                    ends.push_back({v1.x + q.x, v1.y + q.y});
                }
                swept.push_back(manifold::CrossSection::Hull(ends));
            }
        }
        manifold::CrossSection sum =
            manifold::CrossSection::BatchBoolean(swept, manifold::OpType::Add);
        parts.push_back(sum.Translate(shift));
    }

    if (parts.empty()) return manifold::CrossSection();
    return manifold::CrossSection::BatchBoolean(parts, manifold::OpType::Add);
}

} // namespace

// minkowski() over 3D bodies, or over 2D sections when that is what it
// was given. 2D used to be dropped on the floor -- silently, so
// `linear_extrude() minkowski() { square(); circle(); }` produced nothing
// at all where the reference produces the rounded square you asked for.
std::vector<ColoredBody> generateMinkowski(Evaluator& ev, const CSGParams&, const std::vector<std::unique_ptr<CSGNode>>& children,
                                            const oscad::ASTNode& node) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    const RoleSplit split = splitByRole(bodies);

    std::vector<const ColoredBody*> bodies3d;
    std::vector<const ColoredBody*> sections2d;
    for (const ColoredBody& c : split.foreground) {
        if (c.body) bodies3d.push_back(&c);
        else if (c.section) sections2d.push_back(&c);
    }

    std::vector<ColoredBody> passthrough;
    passthrough.insert(passthrough.end(), split.background.begin(), split.background.end());
    passthrough.insert(passthrough.end(), split.highlight.begin(), split.highlight.end());
    passthrough.insert(passthrough.end(), split.showOnly.begin(), split.showOnly.end());
    passthrough.insert(passthrough.end(), split.displayOnly.begin(), split.displayOnly.end());

    // 2D only: sum the sections instead. A mix of 2D and 3D is the
    // reference's error case, and the 3D bodies win here as they do there.
    if (bodies3d.empty() && sections2d.size() >= 2) {
        manifold::CrossSection acc = *sections2d.front()->section;
        for (size_t i = 1; i < sections2d.size(); ++i)
            acc = minkowski2d(acc, *sections2d[i]->section);
        ColoredBody cb;
        cb.section = std::move(acc);
        cb.color = sections2d.front()->color;
        std::vector<ColoredBody> out = {std::move(cb)};
        out.insert(out.end(), passthrough.begin(), passthrough.end());
        return out;
    }
    if (bodies3d.empty() && sections2d.size() == 1) {
        std::vector<ColoredBody> out = {*sections2d.front()};
        out.insert(out.end(), passthrough.begin(), passthrough.end());
        return out;
    }
    if (bodies3d.empty()) return passthrough;
    if (bodies3d.size() == 1) {
        std::vector<ColoredBody> result = {*bodies3d.front()};
        result.insert(result.end(), passthrough.begin(), passthrough.end());
        return result;
    }

    manifold::Manifold result = *bodies3d.front()->body;
    for (size_t i = 1; i < bodies3d.size(); ++i) result = result.MinkowskiSum(*bodies3d[i]->body);
    if (result.Status() != manifold::Manifold::Error::NoError) {
        ev.warn("minkowski: result is not manifold", &node.position());
    }

    ColoredBody cb;
    cb.body = std::move(result);
    cb.color = bodies3d.front()->color;
    std::vector<ColoredBody> out = {std::move(cb)};
    out.insert(out.end(), passthrough.begin(), passthrough.end());
    return out;
}

// minkowski_difference() -- erosion, the operation `minkowski()` has no
// inverse for. A BelfrySCAD extension: OpenSCAD has no equivalent module and
// no way to express it in the language, since minkowski() only ever sums.
// Shrinking a part by a clearance, or hollowing one to a wall thickness,
// otherwise needs hand-built difference() scaffolding that is not the same
// operation.
//
// Reads like minkowski(): the first child is the body being eroded, and
// every child after it is a tool eroded away from it in turn. 3D only, same
// as minkowski()'s own 3D path -- Manifold has no CrossSection erosion, and
// 2D already has one in offset(r=-N).

CSGParams resolveMinkowskiDifference(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    (void)args;
    ev.evalChildren(node.children, effCtx);
    return CSGParams{};
}

std::vector<ColoredBody> generateMinkowskiDifference(Evaluator& ev, const CSGParams&,
                                                      const std::vector<std::unique_ptr<CSGNode>>& children,
                                                      const oscad::ASTNode& node) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    const RoleSplit split = splitByRole(bodies);

    std::vector<const ColoredBody*> bodies3d;
    for (const ColoredBody& c : split.foreground) {
        if (c.body) bodies3d.push_back(&c);
    }

    std::vector<ColoredBody> passthrough;
    passthrough.insert(passthrough.end(), split.background.begin(), split.background.end());
    passthrough.insert(passthrough.end(), split.highlight.begin(), split.highlight.end());
    passthrough.insert(passthrough.end(), split.showOnly.begin(), split.showOnly.end());
    passthrough.insert(passthrough.end(), split.displayOnly.begin(), split.displayOnly.end());

    if (bodies3d.empty()) return passthrough;
    if (bodies3d.size() == 1) {
        // Nothing to erode with, so nothing happens -- the same no-op
        // minkowski() gives for a single child.
        std::vector<ColoredBody> result = {*bodies3d.front()};
        result.insert(result.end(), passthrough.begin(), passthrough.end());
        return result;
    }

    manifold::Manifold result = *bodies3d.front()->body;
    for (size_t i = 1; i < bodies3d.size(); ++i) result = result.MinkowskiDifference(*bodies3d[i]->body);
    if (result.Status() != manifold::Manifold::Error::NoError) {
        ev.warn("minkowski_difference: result is not manifold", &node.position());
    }

    ColoredBody cb;
    cb.body = std::move(result);
    cb.color = bodies3d.front()->color;
    // Per-triangle colour cannot survive an erosion: the surface it was
    // indexed against is gone.
    cb.triColors.reset();
    std::vector<ColoredBody> out = {std::move(cb)};
    out.insert(out.end(), passthrough.begin(), passthrough.end());
    return out;
}

// simplify() -- decimate a mesh within a tolerance. A BelfrySCAD extension:
// nothing in the OpenSCAD language can reduce a mesh's triangle count, and
// library-generated or imported geometry is routinely far denser than the
// model needs, which every later boolean and the exported file both pay for.
//
// Wraps Manifold::Simplify (and CrossSection::Simplify for 2D), so it works
// on sections as well as solids.

CSGParams resolveSimplify(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    // Position 0 so simplify(0.1) reads naturally.
    params["tolerance"] = getArg(args, 0, "tolerance", Value{});
    ev.evalChildren(node.children, effCtx);
    return params;
}

std::vector<ColoredBody> generateSimplify(Evaluator& ev, const CSGParams& params,
                                           const std::vector<std::unique_ptr<CSGNode>>& children,
                                           const oscad::ASTNode& node) {
    const std::vector<ColoredBody> bodies = flattenCsgTree(children);
    if (bodies.empty()) return {};

    const Value tolArg = params.at("tolerance");
    const bool explicitTol = std::holds_alternative<double>(tolArg);
    const double explicitValue = explicitTol ? std::get<double>(tolArg) : 0.0;
    if (!explicitTol && !std::holds_alternative<std::monostate>(tolArg)) {
        ev.warn("simplify: tolerance must be a number", &node.position());
    }
    if (explicitTol && explicitValue < 0.0) {
        ev.warn("simplify: tolerance must not be negative", &node.position());
        return bodies;
    }

    std::vector<ColoredBody> out;
    out.reserve(bodies.size());
    for (const ColoredBody& b : bodies) {
        if (b.role != BodyRole::Normal) {
            out.push_back(b);           // background/highlight untouched
            continue;
        }
        ColoredBody cb = b;
        if (b.body) {
            // Manifold::Simplify(0) falls back to the mesh epsilon and
            // changes nothing, so a bare simplify() cannot default to zero
            // -- it would silently do nothing, which is the worst outcome.
            // Default instead to a fraction of the body's own size, which is
            // the only scale-independent choice available: an absolute
            // default sensible in millimetres is catastrophic in metres.
            double tol = explicitValue;
            if (!explicitTol) {
                const manifold::Box bb = b.body->BoundingBox();
                const manifold::vec3 d = bb.Size();
                tol = kDefaultSimplifyFraction * std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            }
            if (tol > 0.0) cb.body = b.body->Simplify(tol);
        } else if (b.section) {
            double tol = explicitValue;
            if (!explicitTol) {
                const manifold::Rect r = b.section->Bounds();
                const manifold::vec2 d = r.Size();
                tol = kDefaultSimplifyFraction * std::sqrt(d.x * d.x + d.y * d.y);
            }
            if (tol > 0.0) cb.section = b.section->Simplify(tol);
        }
        out.push_back(std::move(cb));
    }
    return out;
}

// -- levelset(field, bounds, isovalue) -------------------------------------
//
// A solid from a grid of sampled values. Wraps Manifold::LevelSet with a C++
// trilinear-sampling lambda over the array the script handed us.
//
// Why a grid rather than Manifold's own SDF-callback shape, which would be
// the obvious mapping: measured, building the field in OpenSCAD costs
// 0.41 us/sample against 0.96 us for a closure call per sample, because the
// arithmetic inlines into the list comprehension. More importantly a grid
// means the C++ side never re-enters the evaluator, so canParallel can be
// TRUE -- Manifold's docs warn that parallel policies "will crash language
// runtimes with runtime locks that expect to not be called back by
// unregistered threads", which is exactly what a callback design forfeits.
//
// The trade is memory: ~67 bytes per cell, so 200^3 is about 550 MB and that
// is the practical ceiling. A callback samples lazily and has no ceiling.
//
// Accuracy is bounded by the GRID, not by Manifold. Manifold samples on a
// body-centred cubic lattice (two interleaved cubic grids, sdf.cpp:440-446)
// and a plain cubic grid does not line up with those points, so the lambda
// interpolates. Same accuracy as a grid-based marching-cubes implementation
// in script; worse than LevelSet given a true SDF. Do not imply otherwise.

namespace {

struct ScalarField {
    std::vector<double> v;      // flat, x fastest
    size_t nx = 0, ny = 0, nz = 0;
    double at(size_t i, size_t j, size_t k) const { return v[(k * ny + j) * nx + i]; }
};

// field[i][j][k] -> flat, checking it is a full rectangular block.
std::optional<ScalarField> readField(const Value& val) {
    const ListPtr* xs = std::get_if<ListPtr>(&val);
    if (!xs || !*xs || (*xs)->items.empty()) return std::nullopt;
    ScalarField f;
    f.nx = (*xs)->items.size();
    for (size_t i = 0; i < f.nx; ++i) {
        const ListPtr* ys = std::get_if<ListPtr>(&(*xs)->items[i]);
        if (!ys || !*ys || (*ys)->items.empty()) return std::nullopt;
        if (i == 0) f.ny = (*ys)->items.size();
        else if ((*ys)->items.size() != f.ny) return std::nullopt;
        for (size_t j = 0; j < f.ny; ++j) {
            const ListPtr* zs = std::get_if<ListPtr>(&(*ys)->items[j]);
            if (!zs || !*zs || (*zs)->items.empty()) return std::nullopt;
            if (i == 0 && j == 0) {
                f.nz = (*zs)->items.size();
                f.v.assign(f.nx * f.ny * f.nz, 0.0);
            } else if ((*zs)->items.size() != f.nz) {
                return std::nullopt;
            }
            for (size_t k = 0; k < f.nz; ++k) {
                const double* d = std::get_if<double>(&(*zs)->items[k]);
                if (!d || !std::isfinite(*d)) return std::nullopt;
                f.v[(k * f.ny + j) * f.nx + i] = *d;
            }
        }
    }
    return f;
}

std::optional<std::array<double, 3>> readVec3(const Value& v) {
    const ListPtr* l = std::get_if<ListPtr>(&v);
    if (!l || !*l || (*l)->items.size() != 3) return std::nullopt;
    std::array<double, 3> out{};
    for (size_t i = 0; i < 3; ++i) {
        const double* d = std::get_if<double>(&(*l)->items[i]);
        if (!d || !std::isfinite(*d)) return std::nullopt;
        out[i] = *d;
    }
    return out;
}

} // namespace

CSGParams resolveLevelSet(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    params["field"] = getArg(args, 0, "field", Value{});
    params["bounds"] = getArg(args, 1, "bounds", Value{});
    params["isovalue"] = getArg(args, 2, "isovalue", Value{0.0});
    params["invert"] = getArg(args, 3, "invert", Value{false});
    params["edge"] = getArg(args, 4, "edge", Value{});
    ev.evalChildren(node.children, effCtx);
    return params;
}

std::vector<ColoredBody> generateLevelSet(Evaluator& ev, const CSGParams& params,
                                           const std::vector<std::unique_ptr<CSGNode>>&,
                                           const oscad::ASTNode& node) {
    // Two shapes of field, and the choice matters beyond taste:
    //
    //   a GRID    -- faster to sample (0.41 vs 0.96 us) and meshes in
    //                PARALLEL, because nothing re-enters the evaluator.
    //                Accuracy is capped by the grid; memory is the ceiling.
    //   a FUNCTION -- Manifold picks its own sample points and can snap to
    //                the true surface, so it is MORE accurate, with no
    //                memory ceiling. But every sample is a closure call, and
    //                canParallel must be false or the evaluator gets called
    //                back from threads it knows nothing about.
    const Value& fieldArg = params.at("field");
    const ClosurePtr* fieldFn = std::get_if<ClosurePtr>(&fieldArg);
    std::optional<ScalarField> field;
    if (!fieldFn) {
        field = readField(fieldArg);
        if (!field) {
            ev.warn("levelset(): field must be a function(x,y,z) or a rectangular field[i][j][k] of numbers",
                    &node.position());
            return {};
        }
        if (field->nx < 2 || field->ny < 2 || field->nz < 2) {
            ev.warn("levelset(): the field needs at least 2 samples along each axis", &node.position());
            return {};
        }
    } else if (!*fieldFn || (*fieldFn)->node == nullptr || (*fieldFn)->node->parameters.size() < 3) {
        ev.warn("levelset(): the field function needs three parameters, as in function(x,y,z) ...",
                &node.position());
        return {};
    }

    const ListPtr* bb = std::get_if<ListPtr>(&params.at("bounds"));
    std::optional<std::array<double, 3>> lo, hi;
    if (bb && *bb && (*bb)->items.size() == 2) {
        lo = readVec3((*bb)->items[0]);
        hi = readVec3((*bb)->items[1]);
    }
    if (!lo || !hi) {
        ev.warn("levelset(): bounds must be [[x0,y0,z0],[x1,y1,z1]]", &node.position());
        return {};
    }
    for (int a = 0; a < 3; ++a) {
        if (!((*hi)[a] > (*lo)[a])) {
            ev.warn("levelset(): bounds must be increasing along every axis", &node.position());
            return {};
        }
    }

    // Same rule as linear_solve's b: an explicitly-undef argument is ABSENT,
    // so a fixed-signature wrapper forwarding every parameter still works.
    const Value& isoVal = params.at("isovalue");
    double isovalue = 0.0;
    if (!std::holds_alternative<std::monostate>(isoVal)) {
        const double* isoArg = std::get_if<double>(&isoVal);
        if (!isoArg) {
            ev.warn("levelset(): isovalue must be a number", &node.position());
            return {};
        }
        isovalue = *isoArg;
    }
    const bool invert = truthy(params.at("invert"));

    const std::array<double, 3> origin = *lo;
    std::array<double, 3> spacing{1.0, 1.0, 1.0};
    double edge = 0.0;
    if (field) {
        // Grid spacing is corner-sample to corner-sample, so n-1 intervals.
        spacing = {((*hi)[0] - (*lo)[0]) / static_cast<double>(field->nx - 1),
                    ((*hi)[1] - (*lo)[1]) / static_cast<double>(field->ny - 1),
                    ((*hi)[2] - (*lo)[2]) / static_cast<double>(field->nz - 1)};
        // Finer than the grid buys nothing -- there is no information between
        // samples -- and coarser throws away what the script paid to compute.
        edge = std::min({spacing[0], spacing[1], spacing[2]});
    }
    if (const double* e = std::get_if<double>(&params.at("edge"))) {
        if (*e > 0.0) edge = *e;
        else ev.warn("levelset(): edge must be positive", &node.position());
    }
    if (edge <= 0.0) {
        // No grid to infer it from. Guessing would silently pick either a
        // useless mesh or a ten-minute one -- the cost is cubic in this
        // number, so it is the caller's call to make.
        ev.warn("levelset(): a function field needs edge= (the sample spacing)", &node.position());
        return {};
    }

    // The function form: one closure call per sample. No live EvalContext at
    // generate time, so a root is built from the closure's own scope -- the
    // same approach the warp experiment used. $-variables are therefore at
    // their defaults inside a field function.
    std::optional<EvalContext> fnCtx;
    std::array<std::string, 3> fnParams;
    if (fieldFn) {
        fnCtx = EvalContext::makeRoot((*fieldFn)->node->scope());
        for (int a = 0; a < 3; ++a) fnParams[static_cast<size_t>(a)] = (*fieldFn)->node->parameters[static_cast<size_t>(a)]->name->name;
    }

    const auto sampleFn = [&](manifold::vec3 p) -> double {
        BoundArgs bound;
        const double xyz[3] = {p.x, p.y, p.z};
        for (int a = 0; a < 3; ++a) bound.set(fnParams[static_cast<size_t>(a)], Value{xyz[a]});
        const Value out = ev.evalFunctionLiteralFromBound(**fieldFn, std::move(bound), *fnCtx, &node.position());
        const double* d = std::get_if<double>(&out);
        // A field that returns junk at one point must not abort the whole
        // build; treat it as far outside instead.
        const double v = (d && std::isfinite(*d)) ? *d : std::numeric_limits<double>::max();
        const double diff = v - isovalue;
        return invert ? diff : -diff;
    };

    // Never dereferenced on the function path, but it must still be a valid
    // reference -- binding one to a null pointer is undefined behaviour even
    // where the lambda is never called.
    const ScalarField emptyField;
    const ScalarField& f = field ? *field : emptyField;
    const auto sampleGrid = [&f, origin, spacing, isovalue, invert](manifold::vec3 p) -> double {
        double g[3];
        size_t i0[3];
        double t[3];
        const double pos[3] = {p.x, p.y, p.z};
        const size_t n[3] = {f.nx, f.ny, f.nz};
        for (int a = 0; a < 3; ++a) {
            g[a] = (pos[a] - origin[a]) / spacing[a];
            if (g[a] < 0.0) g[a] = 0.0;
            const double maxg = static_cast<double>(n[a] - 1);
            if (g[a] > maxg) g[a] = maxg;
            i0[a] = static_cast<size_t>(g[a]);
            if (i0[a] > n[a] - 2) i0[a] = n[a] - 2;
            t[a] = g[a] - static_cast<double>(i0[a]);
        }
        double acc = 0.0;
        for (int c = 0; c < 8; ++c) {
            const size_t di = static_cast<size_t>(c & 1), dj = static_cast<size_t>((c >> 1) & 1),
                          dk = static_cast<size_t>((c >> 2) & 1);
            const double w = (di ? t[0] : 1.0 - t[0]) * (dj ? t[1] : 1.0 - t[1]) * (dk ? t[2] : 1.0 - t[2]);
            acc += w * f.at(i0[0] + di, i0[1] + dj, i0[2] + dk);
        }
        // Manifold takes POSITIVE as inside. A distance field is the other
        // way round, which is the common case, so the default flips it.
        const double d = acc - isovalue;
        return invert ? d : -d;
    };

    manifold::Box bounds(manifold::vec3(origin[0], origin[1], origin[2]),
                          manifold::vec3((*hi)[0], (*hi)[1], (*hi)[2]));
    // tolerance -1: a positive value makes Manifold do EXTRA evaluations per
    // output vertex to snap nearer the true surface. Against a fixed grid
    // those only re-interpolate data already used -- cost, no information.
    // canParallel true: the lambda is pure C++ and never re-enters the
    // evaluator, which is the whole point of taking a grid.
    // canParallel is the crux: safe for a grid because the lambda is pure
    // C++, and NOT safe for a function, which re-enters the evaluator.
    // Manifold: parallel policies "will crash language runtimes with runtime
    // locks that expect to not be called back by unregistered threads".
    manifold::Manifold solid =
        fieldFn ? manifold::Manifold::LevelSet(sampleFn, bounds, edge, 0.0, -1.0, /*canParallel=*/false)
                 : manifold::Manifold::LevelSet(sampleGrid, bounds, edge, 0.0, -1.0, /*canParallel=*/true);

    if (solid.IsEmpty()) return {};
    ColoredBody b;
    b.body = std::move(solid);
    std::vector<ColoredBody> out;
    out.push_back(std::move(b));
    return out;
}

} // namespace oscadeval
