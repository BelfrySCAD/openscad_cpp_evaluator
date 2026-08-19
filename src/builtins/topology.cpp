#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <manifold/polygon.h>

namespace oscadeval {

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

} // namespace oscadeval
