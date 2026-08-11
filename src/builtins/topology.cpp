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
// Manifold has no 2D Minkowski of its own, so: cut both into convex
// pieces, take the convex hull of every pairwise sum of points, and union
// the lot. Correct because the sum of two convex sets is the hull of
// their pairwise sums, and Minkowski distributes over union.
//
// ponytail: pieces multiply, so two 64-segment circles are 62 x 62 hulls.
// Convex shapes stay whole, which is the case that actually turns up
// (sweeping a circle), and the reference is no quicker at this.
manifold::CrossSection minkowski2d(const manifold::CrossSection& a, const manifold::CrossSection& b) {
    const std::vector<manifold::SimplePolygon> pa = convexPieces(a);
    const std::vector<manifold::SimplePolygon> pb = convexPieces(b);
    std::vector<manifold::CrossSection> parts;
    parts.reserve(pa.size() * pb.size());
    for (const manifold::SimplePolygon& x : pa) {
        for (const manifold::SimplePolygon& y : pb) {
            manifold::SimplePolygon sum;
            sum.reserve(x.size() * y.size());
            for (const manifold::vec2& p : x)
                for (const manifold::vec2& q : y) sum.push_back({p.x + q.x, p.y + q.y});
            if (sum.size() >= 3) parts.push_back(manifold::CrossSection::Hull(sum));
        }
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
