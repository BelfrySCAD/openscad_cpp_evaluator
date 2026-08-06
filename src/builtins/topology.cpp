#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

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

// minkowski() only operates on 3D bodies -- 2D sections among the
// foreground children are silently ignored, matching _generate_minkowski
// exactly (it filters `c.body is not None`, never falls back to sections
// the way hull does).
std::vector<ColoredBody> generateMinkowski(Evaluator& ev, const CSGParams&, const std::vector<std::unique_ptr<CSGNode>>& children,
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
