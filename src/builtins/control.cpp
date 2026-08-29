#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <functional>

namespace oscadeval {

// children()/children(N) -- see Evaluator::builtinChildren for the actual
// deferred-evaluation logic; this just resolves the call's own arguments
// and hands off. Mirrors _resolve_children_call.
CSGParams resolveChildren(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    ev.builtinChildren(args, effCtx);
    return CSGParams{};
}

// Never actually reached (evalModularCall always splices "children" --
// see its own doc comment) but kept registered so a lookup for "children"
// is never a dispatch miss.
std::vector<ColoredBody> generateChildren(Evaluator&, const CSGParams&, const std::vector<std::unique_ptr<CSGNode>>& children,
                                           const oscad::ASTNode&) {
    return flattenCsgTree(children);
}

// render() -- a display hint; only its children matter. No generate
// function is registered for "render" (falls to generateTree()'s default
// child-concatenation, exactly reproducing passthrough). Mirrors
// _resolve_render.
CSGParams resolveRender(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    (void)args;
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(effCtx);
    ev.evalChildren(node.children, blockCtx);
    return CSGParams{};
}

// breakpoint([condition]) -- unconditionally forces a debug pause (bypassing
// whatever step/breakpoint-line filtering the injected debugHook itself
// applies) unless `condition` is given and falsy. No generate function
// registered either (breakpoint() has no children to concatenate in
// practice; falls to the default). Mirrors
// _resolve_breakpoint/_builtin_breakpoint.
CSGParams resolveBreakpoint(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    const Value condition = getArg(args, 0, "condition", Value{});
    if (!std::holds_alternative<std::monostate>(condition) && !truthy(condition)) return CSGParams{};
    ev.checkDebug(node, effCtx, /*forced=*/true);
    return CSGParams{};
}

// Merges a single statement's (possibly multiple) bodies into one, for
// intersection_for's per-iteration combine step before intersecting across
// iterations. Mirrors _combine: 3D bodies union together (a lone body
// passes through unchanged, no-op union); otherwise 2D sections union;
// otherwise an empty Manifold placeholder.
ColoredBody combineBodies(const std::vector<ColoredBody>& bodies) {
    std::vector<const ColoredBody*> bodies3d;
    for (const ColoredBody& b : bodies) {
        if (b.body) bodies3d.push_back(&b);
    }
    if (!bodies3d.empty()) {
        if (bodies3d.size() == 1) return *bodies3d.front();
        manifold::Manifold merged = *bodies3d.front()->body;
        for (size_t i = 1; i < bodies3d.size(); ++i) merged = merged + *bodies3d[i]->body;
        ColoredBody cb;
        cb.body = std::move(merged);
        cb.color = bodies3d.front()->color;
        return cb;
    }
    std::vector<const ColoredBody*> sections;
    for (const ColoredBody& b : bodies) {
        if (b.section) sections.push_back(&b);
    }
    if (sections.empty()) {
        ColoredBody cb;
        cb.body = manifold::Manifold();
        return cb;
    }
    manifold::CrossSection cs = *sections.front()->section;
    for (size_t i = 1; i < sections.size(); ++i) cs = cs + *sections[i]->section;
    ColoredBody cb;
    cb.section = std::move(cs);
    cb.color = bodies.front().color;
    return cb;
}

// intersection_for(assignments...) body -- per the reference's own
// comment: unlike for/if/let, this is NOT transparent in the CSG tree (its
// iterations combine into a single `^`-intersected result, so they must
// nest under one intersection_for node the same way union()'s children
// nest under a union node -- otherwise flattenCsgTree() would return the
// pre-intersection per-iteration bodies instead of the real combined
// result). group_sizes records, per iteration, how many CSGNode children
// it contributed (the loop body can itself contain for/if/let, themselves
// transparent, so one iteration can contribute a variable number of tree
// children) -- same rationale as booleans.cpp's own group_sizes. Mirrors
// _resolve_intersection_for/_generate_intersection_for.
CSGParams resolveIntersectionFor(Evaluator& ev, const oscad::ModularIntersectionFor& node, EvalContext& ctx) {
    // Expanded for the same reason resolveCsg expands its own block: a
    // children(..., separate=true) here contributes one statement -- and so
    // one operand group per iteration -- per child it selects.
    const std::vector<const oscad::ASTNode*> bodyNodes = ev.expandChildStatements(node.body, ctx);

    std::vector<Value> groupSizes;

    // Each dimension's own RHS is evaluated against `parentCtx` (not
    // upfront against the original `ctx`), re-evaluated on every entry
    // into this recursion level -- see evalFor's own doc comment
    // (stmt_eval.cpp) for the full "verified against real OpenSCAD.app"
    // rationale; this is the identical bug in intersection_for's own
    // cartesian loop (e.g. `intersection_for (i=[0:2], j=[0:i]) ...`
    // needs `i` visible in `j`'s own range expression).
    std::function<void(size_t, EvalContext&)> recurse = [&](size_t depth, EvalContext& parentCtx) {
        if (depth == node.assignments.size()) {
            // One body-entry marker per full cartesian-product iteration
            // and nothing per individual variable binding (unlike
            // evalFor) -- mirrors _resolve_intersection_for's single
            // `_check_debug(body_node[0], loop_ctx, expr_level=True)`.
            if (!bodyNodes.empty()) ev.checkDebug(*bodyNodes.front(), parentCtx, /*forced=*/false, /*exprLevel=*/true);
            const size_t before = ev.currentTreeFrameSize();
            ev.evalChildren(bodyNodes, parentCtx);
            groupSizes.push_back(Value{static_cast<double>(ev.currentTreeFrameSize() - before)});
            return;
        }
        const auto& assign = node.assignments[depth];
        Value values = ev.evalExpr(*assign->expr, parentCtx);
        const oscad::Position* pos = &assign->position();
        IterableValues iter = expandIterable(values, [&](size_t count) {
            ev.warn("Bad range parameter in for statement: too many elements (" + std::to_string(count) + ")", pos);
        });
        for (const Value& val : iter) {
            EvalContext childCtx = parentCtx.childCtx(nullptr, std::nullopt, ctx.childrenNodes, ctx.childrenCallerCtx);
            childCtx.let_->set(assign->name->name, val);
            recurse(depth + 1, childCtx);
        }
    };
    recurse(0, ctx);

    CSGParams params;
    params["group_sizes"] = Value{std::make_shared<const ValueList>(ValueList{std::move(groupSizes)})};
    return params;
}

std::vector<ColoredBody> generateIntersectionFor(Evaluator&, const CSGParams& params,
                                                  const std::vector<std::unique_ptr<CSGNode>>& children,
                                                  const oscad::ASTNode&) {
    const auto& groupSizes = std::get<ListPtr>(params.at("group_sizes"))->items;
    std::vector<ColoredBody> iterations;
    size_t idx = 0;
    for (const Value& sv : groupSizes) {
        const size_t size = static_cast<size_t>(std::get<double>(sv));
        std::vector<ColoredBody> stmtBodies = flattenCsgTree(children, idx, size);
        idx += size;
        if (!stmtBodies.empty()) iterations.push_back(combineBodies(stmtBodies));
    }
    if (iterations.empty()) return {};

    std::vector<ColoredBody> bodies3d;
    for (const ColoredBody& c : iterations) {
        if (c.body) bodies3d.push_back(c);
    }
    if (!bodies3d.empty()) {
        manifold::Manifold result = *bodies3d.front().body;
        for (size_t i = 1; i < bodies3d.size(); ++i) result = result ^ *bodies3d[i].body;
        ColoredBody cb;
        cb.body = std::move(result);
        cb.color = bodies3d.front().color;
        return {cb};
    }

    std::vector<ColoredBody> sections;
    for (const ColoredBody& c : iterations) {
        if (c.section) sections.push_back(c);
    }
    if (!sections.empty()) {
        manifold::CrossSection result = *sections.front().section;
        for (size_t i = 1; i < sections.size(); ++i) result = result ^ *sections[i].section;
        ColoredBody cb;
        cb.section = std::move(result);
        cb.color = iterations.front().color;
        return {cb};
    }
    return {};
}

} // namespace oscadeval
