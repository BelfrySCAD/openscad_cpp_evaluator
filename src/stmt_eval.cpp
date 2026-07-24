#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"

#include <functional>

namespace oscadeval {

void Evaluator::evalAssignment(const oscad::Assignment& node, EvalContext& ctx) {
    const std::string& name = node.name->name;
    if (!name.empty() && name[0] == '$') {
        (*ctx.dyn)[name] = evalExpr(*node.expr, ctx);
        ctx.dynExplicit->insert(name);
        return;
    }

    const oscad::Position* pos = &node.position();
    auto positionIt = ctx.dynPositions->find(name);
    if (positionIt != ctx.dynPositions->end()) {
        const oscad::Position* firstPos = positionIt->second;
        const int firstLine = firstPos ? firstPos->line : 0;
        warn(name + " was assigned on line " + std::to_string(firstLine) + " but was overwritten", pos);
    }
    (*ctx.let_)[name] = evalExpr(*node.expr, ctx);
    (*ctx.dynPositions)[name] = pos;
}

void Evaluator::doEcho(const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx) {
    std::string msg = "ECHO: ";
    for (size_t i = 0; i < arguments.size(); ++i) {
        if (i) msg += ", ";
        const oscad::Argument& arg = *arguments[i];
        Value val = evalExpr(*argExpr(arg), ctx);
        if (arg.kind() == oscad::NodeKind::NamedArgument) {
            msg += static_cast<const oscad::NamedArgument&>(arg).name->name + " = " + fmtValue(val);
        } else {
            msg += fmtValue(val);
        }
    }
    if (echoFn_) echoFn_(msg);
}

void Evaluator::evalAssertStatement(const oscad::ModularAssert& node, EvalContext& ctx) {
    // The statement form supports named arguments (assert(condition=...,
    // message=...)), unlike the expression form's raw positional indexing
    // -- see evalAssertExpr. Mirrors _eval_statement_impl's ModularAssert
    // branch exactly.
    CallArgs args = resolveArgs(*this, node.arguments, ctx);
    const bool condition = truthy(getArg(args, 0, "condition", Value{true}));
    if (!condition) {
        std::string condText = node.arguments.empty() ? "false" : argExpr(*node.arguments[0])->toString();
        Value msgArg = getArg(args, 1, "message", Value{});
        std::string err = "Assertion '" + condText + "' failed";
        if (!std::holds_alternative<std::monostate>(msgArg)) {
            const std::string* s = std::get_if<std::string>(&msgArg);
            err += ": \"" + (s ? *s : fmtValue(msgArg)) + "\"";
        }
        error(err, node, "assert");
        return;
    }
    // Assertion passed -- propagate any chained child geometry (e.g.
    // `assert(...) translate(...) children();`).
    if (!node.children.empty()) evalChildren(node.children, ctx);
}

void Evaluator::evalFor(const oscad::ModularFor& node, EvalContext& ctx) {
    struct AssignPair {
        std::string name;
        std::vector<Value> values;
    };
    std::vector<AssignPair> pairs;
    pairs.reserve(node.assignments.size());
    for (const auto& assign : node.assignments) {
        Value values = evalExpr(*assign->expr, ctx);
        pairs.push_back(AssignPair{assign->name->name, expandIterable(values)});
    }

    std::vector<const oscad::ASTNode*> bodyNodes;
    bodyNodes.reserve(node.body.size());
    for (const auto& b : node.body) bodyNodes.push_back(b.get());

    // Nested nary loop over each assignment's own value list (cartesian
    // product, last variable varying fastest) -- each level derives a
    // fresh child scope (childrenNodes/childrenCallerCtx always taken from
    // the *original* ctx, not the current recursion level, matching
    // _eval_for's `_nested` closure exactly) and binds just that one
    // variable, so inner levels/the body see all outer bindings already in
    // `let_`.
    std::function<void(size_t, EvalContext&)> recurse = [&](size_t depth, EvalContext& parentCtx) {
        if (depth == pairs.size()) {
            evalChildren(bodyNodes, parentCtx);
            return;
        }
        for (const Value& val : pairs[depth].values) {
            EvalContext childCtx = parentCtx.childCtx(nullptr, std::nullopt, ctx.childrenNodes, ctx.childrenCallerCtx);
            (*childCtx.let_)[pairs[depth].name] = val;
            recurse(depth + 1, childCtx);
        }
    };
    recurse(0, ctx);
}

void Evaluator::evalLetBlock(const oscad::ModularLet& node, EvalContext& ctx) {
    EvalContext childCtx = ctx.childCtx(nullptr, std::nullopt, ctx.childrenNodes, ctx.childrenCallerCtx);
    // ponytail/port-fidelity note: each assignment's RHS is evaluated
    // against the ORIGINAL `ctx`, not the growing `childCtx` -- unlike the
    // let() *expression* form's documented sequential visibility
    // (`let(a=1, b=a+1) expr` sees `a` while evaluating `b`), the
    // *statement* form `let(a=1, b=a+1) { ... }` does NOT: `b`'s `a+1`
    // looks `a` up in `ctx`, which doesn't have this let's own `a` binding
    // yet. This is a direct, deliberate port of the reference's
    // _eval_let_block, not an oversight -- it evaluates every RHS against
    // `ctx` in the loop, only ever writing into `child_ctx`.
    for (const auto& assign : node.assignments) {
        Value v = evalExpr(*assign->expr, ctx);
        const std::string& name = assign->name->name;
        if (!name.empty() && name[0] == '$') {
            (*childCtx.dyn)[name] = v;
            childCtx.dynExplicit->insert(name);
        } else {
            (*childCtx.let_)[name] = v;
        }
    }
    evalChildren(node.children, childCtx);
}

void Evaluator::evalStatement(const oscad::ASTNode& node, EvalContext& ctx) {
    switch (node.kind()) {
        case oscad::NodeKind::Assignment:
            evalAssignment(static_cast<const oscad::Assignment&>(node), ctx);
            return;
        case oscad::NodeKind::ModularCall:
            evalModularCall(static_cast<const oscad::ModularCall&>(node), ctx);
            return;
        case oscad::NodeKind::ModularModifierHighlight: {
            auto& n = static_cast<const oscad::ModularModifierHighlight&>(node);
            evalModifier(*n.child, n, "highlight", ctx);
            return;
        }
        case oscad::NodeKind::ModularModifierBackground: {
            auto& n = static_cast<const oscad::ModularModifierBackground&>(node);
            evalModifier(*n.child, n, "background", ctx);
            return;
        }
        case oscad::NodeKind::ModularModifierShowOnly: {
            auto& n = static_cast<const oscad::ModularModifierShowOnly&>(node);
            evalModifier(*n.child, n, "show_only", ctx);
            return;
        }
        case oscad::NodeKind::ModularModifierDisable:
            // `*` -- never evaluates its child at all, matching the
            // reference exactly (not just "produces no geometry", the
            // child's own side effects -- echo(), assignments -- also
            // never run).
            return;
        case oscad::NodeKind::ModularIntersectionFor:
            evalIntersectionForNode(static_cast<const oscad::ModularIntersectionFor&>(node), ctx);
            return;
        case oscad::NodeKind::ModularIf: {
            auto& n = static_cast<const oscad::ModularIf&>(node);
            if (truthy(evalExpr(*n.condition, ctx))) evalChildren(n.trueBranch, ctx);
            return;
        }
        case oscad::NodeKind::ModularIfElse: {
            auto& n = static_cast<const oscad::ModularIfElse&>(node);
            if (truthy(evalExpr(*n.condition, ctx))) {
                evalChildren(n.trueBranch, ctx);
            } else {
                evalChildren(n.falseBranch, ctx);
            }
            return;
        }
        case oscad::NodeKind::ModularFor:
            evalFor(static_cast<const oscad::ModularFor&>(node), ctx);
            return;
        case oscad::NodeKind::ModularLet:
            evalLetBlock(static_cast<const oscad::ModularLet&>(node), ctx);
            return;
        case oscad::NodeKind::ModularEcho:
            doEcho(static_cast<const oscad::ModularEcho&>(node).arguments, ctx);
            return;
        case oscad::NodeKind::ModularAssert:
            evalAssertStatement(static_cast<const oscad::ModularAssert&>(node), ctx);
            return;
        default:
            // ModuleDeclaration/FunctionDeclaration/UseStatement/
            // IncludeStatement: pure declarations, no-ops at statement-eval
            // time (already hoisted into scope by buildScopes()).
            return;
    }
}

void Evaluator::evalChildren(const std::vector<const oscad::ASTNode*>& children, EvalContext& ctx) {
    // OpenSCAD executes all assignments in a scope before anything else,
    // each group preserving source order -- mirrors evaluate()/
    // _eval_children's `assignments + others` partition.
    std::vector<const oscad::ASTNode*> assignments;
    std::vector<const oscad::ASTNode*> others;
    for (const oscad::ASTNode* child : children) {
        (child->kind() == oscad::NodeKind::Assignment ? assignments : others).push_back(child);
    }

    auto runAll = [&](const std::vector<const oscad::ASTNode*>& nodes) {
        for (const oscad::ASTNode* child : nodes) {
            const oscad::Scope* childScope = child->scope() ? child->scope() : ctx.scope;
            EvalContext childCtx = ctx.withScope(childScope);
            // Safe despite childCtx's own scope ending at this iteration's
            // close: lastCtx_ is only ever read (by error()) synchronously
            // while some evalStatement() call is still active on the C++
            // call stack -- by then either this exact childCtx or a
            // still-live ancestor's is what lastCtx_ points to, and the
            // next loop iteration (or the next nested evalChildren call)
            // always overwrites it before any subsequent read could see a
            // dangling value.
            lastCtx_ = &childCtx;
            // Every real statement flows through this one call site
            // (module/function bodies, for-loop bodies, if/else branches,
            // let-block bodies, the top-level script) -- see
            // debug_hooks.hpp's DebugHookFn doc comment for why this is
            // the port's single statement-level debug checkpoint rather
            // than the reference's many. Declarations are pure no-ops at
            // statement-eval time (already hoisted into scope), so
            // there's nothing useful to pause on.
            if (child->kind() != oscad::NodeKind::ModuleDeclaration && child->kind() != oscad::NodeKind::FunctionDeclaration) {
                checkDebug(*child, childCtx);
            }
            evalStatement(*child, childCtx);
        }
    };
    runAll(assignments);
    runAll(others);
}

void Evaluator::evalChildren(const std::vector<std::unique_ptr<oscad::ASTNode>>& children, EvalContext& ctx) {
    std::vector<const oscad::ASTNode*> raw;
    raw.reserve(children.size());
    for (const auto& child : children) raw.push_back(child.get());
    evalChildren(raw, ctx);
}

} // namespace oscadeval
