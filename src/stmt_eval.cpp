#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"

#include <functional>

namespace oscadeval {

void Evaluator::evalAssignment(const oscad::Assignment& node, EvalContext& ctx) {
    const std::string& name = node.name->name;
    if (!name.empty() && name[0] == '$') {
        ctx.dyn->set(name, evalExprMaybeCompiled(*node.expr, ctx));
        ctx.dynExplicit->set(name, true);
        return;
    }

    const oscad::Position* pos = &node.position();
    if (const oscad::Position* const* firstPosEntry = ctx.dynPositions->find(name)) {
        const oscad::Position* firstPos = *firstPosEntry;
        const int firstLine = firstPos ? firstPos->line : 0;
        warn(name + " was assigned on line " + std::to_string(firstLine) + " but was overwritten", pos);
    }
    ctx.let_->set(name, evalExprMaybeCompiled(*node.expr, ctx));
    ctx.dynPositions->set(name, pos);
}

void Evaluator::emitEcho(const std::vector<std::pair<std::optional<std::string>, Value>>& pairs) {
    std::string msg = "ECHO: ";
    for (size_t i = 0; i < pairs.size(); ++i) {
        if (i) msg += ", ";
        const auto& [name, val] = pairs[i];
        msg += name ? (*name + " = " + fmtValue(val)) : fmtValue(val);
    }
    if (echoFn_) echoFn_(msg);
}

void Evaluator::doEcho(const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx) {
    std::vector<std::pair<std::optional<std::string>, Value>> pairs;
    pairs.reserve(arguments.size());
    for (const auto& argPtr : arguments) {
        const oscad::Argument& arg = *argPtr;
        Value val = evalExprMaybeCompiled(*argExpr(arg), ctx);
        std::optional<std::string> name;
        if (arg.kind() == oscad::NodeKind::NamedArgument) name = static_cast<const oscad::NamedArgument&>(arg).name->name;
        pairs.emplace_back(std::move(name), std::move(val));
    }
    emitEcho(pairs);
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
    //
    // Each dimension's own RHS is evaluated HERE, against `parentCtx` (not
    // upfront against the original `ctx`), and re-evaluated on every entry
    // into this recursion level -- i.e. once per combination of whatever
    // dimensions 0..depth-1 are currently bound to. This is required for
    // real OpenSCAD's own documented/verified behavior: a later `for`
    // clause's range CAN depend on an earlier one (`for (i=[0:2], j=[0:i])`
    // is a standard triangular-loop idiom; `pt = f(p)` inside `for (p=...,
    // pt=f(p))` needs the same). Evaluating all dimensions upfront in one
    // flat pass (the previous shape here) evaluated every RHS against the
    // ORIGINAL ctx before ANY variable was bound, so a later dimension's
    // own reference to an earlier one always failed with "unknown
    // variable" -- confirmed wrong against real OpenSCAD.app directly
    // (verified both `for(i=[0:2],j=[0:i])` and a dependent list-comp
    // case), not just an internal inconsistency.
    std::function<void(size_t, EvalContext&)> recurse = [&](size_t depth, EvalContext& parentCtx) {
        if (depth == node.assignments.size()) {
            // Per-full-iteration "entering the body" marker, separate from
            // (and before) the body's own per-statement checks in
            // evalChildren -- mirrors _eval_for's
            // `_check_debug(node.body[0], parent_ctx, expr_level=True)`.
            if (!bodyNodes.empty()) checkDebug(*bodyNodes.front(), parentCtx, /*forced=*/false, /*exprLevel=*/true);
            evalChildren(bodyNodes, parentCtx);
            return;
        }
        const auto& assign = node.assignments[depth];
        Value values = evalExprMaybeCompiled(*assign->expr, parentCtx);
        const oscad::Position* pos = &assign->position();
        IterableValues iter = expandIterable(values, [&](size_t count) {
            warn("Bad range parameter in for statement: too many elements (" + std::to_string(count) + ")", pos);
        });
        for (const Value& val : iter) {
            EvalContext childCtx = parentCtx.childCtx(nullptr, std::nullopt, ctx.childrenNodes, ctx.childrenCallerCtx);
            childCtx.let_->set(assign->name->name, val);
            // One statement-level stop per (bound-so-far) combination, on
            // the loop-variable assignment itself -- so a breakpoint set
            // directly on an `i=[0:2],`/`j=[0:1]` line fires. Mirrors
            // _eval_for's `_check_debug(assign_node, child)`.
            checkDebug(*assign, childCtx);
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
        // Checked against the ORIGINAL ctx (like the RHS eval below), not
        // childCtx -- mirrors _eval_let_block's `_check_debug(assign, ctx)`.
        checkDebug(*assign, ctx);
        Value v = evalExprMaybeCompiled(*assign->expr, ctx);
        const std::string& name = assign->name->name;
        if (!name.empty() && name[0] == '$') {
            childCtx.dyn->set(name, v);
            childCtx.dynExplicit->set(name, true);
        } else {
            childCtx.let_->set(name, v);
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
        // Both if-forms fire a branch-entry marker after the condition and
        // before the branch's own per-statement checks, on the branch's
        // first statement (or the if node itself for an empty branch) --
        // mirrors _eval_statement_impl's
        // `_check_debug(branch[0] if branch else node, ctx, expr_level=True)`.
        case oscad::NodeKind::ModularIf: {
            auto& n = static_cast<const oscad::ModularIf&>(node);
            if (truthy(evalExprMaybeCompiled(*n.condition, ctx))) {
                checkDebug(n.trueBranch.empty() ? node : *n.trueBranch.front(), ctx, /*forced=*/false,
                            /*exprLevel=*/true);
                evalChildren(n.trueBranch, ctx);
            }
            return;
        }
        case oscad::NodeKind::ModularIfElse: {
            auto& n = static_cast<const oscad::ModularIfElse&>(node);
            const auto& branch = truthy(evalExprMaybeCompiled(*n.condition, ctx)) ? n.trueBranch : n.falseBranch;
            checkDebug(branch.empty() ? node : *branch.front(), ctx, /*forced=*/false, /*exprLevel=*/true);
            evalChildren(branch, ctx);
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
    // Whole-LIST compile first (see tryRunCompiledChildren's own doc
    // comment, evaluator.hpp) -- tries the FULL `children` as one chunk
    // before falling back to assignments-then-others below. This is what
    // lets a resolvable module call anywhere in ANY children list --
    // not just a module/top-level body, but a for-loop's/let-block's own
    // body, or a builtin's own children (translate()/union()/etc., whose
    // own resolve function calls this SAME evalChildren internally) --
    // get real Op::CallModule bytecode instead of always falling through
    // to the native per-statement loop below. Unconditionally correct
    // either way since nothing about these statements' observable
    // behavior differs based on which path ran them (same reasoning as
    // tryRunCompiledAssignmentBlock's own doc comment, just for the WHOLE
    // list instead of its own leading assignment run).
    lastCtx_ = &ctx;
    if (tryRunCompiledChildren(children, ctx)) return;

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
            // The port's statement-level debug checkpoint: every real
            // statement flows through here (module/function bodies,
            // for-loop bodies, if/else branches, let-block bodies, the
            // top-level script). Declarations are pure no-ops at
            // statement-eval time (already hoisted into scope), so there's
            // nothing useful to pause on. ModularLet is excluded for a
            // different reason: the reference's _eval_statement_impl
            // explicitly skips it too (`t is not ModularLet` in its own
            // guard) because evalLetBlock pauses on each of the let's
            // ASSIGNMENTS instead -- checking here as well would produce
            // one extra stop the reference never fires.
            const oscad::NodeKind k = child->kind();
            if (k != oscad::NodeKind::ModuleDeclaration && k != oscad::NodeKind::FunctionDeclaration &&
                k != oscad::NodeKind::ModularLet) {
                checkDebug(*child, childCtx);
            }
            evalStatement(*child, childCtx);
        }
    };
    // Whole-batch compile first (see tryRunCompiledAssignmentBlock's own
    // doc comment, evaluator.hpp) -- falls back to the ordinary per-
    // statement loop whenever compiling/running the batch that way isn't
    // possible or eligible, unconditionally correct either way since
    // nothing about these assignments' observable behavior differs based
    // on which path ran them.
    lastCtx_ = &ctx;
    if (!tryRunCompiledAssignmentBlock(assignments, ctx)) runAll(assignments);
    runAll(others);
}

void Evaluator::evalChildren(const std::vector<std::unique_ptr<oscad::ASTNode>>& children, EvalContext& ctx) {
    std::vector<const oscad::ASTNode*> raw;
    raw.reserve(children.size());
    for (const auto& child : children) raw.push_back(child.get());
    evalChildren(raw, ctx);
}

} // namespace oscadeval
