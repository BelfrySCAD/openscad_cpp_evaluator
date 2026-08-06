#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/stb_font_provider.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>

namespace oscadeval {

namespace {

bool isNumber(const Value& v) { return std::holds_alternative<double>(v); }
bool isBoolValue(const Value& v) { return std::holds_alternative<bool>(v); }

// Converts a Value to the truncated-to-int64 form real OpenSCAD's own
// Value::toInt64() uses for bitwise operations (Value.cc, PR #4833:
// `trunc(toDouble())` cast to int64_t). Only ever called after an isNumber()
// guard -- bool/string/etc. never reach here, matching real OpenSCAD's own
// `type() == Type::NUMBER` gate (bool is a distinct type, never coerced).
std::int64_t toBitwiseInt64(const Value& v) { return static_cast<std::int64_t>(std::trunc(std::get<double>(v))); }

// Python's `%`: result takes the divisor's sign (unlike C++ std::fmod,
// which takes the dividend's), e.g. `-1 % 3` is `2` in Python, `-1` via
// plain fmod.
double pyMod(double a, double b) {
    double r = std::fmod(a, b);
    if (r != 0.0 && ((r < 0.0) != (b < 0.0))) r += b;
    return r;
}

// Ordering for oscComparable()-gated pairs only (same-type number/number,
// string/string, vector/vector, or bool/bool) -- mirrors Python's native
// `<` for these specific paired types, including NaN behavior (`nan < x`
// and `x < nan` are both false). Returns nullopt if a list/list comparison
// hits a pair of nested elements that themselves aren't oscComparable --
// mirrors Python's TypeError-caught-to-undef path one level down (no
// "undefined operation" warning at that depth; only the top-level gate in
// evalExpr's Greater/LessThan cases warns).
std::optional<bool> valueLess(const Value& a, const Value& b) {
    if (const bool* ba = std::get_if<bool>(&a)) return static_cast<int>(*ba) < static_cast<int>(std::get<bool>(b));
    if (const double* da = std::get_if<double>(&a)) return *da < std::get<double>(b);
    if (const std::string* sa = std::get_if<std::string>(&a)) return *sa < std::get<std::string>(b);

    const ListPtr& la = std::get<ListPtr>(a);
    const ListPtr& lb = std::get<ListPtr>(b);
    static const std::vector<Value> kEmpty;
    const auto& ia = la ? la->items : kEmpty;
    const auto& ib = lb ? lb->items : kEmpty;
    const size_t n = std::min(ia.size(), ib.size());
    for (size_t i = 0; i < n; ++i) {
        if (oscEqual(ia[i], ib[i])) continue;
        if (!oscComparable(ia[i], ib[i])) return std::nullopt;
        return valueLess(ia[i], ib[i]);
    }
    return ia.size() < ib.size();
}

bool isListCompClauseKind(oscad::NodeKind kind) {
    using oscad::NodeKind;
    switch (kind) {
        case NodeKind::ListCompIf:
        case NodeKind::ListCompIfElse:
        case NodeKind::ListCompFor:
        case NodeKind::ListCompCFor:
        case NodeKind::ListCompLet:
        case NodeKind::ListCompEach:
            return true;
        default:
            return false;
    }
}

void appendAll(std::vector<Value>& out, std::vector<Value> more) {
    out.insert(out.end(), std::make_move_iterator(more.begin()), std::make_move_iterator(more.end()));
}

} // namespace

Evaluator::Evaluator(EchoFn echoFn, std::shared_ptr<FontProvider> fontProvider, std::shared_ptr<ManifoldCache> manifoldCache,
                     DebugHooks debugHooks, bool profiling)
    : echoFn_(std::move(echoFn)), fontProvider_(std::move(fontProvider)), manifoldCache_(std::move(manifoldCache)),
      debugHooks_(std::move(debugHooks)), profiling_(profiling) {}

FontProvider& Evaluator::fontProvider() {
    if (!fontProvider_) fontProvider_ = std::make_shared<StbFontProvider>();
    return *fontProvider_;
}

void Evaluator::warn(const std::string& message, const oscad::Position* position) {
    if (!echoFn_) return;
    // During generate the call stack has already unwound, so fall back to
    // the entry position resolve captured onto the CSGNode being generated
    // (see CSGNode::warnEntry). Passed as a synthetic single-frame stack so
    // formatWarning needs no second code path -- the frame carries no name,
    // which traceLines renders as a bare "called by ''" line, so it is
    // deliberately NOT included there; only the headline's "from" clause
    // uses it.
    if (callStack_.empty() && generateWarnEntry != nullptr) {
        std::string out = formatWarning(message, position, {});
        const bool sameSpot = position != nullptr && generateWarnEntry->origin == position->origin &&
                               generateWarnEntry->line == position->line;
        if (!sameSpot) {
            out += ", from " + generateWarnEntry->origin + ", line " + std::to_string(generateWarnEntry->line);
        }
        echoFn_(out);
        return;
    }
    echoFn_(formatWarning(message, position, callStack_));
}

Value Evaluator::evalIdentifier(const std::string& name, const oscad::Position* position, EvalContext& ctx,
                                 bool warnIfUndef) {
    if (const Value* v = ctx.let_->find(name)) return *v;

    if (!name.empty() && name[0] == '$') {
        // $-prefixed names are dynamic variables: a completely separate
        // namespace from lexically-scoped identifiers, never requiring a
        // scope declaration (unlike a bare identifier). Found via
        // BelfrySCAD/BOSL2's constants.scad `get_slop()`
        // (`is_undef($slop) ? 0 : $slop` with $slop never assigned
        // anywhere) producing a spurious "Ignoring unknown variable
        // '$slop'" warning here but not through the bytecode VM's
        // Op::LoadDyn (bytecode_vm.cpp), which already does exactly
        // this -- find in ctx.dyn, undef if absent, no scope-lookup
        // fallback, no warning, ever. Verified against real OpenSCAD.app:
        // no warning for a never-set $-variable. Must return here
        // unconditionally rather than falling through to the
        // scope->lookupVariable() check below, which can never resolve a
        // $-name anyway (dynamic variables aren't scope-declarable) and
        // exists only for plain identifiers.
        const Value* v = ctx.dyn->find(name);
        return v ? *v : Value{};
    }

    if (name == "PI") return Value{std::numbers::pi};

    const oscad::ASTNode* decl = ctx.scope->lookupVariable(name);
    if (decl == nullptr) {
        if (warnIfUndef) warn("Ignoring unknown variable '" + name + "'", position);
        return Value{};
    }
    if (decl->kind() == oscad::NodeKind::ParameterDeclaration) return Value{};
    return evalExpr(*static_cast<const oscad::Assignment*>(decl)->expr, ctx);
}

// -- List comprehensions ----------------------------------------------

// Debug checkpoints below mirror the reference's _eval_list_comp AND its
// near-duplicate twin _eval_list_comp_body (both dispatch over the same
// ListComp* kinds with identical _check_debug placement; this port folds
// them into this one function plus evalListCompBody's ListComprehension
// early-out, so one set of calls covers both).
void Evaluator::evalListElement(const oscad::ASTNode& elem, EvalContext& ctx, std::vector<Value>& out) {
    switch (elem.kind()) {
        case oscad::NodeKind::ListCompFor: {
            auto& n = static_cast<const oscad::ListCompFor&>(elem);
            const bool isNestedLc = (n.body->kind() == oscad::NodeKind::ListComprehension);

            // Each dimension's own RHS is evaluated against `parentCtx`
            // (not upfront against the original `ctx`), re-evaluated on
            // every entry into this recursion level -- see evalFor's own
            // doc comment (stmt_eval.cpp) for the full "verified against
            // real OpenSCAD.app" rationale; this is the identical bug in
            // the list-comprehension sibling of that same construct (e.g.
            // `[for (p=[1:N], pt=f(p)) pt]` needs `p` visible in `pt`'s own
            // range expression).
            std::function<void(size_t, EvalContext&)> recurse = [&](size_t depth, EvalContext& parentCtx) {
                if (depth == n.assignments.size()) {
                    if (isNestedLc) {
                        out.push_back(evalListLiteral(static_cast<const oscad::ListComprehension&>(*n.body), parentCtx));
                    } else {
                        appendAll(out, evalListCompBody(*n.body, parentCtx));
                    }
                    return;
                }
                const auto& assign = n.assignments[depth];
                Value values = evalExpr(*assign->expr, parentCtx);
                const oscad::Position* pos = &assign->position();
                IterableValues iter = expandIterable(values, [&](size_t count) {
                    warn("Bad range parameter in for statement: too many elements (" + std::to_string(count) + ")", pos);
                });
                for (const Value& val : iter) {
                    EvalContext childCtx = parentCtx.letChildCtx();
                    childCtx.let_->set(assign->name->name, val);
                    // Per-binding stop, same shape as evalFor's -- but with
                    // NO separate body-entry marker at depth == pairs.size()
                    // (_eval_listcomp_for has none; the body's own element
                    // check below supplies the expr-level stop instead).
                    checkDebug(*assign, childCtx);
                    recurse(depth + 1, childCtx);
                }
            };
            recurse(0, ctx);
            return;
        }
        case oscad::NodeKind::ListCompCFor: {
            // Stop order per _eval_listcomp_cfor, exactly:
            //   init(s) -> [cond(true) -> body -> incr(s)]* -> cond(false)
            // The condition is checked (expr-level) on EVERY pass including
            // the final false one that ends the loop; body entry uses the
            // whole ListCompCFor node, not the body expression.
            auto& n = static_cast<const oscad::ListCompCFor&>(elem);
            EvalContext loopCtx = ctx.letChildCtx();
            for (const auto& assign : n.inits) {
                checkDebug(*assign, loopCtx);
                loopCtx.let_->set(assign->name->name, evalExpr(*assign->expr, loopCtx));
            }

            const bool isNestedLc = (n.body->kind() == oscad::NodeKind::ListComprehension);
            constexpr int kMaxCForIterations = 1'000'000;
            int iterations = 0;
            while (true) {
                checkDebug(*n.condition, loopCtx, /*forced=*/false, /*exprLevel=*/true);
                if (!truthy(evalExpr(*n.condition, loopCtx))) break;
                ++iterations;
                if (iterations > kMaxCForIterations) {
                    error("C-style for loop exceeded maximum iteration count", n);
                }
                checkDebug(n, loopCtx);
                if (isNestedLc) {
                    out.push_back(evalListLiteral(static_cast<const oscad::ListComprehension&>(*n.body), loopCtx));
                } else {
                    appendAll(out, evalListCompBody(*n.body, loopCtx));
                }
                for (const auto& assign : n.incrs) {
                    checkDebug(*assign, loopCtx);
                    loopCtx.let_->set(assign->name->name, evalExpr(*assign->expr, loopCtx));
                }
            }
            return;
        }
        case oscad::NodeKind::ListCompIf: {
            auto& n = static_cast<const oscad::ListCompIf&>(elem);
            checkDebug(n, ctx);
            if (truthy(evalExpr(*n.condition, ctx))) {
                checkDebug(*n.trueExpr, ctx, /*forced=*/false, /*exprLevel=*/true);
                appendAll(out, evalListCompBody(*n.trueExpr, ctx));
            }
            return;
        }
        case oscad::NodeKind::ListCompIfElse: {
            auto& n = static_cast<const oscad::ListCompIfElse&>(elem);
            checkDebug(n, ctx);
            const oscad::ASTNode& branch = truthy(evalExpr(*n.condition, ctx)) ? *n.trueExpr : *n.falseExpr;
            checkDebug(branch, ctx, /*forced=*/false, /*exprLevel=*/true);
            appendAll(out, evalListCompBody(branch, ctx));
            return;
        }
        case oscad::NodeKind::ListCompLet: {
            auto& n = static_cast<const oscad::ListCompLet&>(elem);
            EvalContext letCtx = ctx.letChildCtx();
            // Per-assignment stops only -- no expr-level body marker here,
            // unlike the if/if-else/each clauses (matches the reference).
            for (const auto& assign : n.assignments) {
                checkDebug(*assign, letCtx);
                bindLetName(letCtx, assign->name->name, evalExpr(*assign->expr, letCtx));
            }
            appendAll(out, evalListCompBody(*n.body, letCtx));
            return;
        }
        case oscad::NodeKind::ListCompEach: {
            auto& n = static_cast<const oscad::ListCompEach&>(elem);
            checkDebug(n, ctx, /*forced=*/false, /*exprLevel=*/true);
            const oscad::ASTNode& inner = *n.body;
            if (isListCompClauseKind(inner.kind())) {
                for (const Value& item : evalListCompBody(inner, ctx)) appendEachInto(out, item);
            } else {
                appendEachInto(out, evalExpr(static_cast<const oscad::Expression&>(inner), ctx));
            }
            return;
        }
        default:
            checkDebug(elem, ctx, /*forced=*/false, /*exprLevel=*/true);
            out.push_back(evalExpr(static_cast<const oscad::Expression&>(elem), ctx));
            return;
    }
}

std::vector<Value> Evaluator::evalListCompBody(const oscad::ASTNode& body, EvalContext& ctx) {
    if (body.kind() == oscad::NodeKind::ListComprehension) {
        return {evalListLiteral(static_cast<const oscad::ListComprehension&>(body), ctx)};
    }
    std::vector<Value> out;
    evalListElement(body, ctx, out);
    return out;
}

Value Evaluator::evalListLiteral(const oscad::ListComprehension& node, EvalContext& ctx) {
    std::vector<Value> items;
    items.reserve(node.elements.size());
    for (const auto& elemPtr : node.elements) evalListElement(*elemPtr, ctx, items);
    return Value{std::make_shared<const ValueList>(ValueList{std::move(items)})};
}

Value Evaluator::evalRangeLiteral(const oscad::RangeLiteral& node, EvalContext& ctx) {
    return applyRange(evalExpr(*node.start, ctx), evalExpr(*node.end, ctx), evalExpr(*node.step, ctx));
}

// Shared with the bytecode VM's RANGE opcode -- see applyBinaryOp's own
// comment on why these are factored out as plain Value x Value x Value ->
// Value functions.
Value Evaluator::applyRange(const Value& startV, const Value& endV, const Value& stepV) {
    double start = std::holds_alternative<std::monostate>(startV) ? 0.0 : toDoubleLenient(startV);
    double end = std::holds_alternative<std::monostate>(endV) ? 0.0 : toDoubleLenient(endV);
    double step = std::holds_alternative<std::monostate>(stepV) ? 1.0 : toDoubleLenient(stepV);
    return Value{OscRange{start, step, end}};
}

// -- let()/echo()/assert() expression forms ------------------------------

// Writes one let()-clause binding into the right place ($-prefixed ->
// dyn/dynExplicit, everything else -> let_). Mirrors _bind_let_name, minus
// the copy-on-first-$-write tracking: this port's letChildCtx() always
// copies dyn/dynExplicit eagerly (see its own doc comment), so there's
// nothing to defer here. A member (not a TU-local free function) so
// user_calls.cpp's tail-call trampoline can share it instead of
// duplicating this logic.
void Evaluator::bindLetName(EvalContext& ctx, const std::string& name, const Value& v) {
    if (!name.empty() && name[0] == '$') {
        ctx.dyn->set(name, v);
        ctx.dynExplicit->set(name, true);
    } else {
        ctx.let_->set(name, v);
    }
}

Value Evaluator::evalLetExpr(const oscad::LetOp& node, EvalContext& ctx) {
    EvalContext childCtx = ctx.letChildCtx();
    for (const auto& assign : node.assignments) {
        // Sequential: each RHS is evaluated against childCtx, so it sees
        // earlier bindings from the same let() -- `let(a=1, b=a+1) b` -> 2.
        // Unlike the *statement* form (evalLetBlock), which does not --
        // see that method's own doc comment for why this isn't a copy/paste
        // mistake in one direction or the other.
        // Checked against childCtx (not ctx) -- _expr_let does the same,
        // the mirror image of _eval_let_block's `_check_debug(assign, ctx)`.
        checkDebug(*assign, childCtx);
        Value v = evalExpr(*assign->expr, childCtx);
        bindLetName(childCtx, assign->name->name, v);
    }
    return evalExpr(*node.body, childCtx);
}

Value Evaluator::evalEchoExpr(const oscad::EchoOp& node, EvalContext& ctx) {
    checkDebug(node, ctx); // _expr_echo
    doEcho(node.arguments, ctx);
    return evalExpr(*node.body, ctx);
}

Value Evaluator::evalAssertExpr(const oscad::AssertOp& node, EvalContext& ctx) {
    // Unlike the statement form (evalAssertStatement), which supports named
    // arguments via getArg()/CallArgs, the expression form indexes raw
    // arguments positionally -- mirrors the reference's _expr_assert
    // exactly (`raw[0].expr`/`raw[1].expr`, not _get_arg).
    checkDebug(node, ctx); // _expr_assert, before the condition is evaluated
    const auto& raw = node.arguments;
    const bool condition = raw.empty() || truthy(evalExpr(*argExpr(*raw[0]), ctx));
    if (!condition) {
        std::string condText = raw.empty() ? "false" : argExpr(*raw[0])->toString();
        std::string err = "Assertion '" + condText + "' failed";
        if (raw.size() > 1) {
            Value msg = evalExpr(*argExpr(*raw[1]), ctx);
            const std::string* s = std::get_if<std::string>(&msg);
            err += ": \"" + (s ? *s : fmtValue(msg)) + "\"";
        }
        error(err, node, "assert");
    }
    return evalExpr(*node.body, ctx);
}

// -- Indexing / member access ---------------------------------------------

namespace {
std::optional<int> swizzleIndex(const std::string& member) {
    if (member == "x") return 0;
    if (member == "y") return 1;
    if (member == "z") return 2;
    if (member == "w") return 3;
    return std::nullopt;
}
} // namespace

Value Evaluator::evalExpr(const oscad::Expression& node, EvalContext& ctx) {
    using oscad::NodeKind;
    switch (node.kind()) {
        case NodeKind::NumberLiteral:
            return Value{static_cast<const oscad::NumberLiteral&>(node).val};
        case NodeKind::BooleanLiteral:
            return Value{static_cast<const oscad::BooleanLiteral&>(node).val};
        case NodeKind::StringLiteral:
            return Value{static_cast<const oscad::StringLiteral&>(node).val};
        case NodeKind::UndefinedLiteral:
            return Value{};
        case NodeKind::CommentedExpr:
            return evalExpr(*static_cast<const oscad::CommentedExpr&>(node).expr, ctx);
        case NodeKind::Identifier: {
            auto& n = static_cast<const oscad::Identifier&>(node);
            return evalIdentifier(n.name, &n.position(), ctx, true);
        }
        case NodeKind::RangeLiteral:
            return evalRangeLiteral(static_cast<const oscad::RangeLiteral&>(node), ctx);
        case NodeKind::ListComprehension:
            return evalListLiteral(static_cast<const oscad::ListComprehension&>(node), ctx);
        case NodeKind::FunctionLiteral:
            // A function-literal *value* captures ctx.let_ (extending that
            // trail level's lifetime for as long as this Closure is
            // reachable -- see Closure's own doc comment, value.hpp) so a
            // closure that escapes this call (returned, stored, passed on)
            // can still resolve variables from its defining scope after
            // this call has returned. `dyn`/scope are NOT captured: $-vars
            // are dynamically scoped (see the caller's own ctx.dyn at call
            // time, matching real OpenSCAD), and the lexical scope is
            // static AST data already reachable via the node itself.
            return Value{std::make_shared<const Closure>(
                Closure{static_cast<const oscad::FunctionLiteral*>(&node), ctx.let_})};
        case NodeKind::PrimaryCall:
            return evalFunctionCall(static_cast<const oscad::PrimaryCall&>(node), ctx);
        case NodeKind::LetOp:
            return evalLetExpr(static_cast<const oscad::LetOp&>(node), ctx);
        case NodeKind::EchoOp:
            return evalEchoExpr(static_cast<const oscad::EchoOp&>(node), ctx);
        case NodeKind::AssertOp:
            return evalAssertExpr(static_cast<const oscad::AssertOp&>(node), ctx);

        case NodeKind::PrimaryIndex: {
            auto& n = static_cast<const oscad::PrimaryIndex&>(node);
            return applyIndexAccess(evalExpr(*n.left, ctx), evalExpr(*n.index, ctx));
        }
        case NodeKind::PrimaryMember: {
            auto& n = static_cast<const oscad::PrimaryMember&>(node);
            return applyMemberAccess(evalExpr(*n.left, ctx), n.member->name);
        }

        case NodeKind::AdditionOp: {
            auto& n = static_cast<const oscad::AdditionOp&>(node);
            return applyBinaryOp(NodeKind::AdditionOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::SubtractionOp: {
            auto& n = static_cast<const oscad::SubtractionOp&>(node);
            return applyBinaryOp(NodeKind::SubtractionOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::MultiplicationOp: {
            auto& n = static_cast<const oscad::MultiplicationOp&>(node);
            return applyBinaryOp(NodeKind::MultiplicationOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::DivisionOp: {
            auto& n = static_cast<const oscad::DivisionOp&>(node);
            return applyBinaryOp(NodeKind::DivisionOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::ModuloOp: {
            auto& n = static_cast<const oscad::ModuloOp&>(node);
            return applyBinaryOp(NodeKind::ModuloOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::ExponentOp: {
            auto& n = static_cast<const oscad::ExponentOp&>(node);
            return applyBinaryOp(NodeKind::ExponentOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::UnaryMinusOp: {
            auto& n = static_cast<const oscad::UnaryMinusOp&>(node);
            return applyUnaryOp(NodeKind::UnaryMinusOp, evalExpr(*n.expr, ctx), n.position());
        }

        case NodeKind::LogicalAndOp: {
            // Must short-circuit: the reference's `bool(eval(left)) and
            // bool(eval(right))` relies on Python's own `and` not evaluating
            // `right` when `left` is falsy -- and BOSL2-style code depends on
            // it directly (`is_undef(x) || (assert(is_num(x)) ...)` throws if
            // the assert always runs). Evaluating both sides unconditionally
            // (the previous behavior here) broke exactly that idiom.
            auto& n = static_cast<const oscad::LogicalAndOp&>(node);
            if (!truthy(evalExpr(*n.left, ctx))) return Value{false};
            return Value{truthy(evalExpr(*n.right, ctx))};
        }
        case NodeKind::LogicalOrOp: {
            auto& n = static_cast<const oscad::LogicalOrOp&>(node);
            if (truthy(evalExpr(*n.left, ctx))) return Value{true};
            return Value{truthy(evalExpr(*n.right, ctx))};
        }
        case NodeKind::LogicalNotOp: {
            auto& n = static_cast<const oscad::LogicalNotOp&>(node);
            return applyUnaryOp(NodeKind::LogicalNotOp, evalExpr(*n.expr, ctx), n.position());
        }

        case NodeKind::EqualityOp: {
            auto& n = static_cast<const oscad::EqualityOp&>(node);
            return applyBinaryOp(NodeKind::EqualityOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::InequalityOp: {
            auto& n = static_cast<const oscad::InequalityOp&>(node);
            return applyBinaryOp(NodeKind::InequalityOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::GreaterThanOp: {
            auto& n = static_cast<const oscad::GreaterThanOp&>(node);
            return applyBinaryOp(NodeKind::GreaterThanOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::GreaterThanOrEqualOp: {
            auto& n = static_cast<const oscad::GreaterThanOrEqualOp&>(node);
            return applyBinaryOp(NodeKind::GreaterThanOrEqualOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx),
                                  n.position());
        }
        case NodeKind::LessThanOp: {
            auto& n = static_cast<const oscad::LessThanOp&>(node);
            return applyBinaryOp(NodeKind::LessThanOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::LessThanOrEqualOp: {
            auto& n = static_cast<const oscad::LessThanOrEqualOp&>(node);
            return applyBinaryOp(NodeKind::LessThanOrEqualOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx),
                                  n.position());
        }

        case NodeKind::TernaryOp: {
            // Mirrors _expr_ternary: a statement-level stop on the whole
            // ternary before the condition runs, then an expr-level one on
            // whichever branch was chosen, before evaluating it.
            auto& n = static_cast<const oscad::TernaryOp&>(node);
            checkDebug(n, ctx);
            const oscad::Expression& branch = truthy(evalExpr(*n.condition, ctx)) ? *n.trueExpr : *n.falseExpr;
            checkDebug(branch, ctx, /*forced=*/false, /*exprLevel=*/true);
            return evalExpr(branch, ctx);
        }

        case NodeKind::BitwiseOrOp: {
            auto& n = static_cast<const oscad::BitwiseOrOp&>(node);
            return applyBinaryOp(NodeKind::BitwiseOrOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::BitwiseAndOp: {
            auto& n = static_cast<const oscad::BitwiseAndOp&>(node);
            return applyBinaryOp(NodeKind::BitwiseAndOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx), n.position());
        }
        case NodeKind::BitwiseNotOp: {
            auto& n = static_cast<const oscad::BitwiseNotOp&>(node);
            return applyUnaryOp(NodeKind::BitwiseNotOp, evalExpr(*n.expr, ctx), n.position());
        }
        case NodeKind::BitwiseShiftLeftOp: {
            auto& n = static_cast<const oscad::BitwiseShiftLeftOp&>(node);
            return applyBinaryOp(NodeKind::BitwiseShiftLeftOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx),
                                  n.position());
        }
        case NodeKind::BitwiseShiftRightOp: {
            auto& n = static_cast<const oscad::BitwiseShiftRightOp&>(node);
            return applyBinaryOp(NodeKind::BitwiseShiftRightOp, evalExpr(*n.left, ctx), evalExpr(*n.right, ctx),
                                  n.position());
        }

        default:
            throw std::logic_error(std::string("Evaluator::evalExpr: NodeKind '") + oscad::nodeKindName(node.kind()) +
                                    "' not yet implemented (later phase)");
    }
}

// -- Shared binary/unary/index/member primitives --------------------------
// Moved out of evalExpr's switch verbatim (see this method's own call sites
// above): also called by the bytecode VM's generic BINARY_OP/UNARY_OP/INDEX/
// MEMBER opcodes (bytecode_vm.cpp) so neither copy can drift from the other.

Value Evaluator::applyBinaryOp(oscad::NodeKind kind, const Value& a, const Value& b, const oscad::Position& pos) {
    using oscad::NodeKind;
    switch (kind) {
        case NodeKind::AdditionOp:
            if (isNumber(a) && isNumber(b)) return Value{std::get<double>(a) + std::get<double>(b)};
            return vecAdd(a, b);
        case NodeKind::SubtractionOp:
            if (isNumber(a) && isNumber(b)) return Value{std::get<double>(a) - std::get<double>(b)};
            return vecSub(a, b);
        case NodeKind::MultiplicationOp: {
            if (isNumber(a) && isNumber(b)) return Value{std::get<double>(a) * std::get<double>(b)};
            const bool aList = std::holds_alternative<ListPtr>(a);
            const bool bList = std::holds_alternative<ListPtr>(b);
            if (aList && bList) return matmul(a, b);
            if (aList && isNumber(b)) return scale(std::get<double>(b), a);
            if (bList && isNumber(a)) return scale(std::get<double>(a), b);
            return Value{}; // remaining combos (bool, string*number, ...) never succeed in the reference either
        }
        case NodeKind::DivisionOp: {
            if (isNumber(a) && isNumber(b)) {
                const double av = std::get<double>(a), bv = std::get<double>(b);
                if (bv == 0.0) {
                    return Value{av == 0.0 ? std::numeric_limits<double>::quiet_NaN()
                                            : std::copysign(std::numeric_limits<double>::infinity(), av)};
                }
                return Value{av / bv};
            }
            if (isBoolValue(a) || isBoolValue(b)) return Value{};
            if (std::holds_alternative<ListPtr>(a) && isNumber(b)) return divScale(a, std::get<double>(b));
            return Value{};
        }
        case NodeKind::ModuloOp: {
            if (isBoolValue(a) || isBoolValue(b) || !isNumber(a) || !isNumber(b)) return Value{};
            const double bv = std::get<double>(b);
            if (bv == 0.0) return Value{}; // ZeroDivisionError -> undef
            return Value{pyMod(std::get<double>(a), bv)};
        }
        case NodeKind::ExponentOp: {
            if (isBoolValue(a) || isBoolValue(b) || !isNumber(a) || !isNumber(b)) return Value{};
            const double av = std::get<double>(a), bv = std::get<double>(b);
            if (av == 0.0 && bv < 0.0) return Value{}; // ZeroDivisionError -> undef
            return Value{std::pow(av, bv)};
        }
        case NodeKind::EqualityOp:
            return Value{oscEqual(a, b)};
        case NodeKind::InequalityOp:
            return Value{!oscEqual(a, b)};
        case NodeKind::GreaterThanOp: {
            if (!oscComparable(a, b)) {
                warn("undefined operation (" + oscTypeName(a) + " > " + oscTypeName(b) + ")", &pos);
                return Value{};
            }
            auto gt = valueLess(b, a);
            return gt ? Value{*gt} : Value{};
        }
        case NodeKind::GreaterThanOrEqualOp: {
            if (!oscComparable(a, b)) {
                warn("undefined operation (" + oscTypeName(a) + " >= " + oscTypeName(b) + ")", &pos);
                return Value{};
            }
            auto gt = valueLess(b, a);
            return gt ? Value{*gt || oscEqual(a, b)} : Value{};
        }
        case NodeKind::LessThanOp: {
            if (!oscComparable(a, b)) {
                warn("undefined operation (" + oscTypeName(a) + " < " + oscTypeName(b) + ")", &pos);
                return Value{};
            }
            auto lt = valueLess(a, b);
            return lt ? Value{*lt} : Value{};
        }
        case NodeKind::LessThanOrEqualOp: {
            if (!oscComparable(a, b)) {
                warn("undefined operation (" + oscTypeName(a) + " <= " + oscTypeName(b) + ")", &pos);
                return Value{};
            }
            auto lt = valueLess(a, b);
            return lt ? Value{*lt || oscEqual(a, b)} : Value{};
        }
        // Real OpenSCAD added these in PR #4833 (merged 2025-03-14, "Bitwise
        // operators. Fixes #3345."): both operands truncate-to-int64 (real
        // OpenSCAD's own Value::toInteger()/toInt64(), Value.cc), operate in
        // int64_t two's-complement arithmetic, then cast back to double --
        // matching real OpenSCAD's own storage (no separate integer type;
        // "operates on ... values stored as ordinary OpenSCAD numbers," per
        // the PR description). `bool` is a distinct type from number here
        // exactly as everywhere else in this file, never coerced -- a
        // `bool`/`string`/etc. operand is an "undefined operation" warning,
        // per real OpenSCAD's own `type() == Type::NUMBER` gate.
        case NodeKind::BitwiseOrOp: {
            if (!isNumber(a) || !isNumber(b)) {
                warn("undefined operation (" + oscTypeName(a) + " | " + oscTypeName(b) + ")", &pos);
                return Value{};
            }
            return Value{static_cast<double>(toBitwiseInt64(a) | toBitwiseInt64(b))};
        }
        case NodeKind::BitwiseAndOp: {
            if (!isNumber(a) || !isNumber(b)) {
                warn("undefined operation (" + oscTypeName(a) + " & " + oscTypeName(b) + ")", &pos);
                return Value{};
            }
            return Value{static_cast<double>(toBitwiseInt64(a) & toBitwiseInt64(b))};
        }
        case NodeKind::BitwiseShiftLeftOp: {
            if (!isNumber(a) || !isNumber(b)) {
                warn("undefined operation (" + oscTypeName(a) + " << " + oscTypeName(b) + ")", &pos);
                return Value{};
            }
            const std::int64_t rhs = toBitwiseInt64(b);
            if (rhs < 0) {
                warn("negative shift", &pos);
                return Value{};
            }
            if (rhs >= 64) {
                warn("shift too large", &pos);
                return Value{};
            }
            return Value{static_cast<double>(toBitwiseInt64(a) << rhs)};
        }
        case NodeKind::BitwiseShiftRightOp: {
            if (!isNumber(a) || !isNumber(b)) {
                warn("undefined operation (" + oscTypeName(a) + " >> " + oscTypeName(b) + ")", &pos);
                return Value{};
            }
            const std::int64_t rhs = toBitwiseInt64(b);
            if (rhs < 0) {
                warn("negative shift", &pos);
                return Value{};
            }
            if (rhs >= 64) {
                warn("shift too large", &pos);
                return Value{};
            }
            // Arithmetic (sign-propagating) right shift for a negative lhs
            // is well-defined by the C++20 standard for signed integer
            // types -- the same underlying operation real OpenSCAD's own
            // C++ compiles `int64_t >> rhs` to, so no separate sign-fixup
            // is needed here to match it.
            return Value{static_cast<double>(toBitwiseInt64(a) >> rhs)};
        }
        default:
            throw std::logic_error(std::string("Evaluator::applyBinaryOp: NodeKind '") + oscad::nodeKindName(kind) +
                                    "' is not a binary operator");
    }
}

Value Evaluator::applyUnaryOp(oscad::NodeKind kind, const Value& v, const oscad::Position& pos) {
    using oscad::NodeKind;
    switch (kind) {
        case NodeKind::UnaryMinusOp:
            if (std::holds_alternative<ListPtr>(v)) return scale(-1.0, v);
            if (isBoolValue(v)) return Value{};
            if (isNumber(v)) return Value{-std::get<double>(v)};
            return Value{};
        case NodeKind::LogicalNotOp:
            return Value{!truthy(v)};
        case NodeKind::BitwiseNotOp:
            if (!isNumber(v)) {
                warn("undefined operation (~" + oscTypeName(v) + ")", &pos);
                return Value{};
            }
            return Value{static_cast<double>(~toBitwiseInt64(v))};
        default:
            throw std::logic_error(std::string("Evaluator::applyUnaryOp: NodeKind '") + oscad::nodeKindName(kind) +
                                    "' is not a unary operator");
    }
}

Value Evaluator::applyIndexAccess(const Value& obj, const Value& idx) {
    if (const ListPtr* l = std::get_if<ListPtr>(&obj)) {
        if (const double* d = std::get_if<double>(&idx)) {
            const int i = static_cast<int>(*d);
            if (i < 0 || !*l || static_cast<size_t>(i) >= (*l)->items.size()) return Value{};
            return (*l)->items[static_cast<size_t>(i)];
        }
        return Value{};
    }
    if (const std::string* s = std::get_if<std::string>(&obj)) {
        if (const double* d = std::get_if<double>(&idx)) {
            const int i = static_cast<int>(*d);
            if (i < 0 || static_cast<size_t>(i) >= s->size()) return Value{};
            return Value{std::string(1, (*s)[static_cast<size_t>(i)])};
        }
        return Value{};
    }
    if (const OscRange* r = std::get_if<OscRange>(&obj)) {
        if (const double* d = std::get_if<double>(&idx)) {
            const int i = static_cast<int>(*d);
            if (i == 0) return Value{r->start};
            if (i == 1) return Value{r->step};
            if (i == 2) return Value{r->end};
        }
        return Value{};
    }
    if (const ObjectPtr* o = std::get_if<ObjectPtr>(&obj)) {
        if (const std::string* key = std::get_if<std::string>(&idx); key && *o) {
            for (const auto& [k, v] : (*o)->items) {
                if (k == *key) return v;
            }
        }
        return Value{};
    }
    return Value{};
}

Value Evaluator::applyMemberAccess(const Value& obj, const std::string& member) {
    if (const ListPtr* l = std::get_if<ListPtr>(&obj); l && *l) {
        std::optional<int> idx = swizzleIndex(member);
        if (idx && static_cast<size_t>(*idx) < (*l)->items.size()) return (*l)->items[static_cast<size_t>(*idx)];
        return Value{};
    }
    if (const ObjectPtr* o = std::get_if<ObjectPtr>(&obj); o && *o) {
        for (const auto& [k, v] : (*o)->items) {
            if (k == member) return v;
        }
        return Value{};
    }
    return Value{};
}

} // namespace oscadeval
