#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/function_builtins.hpp"
#include "openscad_cpp_evaluator/import_builtin.hpp"

namespace oscadeval {

std::unordered_map<std::string, Value> Evaluator::bindArgs(
    const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
    const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx) {
    std::unordered_map<std::string, Value> result;
    size_t positionalIdx = 0;
    const size_t nparams = params.size();
    for (const auto& argPtr : arguments) {
        if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
            result[a.name->name] = evalExpr(*a.expr, ctx);
        } else {
            auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
            if (positionalIdx < nparams) {
                result[params[positionalIdx]->name->name] = evalExpr(*a.expr, ctx);
            }
            ++positionalIdx;
        }
    }
    return result;
}

EvalContext Evaluator::callCtxFor(const oscad::ASTNode& decl, EvalContext& ctx, const oscad::Scope* scope,
                                   std::shared_ptr<const ChildrenNodeList> childrenNodes,
                                   const EvalContext* childrenCallerCtx) {
    const oscad::Position& declPos = decl.position();
    for (const CallStackFrame& frame : callStack_) {
        const oscad::Position* outer = frame.declPosition;
        if (outer == nullptr || outer->origin != declPos.origin) continue;
        const bool sameSpan = (outer->start_offset == declPos.start_offset && outer->end_offset == declPos.end_offset);
        if (!sameSpan && outer->start_offset <= declPos.start_offset && declPos.end_offset <= outer->end_offset) {
            return ctx.childCtx(scope, std::nullopt, std::move(childrenNodes), childrenCallerCtx);
        }
    }
    return ctx.callCtx(scope, std::nullopt, std::move(childrenNodes), childrenCallerCtx);
}

void Evaluator::applyDefaults(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                               const std::unordered_map<std::string, Value>& bound, EvalContext& childCtx) {
    std::optional<EvalContext> defaultCtx;
    for (const auto& param : params) {
        const std::string& pname = param->name->name;
        // `bound` (bindArgs' own return value), not childCtx.let_/dyn, is
        // the source of truth for "did the caller actually supply this
        // name" -- see this function's own header doc comment for why
        // checking dyn/let_ directly is wrong either way.
        if (bound.count(pname)) continue;
        const bool isDyn = !pname.empty() && pname[0] == '$';
        if (!param->defaultValue) {
            if (isDyn) {
                childCtx.dyn->set(pname, Value{});
            } else {
                childCtx.let_->set(pname, Value{});
            }
            continue;
        }
        if (!defaultCtx) {
            EvalContext dc;
            dc.scope = childCtx.scope;
            dc.dyn = childCtx.dyn; // shared -- a default expression can't mutate $-vars
            dc.let_ = childCtx.let_->openChild(/*isolate=*/true); // empty: no caller locals, no sibling params
            dc.dynPositions = childCtx.dynPositions;
            dc.dynExplicit = childCtx.dynExplicit;
            dc.color = childCtx.color;
            dc.childrenNodes = childCtx.childrenNodes;
            dc.childrenCallerCtx = childCtx.childrenCallerCtx;
            defaultCtx = std::move(dc);
        }
        Value v = evalExpr(*param->defaultValue, *defaultCtx);
        if (isDyn) {
            childCtx.dyn->set(pname, std::move(v));
        } else {
            childCtx.let_->set(pname, std::move(v));
        }
    }
}

void Evaluator::evalUserModule(const oscad::ModuleDeclaration& decl, const oscad::ModularCall& call, EvalContext& ctx) {
    const oscad::Scope* childScope = decl.scope() ? decl.scope() : ctx.scope;
    auto bound = bindArgs(decl.parameters, call.arguments, ctx);

    auto childrenNodes = std::make_shared<ChildrenNodeList>();
    childrenNodes->reserve(call.children.size());
    for (const auto& c : call.children) childrenNodes->push_back(c.get());

    EvalContext childCtx = callCtxFor(decl, ctx, childScope, childrenNodes, &ctx);

    // $children counts module-instantiation child *statements* passed in
    // `{}`, not the number of geometries they produce -- e.g. children()
    // counts as one child even if it forwards zero bodies.
    int childrenCount = 0;
    for (const auto& c : call.children) {
        if (c->kind() != oscad::NodeKind::Assignment && c->kind() != oscad::NodeKind::ModuleDeclaration &&
            c->kind() != oscad::NodeKind::FunctionDeclaration) {
            ++childrenCount;
        }
    }
    childCtx.dyn->set("$children", Value{static_cast<double>(childrenCount)});

    for (auto& [k, v] : bound) {
        if (!k.empty() && k[0] == '$') {
            childCtx.dyn->set(k, std::move(v));
        } else {
            childCtx.let_->set(k, std::move(v));
        }
    }
    applyDefaults(decl.parameters, bound, childCtx);

    int parentModules = 0;
    for (const CallStackFrame& f : callStack_) {
        if (f.kind == CallStackFrame::Kind::Module) ++parentModules;
    }
    childCtx.dyn->set("$parent_modules", Value{static_cast<double>(parentModules)});

    std::optional<ProfileHandle> prof = profileEnter("module", call.name->name, &call.position(), &decl.position());
    callStack_.push_back(
        CallStackFrame{CallStackFrame::Kind::Module, call.name->name, &call.position(), &decl.position()});
    try {
        evalChildren(decl.children, childCtx);
    } catch (...) {
        callStack_.pop_back();
        if (prof) profileExit(*prof);
        throw;
    }
    callStack_.pop_back();
    if (prof) profileExit(*prof);
}

Value Evaluator::evalUserFunction(const std::string& name, const oscad::FunctionDeclaration& decl,
                                   const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                                   const oscad::ASTNode* callNode) {
    auto bound = bindArgs(decl.parameters, arguments, ctx);
    const oscad::Scope* fnScope = decl.scope() ? decl.scope() : ctx.scope;
    EvalContext childCtx = callCtxFor(decl, ctx, fnScope);
    for (auto& [k, v] : bound) {
        if (!k.empty() && k[0] == '$') {
            childCtx.dyn->set(k, std::move(v));
        } else {
            childCtx.let_->set(k, std::move(v));
        }
    }
    applyDefaults(decl.parameters, bound, childCtx);

    const oscad::Position* callPos = callNode ? &callNode->position() : nullptr;
    std::optional<ProfileHandle> prof = profileEnter("function", name, callPos, &decl.position());
    callStack_.push_back(CallStackFrame{CallStackFrame::Kind::Function, name, callPos, &decl.position()});
    Value result;
    try {
        checkDebug(*decl.expr, childCtx);
        lastCtx_ = &childCtx;
        result = evalExpr(*decl.expr, childCtx);
        if (debugHooks_.returnHook) debugHooks_.returnHook(name, result, static_cast<int>(callStack_.size()));
    } catch (...) {
        callStack_.pop_back();
        if (prof) profileExit(*prof);
        throw;
    }
    callStack_.pop_back();
    if (prof) profileExit(*prof);
    return result;
}

Value Evaluator::evalFunctionLiteral(const oscad::FunctionLiteral& funcNode,
                                      const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                                      const oscad::ASTNode* callNode) {
    auto bound = bindArgs(funcNode.parameters, arguments, ctx);
    const oscad::Scope* fnScope = funcNode.scope() ? funcNode.scope() : ctx.scope;
    EvalContext childCtx = callCtxFor(funcNode, ctx, fnScope);
    for (auto& [k, v] : bound) {
        if (!k.empty() && k[0] == '$') {
            childCtx.dyn->set(k, std::move(v));
        } else {
            childCtx.let_->set(k, std::move(v));
        }
    }
    applyDefaults(funcNode.parameters, bound, childCtx);

    const oscad::Position* callPos = callNode ? &callNode->position() : nullptr;
    // "<function literal>" -- the reference's call-stack entry uses the
    // call site's own callee-expression name when statically known (e.g.
    // `g(3)` for `g = function(x) ...`), which isn't threaded through here
    // (this evaluator doesn't track the binding name a literal happened to
    // be assigned to). Only affects TRACE-line text for an error raised
    // from inside a function-literal call; low value to thread through
    // further given no test depends on it.
    std::optional<ProfileHandle> prof = profileEnter("function", "<function literal>", callPos, &funcNode.position());
    callStack_.push_back(
        CallStackFrame{CallStackFrame::Kind::Function, "<function literal>", callPos, &funcNode.position()});
    Value result;
    try {
        checkDebug(*funcNode.body, childCtx);
        lastCtx_ = &childCtx;
        result = evalExpr(*funcNode.body, childCtx);
        if (debugHooks_.returnHook) debugHooks_.returnHook("<function literal>", result, static_cast<int>(callStack_.size()));
    } catch (...) {
        callStack_.pop_back();
        if (prof) profileExit(*prof);
        throw;
    }
    callStack_.pop_back();
    if (prof) profileExit(*prof);
    return result;
}

Value Evaluator::evalFunctionCall(const oscad::PrimaryCall& node, EvalContext& ctx) {
    const oscad::Identifier* leftId =
        (node.left->kind() == oscad::NodeKind::Identifier) ? static_cast<const oscad::Identifier*>(node.left.get()) : nullptr;

    if (leftId) {
        if (leftId->name == "import") {
            CallArgs args = resolveArgs(*this, node.arguments, ctx);
            return importAsValue(*this, args, node);
        }
        // A builtin function name always wins over a same-named user
        // function -- checked *before* the user-function lookup, not
        // after, matching the reference's _eval_function_call precedence
        // exactly (name in _BUILTIN_FN_NAMES gates which branch runs at
        // all).
        if (isBuiltinFunctionName(leftId->name)) {
            // object() needs the raw argument list, not a pre-resolved
            // CallArgs, to merge positional/named arguments in their exact
            // call-site interleaved order (see builtinObject's own
            // comment) -- called directly, before resolveArgs, so
            // arguments aren't evaluated twice.
            if (leftId->name == "object") return builtinObject(*this, node.arguments, ctx);
            CallArgs args = resolveArgs(*this, node.arguments, ctx);
            return evalBuiltinFunction(*this, leftId->name, args, node);
        }
        const oscad::ASTNode* decl = ctx.scope->lookupFunction(leftId->name);
        if (decl && decl->kind() == oscad::NodeKind::FunctionDeclaration) {
            return evalUserFunction(leftId->name, static_cast<const oscad::FunctionDeclaration&>(*decl), node.arguments,
                                     ctx, &node);
        }
    }

    // Not a builtin or user function -- check whether the callee expression
    // evaluates to a function-literal *value* (e.g. `g = function(x) x*2;
    // g(3)`). warnIfUndef=false: probing here shouldn't itself warn
    // "unknown variable" if `left` isn't bound to anything -- a genuinely
    // unknown callee gets exactly one warning below, not two.
    Value funcNode = leftId ? evalIdentifier(leftId->name, &leftId->position(), ctx, false) : evalExpr(*node.left, ctx);
    if (const auto* flPtr = std::get_if<const oscad::FunctionLiteral*>(&funcNode); flPtr && *flPtr) {
        return evalFunctionLiteral(**flPtr, node.arguments, ctx, &node);
    }

    if (leftId) {
        warn("Ignoring unknown function '" + leftId->name + "'", &node.position());
    }
    return Value{};
}

Value Evaluator::parentModuleName(int idx) const {
    std::vector<std::string> modules;
    for (const CallStackFrame& f : callStack_) {
        if (f.kind == CallStackFrame::Kind::Module) modules.push_back(f.name);
    }
    const int revIdx = static_cast<int>(modules.size()) - 1 - idx;
    if (revIdx < 0 || static_cast<size_t>(revIdx) >= modules.size()) return Value{};
    return Value{modules[static_cast<size_t>(revIdx)]};
}

void Evaluator::builtinChildren(const CallArgs& args, EvalContext& ctx) {
    Value idxArg = getArg(args, 0, "index", Value{});
    if (!ctx.childrenNodes || ctx.childrenNodes->empty()) return;
    const EvalContext* callerCtx = ctx.childrenCallerCtx;
    if (!callerCtx) return;

    // A children() forwarding chain's own dyn/let_/etc. must alias the
    // *caller's* (not this ctx's) -- see EvalContext::withScope's rationale.
    EvalContext evalCtx = callerCtx->childCtx(nullptr, std::nullopt, callerCtx->childrenNodes, callerCtx->childrenCallerCtx);
    for (const auto& [k, v] : ctx.dyn->items()) {
        if (!k.empty() && k[0] == '$') evalCtx.dyn->set(k, v);
    }
    for (const auto& [k, v] : ctx.let_->items()) {
        if (!k.empty() && k[0] == '$') evalCtx.let_->set(k, v);
    }

    if (std::holds_alternative<std::monostate>(idxArg)) {
        evalChildren(*ctx.childrenNodes, evalCtx);
        return;
    }

    // children(N) indexes child *statements*, not output bodies -- a
    // filtered statement may produce 0 bodies, which would shift every
    // subsequent body-index lookup, so the Nth statement is evaluated
    // directly instead.
    const int idx = static_cast<int>(toDoubleLenient(idxArg));
    std::vector<const oscad::ASTNode*> geoNodes;
    for (const oscad::ASTNode* c : *ctx.childrenNodes) {
        if (c->kind() != oscad::NodeKind::Assignment && c->kind() != oscad::NodeKind::ModuleDeclaration &&
            c->kind() != oscad::NodeKind::FunctionDeclaration) {
            geoNodes.push_back(c);
        }
    }
    if (idx < 0 || static_cast<size_t>(idx) >= geoNodes.size()) return;
    evalChildren(std::vector<const oscad::ASTNode*>{geoNodes[static_cast<size_t>(idx)]}, evalCtx);
}

} // namespace oscadeval
