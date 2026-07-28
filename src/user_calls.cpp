#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/bytecode_compiler.hpp"
#include "openscad_cpp_evaluator/bytecode_vm.hpp"
#include "openscad_cpp_evaluator/function_builtins.hpp"
#include "openscad_cpp_evaluator/import_builtin.hpp"

#include <cstdlib>

namespace oscadeval {

const CompiledChunk* Evaluator::lookupOrCompileChunk(const oscad::FunctionDeclaration& decl) {
    auto it = chunkCache_.find(&decl);
    if (it == chunkCache_.end()) {
        it = chunkCache_.emplace(&decl, tryCompileFunction(decl)).first;
        if (it->second) flattenNestedLiterals(*it->second);
    }
    return it->second ? &*it->second : nullptr;
}

const CompiledChunk* Evaluator::lookupCompiledLiteralChunk(const oscad::FunctionLiteral& node) const {
    auto it = literalChunkCache_.find(&node);
    return it != literalChunkCache_.end() ? &it->second : nullptr;
}

void Evaluator::flattenNestedLiterals(CompiledChunk& chunk) {
    for (auto& [litNode, litChunk] : chunk.nestedLiterals) {
        flattenNestedLiterals(litChunk);
        literalChunkCache_.emplace(litNode, std::move(litChunk));
    }
    chunk.nestedLiterals.clear();
}

const Value* Evaluator::findUpvalue(const oscad::ASTNode* targetDecl, int slot) const {
    // Walks upvalueParent links, NOT a blind scan of callStack_ -- see
    // that field's own doc comment for the real bug this distinction
    // fixes (a closure passed through an unrelated, isolating
    // intermediary function must NOT see the enclosing call's locals,
    // even though that call is still technically active on the stack).
    if (callStack_.empty()) return nullptr;
    int idx = static_cast<int>(callStack_.size()) - 1;
    while (idx >= 0) {
        const CallStackFrame& frame = callStack_[static_cast<size_t>(idx)];
        if (frame.declNode == targetDecl && frame.vmFrame) {
            const auto* vmFrame = static_cast<const VmFrame*>(frame.vmFrame);
            if (static_cast<size_t>(slot) < vmFrame->slots.size()) return &vmFrame->slots[static_cast<size_t>(slot)];
            return nullptr;
        }
        idx = frame.upvalueParent;
    }
    return nullptr;
}

namespace {
std::optional<bool> g_bytecodeVmOverride;
}

bool Evaluator::bytecodeVmEnabled() {
    if (g_bytecodeVmOverride.has_value()) return *g_bytecodeVmOverride;
    static const bool enabled = [] {
        const char* v = std::getenv("OSCAD_BYTECODE_VM");
        return v == nullptr || std::string(v) != "0";
    }();
    return enabled;
}

void Evaluator::setBytecodeVmEnabledForTesting(std::optional<bool> enabled) { g_bytecodeVmOverride = enabled; }

std::unique_ptr<VmFrame> Evaluator::acquireVmFrame() {
    if (!vmFramePool_.empty()) {
        auto frame = std::move(vmFramePool_.back());
        vmFramePool_.pop_back();
        return frame;
    }
    return std::make_unique<VmFrame>();
}

void Evaluator::releaseVmFrame(std::unique_ptr<VmFrame> frame) {
    frame->stack.clear();
    frame->slots.clear();
    frame->bound.clear();
    vmFramePool_.push_back(std::move(frame));
}

BoundArgs Evaluator::bindArgs(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                               const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx) {
    BoundArgs result;
    result.reserve(arguments.size());
    size_t positionalIdx = 0;
    const size_t nparams = params.size();
    for (const auto& argPtr : arguments) {
        if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
            result.set(a.name->name, evalExpr(*a.expr, ctx));
        } else {
            auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
            if (positionalIdx < nparams) {
                result.set(params[positionalIdx]->name->name, evalExpr(*a.expr, ctx));
            }
            ++positionalIdx;
        }
    }
    return result;
}

EvalContext Evaluator::callCtxFor(const oscad::ASTNode& decl, EvalContext& ctx, const oscad::Scope* scope,
                                   std::shared_ptr<const ChildrenNodeList> childrenNodes,
                                   const EvalContext* childrenCallerCtx, bool* usedChildCtx,
                                   const std::shared_ptr<TrailView<Value>>& capturedLet) {
    // A closure's own captured environment (capturedLet, non-null for
    // every interpreter-evaluated FunctionLiteral -- see expr_eval.cpp's
    // own FunctionLiteral case) is always the correct root for this call,
    // full stop -- checked BEFORE the live-call-stack walk below.
    // callCtxFromCapturedLet roots the new scope at `capturedLet` itself,
    // never at `ctx.let_`, so it's already correct regardless of whether
    // `ctx` (the CALLER's own context) happens to be ancestor-linked back
    // to the closure's declaring call or not. That distinction matters in
    // exactly the case the live-frame walk below gets wrong: a closure
    // declared inside function A, passed as a Value into a plain function
    // B, and invoked from inside B's body. B's own ctx is isolated from
    // A's (an ordinary call boundary, see EvalContext::callCtx's own
    // isolate=true), so even though A's own frame may still be technically
    // live on callStack_ (matching the containment check below) and this
    // WOULD take the childCtx() branch, `ctx.childCtx()` continues B's own
    // (isolated) ancestry, not A's -- silently losing every variable the
    // closure captured from A. Skipping straight to capturedLet avoids
    // this regardless of which frames happen to still be live.
    if (capturedLet) {
        if (usedChildCtx) *usedChildCtx = false;
        return ctx.callCtxFromCapturedLet(capturedLet, scope, std::nullopt, std::move(childrenNodes), childrenCallerCtx);
    }
    const oscad::Position& declPos = decl.position();
    for (const CallStackFrame& frame : callStack_) {
        const oscad::Position* outer = frame.declPosition;
        if (outer == nullptr || outer->origin != declPos.origin) continue;
        const bool sameSpan = (outer->start_offset == declPos.start_offset && outer->end_offset == declPos.end_offset);
        if (!sameSpan && outer->start_offset <= declPos.start_offset && declPos.end_offset <= outer->end_offset) {
            if (usedChildCtx) *usedChildCtx = true;
            return ctx.childCtx(scope, std::nullopt, std::move(childrenNodes), childrenCallerCtx);
        }
    }
    if (usedChildCtx) *usedChildCtx = false;
    return ctx.callCtx(scope, std::nullopt, std::move(childrenNodes), childrenCallerCtx);
}

std::optional<EvalContext> Evaluator::isolatedCallCtxFor(const oscad::ASTNode& declNode, EvalContext& ctx,
                                                          const std::shared_ptr<TrailView<Value>>& capturedLet) {
    bool usedChildCtx = false;
    EvalContext result = callCtxFor(declNode, ctx, ctx.scope, nullptr, nullptr, &usedChildCtx, capturedLet);
    if (usedChildCtx) return std::nullopt;
    return result;
}

void Evaluator::applyDefaults(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                               const BoundArgs& bound, EvalContext& childCtx) {
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

void Evaluator::bindCallArgsInto(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                                  BoundArgs bound, EvalContext& childCtx) {
    for (auto& [k, v] : bound) {
        if (!k.empty() && k[0] == '$') {
            childCtx.dyn->set(k, std::move(v));
        } else {
            childCtx.let_->set(k, std::move(v));
        }
    }
    applyDefaults(params, bound, childCtx);
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
    callStack_.back().bodyCtx = &childCtx; // per-frame locals for the debugger
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

// -- Tail-call optimization (interpreter path) -----------------------------
//
// See these functions' own declarations in evaluator.hpp for the full
// design rationale (mirrors upstream real OpenSCAD's own
// FunctionCall::evaluate()/simplify_function_body trampoline, with the
// isolated-hop-only restriction this port's stack-scanning closure lookup
// requires).

std::optional<Evaluator::TailStep> Evaluator::tryTailStepFor(
    const std::string& calleeName, const oscad::ASTNode& declNode,
    const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params, const oscad::Expression& body,
    bool hasCompiledChunk, const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
    const oscad::Position& callPos, const std::shared_ptr<TrailView<Value>>& capturedLet) {
    if (hasCompiledChunk) return std::nullopt;
    const oscad::Scope* fnScope = declNode.scope() ? declNode.scope() : ctx.scope;
    bool usedChildCtx = false;
    EvalContext childCtx = callCtxFor(declNode, ctx, fnScope, nullptr, nullptr, &usedChildCtx, capturedLet);
    if (usedChildCtx) return std::nullopt;
    bindCallArgsInto(params, bindArgs(params, arguments, ctx), childCtx);
    TailStep step;
    step.nextExpr = &body;
    step.ctx = std::move(childCtx);
    step.isNewLogicalCall = true;
    step.calleeName = calleeName;
    step.calleeDecl = &declNode;
    step.callPos = &callPos;
    return step;
}

std::variant<Value, Evaluator::TailStep, Evaluator::NotTailStep> Evaluator::simplifyTailStep(
    const oscad::Expression& node, EvalContext& ctx) {
    switch (node.kind()) {
        // Each of these 5 cases duplicates its evalExpr counterpart's own
        // debug checkpoints too -- a tail-position ternary/let/echo/assert/
        // call would otherwise silently lose the stops the reference (which
        // has no TCO at all) fires on every hop.
        case oscad::NodeKind::TernaryOp: {
            auto& n = static_cast<const oscad::TernaryOp&>(node);
            checkDebug(n, ctx);
            const oscad::Expression* branch = truthy(evalExpr(*n.condition, ctx)) ? n.trueExpr.get() : n.falseExpr.get();
            checkDebug(*branch, ctx, /*forced=*/false, /*exprLevel=*/true);
            TailStep step;
            step.nextExpr = branch;
            step.ctx = ctx;
            return step;
        }
        case oscad::NodeKind::LetOp: {
            auto& n = static_cast<const oscad::LetOp&>(node);
            EvalContext childCtx = ctx.letChildCtx();
            for (const auto& assign : n.assignments) {
                checkDebug(*assign, childCtx);
                Value v = evalExpr(*assign->expr, childCtx);
                bindLetName(childCtx, assign->name->name, v);
            }
            TailStep step;
            step.nextExpr = n.body.get();
            step.ctx = std::move(childCtx);
            return step;
        }
        case oscad::NodeKind::EchoOp: {
            auto& n = static_cast<const oscad::EchoOp&>(node);
            checkDebug(n, ctx);
            doEcho(n.arguments, ctx);
            TailStep step;
            step.nextExpr = n.body.get();
            step.ctx = ctx;
            return step;
        }
        case oscad::NodeKind::AssertOp: {
            // Mirrors evalAssertExpr's own condition-check/error-building
            // logic exactly (a small, deliberate duplication -- see this
            // function's own header comment on why it doesn't delegate to
            // evalExpr for these 5 node kinds).
            auto& n = static_cast<const oscad::AssertOp&>(node);
            checkDebug(n, ctx);
            const auto& raw = n.arguments;
            const bool condition = raw.empty() || truthy(evalExpr(*argExpr(*raw[0]), ctx));
            if (!condition) {
                std::string condText = raw.empty() ? "false" : argExpr(*raw[0])->toString();
                std::string err = "Assertion '" + condText + "' failed";
                if (raw.size() > 1) {
                    Value msg = evalExpr(*argExpr(*raw[1]), ctx);
                    const std::string* s = std::get_if<std::string>(&msg);
                    err += ": \"" + (s ? *s : fmtValue(msg)) + "\"";
                }
                error(err, n, "assert");
            }
            TailStep step;
            step.nextExpr = n.body.get();
            step.ctx = ctx;
            return step;
        }
        case oscad::NodeKind::PrimaryCall: {
            auto& n = static_cast<const oscad::PrimaryCall&>(node);
            const oscad::Identifier* leftId = (n.left->kind() == oscad::NodeKind::Identifier)
                                                   ? static_cast<const oscad::Identifier*>(n.left.get())
                                                   : nullptr;

            // import/builtins: nothing evaluated yet (name-only checks),
            // safe to delegate to the existing evalFunctionCall wholesale
            // -- neither ever trampolines, matching upstream's own
            // BuiltinFunction early-return via a genuine call.
            if (leftId && (leftId->name == "import" || isBuiltinFunctionName(leftId->name))) {
                return Value{evalFunctionCall(n, ctx)};
            }

            if (leftId) {
                const oscad::ASTNode* declNode = ctx.scope->lookupFunction(leftId->name);
                if (declNode && declNode->kind() == oscad::NodeKind::FunctionDeclaration) {
                    const auto& decl = static_cast<const oscad::FunctionDeclaration&>(*declNode);
                    checkDebug(n, ctx); // call-site stop, per hop (see evalFunctionCall's)
                    const bool hasChunk = useBytecodeVm() && lookupOrCompileChunk(decl) != nullptr;
                    std::optional<TailStep> step = tryTailStepFor(leftId->name, decl, decl.parameters, *decl.expr,
                                                                   hasChunk, n.arguments, ctx, n.position());
                    if (step) return std::move(*step);
                    // Not eligible (closure-nested or compiled) -- a real
                    // call, same as evalFunctionCall would have made for
                    // this same resolved decl. n.left is never
                    // re-evaluated here (a plain Identifier lookup has no
                    // side effects to duplicate).
                    return Value{evalUserFunction(leftId->name, decl, n.arguments, ctx, &n)};
                }
            }

            // Function-literal *value* callee -- resolve n.left/leftId
            // exactly once (matching evalFunctionCall's own
            // warnIfUndef=false probe), then either trampoline or call it
            // directly. Never re-evaluate n.left below this point -- it
            // may itself be an arbitrary (side-effecting) expression.
            Value funcVal = leftId ? evalIdentifier(leftId->name, &leftId->position(), ctx, false) : evalExpr(*n.left, ctx);
            if (const auto* closurePtr = std::get_if<ClosurePtr>(&funcVal); closurePtr && *closurePtr) {
                const Closure& closure = **closurePtr;
                const oscad::FunctionLiteral& funcNode = *closure.node;
                checkDebug(n, ctx); // call-site stop, function-literal callee
                const bool hasChunk = useBytecodeVm() && lookupCompiledLiteralChunk(funcNode) != nullptr;
                std::optional<TailStep> step =
                    tryTailStepFor("<function literal>", funcNode, funcNode.parameters, *funcNode.body, hasChunk,
                                   n.arguments, ctx, n.position(), capturedLetTrail(closure));
                if (step) return std::move(*step);
                return Value{evalFunctionLiteral(closure, n.arguments, ctx, &n)};
            }

            if (leftId) warn("Ignoring unknown function '" + leftId->name + "'", &n.position());
            return Value{};
        }
        default:
            return NotTailStep{};
    }
}

Value Evaluator::evalFunctionBodyTrampoline(const oscad::Expression& bodyExpr, EvalContext& ctx) {
    const oscad::Expression* expr = &bodyExpr;
    // Every EvalContext this trampoline ever derives must stay alive for
    // its whole run, not just the currently-active one. scope_trail.hpp's
    // levels are normally kept reachable by the C++ call stack itself
    // (each nested call's own local EvalContext coexists with its
    // ancestors' until it returns) -- exactly what this trampoline
    // deliberately avoids growing. $-vars (dyn/dynExplicit) stay
    // dynamically scoped THROUGH even an isolated call (see callCtx()'s
    // own doc comment: dyn's isolate=false, unlike let_'s isolate=true),
    // so a later iteration may still need to walk back through a much
    // earlier one's dyn level to resolve a name. Reusing a single ctx
    // variable via plain assignment would drop the old value's shared_ptr
    // refcount to zero and pop that level (TrailView's custom deleter ->
    // ScopeTrailStorage::popLevel(), scope_trail.hpp) -- permanently
    // erasing its parent-chain link (and any bindings made at it) even
    // though a much later step still needs to see through it. Caught by
    // BytecodeCompiler.VmOffAndVmOnAgreeOnClosureCases turning up "undef"
    // for a name that should still resolve -- not a hypothetical.
    //
    // Trades the native C++ *stack* growth this trampoline exists to
    // avoid for heap growth instead: O(iteration count) EvalContext
    // objects (a handful of shared_ptrs each), bounded by available RAM
    // rather than a ~few-MB thread stack -- not literally O(1) memory,
    // but converts a hard crash into a working (if not minimal-memory)
    // computation. A future pass could shrink this further (only
    // dyn/dynExplicit genuinely need cross-call-boundary lifetime;
    // let_/dynPositions reset at every isolated call and could be
    // dropped there) if memory ever measurably matters more than this
    // simpler all-or-nothing approach.
    std::vector<EvalContext> chain{ctx};
    unsigned recursionGuard = 0;
    while (true) {
        auto result = simplifyTailStep(*expr, ctx);
        if (Value* v = std::get_if<Value>(&result)) return std::move(*v);
        if (std::holds_alternative<NotTailStep>(result)) return evalExpr(*expr, ctx);

        TailStep& step = std::get<TailStep>(result);
        expr = step.nextExpr;
        chain.push_back(std::move(step.ctx));
        ctx = chain.back();
        if (step.isNewLogicalCall) {
            recordTailCallHop(step.calleeName, *step.calleeDecl, step.callPos, recursionGuard);
        }
    }
}

void Evaluator::recordTailCallHop(const std::string& calleeName, const oscad::ASTNode& calleeDecl,
                                   const oscad::Position* callPos, unsigned& recursionGuard) {
    static constexpr unsigned kTcoIterationCap = 1000000;
    if (++recursionGuard > kTcoIterationCap) {
        error("Recursion detected calling function '" + calleeName + "'", calleeDecl);
    }
    CallStackFrame& frame = callStack_.back();
    frame.name = calleeName;
    frame.declNode = &calleeDecl;
    frame.declPosition = &calleeDecl.position();
    frame.callPosition = callPos;
    frame.upvalueParent = -1;
    profileRecordTailHop("function", calleeName, callPos, &calleeDecl.position());
}

Value Evaluator::evalUserFunction(const std::string& name, const oscad::FunctionDeclaration& decl,
                                   const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                                   const oscad::ASTNode* callNode) {
    const oscad::Scope* fnScope = decl.scope() ? decl.scope() : ctx.scope;
    const int callerFrameIdx = callStack_.empty() ? -1 : static_cast<int>(callStack_.size()) - 1;
    bool usedChildCtx = false;
    EvalContext childCtx = callCtxFor(decl, ctx, fnScope, nullptr, nullptr, &usedChildCtx);
    const int upvalueParent = usedChildCtx ? callerFrameIdx : -1;

    // The compiled path (Phase 1) replaces bindArgs+the bound-loop+
    // applyDefaults with slot-based binding entirely (runCompiledFunction
    // does its own, see bytecode_vm.cpp) -- nullptr (VM off, or this
    // declaration doesn't compile -- see tryCompileFunction) falls back to
    // the unchanged interpreter path below.
    const CompiledChunk* chunk = useBytecodeVm() ? lookupOrCompileChunk(decl) : nullptr;
    if (!chunk) {
        bindCallArgsInto(decl.parameters, bindArgs(decl.parameters, arguments, ctx), childCtx);
    }

    const oscad::Position* callPos = callNode ? &callNode->position() : nullptr;
    return evalUserFunctionCore(name, decl, *decl.expr, childCtx, callPos, upvalueParent, [&]() -> Value {
        return chunk ? runCompiledFunctionTrampoline(*this, *chunk, arguments, ctx, childCtx)
                      : evalFunctionBodyTrampoline(*decl.expr, childCtx);
    });
}

Value Evaluator::evalUserFunctionFromBound(const std::string& name, const oscad::FunctionDeclaration& decl,
                                            BoundArgs bound, EvalContext& ctx, const oscad::Position* callPos) {
    const oscad::Scope* fnScope = decl.scope() ? decl.scope() : ctx.scope;
    const int callerFrameIdx = callStack_.empty() ? -1 : static_cast<int>(callStack_.size()) - 1;
    bool usedChildCtx = false;
    EvalContext childCtx = callCtxFor(decl, ctx, fnScope, nullptr, nullptr, &usedChildCtx);
    const int upvalueParent = usedChildCtx ? callerFrameIdx : -1;
    const CompiledChunk* chunk = useBytecodeVm() ? lookupOrCompileChunk(decl) : nullptr;
    if (!chunk) {
        bindCallArgsInto(decl.parameters, std::move(bound), childCtx);
        return evalUserFunctionCore(name, decl, *decl.expr, childCtx, callPos, upvalueParent,
                                     [&]() -> Value { return evalFunctionBodyTrampoline(*decl.expr, childCtx); });
    }
    return evalUserFunctionCore(name, decl, *decl.expr, childCtx, callPos, upvalueParent, [&]() -> Value {
        return runCompiledFunctionFromBoundTrampoline(*this, *chunk, bound, childCtx);
    });
}

Value Evaluator::evalFunctionLiteral(const Closure& closure,
                                      const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                                      const oscad::ASTNode* callNode) {
    const oscad::FunctionLiteral& funcNode = *closure.node;
    const oscad::Scope* fnScope = funcNode.scope() ? funcNode.scope() : ctx.scope;
    const int callerFrameIdx = callStack_.empty() ? -1 : static_cast<int>(callStack_.size()) - 1;
    bool usedChildCtx = false;
    EvalContext childCtx =
        callCtxFor(funcNode, ctx, fnScope, nullptr, nullptr, &usedChildCtx, capturedLetTrail(closure));
    const int upvalueParent = usedChildCtx ? callerFrameIdx : -1;

    // A literal only ever has a compiled chunk if some OTHER declaration's
    // own compile discovered and compiled it as a nested FunctionLiteral
    // (see CompiledChunk::nestedLiterals / Evaluator::flattenNestedLiterals)
    // -- a bare top-level literal (never reached by any compiled container)
    // has no cache entry and always falls to the interpreter below,
    // unconditionally correct either way.
    const CompiledChunk* chunk = useBytecodeVm() ? lookupCompiledLiteralChunk(funcNode) : nullptr;
    if (!chunk) {
        bindCallArgsInto(funcNode.parameters, bindArgs(funcNode.parameters, arguments, ctx), childCtx);
    }

    const oscad::Position* callPos = callNode ? &callNode->position() : nullptr;
    // "<function literal>" -- the reference's call-stack entry uses the
    // call site's own callee-expression name when statically known (e.g.
    // `g(3)` for `g = function(x) ...`), which isn't threaded through here
    // (this evaluator doesn't track the binding name a literal happened to
    // be assigned to). Only affects TRACE-line text for an error raised
    // from inside a function-literal call; low value to thread through
    // further given no test depends on it.
    return evalUserFunctionCore("<function literal>", funcNode, *funcNode.body, childCtx, callPos, upvalueParent,
                                 [&]() -> Value {
                                     return chunk ? runCompiledFunctionTrampoline(*this, *chunk, arguments, ctx, childCtx)
                                                  : evalFunctionBodyTrampoline(*funcNode.body, childCtx);
                                 });
}

Value Evaluator::evalFunctionLiteralFromBound(const Closure& closure, BoundArgs bound,
                                               EvalContext& ctx, const oscad::Position* callPos) {
    const oscad::FunctionLiteral& funcNode = *closure.node;
    const oscad::Scope* fnScope = funcNode.scope() ? funcNode.scope() : ctx.scope;
    const int callerFrameIdx = callStack_.empty() ? -1 : static_cast<int>(callStack_.size()) - 1;
    bool usedChildCtx = false;
    EvalContext childCtx =
        callCtxFor(funcNode, ctx, fnScope, nullptr, nullptr, &usedChildCtx, capturedLetTrail(closure));
    const int upvalueParent = usedChildCtx ? callerFrameIdx : -1;
    const CompiledChunk* chunk = useBytecodeVm() ? lookupCompiledLiteralChunk(funcNode) : nullptr;
    if (!chunk) {
        bindCallArgsInto(funcNode.parameters, std::move(bound), childCtx);
        return evalUserFunctionCore(
            "<function literal>", funcNode, *funcNode.body, childCtx, callPos, upvalueParent,
            [&]() -> Value { return evalFunctionBodyTrampoline(*funcNode.body, childCtx); });
    }
    return evalUserFunctionCore("<function literal>", funcNode, *funcNode.body, childCtx, callPos, upvalueParent,
                                 [&]() -> Value { return runCompiledFunctionFromBoundTrampoline(*this, *chunk, bound, childCtx); });
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
            // Call-site stop, in the CALLER's context, before descending
            // into the callee (whose own body-entry stop comes later, from
            // evalUserFunctionCore). Builtins deliberately get none --
            // mirrors _eval_function_call, where only the user-function and
            // function-literal branches call _check_debug.
            checkDebug(node, ctx);
            return evalUserFunction(leftId->name, static_cast<const oscad::FunctionDeclaration&>(*decl), node.arguments,
                                     ctx, &node);
        }
    }

    // Not a builtin or user function -- check whether the callee expression
    // evaluates to a function-literal *value* (e.g. `g = function(x) x*2;
    // g(3)`). warnIfUndef=false: probing here shouldn't itself warn
    // "unknown variable" if `left` isn't bound to anything -- a genuinely
    // unknown callee gets exactly one warning below, not two.
    Value funcVal = leftId ? evalIdentifier(leftId->name, &leftId->position(), ctx, false) : evalExpr(*node.left, ctx);
    if (const auto* closurePtr = std::get_if<ClosurePtr>(&funcVal); closurePtr && *closurePtr) {
        checkDebug(node, ctx); // same call-site stop, function-literal callee
        return evalFunctionLiteral(**closurePtr, node.arguments, ctx, &node);
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
