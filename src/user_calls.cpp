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
    if (!it->second) return nullptr;
    // Compilation itself is cached forever above (never depends on
    // debugger state); ELIGIBILITY to actually use that compiled chunk
    // right now is re-checked every call instead -- see chunkEligibleNow's
    // own doc comment for why that's still cheap (no debugger: one bool
    // check; fast-continue mode: a handful of integer comparisons against
    // whatever breakpoints happen to be set right now).
    return chunkEligibleNow(*it->second) ? &*it->second : nullptr;
}

const CompiledChunk* Evaluator::lookupCompiledLiteralChunk(const oscad::FunctionLiteral& node) const {
    auto it = literalChunkCache_.find(&node);
    if (it == literalChunkCache_.end()) return nullptr;
    return chunkEligibleNow(it->second) ? &it->second : nullptr;
}

const CompiledChunk* Evaluator::lookupOrCompileModuleChunk(const oscad::ModuleDeclaration& decl) {
    auto it = moduleChunkCache_.find(&decl);
    if (it == moduleChunkCache_.end()) {
        it = moduleChunkCache_.emplace(&decl, tryCompileModuleBody(decl)).first;
        if (it->second) flattenNestedLiterals(*it->second);
    }
    if (!it->second) return nullptr;
    return chunkEligibleNow(*it->second) ? &*it->second : nullptr;
}

void Evaluator::flattenNestedLiterals(CompiledChunk& chunk) {
    for (auto& [litNode, litChunk] : chunk.nestedLiterals) {
        flattenNestedLiterals(litChunk);
        literalChunkCache_.emplace(litNode, std::move(litChunk));
    }
    chunk.nestedLiterals.clear();
}

Value Evaluator::evalExprMaybeCompiled(const oscad::Expression& node, EvalContext& ctx) {
    if (!useBytecodeVm() || !inResolvePass_) return evalExpr(node, ctx);
    auto it = stmtExprChunkCache_.find(&node);
    if (it == stmtExprChunkCache_.end()) {
        it = stmtExprChunkCache_.emplace(&node, tryCompileStatementExpr(node, node.scope())).first;
        // A zero-capture closure literal (e.g. `x = function(y) y + 1;`)
        // still reaches chunk.nestedLiterals even though it never touches
        // closureSites (see tryCompileStatementExpr's own doc comment: only
        // a CAPTURES-HAVING closure is refused) -- flatten it into
        // literalChunkCache_ exactly like lookupOrCompileChunk does, so a
        // later call to that closure value can run compiled too.
        if (it->second) flattenNestedLiterals(*it->second);
    }
    if (!it->second || !chunkEligibleNow(*it->second)) return evalExpr(node, ctx);
    return runCompiledExprChunk(*this, *it->second, ctx);
}

bool Evaluator::tryRunCompiledAssignmentBlock(const std::vector<const oscad::ASTNode*>& assignments,
                                               EvalContext& ctx) {
    if (assignments.empty() || !useBytecodeVm() || !inResolvePass_) return false;
    auto* first = static_cast<const oscad::Assignment*>(assignments.front());
    auto it = assignBlockChunkCache_.find(first);
    if (it == assignBlockChunkCache_.end()) {
        std::vector<const oscad::Assignment*> assigns;
        assigns.reserve(assignments.size());
        for (const oscad::ASTNode* n : assignments) assigns.push_back(static_cast<const oscad::Assignment*>(n));
        it = assignBlockChunkCache_.emplace(first, tryCompileAssignmentBlock(assigns, first->scope())).first;
        if (it->second) flattenNestedLiterals(*it->second);
    }
    if (!it->second || !chunkEligibleNow(*it->second)) return false;
    runCompiledAssignmentBlock(*this, *it->second, ctx);
    return true;
}

bool Evaluator::tryRunCompiledChildren(const std::vector<const oscad::ASTNode*>& children, EvalContext& ctx) {
    if (children.empty() || !useBytecodeVm() || !inResolvePass_) return false;
    const CompiledChunk* chunk = lookupOrCompileChildrenListChunk(children);
    if (!chunk) return false;
    runCompiledModuleBody(*this, *chunk, ctx);
    return true;
}

// The cache-lookup half of tryRunCompiledChildren, shared with
// Op::CallChildren's own runtime handler (bytecode_vm.cpp) -- both need
// exactly this "list -> eligible chunk or nullptr" step, differing only
// in how they then RUN the chunk (native runCompiledModuleBody reentry
// here, a direct vmCallStack_ push there). Caller is responsible for the
// useBytecodeVm()/inResolvePass_ gate (see childrenListChunkCache_'s own
// doc comment for why the pass gate is load-bearing, not defensive).
const CompiledChunk* Evaluator::lookupOrCompileChildrenListChunk(const std::vector<const oscad::ASTNode*>& children) {
    const oscad::ASTNode* first = children.front();
    const auto key = std::make_pair(first, children.size());
    auto it = childrenListChunkCache_.find(key);
    if (it == childrenListChunkCache_.end()) {
        it = childrenListChunkCache_.emplace(key, tryCompileChildrenList(children, first->scope())).first;
        if (it->second) flattenNestedLiterals(*it->second);
    }
    if (!it->second || !chunkEligibleNow(*it->second)) return nullptr;
    return &*it->second;
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
    // A REUSED (pooled) frame does NOT get VmFrame's own default member
    // initializers re-applied -- those only fire for a genuinely fresh
    // std::make_unique<VmFrame>() (acquireVmFrame's own empty-pool
    // fallback). Only pushBracketedModuleFrame ever sets ownsModuleSplice
    // true (and moduleRandsBefore/moduleSpliceCallNode alongside it); no
    // OTHER push site (pushBareFrame, pushBracketedCallFrame,
    // runCompiledFunction(FromBound), a parameter default's own frame)
    // ever resets it back to false. Without this, a frame released here
    // after owning a module's own splice, then reacquired later for an
    // ordinary FUNCTION call, keeps ownsModuleSplice=true -- if THAT
    // function call is later torn down by an exception (teardownVmCall
    // StackDownTo), its own `if (frame->ownsModuleSplice) treeStack_.
    // pop_back();` fires for a frame that never pushed a treeStack_ entry
    // of its own, silently popping one too many and corrupting treeStack_
    // for everything after. Caught for real via a heap-buffer-overflow
    // (ASan) inside teardownVmCallStackDownTo's own treeStack_.pop_back(),
    // triggered by BOSL2's poly_roots() -- a non-tail self-recursive
    // function, reusing a frame previously released by an earlier,
    // unrelated MODULE call -- throwing partway through.
    frame->ownsModuleSplice = false;
    frame->moduleRandsBefore = 0;
    frame->moduleSpliceCallNode = nullptr;
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
            result.set(a.name->name, evalExprMaybeCompiled(*a.expr, ctx));
        } else {
            auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
            if (positionalIdx < nparams) {
                result.set(params[positionalIdx]->name->name, evalExprMaybeCompiled(*a.expr, ctx));
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
    // Was a full callStack_ scan (every frame, checking THAT frame's own
    // declPosition for span containment) -- see activeDeclRefcount_'s own
    // doc comment (evaluator.hpp) for why iterating its DISTINCT keys
    // instead is exactly equivalent, not an approximation: containment for
    // a given declaration depends only on that declaration's own
    // (compile-time-fixed) position, never on which of its possibly-many
    // active occurrences on callStack_ is checked, so deduplicating by
    // declaration identity changes nothing observable -- only how many
    // checks a deeply/self-recursive call chain (the overwhelmingly common
    // shape once Stage 1/2 made deep recursion possible at all) needs to
    // do: a small, roughly-constant number of DISTINCT active
    // declarations, not one check per active call.
    const oscad::Position& declPos = decl.position();
    for (const auto& [activeDecl, count] : activeDeclRefcount_) {
        const oscad::Position* outer = &activeDecl->position();
        if (outer->origin != declPos.origin) continue;
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
    // `declNode.scope()`, not `ctx.scope` -- matches every other call-
    // resolution function's own `fnScope = decl.scope() ? decl.scope() :
    // ctx.scope` pattern (evalUserFunction/evalUserFunctionFromBound/
    // evalFunctionLiteral/evalFunctionLiteralFromBound). Mismatching here
    // was invisible until a captures-having FunctionLiteral's own body
    // could ever actually be trampolined into (Op::MakeClosure's own
    // registration -- bytecode_compiler.cpp's FunctionLiteral case): a
    // self-referential closure's free-variable resolution can fall through
    // to evalIdentifier's ctx.scope->lookupVariable() fallback (see its own
    // doc comment, expr_eval.cpp), which needs the CALLEE's own lexical
    // scope to walk outward from -- the caller's scope at the call site
    // has no relation to it at all.
    bool usedChildCtx = false;
    const oscad::Scope* declScope = declNode.scope() ? declNode.scope() : ctx.scope;
    EvalContext result = callCtxFor(declNode, ctx, declScope, nullptr, nullptr, &usedChildCtx, capturedLet);
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

EvalContext Evaluator::buildModuleChildCtx(const oscad::ModuleDeclaration& decl, const oscad::ModularCall& call,
                                            EvalContext& ctx, BoundArgs bound) {
    const oscad::Scope* childScope = decl.scope() ? decl.scope() : ctx.scope;

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

    // moduleCallDepth_ (maintained incrementally by enterUserCall/
    // exitUserCall*) is already exactly "how many Module frames are on
    // callStack_ right now" -- this call's OWN frame isn't pushed yet, so
    // it correctly counts only ancestors, matching what a callStack_ scan
    // would have found. See moduleCallDepth_'s own doc comment for why a
    // scan here specifically used to be an O(depth) hazard.
    childCtx.dyn->set("$parent_modules", Value{static_cast<double>(moduleCallDepth_)});
    return childCtx;
}

void Evaluator::runModuleBodyNative(const oscad::ModuleDeclaration& decl, EvalContext& childCtx,
                                     const oscad::Position* callPos) {
    // Guards against a genuinely (non-tail) recursive user module the same
    // way evalUserFunctionCore's own guard does for functions (evaluator.hpp)
    // -- each logical module call HERE is a real, unavoidable C++ recursive
    // call (evalModularCall -> evalUserModule -> evalChildren ->
    // evalStatement -> evalModularCall -> ...), so a deep enough one
    // silently overflows the native stack with no exception to catch.
    // Confirmed present for a plain, closure-free `module recur(n) { if
    // (n>0) recur(n-1); else cube(1); }` -- segfaults around depth 5000 on
    // an ordinary desktop build. Recursive tree/pattern-generator modules
    // are a real, common pattern (not just an adversarial script), so this
    // is reachable in practice. Shares kMaxUserCallDepth AND callStack_
    // itself with the function-side guard: a function recursing into a
    // module recursing into a function is the same hazard regardless of
    // which kind of frame is on top. This guard -- unlike the compiled
    // path's own (see pushBracketedModuleFrame, bytecode_vm.cpp,
    // skipDepthGuard) -- stays exactly as-is: this IS still a genuine
    // native recursive call, so native stack margin still matters here.
    // Uses enterUserCall/exitUserCall* (not a hand-rolled profileEnter/
    // callStack_.push_back) so this bracket is identical in shape to the
    // compiled path's own (evalUserModule, below) -- one shared mechanism,
    // not two that could drift.
    UserCallHandle h = enterUserCall(decl.name->name, decl, /*bodyExpr=*/nullptr, childCtx, callPos,
                                       /*upvalueParent=*/-1, /*skipDepthGuard=*/false, CallStackFrame::Kind::Module);
    try {
        evalChildren(decl.children, childCtx);
    } catch (...) {
        exitUserCallException(h);
        throw;
    }
    exitUserCallSuccess(decl.name->name, h, Value{}, /*fireReturnHook=*/false);
}

void Evaluator::evalUserModule(const oscad::ModuleDeclaration& decl, const oscad::ModularCall& call, EvalContext& ctx) {
    auto bound = bindArgs(decl.parameters, call.arguments, ctx);
    EvalContext childCtx = buildModuleChildCtx(decl, call, ctx, std::move(bound));
    const CompiledChunk* chunk = useBytecodeVm() ? lookupOrCompileModuleChunk(decl) : nullptr;
    if (!chunk) {
        runModuleBodyNative(decl, childCtx, &call.position());
        return;
    }
    // Unlike runModuleBodyNative, this bracket wraps runCompiledModuleBody
    // instead of evalChildren -- but it's still REQUIRED here, even though
    // runCompiledModuleBody's own frame is bare (see its own doc comment):
    // this call site IS the "outer caller" that bare frame assumes already
    // did the bracketing (mirrors evalUserFunctionCore's own unconditional
    // enterUserCall wrapping BOTH its compiled and native branches
    // uniformly). Skipping it here was a real bug caught by
    // UserModule.NestedModuleSeesReassignedParameterViaClosure hanging --
    // without `decl`'s own CallStackFrame on callStack_, callCtxFor's own
    // span-containment closure search (used by a module declared INSIDE
    // this one's body) can never find it, silently misclassifying every
    // such lookup.
    UserCallHandle h = enterUserCall(decl.name->name, decl, /*bodyExpr=*/nullptr, childCtx, &call.position(),
                                       /*upvalueParent=*/-1, /*skipDepthGuard=*/false, CallStackFrame::Kind::Module);
    try {
        runCompiledModuleBody(*this, *chunk, childCtx);
    } catch (...) {
        exitUserCallException(h);
        throw;
    }
    exitUserCallSuccess(decl.name->name, h, Value{}, /*fireReturnHook=*/false);
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
    // Tears `chain` down back-to-front on EVERY exit path -- normal return
    // OR exception unwind (recordTailCallHop's own recursion-guard error,
    // below, is the exact case that matters: an infinite tail-recursive
    // function with no base case throws out of this loop entirely,
    // bypassing any teardown code placed after it). Relying on `chain`
    // simply falling out of scope instead is NOT equivalent: std::vector<T>'s
    // own element-destruction order is unspecified by the standard --
    // libc++ destroys back-to-front (matching ScopeTrailStorage::popLevel's
    // own "scan from the back" doc comment, which assumes the level being
    // popped is usually the most recently pushed one -- O(1) per pop that
    // way), but libstdc++ destroys front-to-back, the adversarial order for
    // that same scan -- each pop then walks almost the entire remaining
    // vector, turning an intended O(N) teardown into a real, measured
    // O(N^2) (see issue #50: an N-scaling experiment plus a minimal repro
    // proving libc++/libstdc++ disagree on std::vector<T>::clear()'s
    // destruction order for identical source). Popping back-to-front
    // explicitly, via a destructor that always runs before `chain`'s own,
    // is correct AND O(N) on every standard library and every exit path,
    // not just the ones that happen to agree with libc++ and return
    // normally.
    struct ChainTeardown {
        std::vector<EvalContext>& chain;
        ~ChainTeardown() {
            while (!chain.empty()) chain.pop_back();
        }
    } teardown{chain};
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
    // A tail hop swaps which declaration occupies THIS stack slot without
    // a matching enter/exit pair of its own -- activeDeclRefcount_ (see
    // its own doc comment, evaluator.hpp) has to be updated symmetrically
    // here too, or it would count the OLD declaration as active forever
    // and never see the NEW one at all.
    noteActiveDeclExit(frame.declNode);
    ++activeDeclRefcount_[&calleeDecl];
    frame.name = calleeName;
    frame.declNode = &calleeDecl;
    frame.declPosition = &calleeDecl.position();
    frame.callPosition = callPos;
    frame.upvalueParent = -1;
    profileRecordTailHop("function", calleeName, callPos, &calleeDecl.position());
}

// See these three methods' own doc comment (evaluator.hpp) for the split
// rationale. Body is a direct, behavior-preserving lift of what
// evalUserFunctionCore used to inline -- see this file's own git history
// for the pre-split version if a byte-for-byte diff is ever needed.
Evaluator::UserCallHandle Evaluator::enterUserCall(const std::string& name, const oscad::ASTNode& declNode,
                                                    const oscad::Expression* bodyExpr, EvalContext& childCtx,
                                                    const oscad::Position* callPos, int upvalueParent,
                                                    bool skipDepthGuard, CallStackFrame::Kind kind) {
    const bool isModule = kind == CallStackFrame::Kind::Module;
    // Fixed count, not nativeStackMarginLow() -- see kMaxDriveVmNativeDepth's
    // own doc comment (evaluator.hpp, bytecode_vm.cpp's identical guard)
    // for the full reasoning: the margin-based check was needed only for
    // a native-reentry pattern (BOSL2's attachable() chain) Op::
    // PushBuiltinWrap has since eliminated the native reentry for
    // entirely; for what's left, the margin check is the proven-UNSAFE
    // mechanism on Windows (confirmed via real CI), this fixed count the
    // proven-safe one.
    //
    // Checked against nativeUserCallDepth_, NOT callStack_.size() -- see
    // that field's own doc comment (evaluator.hpp) for why: callStack_
    // also grows from cheap, zero-native-cost skipDepthGuard=true pushes,
    // which must not count toward this native-stack-safety ceiling.
    if (!skipDepthGuard && nativeUserCallDepth_ >= kMaxUserCallDepth) {
        error("Recursion too deep while calling " + std::string(isModule ? "module" : "function") + " '" + name + "'",
              declNode);
    }
    UserCallHandle h;
    h.kind = kind;
    h.declNode = &declNode;
    if (!skipDepthGuard) {
        ++nativeUserCallDepth_;
        h.countedTowardNativeDepth = true;
    }
    h.prof = profileEnter(isModule ? "module" : "function", name, callPos, &declNode.position());
    callStack_.push_back(CallStackFrame{kind, name, callPos, &declNode.position(), &declNode, nullptr, upvalueParent});
    callStack_.back().bodyCtx = &childCtx; // per-frame locals for the debugger
    if (isModule) ++moduleCallDepth_;
    ++activeDeclRefcount_[&declNode];
    if (bodyExpr) checkDebug(*bodyExpr, childCtx);
    lastCtx_ = &childCtx;
    return h;
}

void Evaluator::noteActiveDeclExit(const oscad::ASTNode* declNode) {
    auto it = activeDeclRefcount_.find(declNode);
    if (it == activeDeclRefcount_.end()) return; // defensive; should always be found
    if (--it->second <= 0) activeDeclRefcount_.erase(it);
}

void Evaluator::exitUserCallSuccess(const std::string& name, const UserCallHandle& handle, const Value& result,
                                     bool fireReturnHook) {
    if (fireReturnHook && debugHooks_.returnHook) debugHooks_.returnHook(name, result, static_cast<int>(callStack_.size()));
    callStack_.pop_back();
    if (handle.kind == CallStackFrame::Kind::Module) --moduleCallDepth_;
    if (handle.countedTowardNativeDepth) --nativeUserCallDepth_;
    noteActiveDeclExit(handle.declNode);
    if (handle.prof) profileExit(*handle.prof);
}

void Evaluator::exitUserCallException(const UserCallHandle& handle) {
    callStack_.pop_back();
    if (handle.kind == CallStackFrame::Kind::Module) --moduleCallDepth_;
    if (handle.countedTowardNativeDepth) --nativeUserCallDepth_;
    noteActiveDeclExit(handle.declNode);
    if (handle.prof) profileExit(*handle.prof);
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
        return chunk ? runCompiledFunction(*this, *chunk, arguments, ctx, childCtx)
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
        return runCompiledFunctionFromBound(*this, *chunk, bound, childCtx);
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
                                     return chunk ? runCompiledFunction(*this, *chunk, arguments, ctx, childCtx)
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
                                 [&]() -> Value { return runCompiledFunctionFromBound(*this, *chunk, bound, childCtx); });
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

std::optional<Evaluator::ChildrenForward> Evaluator::prepareChildrenForward(const CallArgs& args, EvalContext& ctx) {
    Value idxArg = getArg(args, 0, "index", Value{});
    if (!ctx.childrenNodes || ctx.childrenNodes->empty()) return std::nullopt;
    const EvalContext* callerCtx = ctx.childrenCallerCtx;
    if (!callerCtx) return std::nullopt;

    // A children() forwarding chain's own dyn/let_/etc. must alias the
    // *caller's* (not this ctx's) -- see EvalContext::withScope's rationale.
    EvalContext evalCtx = callerCtx->childCtx(nullptr, std::nullopt, callerCtx->childrenNodes, callerCtx->childrenCallerCtx);
    // `ctx` here is the post-resolveCallArgs effCtx, deliberately: a
    // `children($fn=12)`-style named-$ override lives only at effCtx's own
    // trail level, and TrailView::items() is ancestry-visible, so reading
    // from effCtx forwards both the call's own overrides AND everything
    // the wrapper module's body set (`$fn = 100; children();`).
    for (const auto& [k, v] : ctx.dyn->items()) {
        if (!k.empty() && k[0] == '$') evalCtx.dyn->set(k, v);
    }
    for (const auto& [k, v] : ctx.let_->items()) {
        if (!k.empty() && k[0] == '$') evalCtx.let_->set(k, v);
    }

    if (std::holds_alternative<std::monostate>(idxArg)) {
        return ChildrenForward{std::move(evalCtx), *ctx.childrenNodes};
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
    if (idx < 0 || static_cast<size_t>(idx) >= geoNodes.size()) return std::nullopt;
    return ChildrenForward{std::move(evalCtx), {geoNodes[static_cast<size_t>(idx)]}};
}

void Evaluator::builtinChildren(const CallArgs& args, EvalContext& ctx) {
    std::optional<ChildrenForward> fwd = prepareChildrenForward(args, ctx);
    if (!fwd) return;
    evalChildren(fwd->nodes, fwd->evalCtx);
}

} // namespace oscadeval
