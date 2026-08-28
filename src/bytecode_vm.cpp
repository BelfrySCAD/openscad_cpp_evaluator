#include <cassert>
#include "openscad_cpp_evaluator/bytecode_vm.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/function_builtins.hpp"
#include "openscad_cpp_evaluator/import_builtin.hpp"
#include "openscad_cpp_evaluator/scope_trail.hpp"

#include "openscad_cpp_parser/ast/declarations.hpp"
#include "openscad_cpp_parser/ast/module_instantiation.hpp"

#include "builtins/builtins.hpp"

#include <algorithm>

namespace oscadeval {

namespace {

// Shared by Op::CallFn's isBuiltin (evalBuiltinFunction) and isImport
// (importAsValue) branches -- both take the same pre-resolved CallArgs
// shape resolveArgs would build for the interpreter path. `args` is
// consumed (each element moved out).
CallArgs buildCallArgs(const CompiledChunk::CallSite& site, std::vector<Value>& args, size_t argCount) {
    CallArgs callArgs;
    int positionalIdx = 0;
    for (size_t i = 0; i < argCount; ++i) {
        if (site.argNames[i]) {
            callArgs.named.emplace_back(*site.argNames[i], std::move(args[i]));
        } else {
            callArgs.positional.emplace_back(positionalIdx++, std::move(args[i]));
        }
    }
    return callArgs;
}

// Replays bindArgs' own positional/named matching rule (positional index
// counted only among positional arguments, later write for a repeated name
// wins) against already-evaluated `args`, for a callee whose own parameter
// LIST is `paramNames` (in declared order -- a FunctionDeclaration's
// parameters or a FunctionLiteral's). Shared by CallFn/CallFnTail (a
// site.decl callee) and CallDynamic/CallDynamicTail (a closure callee).
BoundArgs buildBoundArgs(Evaluator& ev, const CompiledChunk::CallSite& site, std::vector<Value>& args, size_t argCount,
                          const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& paramNames) {
    BoundArgs bound;
    bound.reserve(argCount);
    size_t positionalIdx = 0;
    const size_t nparams = paramNames.size();
    const oscad::Position* pos = site.callNode ? &site.callNode->position() : nullptr;
    for (size_t i = 0; i < argCount; ++i) {
        if (site.argNames[i]) {
            const std::string& name = *site.argNames[i];
            if (!isConfigVariable(name) && !declaresParam(paramNames, name)) {
                warnUnexpectedNamedArg(ev, name, pos);
            }
            bound.set(name, std::move(args[i]));
        } else {
            if (positionalIdx < nparams) {
                bound.set(paramNames[positionalIdx]->name->name, std::move(args[i]));
            } else if (positionalIdx == nparams) {
                warnTooManyPositionalArgs(ev, pos);
            }
            ++positionalIdx;
        }
    }
    return bound;
}

// Binds `bound`'s entries into `frame`'s own slots (matching
// CompiledChunk::Param::name) or, for a declared $-parameter, straight
// into `frame.ctxChain.back().dyn` -- see CompiledChunk::Param::isDyn's
// own doc comment (bytecode.hpp). An undeclared $-named entry is still a
// dynamic-scope override, routed to dyn too; an undeclared plain name has
// no slot and is simply unreferenceable (ponytail: nothing downstream of a
// compiled call ever reads ctx.let_ for a name a compiled body didn't
// itself resolve to a slot, so there's nothing to write there either).
void bindBoundArgsIntoFrame(const CompiledChunk& chunk, const BoundArgs& bound, VmFrame& frame) {
    EvalContext& ctx = frame.ctxChain.back();
    for (size_t i = 0; i < chunk.params.size(); ++i) {
        const CompiledChunk::Param& p = chunk.params[i];
        if (const Value* v = bound.find(p.name)) {
            if (p.isDyn) {
                ctx.dyn->set(p.name, *v);
            } else {
                frame.slots[static_cast<size_t>(p.slot)] = *v;
            }
            frame.bound[i] = true;
        }
    }
    for (const auto& [k, v] : bound) {
        bool matched = false;
        for (const auto& p : chunk.params) {
            if (p.name == k) {
                matched = true;
                break;
            }
        }
        if (!matched && !k.empty() && k[0] == '$') ctx.dyn->set(k, v);
    }
}

// Same, but for a call site whose arguments are still raw AST (evaluated
// here against `callerCtx`) -- only ever used by runCompiledFunction's own
// entry point (a top-level call with raw AST arguments, mirroring
// evalUserFunction's own bindArgs+bound-loop). The Op::CallFn-family
// opcodes always have already-EVALUATED arguments by the time they resolve
// a callee (popped off the caller frame's own operand stack), so they go
// through buildBoundArgs+bindBoundArgsIntoFrame above instead.
void bindAstArgsIntoFrame(Evaluator& ev, const CompiledChunk& chunk,
                          const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& callerCtx,
                          VmFrame& frame) {
    EvalContext& ctx = frame.ctxChain.back();
    size_t positionalIdx = 0;
    const size_t nparams = chunk.params.size();
    for (const auto& argPtr : arguments) {
        if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
            Value v = ev.evalExpr(*a.expr, callerCtx);
            const std::string& name = a.name->name;
            bool matched = false;
            for (size_t i = 0; i < nparams; ++i) {
                if (chunk.params[i].name == name) {
                    if (chunk.params[i].isDyn) {
                        ctx.dyn->set(name, std::move(v));
                    } else {
                        frame.slots[static_cast<size_t>(chunk.params[i].slot)] = std::move(v);
                    }
                    frame.bound[i] = true;
                    matched = true;
                    break;
                }
            }
            if (!matched && !name.empty() && name[0] == '$') {
                ctx.dyn->set(name, v);
            }
            if (!matched && !isConfigVariable(name)) {
                warnUnexpectedNamedArg(ev, name, &argPtr->position());
            }
        } else {
            auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
            Value v = ev.evalExpr(*a.expr, callerCtx);
            if (positionalIdx < nparams) {
                const CompiledChunk::Param& p = chunk.params[positionalIdx];
                if (p.isDyn) {
                    ctx.dyn->set(p.name, std::move(v));
                } else {
                    frame.slots[static_cast<size_t>(p.slot)] = std::move(v);
                }
                frame.bound[positionalIdx] = true;
            } else if (positionalIdx == nparams) {
                warnTooManyPositionalArgs(ev, &argPtr->position());
            }
            ++positionalIdx;
        }
    }
}

Value driveVm(Evaluator& ev, size_t floor);

// Fills in any parameter left unbound after bindBoundArgsIntoFrame/
// bindAstArgsIntoFrame from its own default-value expression
// (chunk.defaultCode[i]) -- mirrors applyDefaults' "declaring a parameter
// always creates a fresh binding" rule exactly, same as before this
// redesign. A default's own code is driven through the SAME explicit-
// frame-stack mechanism as everything else (a bare, unbracketed push+
// drive down to a local floor) rather than a separate execution path --
// so a call made FROM a default value (rare, but not disallowed by the
// grammar) still gets the full no-native-recursion treatment, not just
// function bodies specifically.
void applyCompiledDefaultsToFrame(Evaluator& ev, const CompiledChunk& chunk, VmFrame& frame) {
    for (size_t i = 0; i < chunk.params.size(); ++i) {
        if (frame.bound[i]) continue;
        const CompiledChunk::Param& p = chunk.params[i];
        Value v;
        if (!chunk.defaultCode[i].empty()) {
            auto defaultFrame = ev.acquireVmFrame();
            defaultFrame->chunk = &chunk;
            defaultFrame->code = &chunk.defaultCode[i];
            defaultFrame->pc = 0;
            defaultFrame->slots = frame.slots; // defaults may read earlier SIBLING slots? no -- compiled with an
                                                 // isolated (zero-frame) scope, see bytecode_compiler.cpp's
                                                 // compileFunctionLike -- copied anyway since Op::LoadLocal
                                                 // addresses this SAME chunk's slot numbering and a default's own
                                                 // compiled body never references one (isolated scope guarantees
                                                 // no LoadLocal survives compilation for a default), so this is
                                                 // just "big enough," not "meaningfully shared."
            defaultFrame->stack.clear();
            defaultFrame->bound.assign(chunk.params.size(), false);
            defaultFrame->accumStack.clear();
            defaultFrame->iterLists.assign(static_cast<size_t>(chunk.numIterLists), IterList{});
            defaultFrame->ctxChain.clear();
            defaultFrame->ctxChain.push_back(frame.ctxChain.back());
            defaultFrame->tailHopGuard = 0;
            defaultFrame->hopEligible = false; // call-boundary-free, see VmFrame::hopEligible
            const size_t floor = ev.vmCallStack_.size();
            ev.vmCallStack_.push_back(std::move(defaultFrame));
            ev.vmCallBrackets_.emplace_back(std::nullopt);
            v = driveVm(ev, floor);
        }
        if (p.isDyn) {
            frame.ctxChain.back().dyn->set(p.name, std::move(v));
        } else {
            frame.slots[static_cast<size_t>(p.slot)] = std::move(v);
        }
    }
}

// Constructs (from the pool) and pushes a BARE frame -- no callStack_/
// profiling bracket, see VmFrame's own doc comment (bytecode_vm.hpp) for
// exactly which call shapes this applies to.
void pushBareFrame(Evaluator& ev, const CompiledChunk& chunk, const std::vector<Instruction>& code,
                    EvalContext ctx) {
    auto frame = ev.acquireVmFrame();
    frame->chunk = &chunk;
    frame->code = &code;
    frame->pc = 0;
    frame->slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame->stack.clear();
    frame->bound.assign(chunk.params.size(), false);
    frame->accumStack.clear();
    frame->iterLists.assign(static_cast<size_t>(chunk.numIterLists), IterList{});
    frame->ctxChain.clear();
    frame->ctxChain.push_back(std::move(ctx));
    frame->tailHopGuard = 0;
    frame->logicalName.clear();
    frame->hopEligible = false; // both callers (runCompiledExprChunk/
                                 // runCompiledAssignmentBlock) are genuinely
                                 // call-boundary-free -- see VmFrame::hopEligible.
    ev.vmCallStack_.push_back(std::move(frame));
    ev.vmCallBrackets_.emplace_back(std::nullopt);
}

// Constructs and pushes a BRACKETED frame -- a genuine new logical call,
// exactly mirroring what evalUserFunctionFromBound/
// evalFunctionLiteralFromBound used to do via a native recursive call
// (callCtxFor-derived childCtx, upvalueParent computed from
// usedChildCtx, enterUserCall for the callStack_/profiling/checkDebug
// bracket) -- just pushed onto the explicit stack instead of blocking a
// native C++ frame. `declNode`/`bodyExpr` mirror evalUserFunctionCore's
// own parameters exactly (a FunctionDeclaration's decl/*decl.expr, or a
// FunctionLiteral's funcNode/*funcNode.body).
void pushBracketedCallFrame(Evaluator& ev, const CompiledChunk& chunk, const oscad::ASTNode& declNode,
                             const oscad::Expression& bodyExpr, const std::string& name, BoundArgs bound,
                             EvalContext& callerCtx, const oscad::Scope* fnScope,
                             const std::shared_ptr<TrailView<Value>>& capturedLet, const oscad::Position* callPos) {
    if (ev.vmCallStack_.size() >= Evaluator::kMaxVmCallStackDepth) {
        ev.error("Recursion too deep while calling function '" + name + "'", declNode);
    }
    const int callerFrameIdx = ev.callStackTopIndex();
    bool usedChildCtx = false;
    EvalContext childCtx = ev.callCtxFor(declNode, callerCtx, fnScope, nullptr, nullptr, &usedChildCtx, capturedLet);
    const int upvalueParent = usedChildCtx ? callerFrameIdx : -1;

    auto frame = ev.acquireVmFrame();
    frame->chunk = &chunk;
    frame->code = &chunk.bodyCode;
    frame->pc = 0;
    frame->slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame->stack.clear();
    frame->bound.assign(chunk.params.size(), false);
    frame->accumStack.clear();
    frame->iterLists.assign(static_cast<size_t>(chunk.numIterLists), IterList{});
    frame->ctxChain.clear();
    frame->ctxChain.push_back(std::move(childCtx));
    frame->tailHopGuard = 0;
    frame->logicalName = name;
    frame->hopEligible = true; // just pushed a fresh, correctly-named callStack_ entry
    bindBoundArgsIntoFrame(chunk, bound, *frame);
    applyCompiledDefaultsToFrame(ev, chunk, *frame);

    // enterUserCall's own bodyCtx must point at frame->ctxChain's OWN
    // storage (stable on two counts: this VmFrame lives behind a
    // unique_ptr, so its address never moves even if vmCallStack_ itself
    // reallocates, AND ctxChain is a std::deque, so a later push onto it
    // never moves the element handed over here -- see its own doc comment,
    // bytecode_vm.hpp, for the segfault a vector caused), not
    // a plain local -- a local `childCtx` here would be destroyed the
    // instant this function returns, but CallStackFrame::bodyCtx is read
    // LATER, by ANY subsequent checkDebug() call (this call's own nested
    // calls, or an unrelated sibling's) that walks the WHOLE call stack
    // to build per-frame debugger locals (Evaluator::buildDebugFrames) --
    // reading a dangling pointer there segfaults. Real bug, caught via a
    // targeted ASan build reproducing BelfrySCAD's own "step over a call,
    // but honor a breakpoint set inside it" debug scenario: pausing deep
    // inside a native (interpreter-forced, due to the breakpoint) callee
    // walks the WHOLE call stack, including this compiled caller's own
    // now-stale frame.
    Evaluator::UserCallHandle handle = ev.enterUserCall(name, declNode, &bodyExpr, frame->ctxChain.back(), callPos,
                                                          upvalueParent, /*skipDepthGuard=*/true);
    ev.vmCallStack_.push_back(std::move(frame));
    ev.vmCallBrackets_.push_back(std::move(handle));
}

// Constructs and pushes a BRACKETED MODULE frame -- the module-side analog
// of pushBracketedCallFrame, used only by Op::CallModule for a NESTED
// module call made from within an already-compiled module body. Unlike a
// function call, there's no BoundArgs/defaults to bind into slots here --
// `childCtx` arrives already fully prepared (see Evaluator::
// buildModuleChildCtx, called by Op::CallModule before this runs): a
// module's own parameters were never meant to be slot-addressed in this
// design, only its OWN local variable/for-loop-variable reads inside the
// compiled body go through LoadFree/LoadDyn against ctx, exactly like a
// bare statement-context expression's do. `randsBefore`/`callNode` are
// stashed on the frame itself (moduleRandsBefore/moduleSpliceCallNode) so
// driveVm's own completion branch can run Evaluator::spliceModuleChildren
// once this frame finishes, without needing anywhere else to remember
// them across the (possibly long) run this frame is about to make.
void pushBracketedModuleFrame(Evaluator& ev, const CompiledChunk& chunk, const oscad::ModuleDeclaration& decl,
                               EvalContext& childCtx, const oscad::Position* callPos, std::uint64_t randsBefore,
                               const oscad::ASTNode& callNode) {
    if (ev.vmCallStack_.size() >= Evaluator::kMaxVmCallStackDepth) {
        ev.error("Recursion too deep while calling module '" + decl.name->name + "'", decl);
    }
    auto frame = ev.acquireVmFrame();
    frame->chunk = &chunk;
    frame->code = &chunk.bodyCode;
    frame->pc = 0;
    // See runCompiledModuleBody's own doc comment (below) for why this is
    // .assign(numSlots) now, not .clear() -- a nested let-expression
    // inside a compiled echo/assert/assignment argument can need local
    // slots even though module parameters themselves never do.
    frame->slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame->stack.clear();
    frame->bound.clear();
    frame->accumStack.clear();
    frame->iterLists.assign(static_cast<size_t>(chunk.numIterLists), IterList{});
    frame->ctxChain.clear();
    frame->ctxChain.push_back(std::move(childCtx));
    frame->tailHopGuard = 0;
    frame->logicalName = decl.name->name;
    frame->ownsModuleSplice = true;
    // enterUserCall's own bodyCtx must point at frame->ctxChain's OWN
    // (deque-stable) storage, not the caller's own (about-to-be-destroyed) local
    // `childCtx` -- see pushBracketedCallFrame's own doc comment, above,
    // for the full dangling-pointer hazard this avoids (same fix, same
    // root cause, module side).
    Evaluator::UserCallHandle handle = ev.enterUserCall(decl.name->name, decl, /*bodyExpr=*/nullptr,
                                                          frame->ctxChain.back(), callPos,
                                                          /*upvalueParent=*/-1, /*skipDepthGuard=*/true,
                                                          CallStackFrame::Kind::Module);
    frame->moduleRandsBefore = randsBefore;
    frame->moduleSpliceCallNode = &callNode;
    ev.vmCallStack_.push_back(std::move(frame));
    ev.vmCallBrackets_.push_back(std::move(handle));
}

// Op::CallChildren's own push -- a THIRD frame shape, distinct from both
// existing helpers: splice-owning like pushBracketedModuleFrame's
// (ownsModuleSplice=true, mirroring evalModularCall's own unconditional
// splice branch for "children", csg_resolve.cpp) but BRACKETLESS like
// pushBareFrame's (nullopt in vmCallBrackets_ -- children() never gets a
// callStack_/profiling entry natively either: only enterUserCall pushes
// those, and resolveChildren/builtinChildren never call it, so no TRACE
// frame, no profile site, no $parent_modules bump, exactly matching the
// native path). driveVm's completion branch and teardownVmCallStackDownTo
// both already handle this combination -- their bracket (`if (bracket)`)
// and splice (`if (ownsModuleSplice)`) concerns are independent at both
// sites. `evalCtx` is prepareChildrenForward's result (the caller-derived,
// $-forwarded context the children must run under -- see that helper's
// own doc comment, evaluator.hpp). hopEligible set explicitly false --
// pushBracketedModuleFrame itself omits the reset (benign there only
// because module chunks never contain tail-call opcodes); don't inherit
// a pooled function frame's stale true here either.
void pushChildrenForwardFrame(Evaluator& ev, const CompiledChunk& chunk, EvalContext evalCtx,
                               std::uint64_t randsBefore, const oscad::ASTNode& callNode) {
    if (ev.vmCallStack_.size() >= Evaluator::kMaxVmCallStackDepth) {
        ev.error("Recursion too deep while forwarding children()", callNode);
    }
    auto frame = ev.acquireVmFrame();
    frame->chunk = &chunk;
    frame->code = &chunk.bodyCode;
    frame->pc = 0;
    frame->slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame->stack.clear();
    frame->bound.clear();
    frame->accumStack.clear();
    frame->iterLists.assign(static_cast<size_t>(chunk.numIterLists), IterList{});
    frame->ctxChain.clear();
    frame->ctxChain.push_back(std::move(evalCtx));
    frame->tailHopGuard = 0;
    frame->logicalName.clear();
    frame->hopEligible = false;
    frame->ownsModuleSplice = true;
    frame->moduleRandsBefore = randsBefore;
    frame->moduleSpliceCallNode = &callNode;
    ev.vmCallStack_.push_back(std::move(frame));
    ev.vmCallBrackets_.emplace_back(std::nullopt);
}

// Tears down every frame from vmCallStack_'s own top down to (but not
// including) `floor`, on the exception path -- releases each VmFrame to
// the pool, tears down its ctxChain back-to-front (never relying on
// std::vector's own unspecified destruction order -- see VmFrame's own
// doc comment), and closes out its callStack_/profiling bracket if it had
// one, deliberately WITHOUT firing returnHook (matches
// Evaluator::exitUserCallException's own "only fires on the success path"
// contract). Direct generalization of the pre-redesign ChainTeardown, from
// "one trampoline's own ctx chain" to "the whole explicit call stack."
void teardownVmCallStackDownTo(Evaluator& ev, size_t floor) {
    while (ev.vmCallStack_.size() > floor) {
        std::unique_ptr<VmFrame> frame = std::move(ev.vmCallStack_.back());
        ev.vmCallStack_.pop_back();
        std::optional<Evaluator::UserCallHandle> bracket = std::move(ev.vmCallBrackets_.back());
        ev.vmCallBrackets_.pop_back();
        while (!frame->ctxChain.empty()) frame->ctxChain.pop_back();
        // Any still-open Op::PushBuiltinWrap bracket(s) this frame's own
        // execution left behind -- each pushed its own treeStack_
        // accumulator (Op::PushBuiltinWrap's own runtime handler, above)
        // that would normally be popped by its matching Op::PopBuiltinWrap;
        // on the exception path that never runs. These are the MOST
        // (below). Note these three loops are COUNTS, not targeted pops:
        // popping N off the back of treeStack_ removes the top N whichever
        // group counted them, so the order between the groups is
        // unobservable and adding a fourth kind cannot break it.
        // ownsModuleSplice reads last only because it is the deepest.
        // See Op::PopBuiltinWrap's own doc comment (bytecode.hpp) for why
        // this is a real counter, not the single-bool shape
        // ownsModuleSplice, below, gets away with (N of these can be open
        // at once; at most one module-call splice ever can).
        for (size_t i = 0; i < frame->builtinWrapStack.size(); ++i) ev.treeStack_.pop_back();
        // A Kind::Measure bracket also set ev.measuring_ on the way in, and
        // its matching Op::PopBuiltinWrap -- which would have restored it --
        // never ran. front(), not back(): savedMeasuring is recorded by
        // EVERY kind, so this frame's OUTERMOST still-open bracket holds the
        // value that was live before any of them opened. Nothing else in a
        // frame can change the flag (a nested interpreter evalRenderExpr is
        // scoped; a nested VM frame restores in its own turn of this loop).
        if (!frame->builtinWrapStack.empty()) ev.measuring_ = frame->builtinWrapStack.front().savedMeasuring;
        frame->builtinWrapStack.clear();
        // Same reasoning, same LIFO-order requirement, for any still-open
        // Op::PushCsgWrap bracket(s) -- see PendingCsgWrap's/Op::PushCsgWrap's
        // own doc comments.
        for (size_t i = 0; i < frame->csgWrapStack.size(); ++i) ev.treeStack_.pop_back();
        frame->csgWrapStack.clear();
        // A module frame's own CALLER (Op::CallModule, mirroring
        // evalModularCall/buildTreeNode) always pushes a treeStack_
        // accumulator for it before pushing this frame -- normally popped
        // by this same frame's own completion (driveVm's completion
        // branch); on the exception path that never runs, so it has to
        // happen here instead, exactly like buildTreeNode's own
        // catch(...) { treeStack_.pop_back(); throw; }.
        if (frame->ownsModuleSplice) ev.treeStack_.pop_back();
        if (bracket) ev.exitUserCallException(*bracket);
        ev.releaseVmFrame(std::move(frame));
    }
}

// The driver -- see this project's own session notes for the full
// "explicit frame stack, not the C++ call stack" rationale. Services
// ev.vmCallStack_ down to (but not including) `floor`, returning the
// value the frame originally AT `floor + 1` (i.e. whichever frame the
// caller just pushed before calling this) produced. Every OTHER frame
// this loop itself pushes (a non-tail Op::CallFn/CallDynamic to another
// compiled function) is fully serviced -- pushed, run, popped, its value
// folded into its own parent's operand stack -- before this function
// returns; only the floor frame's own result escapes to the caller.
Value driveVm(Evaluator& ev, size_t floor) {
    Value finalResult;
    // Native-stack-safety guard, distinct from vmCallStack_'s own
    // heap-bounded kMaxVmCallStackDepth check -- see driveVmNativeDepth_'s
    // own doc comment (evaluator.hpp) for exactly what this catches (a
    // NativeStatement-triggered re-entry into the VM, still a genuine
    // native C++ call unlike an ordinary Op::CallModule/CallFn hop).
    struct NativeDepthGuard {
        Evaluator& ev;
        explicit NativeDepthGuard(Evaluator& e) : ev(e) { ++ev.driveVmNativeDepth_; }
        ~NativeDepthGuard() { --ev.driveVmNativeDepth_; }
    } nativeDepthGuard(ev);
    try {
        // Fixed count, not nativeStackMarginLow() -- see kMaxDriveVmNative
        // Depth's own doc comment (evaluator.hpp) for why: the margin-based
        // check was added specifically because BOSL2's attachable() chain
        // (translate/multmatrix/color/modifier-wrapped recursion) needed
        // native reentry depth 55, past this fixed ceiling -- but
        // Op::PushBuiltinWrap (bytecode_compiler.cpp/bytecode_vm.cpp) has
        // since eliminated native reentry for exactly that pattern, so
        // this guard now only has to catch what's genuinely still
        // uncovered (union/difference/intersection-wrapped recursion,
        // other builtins) -- comfortably within this fixed ceiling for any
        // realistic script. The margin-based check was tried here and
        // confirmed, via two separate real Windows CI runs, to still
        // segfault for a deep uncovered-construct chain regardless of how
        // much total stack the process has (this fixed count is the
        // proven-safe mechanism for THIS specific native-reentry chain --
        // see its own doc comment for the real Windows bisection history).
        if (ev.driveVmNativeDepth_ > Evaluator::kMaxDriveVmNativeDepth) {
            const oscad::ASTNode* node = ev.currentCallDeclNode();
            if (!node) node = ev.vmCallStack_.back()->chunk->selfDecl;
            ev.error("Recursion too deep (native call stack)", *node);
        }
        while (ev.vmCallStack_.size() > floor) {
            VmFrame& f = *ev.vmCallStack_.back();
            if (f.pc >= f.code->size()) {
                // A module chunk produces nothing on the stack at all --
                // its whole effect already landed in treeStack_ as a side
                // effect of running its own body (Op::CallModule/
                // NativeStatement). `result` stays Value{} for one; a
                // function chunk's own completion is unchanged.
                const bool isModule = f.chunk->isModule;
                Value result = (!isModule && !f.stack.empty()) ? std::move(f.stack.back()) : Value{};
                const bool isFloorFrame = ev.vmCallStack_.size() == floor + 1;
                std::unique_ptr<VmFrame> finished = std::move(ev.vmCallStack_.back());
                ev.vmCallStack_.pop_back();
                std::optional<Evaluator::UserCallHandle> bracket = std::move(ev.vmCallBrackets_.back());
                ev.vmCallBrackets_.pop_back();
                const std::string finishedName = finished->logicalName;
                const bool ownsModuleSplice = finished->ownsModuleSplice;
                const std::uint64_t moduleRandsBefore = finished->moduleRandsBefore;
                const oscad::ASTNode* moduleSpliceCallNode = finished->moduleSpliceCallNode;
                while (!finished->ctxChain.empty()) finished->ctxChain.pop_back();
                // Module frames never fire returnHook (native evalUserModule
                // never did either -- a module call has no "return value"
                // the debugger reports).
                if (bracket) ev.exitUserCallSuccess(finishedName, *bracket, result, /*fireReturnHook=*/!isModule);
                ev.releaseVmFrame(std::move(finished));
                if (isModule && ownsModuleSplice) {
                    std::vector<std::unique_ptr<CSGNode>> children = std::move(ev.treeStack_.back());
                    ev.treeStack_.pop_back();
                    ev.spliceModuleChildren(std::move(children), moduleRandsBefore, *moduleSpliceCallNode);
                }
                if (isFloorFrame) {
                    finalResult = std::move(result); // unused by a module caller (runCompiledModuleBody ignores it)
                    break;
                }
                if (!isModule) ev.vmCallStack_.back()->stack.push_back(std::move(result));
                ++ev.vmCallStack_.back()->pc;
                continue;
            }

            const Instruction& ins = (*f.code)[f.pc];
            EvalContext& ctx = f.ctxChain.back();
            switch (ins.op) {
                case Op::PushConst:
                    f.stack.push_back(f.chunk->constants[static_cast<size_t>(ins.a)]);
                    ++f.pc;
                    break;
                case Op::PushBool:
                    f.stack.push_back(Value{ins.a != 0});
                    ++f.pc;
                    break;
                case Op::LoadLocal:
                    f.stack.push_back(f.slots[static_cast<size_t>(ins.a)]);
                    ++f.pc;
                    break;
                case Op::StoreLocal: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.slots[static_cast<size_t>(ins.a)] = std::move(v);
                    ++f.pc;
                    break;
                }
                case Op::StoreLocalAndLet: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.slots[static_cast<size_t>(ins.a)] = v;
                    ctx.let_->set(f.chunk->names[static_cast<size_t>(ins.b)], std::move(v));
                    ++f.pc;
                    break;
                }
                case Op::LoadDyn: {
                    const std::string& name = f.chunk->names[static_cast<size_t>(ins.a)];
                    if (const Value* v = ctx.let_->find(name)) {
                        f.stack.push_back(*v);
                    } else if (const Value* v = ctx.dyn->find(name)) {
                        f.stack.push_back(*v);
                    } else {
                        // ins.b = warn flag; see compileIdentifierLoad.
                        if (ins.b) ev.warn("Ignoring unknown variable '" + name + "'", ins.pos);
                        f.stack.push_back(Value{});
                    }
                    ++f.pc;
                    break;
                }
                case Op::StoreDyn: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    const std::string& name = f.chunk->names[static_cast<size_t>(ins.a)];
                    ctx.dyn->set(name, v);
                    ctx.dynExplicit->set(name, true);
                    ++f.pc;
                    break;
                }
                case Op::LoadFree:
                    f.stack.push_back(ev.evalIdentifier(f.chunk->names[static_cast<size_t>(ins.a)], ins.pos, ctx, true));
                    ++f.pc;
                    break;
                case Op::LoadFreeNoWarn:
                    f.stack.push_back(ev.evalIdentifier(f.chunk->names[static_cast<size_t>(ins.a)], ins.pos, ctx, false));
                    ++f.pc;
                    break;
                case Op::LoadUpvalue: {
                    const CompiledChunk::UpvalueRef& uv = f.chunk->upvalues[static_cast<size_t>(ins.a)];
                    const Value* v = ev.findUpvalue(uv.targetDecl, uv.slot);
                    if (!v) v = ctx.let_->find(uv.name);
                    f.stack.push_back(v ? *v : Value{});
                    ++f.pc;
                    break;
                }
                case Op::MakeClosure: {
                    const CompiledChunk::ClosureSite& site = f.chunk->closureSites[static_cast<size_t>(ins.a)];
                    auto capturedTrail = TrailView<Value>::makeRoot();
                    std::vector<const std::string*> selfNames;
                    for (const auto& cap : site.captures) {
                        if (cap.isSelfReference) {
                            selfNames.push_back(&cap.name);
                            continue;
                        }
                        if (cap.targetDecl == f.chunk->selfDecl) {
                            capturedTrail->set(cap.name, f.slots[static_cast<size_t>(cap.slot)]);
                            continue;
                        }
                        const Value* v = ev.findUpvalue(cap.targetDecl, cap.slot);
                        if (!v) v = ctx.let_->find(cap.name);
                        capturedTrail->set(cap.name, v ? *v : Value{});
                    }
                    auto closure = std::make_shared<const Closure>(Closure{site.node, capturedTrail});
                    for (const std::string* selfName : selfNames) {
                        capturedTrail->set(*selfName, Value{closure});
                    }
                    f.stack.push_back(Value{std::move(closure)});
                    ++f.pc;
                    break;
                }
                case Op::PatchClosureCapture: {
                    const Value& consumerVal = f.slots[static_cast<size_t>(ins.a)];
                    if (const auto* closurePtr = std::get_if<ClosurePtr>(&consumerVal); closurePtr && *closurePtr) {
                        if (auto trail = capturedLetTrail(**closurePtr)) {
                            trail->set(f.chunk->names[static_cast<size_t>(ins.b)], f.slots[static_cast<size_t>(ins.c)]);
                        }
                    }
                    ++f.pc;
                    break;
                }
                case Op::Range: {
                    Value step = std::move(f.stack.back());
                    f.stack.pop_back();
                    Value end = std::move(f.stack.back());
                    f.stack.pop_back();
                    Value start = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.stack.push_back(ev.applyRange(start, end, step, ins.a != 0, ins.pos));
                    ++f.pc;
                    break;
                }
                case Op::Index: {
                    Value idx = std::move(f.stack.back());
                    f.stack.pop_back();
                    Value obj = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.stack.push_back(ev.applyIndexAccess(obj, idx));
                    ++f.pc;
                    break;
                }
                case Op::Member: {
                    Value obj = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.stack.push_back(ev.applyMemberAccess(obj, f.chunk->names[static_cast<size_t>(ins.a)]));
                    ++f.pc;
                    break;
                }
                case Op::UnaryOp: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.stack.push_back(ev.applyUnaryOp(static_cast<oscad::NodeKind>(ins.a), v, *ins.pos));
                    ++f.pc;
                    break;
                }
                case Op::BinaryOp: {
                    Value b = std::move(f.stack.back());
                    f.stack.pop_back();
                    Value a = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.stack.push_back(ev.applyBinaryOp(static_cast<oscad::NodeKind>(ins.a), a, b, *ins.pos));
                    ++f.pc;
                    break;
                }
                case Op::Jump:
                    f.pc = static_cast<size_t>(ins.a);
                    break;
                case Op::JumpIfFalse: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.pc = !truthy(v) ? static_cast<size_t>(ins.a) : f.pc + 1;
                    break;
                }
                case Op::JumpIfTrue: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.pc = truthy(v) ? static_cast<size_t>(ins.a) : f.pc + 1;
                    break;
                }
                case Op::OpenLocalScope: {
                    for (int i = 0; i < ins.b; ++i) f.slots[static_cast<size_t>(ins.a + i)] = Value{};
                    ++f.pc;
                    break;
                }
                case Op::BuildList: {
                    const size_t count = static_cast<size_t>(ins.a);
                    std::vector<Value> items(count);
                    for (size_t i = 0; i < count; ++i) {
                        items[count - 1 - i] = std::move(f.stack.back());
                        f.stack.pop_back();
                    }
                    f.stack.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(items)})});
                    ++f.pc;
                    break;
                }
                case Op::AccumOpen:
                    f.accumStack.emplace_back();
                    ++f.pc;
                    break;
                case Op::AccumAppendOne: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    f.accumStack.back().push_back(std::move(v));
                    ++f.pc;
                    break;
                }
                case Op::AccumAppendEach: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    appendEachInto(f.accumStack.back(), v);
                    ++f.pc;
                    break;
                }
                case Op::AccumClose: {
                    std::vector<Value> items = std::move(f.accumStack.back());
                    f.accumStack.pop_back();
                    f.stack.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(items)})});
                    ++f.pc;
                    break;
                }
                case Op::AccumMergeEach: {
                    std::vector<Value> inner = std::move(f.accumStack.back());
                    f.accumStack.pop_back();
                    for (const Value& item : inner) appendEachInto(f.accumStack.back(), item);
                    ++f.pc;
                    break;
                }
                case Op::IterMaterialize: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    IterList& il = f.iterLists[static_cast<size_t>(ins.a)];
                    il.values = expandIterable(v, [&](size_t count) {
                        ev.warn("Bad range parameter in for statement: too many elements (" + std::to_string(count) + ")",
                                ins.pos);
                    });
                    il.index = 0;
                    il.total = il.values.size();
                    ++f.pc;
                    break;
                }
                case Op::IterReset: {
                    f.iterLists[static_cast<size_t>(ins.a)].index = 0;
                    ++f.pc;
                    break;
                }
                case Op::IterNext: {
                    IterList& il = f.iterLists[static_cast<size_t>(ins.b)];
                    if (il.index < il.total) {
                        f.slots[static_cast<size_t>(ins.a)] = il.values[il.index];
                        ++il.index;
                        ++f.pc;
                    } else {
                        f.pc = static_cast<size_t>(ins.c);
                    }
                    break;
                }
                case Op::CheckIterLimit: {
                    double count = std::get<double>(f.slots[static_cast<size_t>(ins.a)]) + 1.0;
                    f.slots[static_cast<size_t>(ins.a)] = Value{count};
                    if (count > static_cast<double>(ins.b)) {
                        ev.error("C-style for loop exceeded maximum iteration count", *ins.node);
                    }
                    ++f.pc;
                    break;
                }
                case Op::CallFn: {
                    const CompiledChunk::CallSite& site = f.chunk->callSites[static_cast<size_t>(ins.a)];
                    const size_t argCount = static_cast<size_t>(ins.b);
                    std::vector<Value> args(argCount);
                    for (size_t i = 0; i < argCount; ++i) {
                        args[argCount - 1 - i] = std::move(f.stack.back());
                        f.stack.pop_back();
                    }
                    if (site.isImport) {
                        CallArgs callArgs = buildCallArgs(site, args, argCount);
                        f.stack.push_back(importAsValue(ev, callArgs, *site.callNode));
                        ++f.pc;
                    } else if (site.isBuiltin && site.calleeName == "object") {
                        std::vector<std::pair<std::optional<std::string>, Value>> pairs;
                        pairs.reserve(argCount);
                        for (size_t i = 0; i < argCount; ++i) pairs.emplace_back(site.argNames[i], std::move(args[i]));
                        f.stack.push_back(mergeObjectArgs(ev, pairs, &site.callNode->position()));
                        ++f.pc;
                    } else if (site.isBuiltin) {
                        CallArgs callArgs = buildCallArgs(site, args, argCount);
                        f.stack.push_back(evalBuiltinFunction(ev, site.calleeName, callArgs, *site.callNode));
                        ++f.pc;
                    } else {
                        BoundArgs bound = buildBoundArgs(ev, site, args, argCount, site.decl->parameters);
                        const CompiledChunk* calleeChunk = ev.useBytecodeVm() ? ev.lookupOrCompileChunk(*site.decl) : nullptr;
                        if (calleeChunk) {
                            const oscad::Scope* fnScope = site.decl->scope() ? site.decl->scope() : ctx.scope;
                            pushBracketedCallFrame(ev, *calleeChunk, *site.decl, *site.decl->expr, site.calleeName,
                                                    std::move(bound), ctx, fnScope, nullptr, &site.callNode->position());
                            // f.pc deliberately NOT advanced -- resumes when the
                            // pushed frame completes (see driveVm's own
                            // completion branch, above).
                        } else {
                            Value result = ev.evalUserFunctionFromBound(site.calleeName, *site.decl, std::move(bound), ctx,
                                                                          &site.callNode->position());
                            f.stack.push_back(std::move(result));
                            ++f.pc;
                        }
                    }
                    break;
                }
                case Op::CallDynamic: {
                    const CompiledChunk::CallSite& site = f.chunk->callSites[static_cast<size_t>(ins.a)];
                    const size_t argCount = static_cast<size_t>(ins.b);
                    std::vector<Value> args(argCount);
                    for (size_t i = 0; i < argCount; ++i) {
                        args[argCount - 1 - i] = std::move(f.stack.back());
                        f.stack.pop_back();
                    }
                    Value callee = std::move(f.stack.back());
                    f.stack.pop_back();
                    if (const auto* closurePtr = std::get_if<ClosurePtr>(&callee); closurePtr && *closurePtr) {
                        const Closure& closure = **closurePtr;
                        const oscad::FunctionLiteral& funcNode = *closure.node;
                        BoundArgs bound = buildBoundArgs(ev, site, args, argCount, funcNode.parameters);
                        const CompiledChunk* calleeChunk = ev.useBytecodeVm() ? ev.lookupCompiledLiteralChunk(funcNode) : nullptr;
                        if (calleeChunk) {
                            const oscad::Scope* fnScope = funcNode.scope() ? funcNode.scope() : ctx.scope;
                            pushBracketedCallFrame(ev, *calleeChunk, funcNode, *funcNode.body, "<function literal>",
                                                    std::move(bound), ctx, fnScope, capturedLetTrail(closure), ins.pos);
                        } else {
                            Value result = ev.evalFunctionLiteralFromBound(closure, std::move(bound), ctx, ins.pos);
                            f.stack.push_back(std::move(result));
                            ++f.pc;
                        }
                    } else {
                        if (!site.calleeName.empty()) ev.warn("Ignoring unknown function '" + site.calleeName + "'", ins.pos);
                        f.stack.push_back(Value{});
                        ++f.pc;
                    }
                    break;
                }
                case Op::CallFnTail: {
                    const CompiledChunk::CallSite& site = f.chunk->callSites[static_cast<size_t>(ins.a)];
                    const size_t argCount = static_cast<size_t>(ins.b);
                    std::vector<Value> args(argCount);
                    for (size_t i = 0; i < argCount; ++i) {
                        args[argCount - 1 - i] = std::move(f.stack.back());
                        f.stack.pop_back();
                    }
                    BoundArgs bound = buildBoundArgs(ev, site, args, argCount, site.decl->parameters);
                    // Tail-hop-in-place requires f.hopEligible -- a frame
                    // that's call-boundary-free (statement expression,
                    // assignment block, parameter default) has no
                    // callStack_ entry that's genuinely its own, so
                    // recordTailCallHop below (which mutates
                    // callStack_.back()) would corrupt whatever UNRELATED
                    // call happens to be on top of callStack_ instead --
                    // see VmFrame::hopEligible's own doc comment for why
                    // this is NOT the same test as "did THIS frame push its
                    // own vmCallBrackets_ entry" (runCompiledFunction's
                    // initial frame is bare in that sense but still
                    // hop-eligible). Falls through to the exact same
                    // push-or-native-fallback behavior as Op::CallFn in that
                    // case -- still gets the full explicit-stack treatment if
                    // the callee compiles, just as a genuine new logical call
                    // rather than a hop, mirroring today's `tailOut==nullptr`
                    // behavior for these same bare call shapes exactly.
                    const bool thisFrameBracketed = f.hopEligible;
                    std::optional<EvalContext> hopCtx =
                        thisFrameBracketed ? ev.isolatedCallCtxFor(*site.decl, ctx) : std::nullopt;
                    const CompiledChunk* calleeChunk =
                        (thisFrameBracketed && hopCtx && ev.useBytecodeVm()) ? ev.lookupOrCompileChunk(*site.decl) : nullptr;
                    if (calleeChunk) {
                        ev.recordTailCallHop(site.calleeName, *site.decl, &site.callNode->position(), f.tailHopGuard);
                        f.chunk = calleeChunk;
                        f.code = &calleeChunk->bodyCode;
                        f.pc = 0;
                        f.slots.assign(static_cast<size_t>(calleeChunk->numSlots), Value{});
                        f.stack.clear();
                        f.bound.assign(calleeChunk->params.size(), false);
                        f.accumStack.clear();
                        f.iterLists.assign(static_cast<size_t>(calleeChunk->numIterLists), IterList{});
                        f.ctxChain.push_back(std::move(*hopCtx));
                        bindBoundArgsIntoFrame(*calleeChunk, bound, f);
                        applyCompiledDefaultsToFrame(ev, *calleeChunk, f);
                        // continue via the outer while -- re-fetches this
                        // same (now-mutated) frame from its new pc=0.
                    } else {
                        const CompiledChunk* fallbackChunk =
                            ev.useBytecodeVm() ? ev.lookupOrCompileChunk(*site.decl) : nullptr;
                        if (fallbackChunk) {
                            const oscad::Scope* fnScope = site.decl->scope() ? site.decl->scope() : ctx.scope;
                            pushBracketedCallFrame(ev, *fallbackChunk, *site.decl, *site.decl->expr, site.calleeName,
                                                    std::move(bound), ctx, fnScope, nullptr, &site.callNode->position());
                        } else {
                            Value result = ev.evalUserFunctionFromBound(site.calleeName, *site.decl, std::move(bound), ctx,
                                                                          &site.callNode->position());
                            f.stack.push_back(std::move(result));
                            ++f.pc;
                        }
                    }
                    break;
                }
                case Op::CallDynamicTail: {
                    const CompiledChunk::CallSite& site = f.chunk->callSites[static_cast<size_t>(ins.a)];
                    const size_t argCount = static_cast<size_t>(ins.b);
                    std::vector<Value> args(argCount);
                    for (size_t i = 0; i < argCount; ++i) {
                        args[argCount - 1 - i] = std::move(f.stack.back());
                        f.stack.pop_back();
                    }
                    Value callee = std::move(f.stack.back());
                    f.stack.pop_back();
                    if (const auto* closurePtr = std::get_if<ClosurePtr>(&callee); closurePtr && *closurePtr) {
                        const Closure& closure = **closurePtr;
                        const oscad::FunctionLiteral& funcNode = *closure.node;
                        BoundArgs bound = buildBoundArgs(ev, site, args, argCount, funcNode.parameters);
                        const bool thisFrameBracketed = f.hopEligible;
                        std::optional<EvalContext> hopCtx = thisFrameBracketed
                                                                 ? ev.isolatedCallCtxFor(funcNode, ctx, capturedLetTrail(closure))
                                                                 : std::nullopt;
                        const CompiledChunk* calleeChunk = (thisFrameBracketed && hopCtx && ev.useBytecodeVm())
                                                                ? ev.lookupCompiledLiteralChunk(funcNode)
                                                                : nullptr;
                        if (calleeChunk) {
                            ev.recordTailCallHop("<function literal>", funcNode, ins.pos, f.tailHopGuard);
                            f.chunk = calleeChunk;
                            f.code = &calleeChunk->bodyCode;
                            f.pc = 0;
                            f.slots.assign(static_cast<size_t>(calleeChunk->numSlots), Value{});
                            f.stack.clear();
                            f.bound.assign(calleeChunk->params.size(), false);
                            f.accumStack.clear();
                            f.iterLists.assign(static_cast<size_t>(calleeChunk->numIterLists), IterList{});
                            f.ctxChain.push_back(std::move(*hopCtx));
                            bindBoundArgsIntoFrame(*calleeChunk, bound, f);
                            applyCompiledDefaultsToFrame(ev, *calleeChunk, f);
                        } else {
                            const CompiledChunk* fallbackChunk =
                                ev.useBytecodeVm() ? ev.lookupCompiledLiteralChunk(funcNode) : nullptr;
                            if (fallbackChunk) {
                                const oscad::Scope* fnScope = funcNode.scope() ? funcNode.scope() : ctx.scope;
                                pushBracketedCallFrame(ev, *fallbackChunk, funcNode, *funcNode.body, "<function literal>",
                                                        std::move(bound), ctx, fnScope, capturedLetTrail(closure), ins.pos);
                            } else {
                                Value result = ev.evalFunctionLiteralFromBound(closure, std::move(bound), ctx, ins.pos);
                                f.stack.push_back(std::move(result));
                                ++f.pc;
                            }
                        }
                    } else {
                        if (!site.calleeName.empty()) ev.warn("Ignoring unknown function '" + site.calleeName + "'", ins.pos);
                        f.stack.push_back(Value{});
                        ++f.pc;
                    }
                    break;
                }
                case Op::Echo: {
                    const CompiledChunk::EchoSite& site = f.chunk->echoSites[static_cast<size_t>(ins.a)];
                    const size_t argCount = static_cast<size_t>(ins.b);
                    std::vector<std::pair<std::optional<std::string>, Value>> pairs(argCount);
                    for (size_t i = 0; i < argCount; ++i) {
                        pairs[argCount - 1 - i] = {site.argNames[argCount - 1 - i], std::move(f.stack.back())};
                        f.stack.pop_back();
                    }
                    ev.emitEcho(pairs);
                    ++f.pc;
                    break;
                }
                case Op::AssertFail: {
                    const bool hasMessage = ins.a == 1;
                    std::string err =
                        "Assertion '" + std::get<std::string>(f.chunk->constants[static_cast<size_t>(ins.b)]) + "' failed";
                    if (hasMessage) {
                        Value msg = std::move(f.stack.back());
                        f.stack.pop_back();
                        const std::string* s = std::get_if<std::string>(&msg);
                        err += ": \"" + (s ? *s : fmtValue(msg)) + "\"";
                    }
                    ev.error(err, *ins.node, "assert");
                }
                case Op::CallModule: {
                    const CompiledChunk::ModuleCallSite& site = f.chunk->moduleCallSites[static_cast<size_t>(ins.a)];
                    BoundArgs bound = ev.bindArgs(site.decl->parameters, site.callNode->arguments, ctx);
                    EvalContext childCtx = ev.buildModuleChildCtx(*site.decl, *site.callNode, ctx, std::move(bound));
                    ev.treeStack_.emplace_back();
                    const std::uint64_t randsBefore = ev.randsCallCount();
                    const CompiledChunk* calleeChunk = ev.useBytecodeVm() ? ev.lookupOrCompileModuleChunk(*site.decl) : nullptr;
                    if (calleeChunk) {
                        pushBracketedModuleFrame(ev, *calleeChunk, *site.decl, childCtx, &site.callNode->position(),
                                                  randsBefore, *site.callNode);
                        // f.pc deliberately NOT advanced -- resumes when the
                        // pushed frame completes (see driveVm's own module
                        // completion branch, above).
                    } else {
                        ev.runModuleBodyNative(*site.decl, childCtx, &site.callNode->position());
                        std::vector<std::unique_ptr<CSGNode>> children = std::move(ev.treeStack_.back());
                        ev.treeStack_.pop_back();
                        ev.spliceModuleChildren(std::move(children), randsBefore, *site.callNode);
                        ++f.pc;
                    }
                    break;
                }
                case Op::CallChildren: {
                    // Ordering is load-bearing throughout -- see this op's
                    // own doc comment (bytecode.hpp): checkDebug against
                    // the SCOPE-WRAPPED ctx (byte-for-byte what Op::
                    // NativeStatement does for this same node today);
                    // randsBefore BEFORE argument resolution (rands-in-args
                    // taint); treeStack_ pushed LAST, immediately before
                    // the frame push / native fallback, so an early-out or
                    // a throw during arg resolution has nothing to clean
                    // up (the native path's own catch{pop;throw} has no
                    // equivalent out here -- this reorder IS the
                    // exception-safety mechanism; arg resolution never
                    // appends CSG nodes, so pushing after it is
                    // unobservable).
                    const auto* callNode =
                        static_cast<const oscad::ModularCall*>(f.chunk->nativeStatements[static_cast<size_t>(ins.a)]);
                    EvalContext scopedCtx = ctx.withScope(callNode->scope() ? callNode->scope() : ctx.scope);
                    ev.checkDebug(*callNode, scopedCtx);
                    // Same order as evalModularCall's own (csg_resolve.cpp):
                    // warn, then resolve. Without this a children() typo
                    // warns only under the interpreter, so no test for that
                    // warning could run under both engines.
                    warnUnexpectedBuiltinArgs(ev, *callNode);
                    const std::uint64_t randsBefore = ev.randsCallCount();
                    auto [args, effCtx] = resolveCallArgs(ev, callNode->arguments, scopedCtx);
                    std::optional<Evaluator::ChildrenForward> fwd = ev.prepareChildrenForward(args, effCtx);
                    if (!fwd) {
                        // No forwarded children in scope / empty / index
                        // out of range -- a silent no-op, exactly matching
                        // builtinChildren's own early returns.
                        ++f.pc;
                        break;
                    }
                    // Pass gate is load-bearing, not defensive -- the
                    // chunk cache is pass-scoped; see inResolvePass()'s
                    // own doc comment (evaluator.hpp).
                    const CompiledChunk* chunk = (ev.useBytecodeVm() && ev.inResolvePass())
                                                     ? ev.lookupOrCompileChildrenListChunk(fwd->nodes)
                                                     : nullptr;
                    if (chunk) {
                        ev.treeStack_.emplace_back();
                        pushChildrenForwardFrame(ev, *chunk, std::move(fwd->evalCtx), randsBefore, *callNode);
                        // f.pc deliberately NOT advanced -- resumes when
                        // the pushed frame completes; driveVm's completion
                        // branch runs the splice (isModule &&
                        // ownsModuleSplice), mirroring evalModularCall's
                        // own "children" splice branch exactly.
                    } else {
                        // Fallback reuses the ALREADY-resolved args --
                        // never evalStatement, which would re-resolve them
                        // (double rands(), double echo/assert side
                        // effects, double expr-level debug stops).
                        // Inlines evalModularCall's own children branch:
                        // push accumulator, run natively, pop, splice.
                        // The native evalChildren still self-gates and
                        // retries tryRunCompiledChildren internally.
                        ev.treeStack_.emplace_back();
                        try {
                            ev.evalChildren(fwd->nodes, fwd->evalCtx);
                        } catch (...) {
                            ev.treeStack_.pop_back();
                            throw;
                        }
                        std::vector<std::unique_ptr<CSGNode>> children = std::move(ev.treeStack_.back());
                        ev.treeStack_.pop_back();
                        ev.spliceModuleChildren(std::move(children), randsBefore, *callNode);
                        ++f.pc;
                    }
                    break;
                }
                case Op::PushBuiltinWrap: {
                    const CompiledChunk::BuiltinWrapSite& site = f.chunk->builtinWrapSites[static_cast<size_t>(ins.a)];
                    // Captured BEFORE any argument resolution -- mirrors
                    // Evaluator::buildTreeNode's own ordering exactly (see
                    // this op's own doc comment, bytecode.hpp, for why
                    // getting this backwards would silently drop
                    // uncacheable/ManifoldCache taint tracking for a
                    // rands() call embedded in the wrapper's own args).
                    const std::uint64_t randsBefore = ev.randsCallCount();
                    warnUnexpectedBuiltinArgs(ev, *site.node);
                    CSGParams params;
                    CallArgs deferredArgs; // Roof only -- see PendingBuiltinWrap's own doc comment
                    switch (site.kind) {
                        case CompiledChunk::BuiltinWrapSite::Kind::Transform: {
                            BuiltinWrapParams result =
                                computeTransformParams(ev, static_cast<const oscad::ModularCall&>(*site.node), ctx);
                            params = std::move(result.params);
                            f.ctxChain.push_back(std::move(result.ctx));
                            break;
                        }
                        case CompiledChunk::BuiltinWrapSite::Kind::Color: {
                            BuiltinWrapParams result =
                                computeColorParams(ev, static_cast<const oscad::ModularCall&>(*site.node), ctx);
                            params = std::move(result.params);
                            f.ctxChain.push_back(std::move(result.ctx));
                            break;
                        }
                        case CompiledChunk::BuiltinWrapSite::Kind::Modifier:
                            // No params, no ctx change -- evalModifier's
                            // own native shape runs its single child
                            // against the untouched ctx too.
                            break;
                        case CompiledChunk::BuiltinWrapSite::Kind::Passthrough: {
                            // hull()/minkowski()/render() -- resolve args
                            // for their own $-propagation-into-children
                            // side effect only (mirrors resolveHull/
                            // resolveMinkowski/resolveRender's own
                            // `(void)args;`); no params of their own ever.
                            auto [args, effCtx] =
                                resolveCallArgs(ev, static_cast<const oscad::ModularCall&>(*site.node).arguments, ctx);
                            (void)args;
                            f.ctxChain.push_back(std::move(effCtx));
                            break;
                        }
                        case CompiledChunk::BuiltinWrapSite::Kind::Measure: {
                            // Arguments resolve purely for the
                            // $-propagation-into-children side effect, as
                            // Passthrough does -- differing only in the
                            // concrete node type they hang off.
                            auto [args, effCtx] = resolveCallArgs(
                                ev, static_cast<const oscad::RenderExpression&>(*site.node).arguments, ctx);
                            (void)args;
                            // Publish this frame's slot locals into the
                            // children's context. The children are STATEMENT
                            // opcodes and resolve names through the
                            // EvalContext, but a compiled function keeps its
                            // parameters and lets in slots, which no context
                            // can see -- so without this,
                            // `function f(w) = render() { cube(w); }.volume;`
                            // resolves `w` to undef and silently measures
                            // nothing. Applied in outermost-first order so an
                            // inner binding shadows an outer one, and only
                            // into effCtx's own fresh trail level, so nothing
                            // leaks back to the caller.
                            for (const auto& [name, slot] : site.capturedLocals) {
                                if (static_cast<size_t>(slot) < f.slots.size()) {
                                    effCtx.let_->set(name, f.slots[static_cast<size_t>(slot)]);
                                }
                            }
                            f.ctxChain.push_back(std::move(effCtx));
                            break;
                        }
                        case CompiledChunk::BuiltinWrapSite::Kind::LinearExtrude: {
                            BuiltinWrapParams result = computeLinearExtrudeParams(
                                ev, static_cast<const oscad::ModularCall&>(*site.node), ctx);
                            params = std::move(result.params);
                            f.ctxChain.push_back(std::move(result.ctx));
                            break;
                        }
                        case CompiledChunk::BuiltinWrapSite::Kind::RotateExtrude: {
                            BuiltinWrapParams result = computeRotateExtrudeParams(
                                ev, static_cast<const oscad::ModularCall&>(*site.node), ctx);
                            params = std::move(result.params);
                            f.ctxChain.push_back(std::move(result.ctx));
                            break;
                        }
                        case CompiledChunk::BuiltinWrapSite::Kind::Projection: {
                            BuiltinWrapParams result =
                                computeProjectionParams(ev, static_cast<const oscad::ModularCall&>(*site.node), ctx);
                            params = std::move(result.params);
                            f.ctxChain.push_back(std::move(result.ctx));
                            break;
                        }
                        case CompiledChunk::BuiltinWrapSite::Kind::Offset: {
                            BuiltinWrapParams result =
                                computeOffsetParams(ev, static_cast<const oscad::ModularCall&>(*site.node), ctx);
                            params = std::move(result.params);
                            f.ctxChain.push_back(std::move(result.ctx));
                            break;
                        }
                        case CompiledChunk::BuiltinWrapSite::Kind::Roof: {
                            // params stays empty here -- computed at Pop
                            // instead (computeRoofParams calls ev.warn(),
                            // which must stay ordered AFTER children; see
                            // this Kind's own doc comment, bytecode.hpp).
                            // `args` must be retained, not re-resolved at
                            // Pop, or its argument expressions would run
                            // twice.
                            auto [args, effCtx] =
                                resolveCallArgs(ev, static_cast<const oscad::ModularCall&>(*site.node).arguments, ctx);
                            deferredArgs = std::move(args);
                            f.ctxChain.push_back(std::move(effCtx));
                            break;
                        }
                    }
                    ev.treeStack_.emplace_back();
                    f.builtinWrapStack.push_back({std::move(params), randsBefore, ins.a, std::move(deferredArgs),
                                                   ev.measuring_, f.stack.size(), ev.treeStack_.size() - 1});
                    // AFTER the push, so a throw from the push itself leaves
                    // the flag untouched rather than stuck on.
                    if (site.kind == CompiledChunk::BuiltinWrapSite::Kind::Measure) ev.measuring_ = true;
                    ++f.pc;
                    break;
                }
                case Op::PopBuiltinWrap: {
                    // Pop this bracket's own bookkeeping FIRST -- before
                    // anything below that can itself throw
                    // (setTreeDepthOrThrow) -- so the exception-teardown
                    // path's own builtinWrapStack.size() is already
                    // correct if THIS throws (see Op::PopBuiltinWrap's own
                    // doc comment, bytecode.hpp).
                    PendingBuiltinWrap pending = std::move(f.builtinWrapStack.back());
                    f.builtinWrapStack.pop_back();
                    const CompiledChunk::BuiltinWrapSite& site =
                        f.chunk->builtinWrapSites[static_cast<size_t>(pending.siteIdx)];
                    if (site.kind == CompiledChunk::BuiltinWrapSite::Kind::Measure) {
                        // Bookkeeping is already popped above, matching this
                        // op's own "pop first, THEN do anything that can
                        // throw" rule: measureCsgSubtree generates real
                        // Manifold geometry and can throw, and if it does,
                        // teardownVmCallStackDownTo must not see a bracket
                        // that is no longer open.
                        //
                        // measuring_ must stay TRUE across measureCsgSubtree
                        // -- that flag is what suppresses the four
                        // provenance writes in csg_generate.cpp -- so it is
                        // restored by a guard scoped AROUND the call, not
                        // before it. Restoring early would leave those
                        // guards inert on the VM path only: no crash, no
                        // wrong geometry, just quietly wrong click-to-source.
                        struct RestoreMeasuring {
                            Evaluator& ev;
                            bool prev;
                            ~RestoreMeasuring() { ev.measuring_ = prev; }
                        } restoreMeasuring{ev, pending.savedMeasuring};

                        f.ctxChain.pop_back(); // the effCtx pushed at Push
                        std::vector<std::unique_ptr<CSGNode>> sub = std::move(ev.treeStack_.back());
                        ev.treeStack_.pop_back();
                        assert(ev.treeStack_.size() == pending.treeStackDepthAtPush);
                        // The children are statement opcodes running mid-
                        // expression; they must be operand-stack-neutral or
                        // this expression's own result lands in the wrong
                        // slot. Every compileOneStatement case is, but
                        // nothing enforces it -- hence the assert.
                        assert(f.stack.size() == pending.stackDepthAtPush);
                        f.stack.push_back(ev.measureCsgSubtree(std::move(sub), *site.node));
                        ++f.pc;
                        break;
                    }
                    // Roof's params computation is deferred to here (see
                    // this Kind's own doc comment, bytecode.hpp) -- ctx is
                    // still on top of f.ctxChain, not yet popped below, so
                    // computeRoofParams sees the exact same effCtx the
                    // children just ran against.
                    if (site.kind == CompiledChunk::BuiltinWrapSite::Kind::Roof) {
                        pending.params = computeRoofParams(ev, pending.deferredArgs, f.ctxChain.back());
                    }
                    if (site.kind != CompiledChunk::BuiltinWrapSite::Kind::Modifier) f.ctxChain.pop_back();
                    std::vector<std::unique_ptr<CSGNode>> children = std::move(ev.treeStack_.back());
                    ev.treeStack_.pop_back();
                    // Mirrors Evaluator::buildTreeNode's own post-
                    // resolveBody() half exactly (csg_resolve.cpp).
                    const bool uncacheable =
                        (ev.randsCallCount() != pending.randsBefore) ||
                        std::any_of(children.begin(), children.end(), [](const auto& c) { return c->uncacheable; });
                    auto treeNode = std::make_unique<CSGNode>();
                    treeNode->kind = site.tagName;
                    treeNode->node = site.node;
                    treeNode->isBuiltin = true;
                    treeNode->warnEntry = ev.currentWarnEntry();
                    treeNode->children = std::move(children);
                    treeNode->params = std::move(pending.params);
                    treeNode->uncacheable = uncacheable;
                    ev.setTreeDepthOrThrow(*treeNode, *site.node);
                    ev.treeStack_.back().push_back(std::move(treeNode));
                    ++f.pc;
                    break;
                }
                case Op::PushCsgWrap: {
                    const CompiledChunk::CsgWrapSite& site = f.chunk->csgWrapSites[static_cast<size_t>(ins.a)];
                    // Captured BEFORE argument resolution -- same
                    // rands-in-args taint reasoning as Op::PushBuiltinWrap.
                    const std::uint64_t randsBefore = ev.randsCallCount();
                    if (site.hasArgs) {
                        warnUnexpectedBuiltinArgs(ev, *site.node);
                        // union/difference/intersection take no positional
                        // arguments in real OpenSCAD -- `args` is discarded,
                        // exactly mirroring resolveCsg's own `(void)args;`
                        // (booleans.cpp). Only `effCtx` (a possibly-$-scoped
                        // child ctx, e.g. `difference($fn=8) {...}`) matters.
                        auto [args, effCtx] =
                            resolveCallArgs(ev, static_cast<const oscad::ModularCall&>(*site.node).arguments, ctx);
                        (void)args;
                        f.ctxChain.push_back(std::move(effCtx));
                    }
                    // intersection_for (hasArgs=false) isn't a call at all
                    // -- no arguments to resolve, no ctx of its own to
                    // push; matches resolveIntersectionFor's own use of
                    // its caller's ctx unchanged (control.cpp). Its per-
                    // ITERATION child ctxs are pushed separately, by the
                    // compiled loop's own Op::ForIterNext.
                    ev.treeStack_.emplace_back();
                    f.csgWrapStack.push_back({site.op, randsBefore, ins.a, {}, 0});
                    ++f.pc;
                    break;
                }
                case Op::CsgGroupStart: {
                    f.csgWrapStack.back().groupStartSize = ev.treeStack_.back().size();
                    ++f.pc;
                    break;
                }
                case Op::CsgGroupChildren: {
                    const auto& call =
                        static_cast<const oscad::ModularCall&>(*f.chunk->nativeStatements[static_cast<size_t>(ins.a)]);
                    PendingCsgWrap& pending = f.csgWrapStack.back();
                    std::optional<std::vector<const oscad::ASTNode*>> expanded =
                        ev.expandSeparatingChildren(call, ctx);
                    // nullopt = `separate` resolved falsey: one group, the
                    // original statement, exactly as CsgGroupStart/End would
                    // have done for it.
                    const std::vector<const oscad::ASTNode*> stmts =
                        expanded ? *expanded : std::vector<const oscad::ASTNode*>{&call};
                    for (const oscad::ASTNode* stmt : stmts) {
                        const size_t before = ev.treeStack_.back().size();
                        ev.evalChildren(std::vector<const oscad::ASTNode*>{stmt}, ctx);
                        pending.groupSizes.push_back(
                            Value{static_cast<double>(ev.treeStack_.back().size() - before)});
                    }
                    ++f.pc;
                    break;
                }
                case Op::CsgGroupEnd: {
                    PendingCsgWrap& pending = f.csgWrapStack.back();
                    const size_t after = ev.treeStack_.back().size();
                    pending.groupSizes.push_back(Value{static_cast<double>(after - pending.groupStartSize)});
                    ++f.pc;
                    break;
                }
                case Op::PopCsgWrap: {
                    // Pop this bracket's own bookkeeping FIRST -- before
                    // anything below that can itself throw
                    // (setTreeDepthOrThrow) -- same ordering reasoning as
                    // Op::PopBuiltinWrap's own doc comment.
                    PendingCsgWrap pending = std::move(f.csgWrapStack.back());
                    f.csgWrapStack.pop_back();
                    const CompiledChunk::CsgWrapSite& site =
                        f.chunk->csgWrapSites[static_cast<size_t>(pending.siteIdx)];
                    if (site.hasArgs) f.ctxChain.pop_back();
                    std::vector<std::unique_ptr<CSGNode>> children = std::move(ev.treeStack_.back());
                    ev.treeStack_.pop_back();
                    // Mirrors resolveCsg's own params exactly for union/
                    // difference/intersection (booleans.cpp) -- and
                    // resolveIntersectionFor's own (no "op" key at all,
                    // control.cpp) when !includeOpParam.
                    CSGParams params;
                    if (site.includeOpParam) params["op"] = Value{pending.op};
                    params["group_sizes"] =
                        Value{std::make_shared<const ValueList>(ValueList{std::move(pending.groupSizes)})};
                    // Mirrors Evaluator::buildTreeNode's own post-
                    // resolveBody() half exactly (csg_resolve.cpp).
                    const bool uncacheable =
                        (ev.randsCallCount() != pending.randsBefore) ||
                        std::any_of(children.begin(), children.end(), [](const auto& c) { return c->uncacheable; });
                    auto treeNode = std::make_unique<CSGNode>();
                    treeNode->kind = site.op;
                    treeNode->node = site.node;
                    treeNode->isBuiltin = true;
                    treeNode->warnEntry = ev.currentWarnEntry();
                    treeNode->children = std::move(children);
                    treeNode->params = std::move(params);
                    treeNode->uncacheable = uncacheable;
                    ev.setTreeDepthOrThrow(*treeNode, *site.node);
                    ev.treeStack_.back().push_back(std::move(treeNode));
                    ++f.pc;
                    break;
                }
                case Op::NativeStatement: {
                    const oscad::ASTNode* stmt = f.chunk->nativeStatements[static_cast<size_t>(ins.a)];
                    EvalContext childCtx = ctx.withScope(stmt->scope() ? stmt->scope() : ctx.scope);
                    // Mirrors evalChildren's own per-statement loop exactly
                    // -- no ModularLet exclusion needed here (unlike that
                    // loop's own): ModularLet has its own compiled form now
                    // (Op::OpenLetScope/StoreLetVar) and never reaches
                    // Op::NativeStatement at all.
                    ev.checkDebug(*stmt, childCtx);
                    ev.evalStatement(*stmt, childCtx);
                    ++f.pc;
                    break;
                }
                case Op::OpenExprScope: {
                    f.ctxChain.push_back(ctx.letChildCtx());
                    ++f.pc;
                    break;
                }
                case Op::CloseExprScope: {
                    f.ctxChain.pop_back();
                    ++f.pc;
                    break;
                }
                case Op::OpenLetScope: {
                    f.ctxChain.push_back(ctx.childCtx(nullptr, std::nullopt, ctx.childrenNodes, ctx.childrenCallerCtx));
                    ++f.pc;
                    break;
                }
                case Op::StoreLetVar: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    const std::string& name = f.chunk->names[static_cast<size_t>(ins.a)];
                    if (!name.empty() && name[0] == '$') {
                        ctx.dyn->set(name, std::move(v));
                        ctx.dynExplicit->set(name, true);
                    } else {
                        ctx.let_->set(name, std::move(v));
                    }
                    ++f.pc;
                    break;
                }
                case Op::CheckDebugStatement: {
                    const oscad::ASTNode* stmt = f.chunk->nativeStatements[static_cast<size_t>(ins.a)];
                    ev.checkDebug(*stmt, ctx);
                    ++f.pc;
                    break;
                }
                case Op::StoreModuleVar: {
                    Value v = std::move(f.stack.back());
                    f.stack.pop_back();
                    const std::string& name = f.chunk->names[static_cast<size_t>(ins.a)];
                    // Mirrors Evaluator::evalAssignment exactly (stmt_eval.cpp) --
                    // see StoreModuleVar's own doc comment (bytecode.hpp).
                    if (!name.empty() && name[0] == '$') {
                        ctx.dyn->set(name, std::move(v));
                        ctx.dynExplicit->set(name, true);
                    } else {
                        if (const oscad::Position* const* firstPosEntry = ctx.dynPositions->find(name)) {
                            const oscad::Position* firstPos = *firstPosEntry;
                            const int firstLine = firstPos ? firstPos->line : 0;
                            ev.warn(name + " was assigned on line " + std::to_string(firstLine) + " but was overwritten",
                                    ins.pos);
                        }
                        ctx.let_->set(name, std::move(v));
                        ctx.dynPositions->set(name, ins.pos);
                    }
                    ++f.pc;
                    break;
                }
                case Op::AssertStatement: {
                    const CompiledChunk::AssertSite& site = f.chunk->assertSites[static_cast<size_t>(ins.a)];
                    std::vector<Value> args(static_cast<size_t>(site.argCount));
                    for (int i = site.argCount - 1; i >= 0; --i) {
                        args[static_cast<size_t>(i)] = std::move(f.stack.back());
                        f.stack.pop_back();
                    }
                    const Value condVal = site.conditionArgIndex ? std::move(args[static_cast<size_t>(*site.conditionArgIndex)])
                                                                   : Value{true};
                    if (!truthy(condVal)) {
                        std::string err = "Assertion '" +
                                           std::get<std::string>(f.chunk->constants[static_cast<size_t>(site.condTextConstIdx)]) +
                                           "' failed";
                        if (site.messageArgIndex) {
                            const Value& msgArg = args[static_cast<size_t>(*site.messageArgIndex)];
                            if (!std::holds_alternative<std::monostate>(msgArg)) {
                                const std::string* s = std::get_if<std::string>(&msgArg);
                                err += ": \"" + (s ? *s : fmtValue(msgArg)) + "\"";
                            }
                        }
                        ev.error(err, *site.node, "assert");
                    }
                    // Rare (assert(...) translate(...) children();-shaped) --
                    // not worth its own compiled path; evalChildren already
                    // tries ITS OWN compiled fast path internally regardless.
                    if (!site.node->children.empty()) ev.evalChildren(site.node->children, ctx);
                    ++f.pc;
                    break;
                }
                case Op::NativeCondJumpIfFalse: {
                    const oscad::Expression* cond = f.chunk->nativeExprs[static_cast<size_t>(ins.a)];
                    const bool truthyVal = truthy(ev.evalExprMaybeCompiled(*cond, ctx));
                    f.pc = truthyVal ? f.pc + 1 : static_cast<size_t>(ins.b);
                    break;
                }
                case Op::NativeCheckDebugExprLevel: {
                    const oscad::ASTNode* marker = f.chunk->nativeStatements[static_cast<size_t>(ins.a)];
                    ev.checkDebug(*marker, ctx, /*forced=*/false, /*exprLevel=*/true);
                    ++f.pc;
                    break;
                }
                case Op::NativeIterMaterialize: {
                    const oscad::Expression* rangeExpr = f.chunk->nativeExprs[static_cast<size_t>(ins.a)];
                    Value v = ev.evalExprMaybeCompiled(*rangeExpr, ctx);
                    IterList& il = f.iterLists[static_cast<size_t>(ins.b)];
                    il.values = expandIterable(v, [&](size_t count) {
                        ev.warn("Bad range parameter in for statement: too many elements (" + std::to_string(count) + ")",
                                ins.pos);
                    });
                    il.index = 0;
                    il.total = il.values.size();
                    ++f.pc;
                    break;
                }
                case Op::ForIterNext: {
                    IterList& il = f.iterLists[static_cast<size_t>(ins.b)];
                    if (il.index < il.total) {
                        // Fresh child ctx per iteration -- mirrors evalFor's
                        // own parentCtx.childCtx(...) exactly (see
                        // Op::ForIterNext's own doc comment, bytecode.hpp,
                        // for why reusing the SAME ctx across iterations
                        // would be observably wrong, not just slower).
                        EvalContext iterCtx = ctx.childCtx(nullptr, std::nullopt, ctx.childrenNodes, ctx.childrenCallerCtx);
                        iterCtx.let_->set(f.chunk->names[static_cast<size_t>(ins.a)], il.values[il.index]);
                        ++il.index;
                        f.ctxChain.push_back(std::move(iterCtx));
                        ev.checkDebug(*ins.node, f.ctxChain.back());
                        ++f.pc;
                    } else {
                        f.pc = static_cast<size_t>(ins.c);
                    }
                    break;
                }
                case Op::ForIterEnd: {
                    f.ctxChain.pop_back();
                    f.pc = static_cast<size_t>(ins.a);
                    break;
                }
            }
        }
    } catch (...) {
        teardownVmCallStackDownTo(ev, floor);
        throw;
    }
    return finalResult;
}

} // namespace

Value runCompiledFunction(Evaluator& ev, const CompiledChunk& chunk,
                          const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& callerCtx,
                          EvalContext& childCtx) {
    auto frame = ev.acquireVmFrame();
    frame->chunk = &chunk;
    frame->code = &chunk.bodyCode;
    frame->pc = 0;
    frame->slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame->stack.clear();
    frame->bound.assign(chunk.params.size(), false);
    frame->accumStack.clear();
    frame->iterLists.assign(static_cast<size_t>(chunk.numIterLists), IterList{});
    frame->ctxChain.clear();
    frame->ctxChain.push_back(childCtx);
    frame->tailHopGuard = 0;
    frame->logicalName.clear();
    frame->hopEligible = true; // caller's own enterUserCall already made
                                // callStack_.back() this call -- see
                                // VmFrame::hopEligible.
    bindAstArgsIntoFrame(ev, chunk, arguments, callerCtx, *frame);
    applyCompiledDefaultsToFrame(ev, chunk, *frame);
    const size_t floor = ev.vmCallStack_.size();
    ev.vmCallStack_.push_back(std::move(frame));
    ev.vmCallBrackets_.emplace_back(std::nullopt);
    return driveVm(ev, floor);
}

Value runCompiledFunctionFromBound(Evaluator& ev, const CompiledChunk& chunk, const BoundArgs& bound,
                                    EvalContext& childCtx) {
    auto frame = ev.acquireVmFrame();
    frame->chunk = &chunk;
    frame->code = &chunk.bodyCode;
    frame->pc = 0;
    frame->slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame->stack.clear();
    frame->bound.assign(chunk.params.size(), false);
    frame->accumStack.clear();
    frame->iterLists.assign(static_cast<size_t>(chunk.numIterLists), IterList{});
    frame->ctxChain.clear();
    frame->ctxChain.push_back(childCtx);
    frame->tailHopGuard = 0;
    frame->logicalName.clear();
    frame->hopEligible = true; // caller's own enterUserCall already made
                                // callStack_.back() this call -- see
                                // VmFrame::hopEligible.
    bindBoundArgsIntoFrame(chunk, bound, *frame);
    applyCompiledDefaultsToFrame(ev, chunk, *frame);
    const size_t floor = ev.vmCallStack_.size();
    ev.vmCallStack_.push_back(std::move(frame));
    ev.vmCallBrackets_.emplace_back(std::nullopt);
    return driveVm(ev, floor);
}

Value runCompiledExprChunk(Evaluator& ev, const CompiledChunk& chunk, EvalContext& ctx) {
    // A fresh letChildCtx(), not `ctx` directly -- Op::OpenLocalScope (a
    // nested let()'s own compiled entry) only resets SLOTS, never opens a
    // new ctx.dyn trail level: a `let($fn=...)` inside the expression
    // writes straight into whatever dyn level is current. That's harmless
    // for a genuine call frame (each of THOSE already derives its own
    // fresh, call-scoped childCtx via callCtxFor), but this wrapper is
    // handed the CALLER's own ctx directly (a statement-context expression
    // runs in the same scope as its statement, not a new call boundary),
    // so without this, a `$fn` override would leak into every statement
    // sharing `ctx` AFTER this one. Caught for real this session:
    // `v1 = let($fn=55) f(); v2 = f();` returned 55 for v2 too.
    const size_t floor = ev.vmCallStack_.size();
    pushBareFrame(ev, chunk, chunk.bodyCode, ctx.letChildCtx());
    return driveVm(ev, floor);
}

void runCompiledAssignmentBlock(Evaluator& ev, const CompiledChunk& chunk, EvalContext& ctx) {
    const size_t floor = ev.vmCallStack_.size();
    // `ctx` directly -- Op::StoreLocalAndLet/Op::StoreDyn's own writes must
    // persist into the caller's own scope; see tryCompileAssignmentBlock's
    // own doc comment (bytecode_compiler.hpp) for why a nested let()'s own
    // dyn-scoping hazard doesn't apply here (refused at compile time
    // instead of needing runtime containment).
    pushBareFrame(ev, chunk, chunk.bodyCode, ctx);
    driveVm(ev, floor);
}

void runCompiledModuleBody(Evaluator& ev, const CompiledChunk& chunk, EvalContext& childCtx) {
    // Bare push -- unlike pushBracketedModuleFrame, this frame's own
    // splice responsibility belongs to whichever NATIVE evalModularCall
    // called Evaluator::evalUserModule in the first place (it already
    // pushed treeStack_'s own accumulator before calling in here) --
    // mirrors runCompiledFunction's own bare/bracket-owned-by-caller split
    // exactly (VmFrame::ownsModuleSplice's own doc comment, bytecode_vm.hpp).
    auto frame = ev.acquireVmFrame();
    frame->chunk = &chunk;
    frame->code = &chunk.bodyCode;
    frame->pc = 0;
    // .assign, not .clear() -- a module chunk's own bodyCode has no
    // PARAMETER slots (module params are always bound natively, see this
    // function's own doc comment above), but CAN still declare LOCAL slots
    // via a nested let-EXPRESSION inside a compiled echo/assert/assignment
    // argument (Op::LetOp's own compile case, reused as-is for these --
    // see compileOneStatement's Assignment/ModularEcho/ModularAssert/
    // ModularLet cases). chunk.numSlots is 0 for the (still-common) case
    // where nothing inside this body ever does that.
    frame->slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame->stack.clear();
    frame->bound.clear();
    frame->accumStack.clear();
    frame->iterLists.assign(static_cast<size_t>(chunk.numIterLists), IterList{});
    frame->ctxChain.clear();
    frame->ctxChain.push_back(childCtx);
    frame->tailHopGuard = 0;
    frame->logicalName.clear();
    frame->ownsModuleSplice = false;
    frame->moduleRandsBefore = 0;
    frame->moduleSpliceCallNode = nullptr;
    const size_t floor = ev.vmCallStack_.size();
    ev.vmCallStack_.push_back(std::move(frame));
    ev.vmCallBrackets_.emplace_back(std::nullopt);
    driveVm(ev, floor);
}

} // namespace oscadeval
