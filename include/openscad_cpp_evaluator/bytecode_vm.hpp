#pragma once

#include "openscad_cpp_evaluator/bound_args.hpp"
#include "openscad_cpp_evaluator/bytecode.hpp"
#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/csg_node.hpp"
#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast/expression.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace oscadeval {

class Evaluator;

// One still-open Op::PushBuiltinWrap bracket's own state, needed again by
// its matching Op::PopBuiltinWrap -- see that op's own doc comment
// (bytecode.hpp) for the full contract. Lives in the header for the same
// reason IterList does, just above: it travels with a VmFrame (VmFrame::
// builtinWrapStack, below), not a free-standing local, since Push/
// PopBuiltinWrap pairs can nest or sequence within one frame's own
// instruction stream.
struct PendingBuiltinWrap {
    CSGParams params;
    std::uint64_t randsBefore = 0;
    int siteIdx = -1;
    // Only populated for BuiltinWrapSite::Kind::Roof -- the already-
    // resolved call arguments, retained across the whole bracket so
    // Op::PopBuiltinWrap's handler can compute roof()'s own params AFTER
    // children finish (computeRoofParams needs them; re-running
    // resolveCallArgs at Pop time instead would re-evaluate every argument
    // expression a second time -- double rands()/side effects). Every
    // other kind computes its params at Push time and leaves this default-
    // empty.
    CallArgs deferredArgs;

    // ev.measuring_ as it stood immediately BEFORE this bracket opened.
    // Recorded for EVERY kind, not just Measure, so the exception teardown
    // can restore from front() without inspecting kinds -- a non-Measure
    // entry simply records and restores the same value.
    bool savedMeasuring = false;
    // f.stack.size() and ev.treeStack_.size() at Push time. Kind::Measure's
    // Pop asserts both: it is the first bracket that runs STATEMENT opcodes
    // with a non-empty operand stack beneath it, and nothing structurally
    // enforces that those statements are operand-stack-neutral.
    size_t stackDepthAtPush = 0;
    size_t treeStackDepthAtPush = 0;
};

// One still-open Op::PushCsgWrap bracket's own state -- see that op's own
// doc comment (bytecode.hpp) for the full contract. `groupSizes` is built
// up incrementally, one entry per Op::CsgGroupStart/CsgGroupEnd pair (one
// per top-level GEOMETRY child statement); `groupStartSize` is scratch
// space for the CURRENTLY OPEN group only (set by CsgGroupStart, consumed
// by the matching CsgGroupEnd) -- safe as a single scalar, not a stack of
// its own, because groups within one CSG wrap are siblings in sequence,
// never nested (unlike PushCsgWrap brackets themselves, which CAN nest,
// e.g. `union() { difference() {...} }` -- that's what makes
// VmFrame::csgWrapStack itself a real LIFO, below).
struct PendingCsgWrap {
    std::string op;
    std::uint64_t randsBefore = 0;
    int siteIdx = -1;
    std::vector<Value> groupSizes;
    size_t groupStartSize = 0;
};

// One ListCompFor/statement-for assignment's own materialized iteration
// state -- see Op::IterMaterialize/IterReset/IterNext's own doc comments
// (bytecode.hpp). Lives in the header (not bytecode_vm.cpp's own anonymous
// namespace, where it used to be a purely internal type) because VmFrame,
// below, now owns one vector of these per frame -- iteration state belongs
// to whichever logical call is running it, exactly like `slots`/`stack`
// already do, so it has to travel with the frame across a push/pop the same
// way.
struct IterList {
    IterableValues values;
    size_t index = 0;
    // Cached once at IterMaterialize time -- IterableValues::size() is O(1)
    // for a list/owned vector, but walks a RANGE's whole sequence from
    // scratch every call. Recomputing it once per IterNext call turned a
    // single range-based for()/list-comp `for` into an O(n^2) walk (a
    // 100,000-element range: interpreter ~0.03s, VM ~9.5s) -- caught via a
    // BOSL2 corpus sweep (test_math.scadtest's own gaussian_rands(), which
    // builds a 100,000-element list via `count()`).
    size_t total = 0;
};

// One activation record in the explicit, heap-allocated VM call stack
// (Evaluator::vmCallStack_) that replaced native C++ recursion for calls
// between two compiled chunks -- see this project's own session notes for
// the full "iterate over the code, don't use the C++ call stack" rationale.
// Pooled (Evaluator::acquireVmFrame()/releaseVmFrame(), a vmFramePool_ of
// unique_ptr<VmFrame>) rather than freshly heap-allocated per logical call
// -- profiling found 3 fresh vector allocations per call was enough to
// erase the whole point of the VM (measured, not assumed) -- `slots`/
// `stack` are `.assign()`/`.clear()`-reset on reuse rather than
// reallocated, exactly as before this redesign.
//
// `chunk` vs. `code`: `chunk` is this frame's own CompiledChunk (constants/
// names/callSites/closureSites/etc.); `code` is the SPECIFIC instruction
// vector actually being run -- almost always `&chunk->bodyCode`, but a
// parameter-default-value evaluation runs `&chunk->defaultCode[i]` instead
// (a genuinely different instruction sequence belonging to the SAME
// chunk). Kept as two separate pointers rather than assuming "code is
// always this chunk's own body" specifically so default-value evaluation
// can be driven by the exact same push/pop machinery as everything else
// (including a call made FROM a default value, however rare) instead of a
// second, parallel execution path.
//
// `ctxChain`: NEVER overwrite `ctxChain.back()` in place to "reset" a
// frame's context on a tail hop -- EvalContext's own let_/dyn/dynExplicit
// are shared_ptr<TrailView<T>> views onto a shared, mutable
// ScopeTrailStorage, and ~TrailView() pops its own trail level the INSTANT
// its last reference drops (scope_trail.hpp). Overwriting `ctx` in place
// would drop the old EvalContext's refcount to zero immediately, popping a
// trail level a still-reachable $-var lookup may need to walk back through
// later in the same tail chain -- exactly the bug
// runCompiledFunctionTrampoline's own (now-retired) `chain` vector existed
// to prevent, one layer down. Every hop (tail or not) does
// `ctxChain.push_back(newCtx)`; "current ctx" is always `ctxChain.back()`;
// torn down back-to-front (never relying on the vector's own unspecified
// destruction order -- libstdc++/libc++ disagree, see issue #50) whenever
// this frame is popped, on every exit path including exceptions.
//
// Whether (and under what name) THIS frame carries its own callStack_/
// profiling bracket lives OUTSIDE VmFrame itself, in a parallel vector on
// Evaluator (vmCallBrackets_, pushed/popped in lockstep with
// vmCallStack_ by the driver) -- VmFrame can't hold an
// Evaluator::UserCallHandle directly (evaluator.hpp includes THIS header,
// so the reverse include would be circular). A frame gets a real bracket
// (via Evaluator::enterUserCall) only when it represents a genuine NEW
// logical function/closure call -- pushed by the driver's own non-tail
// Op::CallFn/CallDynamic handling, exactly mirroring what
// evalUserFunctionFromBound/evalFunctionLiteralFromBound used to do via a
// native recursive call. A "bare" frame (no entry in vmCallBrackets_) --
// the very first frame of a top-level runCompiledFunction(FromBound) call
// (its own bracket belongs to the OUTER evalUserFunctionCore/
// enterUserCall pair that invoked it), a bare statement-context expression
// (runCompiledExprChunk), an assignment block (runCompiledAssignmentBlock),
// or a parameter-default-value evaluation -- never touches callStack_/
// profiling on its own, matching each of those call shapes' existing
// (pre-redesign) behavior exactly.
struct VmFrame {
    const CompiledChunk* chunk = nullptr;
    const std::vector<Instruction>* code = nullptr;
    size_t pc = 0;
    std::vector<Value> slots;
    std::vector<Value> stack;
    std::vector<bool> bound;
    std::vector<std::vector<Value>> accumStack;
    std::vector<IterList> iterLists;
    std::vector<EvalContext> ctxChain;
    // Still-open Op::PushBuiltinWrap brackets, LIFO, scoped to THIS frame's
    // own instruction stream -- mirrors accumStack/iterLists' own role for
    // their respective bracket pairs. Normal execution always drains this
    // back to empty (every Push has a compile-time-matched Pop); only the
    // exception-teardown path (teardownVmCallStackDownTo, bytecode_vm.cpp)
    // needs to know its size, to pop the matching number of extra
    // treeStack_ entries for a frame torn down mid-bracket. Not explicitly
    // cleared in releaseVmFrame, same as accumStack/ctxChain -- the
    // existing invariant ("whoever pops this frame drains its own open
    // brackets first") already covers it.
    std::vector<PendingBuiltinWrap> builtinWrapStack;
    // Op::PushCsgWrap's own per-frame LIFO -- same role/lifetime/teardown
    // discipline as builtinWrapStack, just for union()/difference()/
    // intersection() (see PendingCsgWrap's own doc comment, above, and
    // Op::PushCsgWrap's, bytecode.hpp). Not explicitly cleared in
    // releaseVmFrame, same as builtinWrapStack/accumStack/ctxChain -- the
    // existing invariant ("whoever pops this frame drains its own open
    // brackets first", normally via matched Push/Pop, or via
    // teardownVmCallStackDownTo on the exception path) already covers it.
    std::vector<PendingCsgWrap> csgWrapStack;
    // The ORIGINAL callee name at push time, used by
    // Evaluator::exitUserCallSuccess's own returnHook call when this frame
    // carries a bracket -- deliberately NOT updated by a later tail hop
    // within this SAME frame (recordTailCallHop only mutates callStack_'s
    // own entry), mirroring evalUserFunctionCore's existing behavior of
    // always reporting the original entry name regardless of how many tail
    // hops happened inside it. Unused for a bare frame.
    std::string logicalName;
    // Evaluator::recordTailCallHop's own runaway-loop counter (the
    // existing 1,000,000-iteration cap, user_calls.cpp) -- used to live as
    // a local variable in the now-retired runCompiledFunctionTrampoline's
    // own while loop, persisting for that trampoline's whole run; now
    // persists here instead, since a tail hop replaces THIS frame in place
    // rather than looping in a separate function. Reset to 0 whenever this
    // frame is (re)constructed for a NEW logical call (a tail hop within
    // an already-running frame does NOT reset it -- same "one counter per
    // trampoline run" semantics as before).
    unsigned tailHopGuard = 0;

    // -- Module-body compilation (Stage 2) --------------------------------
    // True only for a frame pushed by Op::CallModule (bytecode_vm.cpp) --
    // i.e. a NESTED module call made from within another already-compiled
    // module body, where there's no native evalModularCall left around it
    // to do the splice-vs-union post-processing. driveVm's own completion
    // branch checks this to decide whether popping this frame should ALSO
    // run Evaluator::spliceModuleChildren before resuming the caller.
    // False for runCompiledModuleBody's own (bare) top-level frame -- its
    // splice responsibility belongs to whatever NATIVE evalModularCall
    // called Evaluator::evalUserModule in the first place, exactly
    // mirroring runCompiledFunction's own bare/bracket-owned-by-caller
    // split (see VmFrame's own doc comment above, and enterUserCall's
    // skipDepthGuard).
    bool ownsModuleSplice = false;
    // Evaluator::randsCallCount_ captured at THIS frame's own push time
    // (mirrors evalModularCall's own `randsBefore` local) -- only
    // meaningful when ownsModuleSplice is true; read back by
    // spliceModuleChildren when this frame completes.
    std::uint64_t moduleRandsBefore = 0;
    // The ModularCall AST node this frame's own splice (if any) should
    // stamp onto a synthetic ">1 spliced child" union-wrapper CSGNode --
    // mirrors evalModularCall's own `&node` (csg_resolve.cpp). Only
    // meaningful when ownsModuleSplice is true.
    const oscad::ASTNode* moduleSpliceCallNode = nullptr;

    // Whether callStack_.back() (at push time) genuinely IS this frame's
    // own logical call, and therefore safe for a tail hop inside this
    // frame to overwrite via recordTailCallHop. This is NOT the same
    // question as "does this frame own a vmCallBrackets_ entry to
    // release" (Evaluator::vmCallBrackets_, see its own push sites) --
    // runCompiledFunction/runCompiledFunctionFromBound's own initial
    // frame is bare (no entry of ITS own in vmCallBrackets_, since the
    // bracket belongs to the CALLER's enterUserCall, already active
    // before either function is even invoked) but callStack_.back() is
    // still correctly THIS call the moment either one starts running --
    // so a tail call made directly from a top-level compiled function's
    // own body must still be hop-eligible, exactly like a nested
    // pushBracketedCallFrame call. Only genuinely call-boundary-free
    // pushes (runCompiledExprChunk, runCompiledAssignmentBlock, a
    // parameter default's own nested frame) leave this false -- for
    // those, callStack_.back() (if anything) belongs to some unrelated
    // enclosing call, and recordTailCallHop must not touch it. Found via
    // BytecodeCompiler.ClosureWithDollarParameterNowCompilesToo: without
    // this distinction EVERY top-level compiled call's own first tail
    // call fell through to a genuine (bracketed) push instead of hopping,
    // a correct-but-wasteful extra frame+checkDebug stop, not a value bug.
    bool hopEligible = false;
};

// Binds `arguments` (raw AST, evaluated against `callerCtx`) into a fresh
// compiled call and runs it to completion -- replaces
// runCompiledFunctionTrampoline (retired, folded in here: tail hops are
// now handled entirely internally by the shared VM driver, never surfaced
// to callers). Mirrors evalUserFunction's own bindArgs+bound-loop+
// applyDefaults+evalExpr sequence exactly in observable effect, just via
// slots instead of a per-call unordered_map + ctx.let_. This call's OWN
// callStack_/profiling bracket is the CALLER's (evalUserFunctionCore,
// already active by the time this runs) -- this function pushes a single
// BARE frame (see VmFrame::callBracket's own doc comment) and drives it;
// only a NON-TAIL call made from within its body to another compiled
// function gets its own bracket, pushed by the driver itself.
Value runCompiledFunction(Evaluator& ev, const CompiledChunk& chunk,
                          const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& callerCtx,
                          EvalContext& childCtx);

// Same, but for a call site whose arguments are already evaluated (a
// compiled Op::CallFn opcode's own callee, or
// Evaluator::evalUserFunctionFromBound's interpreter-callee sibling path,
// or a tail hop) -- binds `bound`'s entries into slots by matching
// CompiledChunk::Param::name (an undeclared $-prefixed entry goes straight
// to childCtx.dyn; an undeclared plain name has no slot and is simply
// unreferenceable), applies defaults for anything left unbound, then runs
// the body to completion. Replaces runCompiledFunctionFromBoundTrampoline
// (retired) for the same reason as runCompiledFunction, above.
Value runCompiledFunctionFromBound(Evaluator& ev, const CompiledChunk& chunk, const BoundArgs& bound,
                                    EvalContext& childCtx);

// Runs a bare statement-context chunk (see tryCompileStatementExpr,
// bytecode_compiler.hpp) -- no parameters to bind, no defaults, never
// itself a tail-hop target. `ctx` is used AS-IS, not derived via
// callCtxFor -- a statement-context expression runs in the same scope as
// the statement it belongs to, not a new call boundary, exactly like the
// interpreter's own evalExpr(*node.expr, ctx) call it replaces. A call
// made FROM within this expression to a compiled function still gets the
// full explicit-frame-stack treatment (pushed by the shared driver) --
// only THIS bare wrapper frame itself carries no callStack_/profiling
// bracket of its own.
Value runCompiledExprChunk(Evaluator& ev, const CompiledChunk& chunk, EvalContext& ctx);

// Runs a compiled ASSIGNMENT BLOCK (see Evaluator::tryCompileAssignmentBlock,
// user_calls.cpp) -- a run of sibling assignment statements sharing one
// scope. Unlike runCompiledExprChunk, `ctx` is used directly, NOT via a
// fresh letChildCtx(): every Op::StoreLocalAndLet writes straight into
// ctx.let_ (and any $-assignment, if this block has one, straight into
// ctx.dyn via Op::StoreDyn) precisely so those writes OUTLIVE this call and
// stay visible to whatever runs after it in the same scope -- the entire
// point of compiling the block at all. Safe from the same nested-`let()`
// dyn-leak hazard runCompiledExprChunk's own letChildCtx() guards against
// only because tryCompileAssignmentBlock refuses to compile a block
// containing one in the first place (scans the compiled bodyCode for any
// Op::StoreDyn).
void runCompiledAssignmentBlock(Evaluator& ev, const CompiledChunk& chunk, EvalContext& ctx);

// Runs a compiled MODULE body (see tryCompileModuleBody, bytecode_
// compiler.hpp) to completion -- the module-side analog of
// runCompiledFunction/runCompiledFunctionFromBound. Pushes a single BARE
// frame (ownsModuleSplice=false: this call's own splice responsibility
// belongs to whichever NATIVE evalModularCall invoked
// Evaluator::evalUserModule, which already pushed treeStack_'s own
// accumulator before calling in here) and drives it; only a NESTED
// Op::CallModule made from within its body gets its own bracket+splice
// responsibility, pushed by the driver itself. `childCtx` is the fully
// prepared child scope (parameters bound, $children/$parent_modules set,
// defaults applied) -- see Evaluator::buildModuleChildCtx, which both
// this entry point's caller (evalUserModule) and Op::CallModule's own
// handler use to build it identically either way. No return value: a
// module body's whole effect is the side effect of whatever CSGNodes
// landed in treeStack_ while it ran.
void runCompiledModuleBody(Evaluator& ev, const CompiledChunk& chunk, EvalContext& childCtx);

} // namespace oscadeval
