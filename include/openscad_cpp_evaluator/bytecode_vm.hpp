#pragma once

#include "openscad_cpp_evaluator/bound_args.hpp"
#include "openscad_cpp_evaluator/bytecode.hpp"
#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast/expression.hpp"

#include <memory>
#include <vector>

namespace oscadeval {

class Evaluator;

// A reusable scratch buffer set for one compiled-call activation: the VM
// operand stack, the parameter/local slot array, and the "was this param
// actually bound by the caller" mask. Pooled on Evaluator (see
// acquireVmFrame()/releaseVmFrame()) rather than freshly heap-allocated per
// call -- profiling found 3 fresh vector allocations per call was enough to
// erase the whole point of avoiding bindArgs' single unordered_map
// allocation (see this project's own session notes / CLAUDE.md for the
// measured-not-assumed finding that motivated this). Pooling is safe under
// recursion: a still-executing (nested/recursive) call's own frame is
// never in the pool to be handed out again -- it's only returned once that
// call's own RAII guard destructs, and each acquire either reuses an
// already-returned frame or allocates a genuinely new one, never aliasing
// a frame two active calls both hold.
struct VmFrame {
    std::vector<Value> stack;
    std::vector<Value> slots;
    std::vector<bool> bound;
};

// Filled by runChunk's Op::CallFnTail/CallDynamicTail handler when a tail
// hop is eligible to trampoline (isolated -- see Evaluator::
// isolatedCallCtxFor's own doc comment) instead of recursing for real.
// `ctx` is the ALREADY-DERIVED isolated-call EvalContext (callCtxFor's own
// callCtx() branch, computed once by isolatedCallCtxFor rather than
// re-derived by the trampoline loop) -- a fresh $-var (dyn) trail level
// for this hop, matching what a real (non-tail) call would have built via
// evalUserFunctionFromBound/evalFunctionLiteralFromBound's own callCtxFor
// call. Exactly one of `decl`/`literal` is non-null; both null means "no
// tail request" (the sentinel runCompiledFunctionTrampoline/
// runCompiledFunctionFromBoundTrampoline check for after each hop).
struct TailCallRequest {
    const oscad::FunctionDeclaration* decl = nullptr;
    const oscad::FunctionLiteral* literal = nullptr;
    BoundArgs bound;
    EvalContext ctx;
    std::string name;
    const oscad::Position* callPos = nullptr;
};

// Binds `arguments` (evaluated against `callerCtx`, exactly like bindArgs)
// into a fresh slot array per chunk.params, applies any unbound parameter's
// compiled default, then runs the compiled body -- all against `childCtx`
// (the same callCtxFor()-derived context evalUserFunction already builds,
// for dyn/scope/color/childrenNodes; the interpreter's ctx.let_ is never
// written to for a compiled call, since every plain local this chunk knows
// about lives in the slot array instead). Mirrors evalUserFunction's own
// bindArgs+bound-loop+applyDefaults+evalExpr sequence exactly in observable
// effect, just via slots instead of a per-call unordered_map + ctx.let_.
// `tailOut`: forwarded to the body's own runChunk call -- see
// runCompiledFunctionTrampoline for the only caller that passes non-null.
Value runCompiledFunction(Evaluator& ev, const CompiledChunk& chunk,
                          const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& callerCtx,
                          EvalContext& childCtx, TailCallRequest* tailOut = nullptr);

// Same, but for a call site whose arguments are already evaluated (a
// compiled CALL_FN opcode's own callee, or Evaluator::evalUserFunctionFromBound's
// interpreter-callee sibling path) -- binds `bound`'s entries into slots by
// matching CompiledChunk::Param::name (an undeclared $-prefixed entry goes
// straight to childCtx.dyn, exactly like bindCompiledArgs' own handling;
// an undeclared plain name has no slot and is simply unreferenceable, same
// reasoning as bindCompiledArgs), applies defaults for anything left
// unbound, then runs the body. `tailOut`: see runCompiledFunction's own
// note -- this is also the function every trampoline hop AFTER the first
// reuses directly (see runCompiledFunctionTrampoline/
// runCompiledFunctionFromBoundTrampoline), since a tail request's own
// bound-args shape is identical to this function's own parameter shape.
Value runCompiledFunctionFromBound(Evaluator& ev, const CompiledChunk& chunk, const BoundArgs& bound,
                                    EvalContext& childCtx, TailCallRequest* tailOut = nullptr);

// The trampoline entry points -- replace runCompiledFunction/
// runCompiledFunctionFromBound at evalUserFunction's/evalUserFunctionFromBound's/
// evalFunctionLiteral's/evalFunctionLiteralFromBound's own call sites
// (user_calls.cpp). Runs the first (real) call normally, then loops on
// any TailCallRequest that comes back -- each subsequent hop reuses
// runCompiledFunctionFromBound directly (a tail request's bound-args
// shape already matches its own parameter shape exactly), acquiring a
// fresh VmFrame each time (releasing the previous one -- safe, since only
// an ISOLATED hop ever reaches here, meaning nothing can hold an upvalue
// into a tail-discarded frame's own slots) and recording the hop via
// Evaluator::recordTailCallHop (callStack_ frame mutation, the
// 1,000,000-iteration cap, profiling). See these functions' own .cpp
// definitions for why every hop's own EvalContext must be kept alive for
// the whole trampoline run (a std::vector<EvalContext> chain, mirroring
// evalFunctionBodyTrampoline's identical fix on the interpreter side).
Value runCompiledFunctionTrampoline(Evaluator& ev, const CompiledChunk& chunk,
                                     const std::vector<std::unique_ptr<oscad::Argument>>& arguments,
                                     EvalContext& callerCtx, EvalContext& childCtx);
Value runCompiledFunctionFromBoundTrampoline(Evaluator& ev, const CompiledChunk& chunk, const BoundArgs& bound,
                                              EvalContext& childCtx);

} // namespace oscadeval
