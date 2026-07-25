#pragma once

#include "openscad_cpp_evaluator/bytecode.hpp"
#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast/expression.hpp"

#include <memory>
#include <unordered_map>
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

// Binds `arguments` (evaluated against `callerCtx`, exactly like bindArgs)
// into a fresh slot array per chunk.params, applies any unbound parameter's
// compiled default, then runs the compiled body -- all against `childCtx`
// (the same callCtxFor()-derived context evalUserFunction already builds,
// for dyn/scope/color/childrenNodes; the interpreter's ctx.let_ is never
// written to for a compiled call, since every plain local this chunk knows
// about lives in the slot array instead). Mirrors evalUserFunction's own
// bindArgs+bound-loop+applyDefaults+evalExpr sequence exactly in observable
// effect, just via slots instead of a per-call unordered_map + ctx.let_.
Value runCompiledFunction(Evaluator& ev, const CompiledChunk& chunk,
                          const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& callerCtx,
                          EvalContext& childCtx);

// Same, but for a call site whose arguments are already evaluated (a
// compiled CALL_FN opcode's own callee, or Evaluator::evalUserFunctionFromBound's
// interpreter-callee sibling path) -- binds `bound`'s entries into slots by
// matching CompiledChunk::Param::name (an undeclared $-prefixed entry goes
// straight to childCtx.dyn, exactly like bindCompiledArgs' own handling;
// an undeclared plain name has no slot and is simply unreferenceable, same
// reasoning as bindCompiledArgs), applies defaults for anything left
// unbound, then runs the body.
Value runCompiledFunctionFromBound(Evaluator& ev, const CompiledChunk& chunk,
                                    const std::unordered_map<std::string, Value>& bound, EvalContext& childCtx);

} // namespace oscadeval
