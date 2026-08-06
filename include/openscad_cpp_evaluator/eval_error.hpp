#pragma once

#include "openscad_cpp_parser/position.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace oscad {
class ASTNode;
} // namespace oscad

namespace oscadeval {

struct EvalContext; // for CallStackFrame::bodyCtx (non-owning debug-frame ptr)

// Thrown for any OpenSCAD-level runtime error (assertion failure, a raised
// error(), a debugger "stop" command in a later phase). Carries the fully
// formatted "ERROR: ...\nTRACE: ..." message -- the same text a caller
// prints to the console, matching real OpenSCAD's format exactly.
class EvalError : public std::runtime_error {
public:
    explicit EvalError(const std::string& message) : std::runtime_error(message) {}
};

// One call-stack frame (module or function invocation). `declPosition` (the
// callee's own declaration start) is only meaningful for Module frames --
// see traceLines()'s TRACE-line-pair behavior below. Non-owning: valid only
// as long as the AST it points into is alive, same contract as every other
// raw AST pointer this evaluator holds (see evaluator.hpp).
struct CallStackFrame {
    enum class Kind { Module, Function };
    Kind kind;
    std::string name;
    const oscad::Position* callPosition = nullptr;
    const oscad::Position* declPosition = nullptr; // Module frames only

    // -- Bytecode VM upvalue support (Phase 2) ---------------------------
    // Set for a Function frame regardless of compiled-or-not (`declNode`
    // is cheap and harmless to always set); `vmFrame` is only non-null
    // while this specific call is actually executing compiled bytecode.
    // `declNode` is this call's own FunctionDeclaration/FunctionLiteral
    // identity -- used for LOAD_UPVALUE's exact-match search
    // (Evaluator::findUpvalue), distinct from callCtxFor's own
    // span-containment use of `declPosition`. `vmFrame` is a type-erased
    // `VmFrame*` (bytecode_vm.hpp isn't included here, to avoid a header
    // coupling this foundational header doesn't otherwise need) -- cast
    // back in bytecode_vm.cpp/user_calls.cpp, the only places that ever
    // read it.
    const oscad::ASTNode* declNode = nullptr;
    void* vmFrame = nullptr;

    // Upvalue-ancestry parent: the callStack_ INDEX (not pointer -- the
    // vector can reallocate) of the frame this one's own closure-visibility
    // chain continues from, or -1 if it terminates here. Set to the
    // CALLING frame's own index (the one active immediately before this
    // one was pushed) exactly when callCtxFor chose childCtx() for this
    // call, else -1 (callCtx() was chosen -- an isolated call, matching
    // real OpenSCAD's own "no escaping closures... only reaches through an
    // unbroken chain of lexically-nested calls" semantics).
    //
    // This is NOT the same thing as "is declNode still active somewhere on
    // callStack_" -- a real bug caught via a differential VM-off-vs-VM-on
    // test: `function apply(f,v)=f(v); function make(x)=apply(function(y)
    // y+x, 100);` -- when `f(v)` (the closure) runs INSIDE apply's own
    // (isolated, since apply isn't lexically nested in make) frame, a
    // naive "scan callStack_ for any frame whose declNode matches the
    // upvalue's target" finds `make` (still on the stack) and WRONGLY
    // resolves `x`, even though real OpenSCAD's own ctx-ancestry-based
    // closures give `x` as undef here (apply's own isolation already
    // severed the chain before the closure was ever invoked). Walking
    // upvalueParent links instead reproduces this exactly: apply's own
    // frame has upvalueParent=-1 (it's isolated from make), so the
    // closure's own chain (parent = apply's frame) terminates there
    // without ever reaching make, matching the interpreter byte-for-byte.
    int upvalueParent = -1;

    // The body EvalContext of this active call, set right after the frame is
    // pushed -- lets the debugger build per-frame local variable snapshots
    // for the whole call stack (not just the innermost frame). Non-owning;
    // valid only while this call is on callStack_ (same lifetime as every
    // other pointer here).
    const EvalContext* bodyCtx = nullptr;
};

// " in file {origin}, line {line}" -- "" if pos is null. Mirrors the Python
// reference's Evaluator._loc.
std::string locSuffix(const oscad::Position* pos);

// Builds the TRACE lines for an error/warning, walking the call stack
// innermost-first (mirrors Evaluator._trace_lines):
//   - innermostFrame, if non-empty (e.g. "assert"):
//       "TRACE: called by '<innermostFrame>'<locSuffix(nodePosition)>"
//   - then each stack frame, innermost (last pushed) first:
//       Module:   "TRACE: call of '<name>()'<locSuffix(declPosition)>"
//                 "TRACE: called by '<name>'<locSuffix(callPosition)>"
//       Function: "TRACE: called by '<name>'<locSuffix(callPosition)>"
// One deliberate departure from the reference: once callStack itself has
// more than a small number of frames, only the innermost and outermost
// few are shown, with a single "... N more frames ..." marker in
// between -- see traceLines()'s own definition (eval_error.cpp) for why.
// The reference never needed this (Python's own recursion limit makes a
// callStack this deep unreachable there); this port's explicit-stack VM
// (Stage 1/2) can legitimately recurse thousands of calls deep, and
// printing one line per frame turned a single error into a
// multi-megabyte message.
std::vector<std::string> traceLines(const oscad::Position* nodePosition,
                                     const std::vector<CallStackFrame>& callStack,
                                     const std::string& innermostFrame = "");

// "ERROR: {msg}{locSuffix(nodePosition)}" followed by traceLines(), each on
// its own line -- the exact multi-line format real OpenSCAD prints. Callers
// needing the full Evaluator::error() behavior (recording to an error log,
// invoking an error-break callback, then throwing) build that on top of
// this once the Evaluator class carries a real call stack (Phase 4+).
std::string formatError(const std::string& msg, const oscad::Position* nodePosition,
                         const std::vector<CallStackFrame>& callStack,
                         const std::string& innermostFrame = "");

// "WARNING: {msg}{locSuffix(nodePosition)}, from {entry file}, line {N}"
// followed by traceLines(). The "from" clause names the OUTERMOST frame's
// call site -- the user's own line that entered the call chain -- because
// a warning raised inside a library otherwise reports only the library's
// own line, which the reader can neither act on nor trace back.
//
// Omitted when the stack is empty (a top-level warning, the common case,
// which stays a single line) or when it would merely repeat the headline.
// Deliberately richer than real OpenSCAD, which prints neither.
std::string formatWarning(const std::string& msg, const oscad::Position* nodePosition,
                           const std::vector<CallStackFrame>& callStack);

} // namespace oscadeval
