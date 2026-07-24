#pragma once

#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace oscadeval {

// Callback injection points, mirroring the Python reference's
// Evaluator.__init__(echo_fn=None, debug_hook=None, error_break_fn=None,
// return_hook=None): callback injection, not GUI/toolkit coupling. No
// QObject, no signals -- a caller (e.g. a Qt-based host app, or this
// project's own CLI --debug REPL, debug_repl.hpp) wires these to its own
// event system; all default to empty/no-op so a bare Evaluator
// construction needs none of them.

// echo()/WARNING output. Mirrors Evaluator._echo_fn(message) -- message
// already has any "WARNING: "/formatting applied, this is just "print it
// somewhere."
using EchoFn = std::function<void(const std::string& message)>;

// -- Debugging (Phase 9) -----------------------------------------------

// Currently-visible variable names -> values at a debug pause, already
// merged the way the reference's own REPL actually consumes them: plain
// (`let`) locals plus `$`-prefixed dynamic vars for the *innermost* active
// frame, plus (if that frame is nested inside a user call) any
// unshadowed top-level script variable. Mirrors what
// Evaluator._build_frame_locals computes for `all_frame_locals[0]` --
// deliberately not the *whole* call-stack's own per-frame locals, since
// neither this port's debug_repl.cpp nor the reference's own
// _debug_repl.py ever look past frame 0 (only backtrace walks the rest,
// and that only needs names/positions, already on CallStackFrame).
struct DebugFrame {
    std::unordered_map<std::string, Value> locals;
};

// Lazily computes DebugFrame -- mirrors the reference's own `get_frames`
// callable parameter: a hook that only traces line numbers (never
// inspects variables) shouldn't pay for a locals snapshot on every
// statement.
using DebugFramesFn = std::function<DebugFrame()>;

// A debug hook's response: `stop` aborts the whole evaluate() call
// (mirrors returning cmd="stop", which raises EvalError); `mods` are
// applied to the paused statement's own `let` scope before resuming
// (mirrors the returned `mods` dict -- a debugger's "set variable"
// command).
struct DebugAction {
    bool stop = false;
    std::unordered_map<std::string, Value> mods;
};

// Called at (approximately) every top-level statement in every block --
// see Evaluator::evalChildren's single call site for the exact mechanism,
// and its own comment for why this port checks at statement granularity
// only, not the reference's additional expr_level sub-expression checks
// (list-comprehension clauses, ternary branches, ...): those exist in the
// reference to let a debugger single-step through individual expression
// evaluation, which no consumer of this seam (debug_repl.cpp's REPL,
// nor a plausible future GUI one) actually needs -- "pause on any
// statement, inspect locals, set a breakpoint" is fully served by
// statement-level granularity alone, at a small fraction of the
// reference's own call-site count. Also called once, with `forced=true`,
// right before evaluating a user function/function-literal's body
// expression (functions have no statement-level granularity to hang a
// hook off otherwise) and by breakpoint() (forced=true, unconditionally,
// matching the reference's own _check_debug(..., forced=True) call).
// `depth` is the live user-call-stack depth (Evaluator::callStack_.size())
// at the pause point. Mirrors Evaluator._check_debug/the debug_hook
// callback contract.
using DebugHookFn = std::function<DebugAction(int line, int depth, bool forced, const std::string& origin,
                                               const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame)>;

// Called from Evaluator::error() right before it throws, if set -- gives a
// debugger one last chance to inspect state at the failure site. Mirrors
// error_break_fn.
using ErrorBreakFn = std::function<void(int line, const std::string& header, const std::string& origin,
                                         const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame)>;

// Called right after a user function/function-literal call computes its
// result, before returning -- lets a debugger's `finish` command report
// "value returned is ...". Mirrors return_hook(name, value, depth).
using ReturnHookFn = std::function<void(const std::string& name, const Value& result, int depth)>;

// Bundles the 3 debugging-related callbacks into one constructor
// parameter (rather than 3 more positional Evaluator(...) arguments) --
// default-constructed (every field null) means "debugging off", mirroring
// the reference's `self._debugging = debug_hook is not None`.
struct DebugHooks {
    DebugHookFn debugHook;
    ErrorBreakFn errorBreak;
    ReturnHookFn returnHook;
};

} // namespace oscadeval
