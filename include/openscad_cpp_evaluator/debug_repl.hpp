#pragma once

#include "openscad_cpp_evaluator/debug_hooks.hpp"

#include <atomic>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace oscadeval {

class Evaluator;

// A minimal, gdb-style interactive debugger for the CLI's `--debug` flag.
// Wires into Evaluator via the same DebugHooks contract any caller uses
// (see examples/minimal_debugger.cpp). Unlike a GUI debugger, which needs
// a worker thread so pausing doesn't block the event loop, this blocks
// synchronously on stdin from inside the hook itself -- evaluate() is on
// the same thread as the prompt, so there's nothing else that needs to
// keep running while paused.
//
// Breakpoint/step-into/step-over/step-out semantics are a close port of
// the Python reference's own DebugRepl (src/openscad_evaluator/
// _debug_repl.py), not re-derived -- see debug_hooks.hpp's DebugHookFn doc
// comment for the one deliberate scope reduction (statement-level pausing
// only, no expr-level sub-statement granularity).
class DebugRepl {
public:
    // `in`/`out` default to std::cin/std::cout for real interactive use;
    // a caller (this project's own test suite, mirroring the reference's
    // own `_feed_input`-monkeypatched-`input()` pytest fixtures) can
    // inject any istream/ostream instead -- e.g. an std::istringstream
    // pre-loaded with canned REPL commands and an std::ostringstream to
    // assert against, driving the whole class in-process with no
    // subprocess spawned and no real terminal needed.
    explicit DebugRepl(const std::string& sourcePath, std::istream& in = std::cin, std::ostream& out = std::cout);

    // Lets "child" (step-to-child) read Evaluator::lastChildrenPositions().
    // Set once, after the caller (cli_lib.cpp) constructs the Evaluator it's
    // about to wire this repl's hooks into -- DebugRepl itself is
    // constructed, and its pre-run prompt run, BEFORE that Evaluator exists,
    // so this can't happen at construction time. Left unset (nullptr),
    // "child" behaves exactly like "finish" (the depth-drop fallback still
    // applies) -- matching what happens if the paused call never invokes
    // children() at all.
    void attachEvaluator(Evaluator& ev) { evaluator_ = &ev; }

    // Installs a process-wide SIGINT handler so Ctrl+C during a running
    // evaluate() call requests a pause at the next statement boundary --
    // folded into the exact same shouldPause OR-chain debugHook() already
    // uses for breakpoints/steps, so the result behaves identically to
    // hitting a breakpoint (print/backtrace/continue/step all just work).
    // Mirrors BelfrySCAD's own DebugSession::pause(), just signal-triggered
    // here instead of a GUI button click. The handler itself only sets
    // pauseRequested_ (an atomic bool -- the async-signal-safe minimum);
    // only one DebugRepl may have this active at a time (a plain C
    // function can't capture `this`, so a single "currently active
    // instance" pointer backs it) -- fine for this CLI's own
    // single-`--debug`-session-per-process usage.
    void installInterruptHandler();

    // Sets the exact same flag installInterruptHandler()'s real signal
    // handler does, without going through a real OS signal -- lets
    // tests (an in-process istream/ostream harness, no subprocess, so a
    // genuine Ctrl+C can't be delivered) simulate "the user just pressed
    // Ctrl+C" and assert the next debugHook() call pauses like a
    // breakpoint.
    void requestPause() { pauseRequested_.store(true, std::memory_order_relaxed); }

    // Interactive prompt shown before evaluation starts. Returns false if
    // the user quit without running -- the caller shouldn't call
    // evaluate() at all in that case. Mirrors run_prompt().
    bool runPrompt();

    // Wire these three into Evaluator's DebugHooks (see debug_hooks.hpp).
    DebugAction debugHook(int line, int depth, bool forced, const std::string& origin,
                           const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame);
    void errorBreak(int line, const std::string& header, const std::string& origin, const std::vector<CallStackFrame>& callStack,
                     const DebugFramesFn& getFrame);
    void returnHook(const std::string& name, const Value& result, int depth);

private:
    std::string resolveOrigin(const std::string& origin) const;
    std::pair<std::string, std::optional<int>> parseLocation(const std::string& arg) const;
    void addBreakpoint(const std::string& arg);
    void deleteBreakpoint(const std::string& arg);
    void printBreakpoints() const;
    // `origin`: which file to read lines from -- empty means sourcePath_.
    // A breakpoint/step can land inside a `use <file>`-injected function or
    // module's own body, which lives in a different file than sourcePath_;
    // this must show *that* file's lines, not always the main script's.
    void listSource(const std::string& arg, std::optional<int> currentLine = std::nullopt, const std::string& origin = "") const;
    const std::vector<std::string>& linesFor(const std::string& origin) const;
    void setVar(const std::string& arg);
    void printVar(const std::string& arg, const std::unordered_map<std::string, Value>& visibleVars);
    void printBacktrace(const std::vector<CallStackFrame>& callStack, const std::string& origin, int line) const;

    // The paused-prompt command loop -- shared by debugHook() (returns its
    // result to the evaluator) and errorBreak() (result discarded,
    // evaluation is aborting regardless; used only for its printing/
    // inspection side effects).
    DebugAction interact(int line, int depth, const std::string& origin, const std::unordered_map<std::string, Value>& visibleVars,
                          const std::vector<CallStackFrame>& callStack);
    DebugAction resume(std::optional<std::string> stepCmd, int line = 0, int depth = 0, const std::string& origin = "");

    std::istream& in_;
    std::ostream& out_;
    std::string sourcePath_;
    // Lazily populated per-origin (see linesFor()) -- mutable so listSource()
    // (a const method) can fill it in on first use of a given origin.
    mutable std::unordered_map<std::string, std::vector<std::string>> sourceLinesByOrigin_;
    std::map<std::string, std::set<int>> breakpoints_;
    bool breakOnFirst_ = true;
    std::optional<std::string> stepCmd_; // "into" | "over" | "out" | "to_child"
    int stepLine_ = 0;
    int stepDepth_ = 0;
    std::string stepOrigin_;
    // Snapshotted from Evaluator::lastChildrenPositions() when "child" is
    // issued, each origin normalized through resolveOrigin() to match how
    // debugHook()'s own `resolved` is normalized -- otherwise a raw,
    // un-normalized origin string (e.g. differing from `resolved` only by
    // a symlink, such as macOS's /var -> /private/var) would never compare
    // equal even for the objectively same file.
    std::set<std::pair<std::string, int>> stepToChildTargets_;
    std::unordered_map<std::string, Value> pendingMods_;
    int printCount_ = 0;
    bool quit_ = false;
    Evaluator* evaluator_ = nullptr;
    std::atomic<bool> pauseRequested_{false};
};

} // namespace oscadeval
