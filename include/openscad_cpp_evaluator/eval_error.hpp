#pragma once

#include "openscad_cpp_parser/position.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace oscadeval {

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

} // namespace oscadeval
