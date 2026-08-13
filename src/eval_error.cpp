#include "openscad_cpp_evaluator/eval_error.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include <algorithm>

namespace oscadeval {

void Evaluator::error(const std::string& msg, const oscad::ASTNode& node, const std::string& innermostFrame) {
    const oscad::Position& pos = node.position();
    if (debugHooks_.errorBreak) {
        const std::string header = "ERROR: " + msg + locSuffix(&pos);
        const DebugFramesFn getFrame = [this]() { return buildDebugFrames(lastCtx_); };
        debugHooks_.errorBreak(pos.line, header, pos.origin, callStack_, getFrame);
    }
    throw EvalError(formatError(msg, &pos, callStack_, innermostFrame));
}

std::string locSuffix(const oscad::Position* pos) {
    if (pos == nullptr) return "";
    return " in file " + pos->origin + ", line " + std::to_string(pos->line);
}

std::vector<std::string> traceLines(const oscad::Position* nodePosition,
                                     const std::vector<CallStackFrame>& callStack,
                                     const std::string& innermostFrame) {
    std::vector<std::string> lines;
    if (!innermostFrame.empty()) {
        lines.push_back("TRACE: called by '" + innermostFrame + "'" + locSuffix(nodePosition));
    }

    // A tail-recursive chain mutates ONE CallStackFrame in place per hop
    // (recordTailCallHop, user_calls.cpp), so callStack itself never
    // reflects how many hops actually happened -- its trace is already
    // small regardless of depth (see TailCalls.
    // ErrorThrownDeepInATailChainProducesABoundedTrace). A genuinely
    // non-tail recursive chain has no such collapsing: every call is a
    // real, distinct CallStackFrame, and Stage 1/2's own explicit-stack
    // VM means that chain can legitimately run thousands of frames deep
    // now (confirmed reachable, not hypothetical: CsgTree.
    // IncrementalGeometryChainHitsTheDepthGuardCleanlyInsteadOfCrashing
    // produced a multi-megabyte error message before this bound existed).
    // Shows the innermost kFramesEachEnd frames (closest to the actual
    // error -- generally the most useful for debugging) and the
    // outermost kFramesEachEnd (the original entry point, for overall
    // context), with one marker line summarizing how many were skipped
    // in between -- the same shape most language tracebacks use for a
    // pathologically deep stack.
    constexpr size_t kFramesEachEnd = 10;
    const size_t total = callStack.size();
    const bool truncate = total > 2 * kFramesEachEnd;

    size_t idx = 0;
    for (auto it = callStack.rbegin(); it != callStack.rend(); ++it, ++idx) {
        if (truncate && idx >= kFramesEachEnd && idx < total - kFramesEachEnd) {
            if (idx == kFramesEachEnd) {
                const size_t skipped = total - 2 * kFramesEachEnd;
                lines.push_back("TRACE: ... " + std::to_string(skipped) + " more frame" + (skipped == 1 ? "" : "s") +
                                 " ...");
            }
            continue;
        }
        const CallStackFrame& frame = *it;
        if (frame.kind == CallStackFrame::Kind::Module) {
            lines.push_back("TRACE: call of '" + frame.name + "()'" + locSuffix(frame.declPosition));
            lines.push_back("TRACE: called by '" + frame.name + "'" + locSuffix(frame.callPosition));
        } else {
            lines.push_back("TRACE: called by '" + frame.name + "'" + locSuffix(frame.callPosition));
        }
    }
    return lines;
}

std::string formatError(const std::string& msg, const oscad::Position* nodePosition,
                         const std::vector<CallStackFrame>& callStack, const std::string& innermostFrame) {
    std::string full = "ERROR: " + msg + locSuffix(nodePosition);
    for (const std::string& line : traceLines(nodePosition, callStack, innermostFrame)) {
        full += "\n" + line;
    }
    return full;
}

std::string formatWarning(const std::string& msg, const oscad::Position* nodePosition,
                           const std::vector<CallStackFrame>& callStack) {
    // The warning keeps its OWN location in the headline: that is where the
    // problem actually is, and it's what you need in order to fix a genuine
    // library bug. But on its own, a warning raised deep inside a library
    // ("in file BOSL2/vnf.scad, line 1624") is close to useless -- the
    // reader's next question is always "which line of MINE caused that?",
    // and nothing in the message answered it.
    //
    // callStack is outermost-first, so front() is the call that entered
    // this chain from the top level -- the user's own line, however deep
    // the warning surfaced. Naming it inline makes the common case a
    // one-line answer; the TRACE lines below carry the full chain for when
    // an intermediate frame is the real culprit.
    //
    // Real OpenSCAD reports only the raising location and no trace at all
    // (verified against 2022.08.22), so this is deliberately more than the
    // reference gives: warnings there routinely point into library
    // internals with no way back to the caller.
    std::string full = "WARNING: " + msg + locSuffix(nodePosition);

    if (!callStack.empty()) {
        const oscad::Position* entry = callStack.front().callPosition;
        // Skip the clause when it would only repeat the headline (a warning
        // raised directly at the top-level call site), or when the entry
        // frame recorded no call position.
        const bool sameSpot = entry != nullptr && nodePosition != nullptr &&
                               entry->origin == nodePosition->origin && entry->line == nodePosition->line;
        if (entry != nullptr && !sameSpot) {
            full += ", from " + entry->origin + ", line " + std::to_string(entry->line);
        }
    }

    // Same shape and truncation as an error's trace, including the
    // 10-frames-each-end bound for pathologically deep chains. A warning
    // raised outside any call has an empty stack and stays a single line --
    // the overwhelmingly common case, which keeps ordinary warnings from
    // sprouting a trace nobody needs.
    for (const std::string& line : traceLines(nodePosition, callStack, "")) {
        full += "\n" + line;
    }
    return full;
}

bool isConfigVariable(const std::string& name) {
    return !name.empty() && name[0] == '$' && name != "$children";
}

bool declaresParam(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params, const std::string& name) {
    for (const auto& p : params) {
        if (p->name->name == name) return true;
    }
    return false;
}

void warnUnexpectedNamedArg(Evaluator& ev, const std::string& name, const oscad::Position* pos) {
    ev.warn("variable " + name + " not specified as parameter", pos);
}

void warnTooManyPositionalArgs(Evaluator& ev, const oscad::Position* pos) {
    ev.warn("Too many unnamed arguments supplied", pos);
}

void warnUnexpectedArgs(Evaluator& ev, const std::vector<std::string>& params,
                         const std::vector<std::unique_ptr<oscad::Argument>>& arguments) {
    size_t positionalIdx = 0;
    for (const auto& argPtr : arguments) {
        if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            const std::string& name = static_cast<const oscad::NamedArgument&>(*argPtr).name->name;
            if (!isConfigVariable(name) &&
                std::find(params.begin(), params.end(), name) == params.end()) {
                warnUnexpectedNamedArg(ev, name, &argPtr->position());
            }
        } else if (positionalIdx++ == params.size()) {
            warnTooManyPositionalArgs(ev, &argPtr->position());
        }
    }
}

} // namespace oscadeval
