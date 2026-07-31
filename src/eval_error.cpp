#include "openscad_cpp_evaluator/eval_error.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

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

} // namespace oscadeval
