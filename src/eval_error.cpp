#include "openscad_cpp_evaluator/eval_error.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

namespace oscadeval {

void Evaluator::error(const std::string& msg, const oscad::ASTNode& node, const std::string& innermostFrame) {
    const oscad::Position& pos = node.position();
    if (debugHooks_.errorBreak) {
        const std::string header = "ERROR: " + msg + locSuffix(&pos);
        const DebugFramesFn getFrame = [this]() { return buildDebugFrame(lastCtx_); };
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
    for (auto it = callStack.rbegin(); it != callStack.rend(); ++it) {
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
