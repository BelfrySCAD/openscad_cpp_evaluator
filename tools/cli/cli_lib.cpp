#include "cli_lib.hpp"

#include "openscad_cpp_evaluator/debug_repl.hpp"
#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/eval_use.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/export.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <algorithm>
#include <filesystem>
#include <optional>

// Mirrors just enough of openscad_evaluator's own cli.py to prove the
// pipeline end to end: format-from-extension dispatch, and --debug
// dropping into a gdb-style REPL (debug_repl.hpp) instead of running
// straight through.
namespace oscadeval {

namespace {

std::string formatForPath(const std::string& explicitFormat, const std::string& outputPath) {
    if (!explicitFormat.empty()) return explicitFormat;
    std::string ext = std::filesystem::path(outputPath).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".stl") return "stl";
    if (ext == ".obj") return "obj";
    if (ext == ".off") return "off";
    if (ext == ".3mf") return "3mf";
    return "";
}

} // namespace

int runCli(const std::vector<std::string>& args, std::istream& in, std::ostream& out, std::ostream& err) {
    std::string inputPath;
    std::string outputPath;
    std::string format;
    bool debug = false;
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "-o" && i + 1 < args.size()) {
            outputPath = args[++i];
        } else if (arg == "--format" && i + 1 < args.size()) {
            format = args[++i];
        } else if (arg == "--debug") {
            debug = true;
        } else if (inputPath.empty()) {
            inputPath = arg;
        }
    }
    if (inputPath.empty() || outputPath.empty()) {
        err << "usage: openscad-cpp-evaluator <input.scad> -o <output.{stl,obj,off,3mf}> [--format stl|obj|off|3mf] [--debug]\n";
        return 1;
    }
    const std::string fmt = formatForPath(format, outputPath);
    if (fmt.empty()) {
        err << "error: cannot infer output format from '" << outputPath << "' -- pass --format\n";
        return 1;
    }

    try {
        std::vector<std::unique_ptr<oscad::ASTNode>> ast = oscad::getASTFromFile(inputPath);
        ResolvedUseScopes used = resolveUseScopes(ast, inputPath, [&out](const std::string& msg) { out << msg << "\n"; });

        std::optional<DebugRepl> repl;
        DebugHooks hooks;
        if (debug) {
            repl.emplace(inputPath, in, out);
            if (!repl->runPrompt()) return 0; // user quit before "run"
            hooks.debugHook = [&](int line, int depth, bool forced, const std::string& origin,
                                   const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame) {
                return repl->debugHook(line, depth, forced, origin, callStack, getFrame);
            };
            hooks.errorBreak = [&](int line, const std::string& header, const std::string& origin,
                                    const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame) {
                repl->errorBreak(line, header, origin, callStack, getFrame);
            };
            hooks.returnHook = [&](const std::string& name, const Value& result, int depth) {
                repl->returnHook(name, result, depth);
            };
        }

        Evaluator evaluator([&out](const std::string& msg) { out << msg << "\n"; }, nullptr, nullptr, hooks);
        if (repl) repl->attachEvaluator(evaluator); // lets "child" read Evaluator::lastChildrenPositions()
        EvalContext ctx = EvalContext::makeRoot(used.rootScope.get());

        std::vector<ColoredBody> bodies = toRenderableBodies(evaluator.evaluate(used.processedNodes, ctx));

        if (fmt == "stl") {
            writeStl(outputPath, bodies);
        } else if (fmt == "obj") {
            writeObj(outputPath, bodies);
        } else if (fmt == "off") {
            writeOff(outputPath, bodies);
        } else {
            writeThreeMf(outputPath, bodies);
        }
        out << "Exported to " << outputPath << "\n";
        return 0;
    } catch (const oscad::ParseError& e) {
        err << e.what() << "\n";
        return 1;
    } catch (const EvalError& e) {
        // e.what() is already the fully formatted "ERROR: ...\nTRACE: ..."
        // message (see eval_error.hpp) -- no extra prefix here, unlike the
        // generic catch below.
        err << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        err << "ERROR: " << e.what() << "\n";
        return 1;
    }
}

} // namespace oscadeval
