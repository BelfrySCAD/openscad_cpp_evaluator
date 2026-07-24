// Minimal example: hook a debugger into Evaluator via DebugHooks::debugHook.
//
// debugHook is called at (approximately) every top-level statement -- see
// debug_hooks.hpp's DebugHookFn doc comment for the exact scope -- before
// it runs. It receives enough context to log/inspect current state,
// optionally override a variable's value for that scope via the returned
// DebugAction::mods, and decide whether to keep going (stop=false) or
// abort the whole evaluate() call (stop=true, which raises EvalError).
//
// Real debuggers (breakpoints, step-over/step-into, blocking on user
// input) are built entirely on top of this one hook -- see debug_repl.hpp/
// debug_repl.cpp for exactly that (wired into this project's own CLI via
// its --debug flag). The evaluator itself has no stepping state of its
// own; it just calls debugHook and does whatever it says.
//
// Build & run: this target is only built if BUILD_EXAMPLES is ON (see
// CMakeLists.txt) -- `build/examples/minimal_debugger`.

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <cassert>
#include <iostream>

using namespace oscadeval;

namespace {

const char* kScript = "width = 10;\n"
                       "cube([width, width, width]);\n"
                       "translate([20, 0, 0]) sphere(r=5, $fn=16);\n";

std::string fmtLocals(const std::unordered_map<std::string, Value>& locals) {
    std::string out = "{";
    bool first = true;
    for (const auto& [k, v] : locals) {
        if (!first) out += ", ";
        first = false;
        out += k + ": " + fmtValue(v);
    }
    return out + "}";
}

// 1. Print (line, call depth, in-scope variables) at every statement.
void traceEveryStatement() {
    std::cout << "1. trace every statement:\n";
    DebugHooks hooks;
    hooks.debugHook = [](int line, int depth, bool, const std::string&, const std::vector<CallStackFrame>&,
                          const DebugFramesFn& getFrame) {
        DebugFrame frame = getFrame();
        std::cout << "  line " << line << "  depth=" << depth << "  locals=" << fmtLocals(frame.locals) << "\n";
        return DebugAction{};
    };

    auto ast = oscad::getASTFromString(kScript);
    auto scope = oscad::buildScopes(ast);
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    std::vector<ColoredBody> bodies = ev.evaluate(ast, ctx);
    std::cout << "  -> " << bodies.size() << " bodies\n\n";
    assert(bodies.size() == 2);
}

// 2. Abort evaluation as soon as a chosen line is reached.
void stopAtBreakpoint() {
    std::cout << "2. stop at a breakpoint:\n";
    constexpr int kBreakLine = 3; // the `translate(...) sphere(...)` statement

    DebugHooks hooks;
    hooks.debugHook = [](int line, int, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        DebugAction action;
        if (line == kBreakLine) {
            std::cout << "  breakpoint hit at line " << line << "\n";
            action.stop = true;
        }
        return action;
    };

    auto ast = oscad::getASTFromString(kScript);
    auto scope = oscad::buildScopes(ast);
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    try {
        ev.evaluate(ast, ctx);
        std::cerr << "expected evaluation to stop at the breakpoint\n";
        std::abort();
    } catch (const EvalError&) {
        std::cout << "  evaluation aborted as expected\n\n";
    }
}

// 3. The mods returned from a hook let a debugger inject a "set variable"
// command mid-run -- e.g. a "change this value and keep going" watch
// expression.
void overrideVariableViaMods() {
    std::cout << "3. override a variable via mods:\n";
    DebugHooks hooks;
    hooks.debugHook = [](int line, int, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        DebugAction action;
        if (line == 2) action.mods["width"] = Value{2.0}; // about to run `cube([width, width, width])`
        return action;
    };

    auto ast = oscad::getASTFromString(kScript);
    auto scope = oscad::buildScopes(ast);
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    std::vector<ColoredBody> bodies = ev.evaluate(ast, ctx);
    manifold::Box bbox = bodies[0].body->BoundingBox();
    const double size = bbox.max.x - bbox.min.x;
    std::cout << "  cube size after override: " << size << " (script said width=10)\n";
    assert(size == 2.0);
}

} // namespace

int main() {
    traceEveryStatement();
    stopAtBreakpoint();
    overrideVariableViaMods();
    return 0;
}
