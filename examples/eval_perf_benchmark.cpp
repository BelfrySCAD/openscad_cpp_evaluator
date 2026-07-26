// Timing smoke-test for the evaluator's hot path -- a regression tripwire
// for the interpreter-speed work covered by CLAUDE.md's "quick wins" pass
// (Release-by-default build, ManifoldCache key memoization, hash-dispatch
// builtin functions, flat-vector CallArgs, copy-free expandIterable()).
// None of those touch geometry correctness (already covered by the ctest
// suite) -- this only exercises the call-heavy/loop-heavy paths they
// speed up: deep non-tail recursion (fib, stresses user-function call
// machinery + CallArgs), a list comprehension over a range (stresses
// expandIterable's synthesized-vector path), and a for loop over the
// resulting list instantiating cube()/translate() every iteration
// (stresses expandIterable's zero-copy ListPtr path, builtin *module*
// dispatch, and builtin *function* dispatch via sin()/cos()/abs()).
//
// Build & run: this target is only built if BUILD_EXAMPLES is ON (see
// CMakeLists.txt) -- `build/examples/eval_perf_benchmark`.

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <iostream>

using namespace oscadeval;

namespace {

// fib() is deliberately small here -- non-tail user-function recursion goes
// through the bytecode VM's own CallFn machinery (EvalContext/call-stack
// overhead), not CallArgs/evalBuiltinFunction/expandIterable, so it isn't a
// useful stress test for the changes this benchmark exists to guard
// (dispatch-table lookup, flat-vector CallArgs, copy-free expandIterable).
// It stays in only as a light recursion sanity check. The for loop is the
// real payload: each of its many iterations evaluates a builtin function
// call (sin/cos/abs), a `for`-over-list expandIterable expansion, and two
// builtin *module* calls (translate/cube) with their own CallArgs.
const char* kScript = R"scad(
function fib(n) = n < 2 ? n : fib(n - 1) + fib(n - 2);
echo(fib(16));

pts = [for (i = [0:49999]) [i, sin(i), cos(i)]];
for (p = pts)
    translate([p[0], abs(p[1]), p[2]]) cube([1, 1, 1]);
)scad";

} // namespace

int main() {
    auto ast = oscad::getASTFromString(kScript);
    auto scope = oscad::buildScopes(ast);
    Evaluator ev;
    EvalContext ctx = EvalContext::makeRoot(scope.get());

    const auto start = std::chrono::steady_clock::now();
    std::vector<ColoredBody> bodies = ev.evaluate(ast, ctx);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double ms = std::chrono::duration<double, std::milli>(elapsed).count();

    // One top-level body per for-loop iteration (translate/cube splice
    // transparently into the top-level CSGNode list, one per iteration) --
    // a correctness sanity check, not this file's main point.
    assert(bodies.size() == 50000);

    std::printf("eval_perf_benchmark: %.1fms (fib(16) + 50000-iteration for-loop)\n", ms);

    // ponytail: generous fixed ceiling, not a strict perf target -- trips
    // only on a real regression (e.g. an O(n^2) reintroduced somewhere in
    // this path), not machine noise. Raise if a slower CI runner ever
    // needs it.
    constexpr double kCeilingMs = 5000.0;
    if (ms > kCeilingMs) {
        std::cerr << "eval_perf_benchmark: took " << ms << "ms, expected under " << kCeilingMs << "ms\n";
        return 1;
    }
    return 0;
}
