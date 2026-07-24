// Minimal example: reuse ManifoldCache across repeated evaluate() calls.
//
// A real editor re-parses the whole script and builds a brand-new
// Evaluator on every render, so nothing is cached by default. Passing the
// *same* ManifoldCache instance into successive Evaluator(...,
// manifoldCache) calls lets generateTree() skip Manifold work for any
// CSGNode subtree whose *content* (kind/params/children) is byte-for-byte
// the same as a previous render -- exactly what happens when a user edits
// one part of a script and re-renders the whole thing: everything under
// the edit still has to be regenerated, but everything else is served
// straight from cache.
//
// Build & run: this target is only built if BUILD_EXAMPLES is ON (see
// CMakeLists.txt) -- `build/examples/manifold_cache_reuse`.

#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/manifold_cache.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>

using namespace oscadeval;

namespace {

// minkowski() of two moderately dense spheres is deliberately expensive;
// translate's own offset is the only thing that changes between renders,
// so the minkowski subtree underneath is identical both times.
std::string script(int x) {
    std::ostringstream ss;
    ss << "translate([" << x << ", 0, 0])\n"
       << "  minkowski() {\n"
       << "    sphere(r=6, $fn=48);\n"
       << "    sphere(r=2, $fn=48);\n"
       << "  }\n";
    return ss.str();
}

double renderMs(const std::shared_ptr<ManifoldCache>& cache, int x) {
    auto ast = oscad::getASTFromString(script(x));
    auto scope = oscad::buildScopes(ast);
    Evaluator ev(EchoFn{}, nullptr, cache);
    EvalContext ctx = EvalContext::makeRoot(scope.get());

    const auto start = std::chrono::steady_clock::now();
    std::vector<ColoredBody> bodies = ev.evaluate(ast, ctx);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    assert(bodies.size() == 1);
    return std::chrono::duration<double, std::milli>(elapsed).count();
}

} // namespace

int main() {
    auto cache = std::make_shared<ManifoldCache>();

    const double cold = renderMs(cache, 0);
    std::printf("cold render (nothing cached yet):       %6.1fms\n", cold);

    // Same minkowski content, only translate's own offset differs --
    // translate misses (its own params changed) but the expensive
    // minkowski subtree underneath is served from cache.
    const double warm = renderMs(cache, 10);
    std::printf("warm render (minkowski subtree cached): %6.1fms\n", warm);

    if (warm >= cold) {
        std::cerr << "expected the cached minkowski to make the second render faster\n";
        return 1;
    }
    std::printf("speedup: %.1fx\n", cold / warm);
    return 0;
}
