#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

const CallSiteProfile* findSite(const ProfileResult& r, const std::string& kind, const std::string& name) {
    for (const auto& s : r.callSites) {
        if (s.kind == kind && s.name == name) return &s;
    }
    return nullptr;
}

} // namespace

TEST(Profiling, DisabledByDefaultLeavesProfileResultEmpty) {
    Evaluator ev; // profiling=false by default
    auto ast = parseSrc("cube(1);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    EXPECT_FALSE(ev.profileResult.has_value());
}

TEST(Profiling, ArithmeticIdentityAlwaysHolds) {
    // resolveTime == sum(selfTime) + unattributedTime is an exact
    // arithmetic identity by construction (unattributedTime =
    // max(0, resolveTime - selfSum)), not a timing-dependent assertion --
    // verified regardless of how long the real evaluate() call actually
    // took. Mirrors the reference's own
    // test_self_times_plus_unattributed_equal_resolve_time.
    Evaluator ev(EchoFn{}, nullptr, nullptr, DebugHooks{}, /*profiling=*/true);
    auto ast = parseSrc("module m(n) { cube(n); } m(1); m(2); translate([1,0,0]) sphere(r=1,$fn=8);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);

    ASSERT_TRUE(ev.profileResult.has_value());
    double selfSum = 0.0;
    for (const auto& s : ev.profileResult->callSites) selfSum += s.selfTime;
    EXPECT_GE(ev.profileResult->unattributedTime, 0.0);
    if (selfSum <= ev.profileResult->resolveTime) {
        EXPECT_NEAR(selfSum + ev.profileResult->unattributedTime, ev.profileResult->resolveTime, 1e-9);
    } else {
        EXPECT_DOUBLE_EQ(ev.profileResult->unattributedTime, 0.0);
    }
}

TEST(Profiling, RecursiveFunctionNotDoubleCounted) {
    Evaluator ev(EchoFn{}, nullptr, nullptr, DebugHooks{}, /*profiling=*/true);
    auto ast = parseSrc("function fact(n) = n <= 1 ? 1 : n * fact(n - 1);\nresult = fact(15);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);

    ASSERT_TRUE(ev.profileResult.has_value());
    // Two distinct call SITES for "fact" -- aggregation is per source
    // location, not per declaration (see CallSiteProfile's own doc
    // comment): the initial call at `result = fact(15)`'s line (1
    // invocation) and the recursive `fact(n - 1)` call inside the
    // function body's own line (14 invocations, for n=15 down to n=2 --
    // the n<=1 base case makes no further call). 1 + 14 = 15 total.
    int totalCalls = 0;
    const CallSiteProfile* recursiveSite = nullptr;
    for (const auto& s : ev.profileResult->callSites) {
        if (s.kind == "function" && s.name == "fact") {
            totalCalls += s.callCount;
            if (s.callCount > 1) recursiveSite = &s;
        }
    }
    EXPECT_EQ(totalCalls, 15);
    ASSERT_NE(recursiveSite, nullptr);
    EXPECT_EQ(recursiveSite->callCount, 14);
    // The recursion guard must keep cumulative_time from ballooning past
    // total elapsed resolve time (each recursive re-entry's own elapsed
    // is already included in the outermost invocation's, via child-time
    // propagation) -- a real bug would multiply this well past resolveTime.
    EXPECT_LE(recursiveSite->cumulativeTime, ev.profileResult->resolveTime + 1e-6);
    EXPECT_GE(recursiveSite->selfTime, 0.0);
}

TEST(Profiling, CallSiteAggregatesRepeatedInvocationsAtSameCallExpression) {
    Evaluator ev(EchoFn{}, nullptr, nullptr, DebugHooks{}, /*profiling=*/true);
    auto ast = parseSrc("module leaf() { cube(1); } for (i = [0:4]) leaf();");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);

    ASSERT_TRUE(ev.profileResult.has_value());
    const CallSiteProfile* site = findSite(*ev.profileResult, "module", "leaf");
    ASSERT_NE(site, nullptr);
    EXPECT_EQ(site->callCount, 5); // [0:4] is 5 iterations, same AST call node re-executed
}

TEST(Profiling, CallerNameReflectsEnclosingScope) {
    Evaluator ev(EchoFn{}, nullptr, nullptr, DebugHooks{}, /*profiling=*/true);
    auto ast = parseSrc("module inner() { cube(1); } module outer() { inner(); } outer();");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);

    ASSERT_TRUE(ev.profileResult.has_value());
    const CallSiteProfile* outerSite = findSite(*ev.profileResult, "module", "outer");
    const CallSiteProfile* innerSite = findSite(*ev.profileResult, "module", "inner");
    ASSERT_NE(outerSite, nullptr);
    ASSERT_NE(innerSite, nullptr);
    EXPECT_EQ(outerSite->callerName, "<toplevel>");
    EXPECT_EQ(innerSite->callerName, "outer");
}

TEST(Profiling, GenerateTimeAndTotalTimeAreNonNegative) {
    Evaluator ev(EchoFn{}, nullptr, nullptr, DebugHooks{}, /*profiling=*/true);
    auto ast = parseSrc("cube(2); sphere(r=1, $fn=16);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);

    ASSERT_TRUE(ev.profileResult.has_value());
    EXPECT_GE(ev.profileResult->generateTime, 0.0);
    EXPECT_GE(ev.profileResult->resolveTime, 0.0);
    EXPECT_NEAR(ev.profileResult->totalTime, ev.profileResult->resolveTime + ev.profileResult->generateTime, 1e-9);
}
