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

// Runs `code` with profiling on and returns the result. `ast`/`scope` are
// kept alive by the caller's Evaluator, which owns nothing that outlives
// this -- ProfileResult is plain data.
ProfileResult profileSrc(const std::string& code) {
    Evaluator ev(EchoFn{}, nullptr, nullptr, DebugHooks{}, /*profiling=*/true);
    auto ast = parseSrc(code);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    return ev.profileResult.value_or(ProfileResult{});
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

// -- Calling-context tree (per-path attribution) ---------------------------

namespace {
const oscadeval::ProfilePathNode* childNamed(const oscadeval::ProfileResult& r,
                                              const oscadeval::ProfilePathNode& parent,
                                              const std::string& name) {
    for (int idx : parent.children) {
        const auto& n = r.paths[static_cast<size_t>(idx)];
        if (n.name == name) return &n;
    }
    return nullptr;
}
} // namespace

// The point of the tree: the same callee reached by two different callers
// gets two nodes with their own times, where callSites has one entry
// totalling both. Without that, a report can say "helper is expensive" but
// never "expensive *when called from here*".
TEST(ProfilePaths, SameCalleeUnderTwoCallersGetsSeparateNodes) {
    const ProfileResult r = profileSrc(
        "function work(n) = n <= 0 ? 0 : work(n - 1) + 1;\n"
        "module light() { x = work(20); cube(1); }\n"
        "module heavy() { x = work(400); cube(1); }\n"
        "light();\n"
        "heavy();\n");
    ASSERT_FALSE(r.paths.empty());
    const auto& root = r.paths[0];
    EXPECT_EQ(root.name, "<toplevel>");

    const auto* light = childNamed(r, root, "light");
    const auto* heavy = childNamed(r, root, "heavy");
    ASSERT_NE(light, nullptr);
    ASSERT_NE(heavy, nullptr);

    const auto* workUnderLight = childNamed(r, *light, "work");
    const auto* workUnderHeavy = childNamed(r, *heavy, "work");
    ASSERT_NE(workUnderLight, nullptr);
    ASSERT_NE(workUnderHeavy, nullptr);
    // Two distinct nodes for the same callee -- the whole point.
    EXPECT_NE(workUnderLight, workUnderHeavy);
    // ...and the expensive path is attributed the larger share. This is the
    // fact the aggregated view cannot express at all.
    EXPECT_GT(workUnderHeavy->cumulativeTime, workUnderLight->cumulativeTime);

    // The flat view still totals both, unchanged.
    double siteTotal = 0.0;
    for (const auto& s : r.callSites) {
        if (s.name == "work") siteTotal += s.cumulativeTime;
    }
    EXPECT_GT(siteTotal, 0.0);
}

// Recursion folds onto one node instead of unrolling into a chain per
// level, or a 400-deep recursion would be 400 nodes.
TEST(ProfilePaths, RecursionFoldsOntoASingleNode) {
    const ProfileResult r = profileSrc(
        "function down(n) = n <= 0 ? 0 : down(n - 1) + 1;\n"
        "x = down(200);\n"
        "cube(x > 0 ? 1 : 2);\n");
    ASSERT_FALSE(r.paths.empty());
    size_t downNodes = 0;
    for (const auto& n : r.paths) {
        if (n.name == "down") ++downNodes;
    }
    // One node for the outer call plus one for the folded recursive site --
    // emphatically not one per level.
    EXPECT_LE(downNodes, 2u) << downNodes;
    EXPECT_LT(r.paths.size(), 20u) << r.paths.size();
}

// A node's cumulative time must cover its subtree, or percentages in a
// report are meaningless.
//
// The fixture deliberately RECURSES THROUGH A MODULE CALL, because that is
// what broke the first implementation and what a simple non-recursive
// fixture cannot catch. Recursion makes every level fold onto the same
// node for the sites inside the recursive body (here the `translate` and
// its internal `_translate`), so those nodes are entered many times while
// the calls each level makes still attach as their children. Measuring
// cumulative per entry -- counting only the outermost, to avoid double-
// counting nested time -- made the node read SMALLER than its own
// subtree. On a real model a `_translate` showed 22.85ms with 90.09ms of
// children. Cumulative is derived from the subtree now, so this holds by
// construction.
TEST(ProfilePaths, CumulativeTimeContainsChildrenEvenThroughRecursion) {
    const ProfileResult r = profileSrc(
        "function work(n) = n <= 0 ? 0 : work(n - 1) + 1;\n"
        "module step(n) {\n"
        "    x = work(40);\n"
        "    cube(1);\n"
        "    if (n > 0) translate([0, 0, 2]) step(n - 1);\n"
        "}\n"
        "module outer() { step(6); }\n"
        "outer();\n");
    ASSERT_FALSE(r.paths.empty());
    EXPECT_GT(r.paths[0].cumulativeTime, 0.0);

    size_t multiEntry = 0;
    for (const auto& n : r.paths) {
        double childSum = 0.0;
        for (int c : n.children) childSum += r.paths[static_cast<size_t>(c)].cumulativeTime;
        EXPECT_LE(childSum, n.cumulativeTime + 1e-9)
            << n.name << " (cum " << n.cumulativeTime << " < children " << childSum << ")";
        EXPECT_GE(n.cumulativeTime, n.selfTime - 1e-9) << n.name;
        if (n.callCount > 1) ++multiEntry;
    }
    // Guard the fixture itself: if nothing folded, this test would pass
    // without ever exercising the case it exists for.
    EXPECT_GT(multiEntry, 0u) << "fixture never produced a multi-entry node";
}

// The root accounts for essentially the whole resolve pass -- the gap is
// top-level work outside any user call, which is what unattributedTime is.
TEST(ProfilePaths, RootCoversTheProfiledWork) {
    const ProfileResult r = profileSrc(
        "module m() { cube(1); }\nm(); m(); m();\n");
    ASSERT_FALSE(r.paths.empty());
    EXPECT_GT(r.paths[0].cumulativeTime, 0.0);
    EXPECT_LE(r.paths[0].cumulativeTime, r.resolveTime + 1e-9);
}

// Profiling off must cost nothing and produce no tree.
TEST(ProfilePaths, NoTreeWhenProfilingIsOff) {
    Evaluator ev;  // profiling off
    auto ast = parseSrc("module m() { cube(1); }\nm();");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    EXPECT_FALSE(ev.profileResult.has_value());
}
