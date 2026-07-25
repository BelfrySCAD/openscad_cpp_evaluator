#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

// Mirrors the reference's TestDynExplicit + TestVariables::
// test_animation_t_set_via_viewport_params: EvalContext::dynExplicit
// distinguishes "the script itself assigned this $-variable" from "this
// $-variable is merely present in `dyn`" (e.g. $vp* seeded from the
// current camera via Evaluator::evaluate()'s viewportParams parameter, or
// $fn/$fa/$fs/$t/$parent_modules seeded from EvalContext::makeRoot's own
// defaults) -- used by a GUI host to decide whether a script's $vp*
// assignment should move the viewport camera, vs. leaving a manually-
// adjusted camera alone.
//
// Unlike the reference (whose own `ctx` is constructed inside evaluate()
// itself, forcing it to expose a separate `_root_ctx` escape hatch for
// tests to inspect post-evaluate dyn/dynExplicit state), this port's `ctx`
// is always caller-owned already -- these tests just inspect the same
// `ctx` object passed into evaluate() directly.

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::unordered_map<std::string, Value> vpSeed() {
    return {
        {"$vpt", Value{std::make_shared<const ValueList>(ValueList{{Value{0.0}, Value{0.0}, Value{0.0}}})}},
        {"$vpr", Value{std::make_shared<const ValueList>(ValueList{{Value{55.0}, Value{0.0}, Value{25.0}}})}},
        {"$vpd", Value{140.0}},
        {"$vpf", Value{22.5}},
    };
}

} // namespace

TEST(ViewportParams, SeededValueIsPresentButNotExplicit) {
    Evaluator ev;
    auto ast = parseSrc("cube(10);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx, vpSeed());
    EXPECT_TRUE(ctx.dyn->count("$vpt") > 0);
    EXPECT_TRUE(ctx.dynExplicit->empty());
}

TEST(ViewportParams, ScriptAssignmentIsExplicit) {
    Evaluator ev;
    auto ast = parseSrc("$vpt = [1, 2, 3]; cube(10);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx, vpSeed());
    const auto& items = std::get<ListPtr>(ctx.dyn->at("$vpt"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(std::get<double>(items[0]), 1.0);
    EXPECT_DOUBLE_EQ(std::get<double>(items[1]), 2.0);
    EXPECT_DOUBLE_EQ(std::get<double>(items[2]), 3.0);
    EXPECT_EQ(explicitSnapshot(*ctx.dynExplicit), NameSet{"$vpt"});
}

TEST(ViewportParams, OnlyTheAssignedNameIsExplicit) {
    Evaluator ev;
    auto ast = parseSrc("$vpd = 50; cube(10);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx, vpSeed());
    EXPECT_EQ(explicitSnapshot(*ctx.dynExplicit), NameSet{"$vpd"});
    EXPECT_FALSE(ctx.dynExplicit->count("$vpt"));
    EXPECT_TRUE(ctx.dyn->count("$vpt") > 0); // still present (seeded), just not explicit
}

TEST(ViewportParams, RegularSpecialVarAssignmentAlsoTracked) {
    Evaluator ev;
    auto ast = parseSrc("$fn = 64; cube(10);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx); // no viewportParams at all
    EXPECT_EQ(explicitSnapshot(*ctx.dynExplicit), NameSet{"$fn"});
}

TEST(ViewportParams, AnimationTSetViaViewportParams) {
    std::string lastEcho;
    Evaluator ev([&](const std::string& msg) { lastEcho = msg; });
    auto ast = parseSrc("echo($t);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx, {{"$t", Value{0.25}}});
    EXPECT_EQ(lastEcho, "ECHO: 0.25");
}

TEST(ViewportParams, NoViewportParamsLeavesDefaultsUntouched) {
    std::string lastEcho;
    Evaluator ev([&](const std::string& msg) { lastEcho = msg; });
    auto ast = parseSrc("echo($t);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    EXPECT_EQ(lastEcho, "ECHO: 0");
}
