// profile_time(): `profile_time(label) { ... }` prints its children's script
// and geometry time; `x = profile_time(label) expr;` prints how long `expr`
// took and evaluates to its value.

#include "test_helpers.hpp"
#include "../src/builtins/builtins.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::vector<std::string> profileLines(const std::vector<std::string>& all) {
    std::vector<std::string> out;
    for (const std::string& m : all)
        if (m.rfind("PROFILE: ", 0) == 0) out.push_back(m);
    return out;
}

bool contains(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

} // namespace

TEST(ProfileTime, ExpressionFormYieldsTheBodysValueAndReportsIt) {
    std::vector<std::string> msgs;
    auto e = evalSrc("x = profile_time(\"sum\") 1 + 2; echo(x = x);", [&](const std::string& m) { msgs.push_back(m); });
    auto p = profileLines(msgs);
    ASSERT_EQ(p.size(), 1u);
    EXPECT_TRUE(contains(p[0], "PROFILE: sum: ")) << p[0];
    EXPECT_TRUE(contains(p[0], " ms") || contains(p[0], " \u00b5s")) << p[0];
    EXPECT_NE(std::find(msgs.begin(), msgs.end(), "ECHO: x = 3"), msgs.end());
}

TEST(ProfileTime, ExpressionFormWorksInsideAFunction) {
    std::vector<std::string> msgs;
    auto e = evalSrc("function f(n) = profile_time(\"f\") n * 2; echo(v = f(21));",
                     [&](const std::string& m) { msgs.push_back(m); });
    EXPECT_EQ(profileLines(msgs).size(), 1u);
    EXPECT_NE(std::find(msgs.begin(), msgs.end(), "ECHO: v = 42"), msgs.end());
}

TEST(ProfileTime, WithoutALabelItNamesTheLine) {
    std::vector<std::string> msgs;
    auto e = evalSrc("\n\nx = profile_time() 1;", [&](const std::string& m) { msgs.push_back(m); });
    auto p = profileLines(msgs);
    ASSERT_EQ(p.size(), 1u);
    EXPECT_TRUE(contains(p[0], "PROFILE: line 3")) << p[0];
}

TEST(ProfileTime, ModuleFormReportsScriptAndGeometry) {
    std::vector<std::string> msgs;
    auto e = evalSrc("profile_time(\"parts\") { cube(1); translate([3,0,0]) sphere(1); }",
                     [&](const std::string& m) { msgs.push_back(m); });
    auto p = profileLines(msgs);
    ASSERT_EQ(p.size(), 1u);
    EXPECT_TRUE(contains(p[0], "PROFILE: parts: ")) << p[0];
    EXPECT_TRUE(contains(p[0], "(script ")) << p[0];
    EXPECT_TRUE(contains(p[0], ", geometry ")) << p[0];
    EXPECT_FALSE(contains(p[0], "cached")) << p[0];
    // Its children's geometry passes straight through.
    EXPECT_EQ(e.bodies.size(), 2u);
}

TEST(ProfileTime, NamedLabelIsAccepted) {
    std::vector<std::string> msgs;
    auto e = evalSrc("profile_time(label=\"n\") cube(1);", [&](const std::string& m) { msgs.push_back(m); });
    auto p = profileLines(msgs);
    ASSERT_EQ(p.size(), 1u);
    EXPECT_TRUE(contains(p[0], "PROFILE: n: ")) << p[0];
    for (const std::string& m : msgs) EXPECT_FALSE(contains(m, "WARNING")) << m;
}

TEST(ProfileTime, ACacheHitSaysSo) {
    auto cache = std::make_shared<ManifoldCache>();
    const std::string src = "profile_time(\"c\") cube(2);";
    std::vector<std::string> first, second;
    { auto e = evalSrcWithCache(src, cache, [&](const std::string& m) { first.push_back(m); }); }
    { auto e = evalSrcWithCache(src, cache, [&](const std::string& m) { second.push_back(m); }); }
    ASSERT_EQ(profileLines(first).size(), 1u);
    EXPECT_FALSE(contains(profileLines(first)[0], "cached"));
    ASSERT_EQ(profileLines(second).size(), 1u);
    EXPECT_TRUE(contains(profileLines(second)[0], "geometry cached")) << profileLines(second)[0];
}

TEST(ProfileTime, ACacheHitAboveItStillReportsIt) {
    // The hit is at translate(), so generation never reaches profile_time().
    auto cache = std::make_shared<ManifoldCache>();
    const std::string src = "translate([1,0,0]) profile_time(\"inner\") cube(2);";
    std::vector<std::string> second;
    { auto e = evalSrcWithCache(src, cache); }
    { auto e = evalSrcWithCache(src, cache, [&](const std::string& m) { second.push_back(m); }); }
    auto p = profileLines(second);
    ASSERT_EQ(p.size(), 1u);
    EXPECT_TRUE(contains(p[0], "PROFILE: inner: ")) << p[0];
    EXPECT_TRUE(contains(p[0], "geometry cached")) << p[0];
}

TEST(ProfileTime, ResolveOnlyReportsScriptTimeAlone) {
    std::vector<std::string> msgs;
    auto ast = parseSrc("profile_time(\"r\") cube(1);");
    auto scope = oscad::buildScopes(ast);
    Evaluator ev([&](const std::string& m) { msgs.push_back(m); });
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx, {}, /*generate=*/false);
    auto p = profileLines(msgs);
    ASSERT_EQ(p.size(), 1u);
    EXPECT_TRUE(contains(p[0], "(no geometry built)")) << p[0];
}

TEST(ProfileTime, IsAFeature) {
    std::vector<std::string> msgs;
    auto e = evalSrc("echo(supported_feature(\"profile-time\"));", [&](const std::string& m) { msgs.push_back(m); });
    EXPECT_NE(std::find(msgs.begin(), msgs.end(), "ECHO: 1"), msgs.end());
}

TEST(ProfileTime, ExpressionFormRunsUnderTheBytecodeVm) {
    // The compiler has no opcode for it, so a function containing one falls
    // back to the interpreter; the result must not change.
    Evaluator::setBytecodeVmEnabledForTesting(true);
    std::vector<std::string> msgs;
    {
        auto e = evalSrc("function g(n) = n <= 0 ? 0 : profile_time(\"g\") n + g(n - 1); echo(v = g(4));",
                         [&](const std::string& m) { msgs.push_back(m); });
    }
    Evaluator::setBytecodeVmEnabledForTesting(std::nullopt);
    EXPECT_EQ(profileLines(msgs).size(), 4u);
    EXPECT_NE(std::find(msgs.begin(), msgs.end(), "ECHO: v = 10"), msgs.end());
}

TEST(ProfileTime, UnderAMillisecondIsInMicroseconds) {
    EXPECT_EQ(fmtMs(0.0047), "4.7 \u00b5s");
    EXPECT_EQ(fmtMs(0.5), "500.0 \u00b5s");
    EXPECT_EQ(fmtMs(12.4), "12.40 ms");
}
