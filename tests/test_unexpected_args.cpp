#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

// Real OpenSCAD (Parameters.cc's parse_without_defaults) warns for a named
// argument the callee doesn't declare, and once per call for positional
// arguments past the last parameter. Every expectation below was checked
// against a real OpenSCAD 2022.08.22 run of the same script -- same message
// text, same set of calls warned about.
//
// Every case runs twice, VM off and VM on: user calls bind through
// bindArgs (interpreter) or buildBoundArgs/bindAstArgsIntoFrame (VM), and
// the transform/color/hull/extrude/CSG builtins bypass evalModularCall
// entirely once compiled (Op::PushBuiltinWrap/Op::PushCsgWrap). Any of those
// paths missing the check is exactly the bug this file exists to catch.
using namespace oscadeval;
using namespace oscadeval::test;

namespace {

class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

std::vector<std::string> warningsFor(const std::string& code) {
    std::vector<std::string> out;
    Evaluated e = evalSrc(code, [&](const std::string& msg) {
        if (msg.rfind("WARNING: ", 0) == 0) out.push_back(msg);
    });
    (void)e;
    return out;
}

// Every warning line, both VM settings, asserted to be identical between
// them -- the two binding paths must not disagree about what is unexpected.
std::vector<std::string> warningsBothPaths(const std::string& code) {
    std::vector<std::string> interpreted;
    {
        ScopedVm vm(false);
        interpreted = warningsFor(code);
    }
    std::vector<std::string> compiled;
    {
        ScopedVm vm(true);
        compiled = warningsFor(code);
    }
    EXPECT_EQ(interpreted, compiled) << "VM-on and VM-off disagree for: " << code;
    return interpreted;
}

// True if some warning contains `needle`.
bool has(const std::vector<std::string>& warnings, const std::string& needle) {
    for (const std::string& w : warnings) {
        if (w.find(needle) != std::string::npos) return true;
    }
    return false;
}

} // namespace

TEST(UnexpectedArgs, UserModuleNamedArgNotAParameter) {
    auto w = warningsBothPaths("module m(a=1) { } m(a=2, b=3);");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: variable b not specified as parameter"), std::string::npos);
}

TEST(UnexpectedArgs, UserFunctionNamedArgNotAParameter) {
    auto w = warningsBothPaths("function f(x=1) = x; echo(f(x=2, y=3));");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: variable y not specified as parameter"), std::string::npos);
}

// A call from inside another function is the case the bytecode VM's own
// Op::CallFn path (buildBoundArgs) handles, distinct from the top-level
// entry (bindAstArgsIntoFrame).
TEST(UnexpectedArgs, NestedUserFunctionCallStillWarns) {
    auto w = warningsBothPaths("function inner(n=1) = n; function outer(k) = inner(n=k, bogus=1); echo(outer(2));");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: variable bogus not specified as parameter"), std::string::npos);
}

TEST(UnexpectedArgs, FunctionLiteralWarnsToo) {
    auto w = warningsBothPaths("g = function(p) p * 2; echo(g(3, extra=9));");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: variable extra not specified as parameter"), std::string::npos);
}

TEST(UnexpectedArgs, TooManyPositionalArgsWarnsExactlyOnce) {
    // One warning for the whole call, however many extra arguments -- the
    // reference's own warned_for_extra_arguments latch.
    auto w = warningsBothPaths("module m(a) { } m(1, 2, 3);");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: Too many unnamed arguments supplied"), std::string::npos);
}

TEST(UnexpectedArgs, DollarVariablesAreNeverUnexpected) {
    // ContextFrame::is_config_variable: a $-name is a dynamic-scope
    // override, not a parameter, so passing one to anything is fine.
    EXPECT_TRUE(warningsBothPaths("module m(a=1) { } m(a=2, $custom=3, $fn=8);").empty());
    EXPECT_TRUE(warningsBothPaths("function f(x=1) = x; echo(f(x=2, $custom=3));").empty());
    EXPECT_TRUE(warningsBothPaths("sphere(r=1, $fn=8);").empty());
}

TEST(UnexpectedArgs, DeclaredParametersNeverWarn) {
    EXPECT_TRUE(warningsBothPaths("module m(a, b=2) { } m(1, b=3);").empty());
    EXPECT_TRUE(warningsBothPaths("cylinder(h=2, r1=1, r2=0, center=true);").empty());
    EXPECT_TRUE(warningsBothPaths("linear_extrude(height=1, twist=10, convexity=2) square(1);").empty());
}

TEST(UnexpectedArgs, BuiltinPrimitiveWarns) {
    auto w = warningsBothPaths("cube(size=2, bogus=9);");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: variable bogus not specified as parameter"), std::string::npos);
}

// The transform/color/hull/CSG family never reaches evalModularCall once
// the bytecode compiler turns it into Op::PushBuiltinWrap/Op::PushCsgWrap
// -- the whole point of running these VM-on as well.
TEST(UnexpectedArgs, BuiltinsWithChildrenWarnOnBothPaths) {
    EXPECT_TRUE(has(warningsBothPaths("translate([1,0,0], junk=1) cube(1);"), "variable junk"));
    EXPECT_TRUE(has(warningsBothPaths("color(\"red\", junk=1) cube(1);"), "variable junk"));
    EXPECT_TRUE(has(warningsBothPaths("hull(junk=1) cube(1);"), "variable junk"));
    EXPECT_TRUE(has(warningsBothPaths("union(junk=1) cube(1);"), "variable junk"));
    EXPECT_TRUE(has(warningsBothPaths("difference(junk=1) { cube(1); }"), "variable junk"));
    EXPECT_TRUE(has(warningsBothPaths("offset(r=1, junk=1) square(1);"), "variable junk"));
}

// The `#`/`%`/`!` modifiers share BuiltinWrapSite with the transforms but
// carry no argument list at all -- must not trip the ModularCall cast.
TEST(UnexpectedArgs, ModifiersAreNotCalls) {
    EXPECT_TRUE(warningsBothPaths("#translate([1,0,0]) cube(1);").empty());
    EXPECT_TRUE(warningsBothPaths("%cube(1);").empty());
}

// Real OpenSCAD's builtin FUNCTIONS read their arguments positionally
// without Parameters::parse, so they warn about nothing -- verified against
// 2022.08.22, where both `sin(x=30)` and `sin(bogus=30)` return 0.5 in
// silence. textmetrics/fontmetrics are the exceptions that do parse.
TEST(UnexpectedArgs, BuiltinFunctionsDoNotWarn) {
    EXPECT_TRUE(warningsBothPaths("echo(sin(bogus=30));").empty());
    EXPECT_TRUE(warningsBothPaths("echo(len(bogus=[1,2]));").empty());
}

TEST(UnexpectedArgs, TextMetricsWarns) {
    auto w = warningsBothPaths("echo(textmetrics(text=\"hi\", size=5, nope=1));");
    EXPECT_TRUE(has(w, "variable nope not specified as parameter"));
}
