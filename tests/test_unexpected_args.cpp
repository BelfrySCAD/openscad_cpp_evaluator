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
    EXPECT_NE(w[0].find("WARNING: variable \"b\" not specified as parameter"), std::string::npos);
}

TEST(UnexpectedArgs, UserFunctionNamedArgNotAParameter) {
    auto w = warningsBothPaths("function f(x=1) = x; echo(f(x=2, y=3));");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: variable \"y\" not specified as parameter"), std::string::npos);
}

// A call from inside another function is the case the bytecode VM's own
// Op::CallFn path (buildBoundArgs) handles, distinct from the top-level
// entry (bindAstArgsIntoFrame).
TEST(UnexpectedArgs, NestedUserFunctionCallStillWarns) {
    auto w = warningsBothPaths("function inner(n=1) = n; function outer(k) = inner(n=k, bogus=1); echo(outer(2));");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: variable \"bogus\" not specified as parameter"), std::string::npos);
}

TEST(UnexpectedArgs, FunctionLiteralWarnsToo) {
    auto w = warningsBothPaths("g = function(p) p * 2; echo(g(3, extra=9));");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("WARNING: variable \"extra\" not specified as parameter"), std::string::npos);
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
    EXPECT_NE(w[0].find("WARNING: variable \"bogus\" not specified as parameter"), std::string::npos);
}

// The transform/color/hull/CSG family never reaches evalModularCall once
// the bytecode compiler turns it into Op::PushBuiltinWrap/Op::PushCsgWrap
// -- the whole point of running these VM-on as well.
TEST(UnexpectedArgs, BuiltinsWithChildrenWarnOnBothPaths) {
    EXPECT_TRUE(has(warningsBothPaths("translate([1,0,0], junk=1) cube(1);"), "variable \"junk\""));
    EXPECT_TRUE(has(warningsBothPaths("color(\"red\", junk=1) cube(1);"), "variable \"junk\""));
    EXPECT_TRUE(has(warningsBothPaths("hull(junk=1) cube(1);"), "variable \"junk\""));
    EXPECT_TRUE(has(warningsBothPaths("union(junk=1) cube(1);"), "variable \"junk\""));
    EXPECT_TRUE(has(warningsBothPaths("difference(junk=1) { cube(1); }"), "variable \"junk\""));
    EXPECT_TRUE(has(warningsBothPaths("offset(r=1, junk=1) square(1);"), "variable \"junk\""));
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
    EXPECT_TRUE(has(w, "variable \"nope\" not specified as parameter"));
}

// An unparseable font spec: undef and nothing drawn, with the reference's
// warning (#1161).
TEST(UnexpectedArgs, AnUnparseableFontWarns) {
    auto w = warningsBothPaths("echo(textmetrics(\"Hg\", font=\"Liberation Sans:bogus=3\"));");
    EXPECT_TRUE(has(w, "Can't get font Liberation Sans:bogus=3"));
    w = warningsBothPaths("text(\"Hg\", font=\"Liberation Sans:SLANT=italic\");");
    EXPECT_TRUE(has(w, "Can't get font Liberation Sans:SLANT=italic"));
    w = warningsBothPaths("echo(fontmetrics(font=\"Liberation Sans:weight=heavyish\"));");
    EXPECT_TRUE(has(w, "Can't get font Liberation Sans:weight=heavyish"));
}

// A positional argument takes the next parameter not already bound, as in
// Parameters::parse: `f(a=1, b=2, 3)` binds c. It used to count only the
// positional arguments, so it bound a and silently replaced a=1
// (BelfrySCAD #1128). Expected values are OpenSCAD-dev 2026.02.01's.
TEST(UnexpectedArgs, PositionalArgumentSkipsParametersAlreadyNamed) {
    const std::string code = R"(
function f(a, b, c) = [a, b, c];
module m(a, b, c) echo(m=[a, b, c]);
g = function(a, b, c) [a, b, c];
function wrap(s, k) = s ? [f(a=1, b=2, k)] : [];
echo(f1=f(a=1, b=2, 3));
echo(f2=f(b=2, 3));
echo(f4=f(c=3, 1, 2));
echo(f5=f(b=2, 1, 3));
echo(g1=g(a=1, b=2, 3));
echo(w=wrap(true, 3));
m(a=1, b=2, 3);
m(b=2, 3);
echo(x=f(a=1, b=2, c=3, 4));
)";
    for (bool vm : {false, true}) {
        ScopedVm scoped(vm);
        std::vector<std::string> echoes, warnings;
        evalSrc(code, [&](const std::string& msg) {
            if (msg.rfind("ECHO: ", 0) == 0) echoes.push_back(msg);
            if (msg.rfind("WARNING: ", 0) == 0) warnings.push_back(msg);
        });
        EXPECT_EQ(echoes, (std::vector<std::string>{
            "ECHO: f1 = [1, 2, 3]", "ECHO: f2 = [3, 2, undef]", "ECHO: f4 = [1, 2, 3]",
            "ECHO: f5 = [1, 2, 3]", "ECHO: g1 = [1, 2, 3]", "ECHO: w = [[1, 2, 3]]",
            "ECHO: m = [1, 2, 3]", "ECHO: m = [3, 2, undef]", "ECHO: x = [1, 2, 3]"}))
            << "vm=" << vm;
        ASSERT_EQ(warnings.size(), 1u) << "vm=" << vm;
        EXPECT_NE(warnings[0].find("Too many unnamed arguments supplied"), std::string::npos);
    }
}

// The same rule for builtin modules, whose arguments bind through CallArgs:
// `cylinder(h=10, 5)` is r1=5 in OpenSCAD, and lost the 5 to h here. The
// bounding boxes are OpenSCAD-dev 2026.02.01's for the same calls.
TEST(UnexpectedArgs, BuiltinPositionalArgumentSkipsParametersAlreadyNamed) {
    const std::string code = R"(
echo(cyl=render() { cylinder(h=10, 5); }.boundingbox);
echo(sq=render() { square(size=3, true); }.boundingbox);
echo(lx=render() { linear_extrude(height=6, [0,0,1], 2) square(2); }.boundingbox);
echo(rot=render() { rotate(v=[0,0,1], 90) cube([4,1,1]); }.boundingbox);
)";
    for (bool vm : {false, true}) {
        ScopedVm scoped(vm);
        std::vector<std::string> echoes;
        evalSrc(code, [&](const std::string& msg) {
            if (msg.rfind("ECHO: ", 0) == 0) echoes.push_back(msg);
        });
        EXPECT_EQ(echoes, (std::vector<std::string>{
            "ECHO: cyl = [[-5, -5, 0], [5, 5, 10]]", "ECHO: sq = [[-1.5, -1.5], [1.5, 1.5]]",
            "ECHO: lx = [[0, 0, 0], [4, 4, 6]]", "ECHO: rot = [[-1, 0, 0], [0, 4, 1]]"}))
            << "vm=" << vm;
    }
}

// OpenSCAD 2026.11's other two named-argument warnings, checked against
// 2026.11-RC1 with the same calls: a name given twice, and a name filling a
// parameter a positional argument already filled. The last value wins.
namespace {
size_t count(const std::vector<std::string>& w, const std::string& needle) {
    size_t n = 0;
    for (const std::string& s : w) n += s.find(needle) != std::string::npos;
    return n;
}
} // namespace

TEST(UnexpectedArgs, RepeatedAndOverridingNamesWarnOnEveryPath) {
    const std::string decls = "module m(a) echo(a);\nfunction f(a) = a;\ng = function(a) a;\n"
                              "function viaPlan() = f(a=1, a=2);\n";  // a compiled call site
    struct Case {
        const char* code;
        const char* needle;
        size_t times;
    };
    for (const Case& c : {
             Case{"m(a=1, a=2);", "argument \"a\" supplied more than once", 1},
             Case{"m(a=1, a=2, a=3);", "argument \"a\" supplied more than once", 2},
             Case{"m(1, a=2);", "argument \"a\" overrides positional argument", 1},
             Case{"echo(f(a=1, a=2));", "argument \"a\" supplied more than once", 1},
             Case{"echo(f(1, a=2));", "argument \"a\" overrides positional argument", 1},
             Case{"echo(g(a=1, a=2));", "argument \"a\" supplied more than once", 1},
             Case{"echo(viaPlan());", "argument \"a\" supplied more than once", 1},
             Case{"m($fn=1, $fn=2);", "argument \"$fn\" supplied more than once", 1},
             Case{"cube(size=1, size=2);", "argument \"size\" supplied more than once", 1},
             Case{"cube(1, size=2);", "argument \"size\" overrides positional argument", 1},
             Case{"translate(v=[1,0,0], v=[2,0,0]) cube(1);", "argument \"v\" supplied more than once", 1},
             Case{"echo(fragments(r=1, r=10));", "argument \"r\" supplied more than once", 1},
             Case{"echo(fragments(1, r=10));", "argument \"r\" overrides positional argument", 1},
         }) {
        const auto w = warningsBothPaths(decls + c.code);
        EXPECT_EQ(count(w, c.needle), c.times) << c.code << ::testing::PrintToString(w);
    }
    // Unknown AND repeated: "not specified" for the first, "more than once"
    // for the second -- never "not specified" twice.
    const auto w = warningsBothPaths(decls + "m(b=1, b=2);");
    EXPECT_EQ(count(w, "variable \"b\" not specified as parameter"), 1u) << ::testing::PrintToString(w);
    EXPECT_EQ(count(w, "argument \"b\" supplied more than once"), 1u) << ::testing::PrintToString(w);
    // A positional argument AFTER the name skips it, as before: no warning.
    EXPECT_TRUE(warningsBothPaths("module n(a, b) {}\nn(a=1, 2);").empty());
    // object() and str() take any names: nothing to repeat.
    EXPECT_TRUE(warningsBothPaths("echo(object(a=1, a=2), str(a=1, a=1));").empty());
}

TEST(UnexpectedArgs, TheLastRepeatedValueWins) {
    for (const bool vm : {false, true}) {
        ScopedVm scoped(vm);
        Evaluated e = evalSrc("cube(size=1, size=2);\ntranslate(v=[1,0,0], v=[5,0,0]) cube(1);");
        ASSERT_EQ(e.bodies.size(), 2u);
        EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-9) << "vm " << vm;
        EXPECT_NEAR(e.bodies[1].body->BoundingBox().min.x, 5.0, 1e-9) << "vm " << vm;
        std::vector<std::string> echoes;
        evalSrc("module m(a) echo(a);\nm(a=1, a=2);\necho(fragments(r=1, r=10));\n"
                "echo(textmetrics(text=\"a\", text=\"bb\").size == textmetrics(\"bb\").size);",
                [&](const std::string& msg) { if (msg.rfind("ECHO: ", 0) == 0) echoes.push_back(msg); });
        EXPECT_EQ(echoes, (std::vector<std::string>{"ECHO: 2", "ECHO: 30", "ECHO: true"})) << "vm " << vm;
    }
}
