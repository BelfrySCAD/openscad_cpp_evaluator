// Tests for `use <file>` statement resolution (eval_use.hpp/.cpp) --
// cross-checked directly against the Python reference's resolve_use_scopes
// on identical fixtures before being ported here (no existing Python test
// suite for this feature to port test cases *from* -- these were written
// from scratch, then verified byte-for-byte against real Python output).

#include "openscad_cpp_evaluator/eval_use.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

using namespace oscadeval;

namespace {

std::filesystem::path writeFile(const std::string& name, const std::string& content) {
    const auto p = std::filesystem::temp_directory_path() / ("oscad_use_test_" + name);
    std::ofstream out(p);
    out << content;
    return p;
}

struct UseEvaluated {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    ResolvedUseScopes used;
    Evaluator ev;
    std::vector<ColoredBody> bodies;
};

UseEvaluated evalFile(const std::filesystem::path& path, EchoFn echoFn = {}) {
    UseEvaluated e{oscad::getASTFromFile(path.string()), {}, Evaluator(echoFn), {}};
    e.used = resolveUseScopes(e.ast, path.string(), echoFn);
    e.ev.setUsedFileGlobals(e.used.usedFileGlobals);
    EvalContext ctx = EvalContext::makeRoot(e.used.rootScope.get());
    e.bodies = e.ev.evaluate(e.used.processedNodes, ctx);
    return e;
}

} // namespace

TEST(UseStatement, InjectsFunctionAndModuleButNotVariables) {
    auto lib = writeFile("lib1.scad", "lib_var = 100;\n"
                                       "function lib_add(a, b) = a + b + lib_var;\n"
                                       "module lib_cube(s) { cube(s); }\n");
    auto main = writeFile("main1.scad", "use <" + lib.string() + ">\n"
                                         "echo(lib_add(1, 2));\n"
                                         "echo(is_undef(lib_var));\n"
                                         "lib_cube(5);\n");
    std::vector<std::string> echoed;
    UseEvaluated e = evalFile(main, [&](const std::string& m) { echoed.push_back(m); });

    // Two, not three: is_undef() is a probe and does not warn about the name
    // it is asking about -- matching the reference, which is silent for this
    // exact script. `ECHO: true` is what proves lib_var wasn't injected; the
    // warning this used to also assert was our own divergence, not the point.
    ASSERT_EQ(echoed.size(), 2u);
    EXPECT_EQ(echoed[0], "ECHO: 103"); // 1 + 2 + lib_var(100) -- proves lib_add's body resolves
                                        // lib_var from *its own* file's scope (reanchoring)
    EXPECT_EQ(echoed[1], "ECHO: true"); // is_undef(lib_var) in main's own scope -- NOT injected
    EXPECT_EQ(e.bodies.size(), 1u);     // lib_cube(5) still produced geometry
    std::filesystem::remove(lib);
    std::filesystem::remove(main);
}

TEST(UseStatement, NestedUseNotReExported) {
    auto base = writeFile("base2.scad", "function base_fn(x) = x * 10;\n");
    auto mid = writeFile("mid2.scad", "use <" + base.string() + ">\n"
                                       "function mid_fn(x) = base_fn(x) + 1;\n");
    auto main = writeFile("main2.scad", "use <" + mid.string() + ">\n"
                                         "echo(mid_fn(5));\n"
                                         "echo(is_undef(base_fn(5)));\n");
    std::vector<std::string> echoed;
    UseEvaluated e = evalFile(main, [&](const std::string& m) { echoed.push_back(m); });

    ASSERT_EQ(echoed.size(), 3u);
    EXPECT_EQ(echoed[0], "ECHO: 51"); // mid_fn can call base_fn (its own use)
    EXPECT_NE(echoed[1].find("WARNING: Ignoring unknown function 'base_fn'"), std::string::npos); // not re-exported
    EXPECT_EQ(echoed[2], "ECHO: true");
    std::filesystem::remove(base);
    std::filesystem::remove(mid);
    std::filesystem::remove(main);
}

TEST(UseStatement, MissingFileSilentlySkipped) {
    // Real OpenSCAD (and this port's own Python reference) doesn't warn at
    // all for a use target that can't be found -- verified directly
    // against the Python reference.
    auto main = writeFile("main3.scad", "use <does_not_exist_xyz_12345.scad>\n"
                                         "cube(2);\n");
    std::vector<std::string> echoed;
    UseEvaluated e = evalFile(main, [&](const std::string& m) { echoed.push_back(m); });
    EXPECT_TRUE(echoed.empty());
    EXPECT_EQ(e.bodies.size(), 1u);
    std::filesystem::remove(main);
}

TEST(UseStatement, SyntaxErrorInUsedFileLogsAndContinues) {
    auto bad = writeFile("bad4.scad", "this is not valid scad ((((\n");
    auto main = writeFile("main4.scad", "use <" + bad.string() + ">\n"
                                         "cube(2);\n");
    std::vector<std::string> echoed;
    UseEvaluated e = evalFile(main, [&](const std::string& m) { echoed.push_back(m); });
    ASSERT_EQ(echoed.size(), 1u);
    EXPECT_NE(echoed[0].find("use error:"), std::string::npos);
    EXPECT_EQ(e.bodies.size(), 1u); // evaluation continues with just main's own nodes
    std::filesystem::remove(bad);
    std::filesystem::remove(main);
}

TEST(UseStatement, NoUseStatementsLeavesOwnNodesUnchanged) {
    auto main = writeFile("main5.scad", "cube(3);\n");
    UseEvaluated e = evalFile(main);
    EXPECT_EQ(e.used.processedNodes.size(), e.ast.size());
    EXPECT_EQ(e.bodies.size(), 1u);
    std::filesystem::remove(main);
}

// A file's globals are evaluated ONCE per run. OpenSCAD re-evaluates a used
// file's globals on every call into it (a long-standing upstream bug, not
// reference behaviour); this evaluator used to re-evaluate ANY global on
// every READ from inside a function, so `g() != g()` below.
TEST(UseStatement, UsedFileGlobalsEvaluatedOncePerRun) {
    auto lib = writeFile("lib_once.scad", "r = rands(0, 1, 1);\n"
                                           "function g() = r;\n"
                                           "function h() = r;\n");
    auto main = writeFile("main_once.scad", "use <" + lib.string() + ">\n"
                                             "echo(g() == g(), g() == h());\n");
    std::vector<std::string> echoed;
    evalFile(main, [&](const std::string& m) { echoed.push_back(m); });
    ASSERT_EQ(echoed.size(), 1u);
    EXPECT_EQ(echoed[0], "ECHO: true, true");
    std::filesystem::remove(lib);
    std::filesystem::remove(main);
}

// Source order, eagerly, so a later global read from an earlier one's
// initializer is undef with one warning -- what OpenSCAD prints too (three
// times there, once per call).
TEST(UseStatement, UsedFileForwardReadIsUndef) {
    auto lib = writeFile("lib_fwd.scad", "q = is_undef(w) ? \"early\" : w;\n"
                                          "w = 5;\n"
                                          "function uq() = q;\n"
                                          "function uw() = w;\n");
    auto main = writeFile("main_fwd.scad", "use <" + lib.string() + ">\n"
                                            "echo(uq(), uw(), uq());\n");
    std::vector<std::string> echoed;
    evalFile(main, [&](const std::string& m) { echoed.push_back(m); });
    ASSERT_EQ(echoed.size(), 1u);
    EXPECT_EQ(echoed[0], "ECHO: \"early\", 5, \"early\"");
    std::filesystem::remove(lib);
    std::filesystem::remove(main);
}

// The main file's globals go the same way: read back from the values the
// top-level pass already computed, never re-evaluated.
TEST(UseStatement, MainFileGlobalReadFromFunctionIsStable) {
    auto main = writeFile("main_stable.scad", "r = rands(0, 1, 1);\n"
                                               "function f() = r;\n"
                                               "echo(f() == f(), f() == r);\n"
                                               "x = g();\n"
                                               "function g() = y;\n"
                                               "y = 1;\n"
                                               "echo(x);\n");
    std::vector<std::string> echoed;
    evalFile(main, [&](const std::string& m) { echoed.push_back(m); });
    ASSERT_EQ(echoed.size(), 3u);
    EXPECT_NE(echoed[0].find("Ignoring unknown variable 'y'"), std::string::npos); // forward read: undef, as in OpenSCAD
    EXPECT_EQ(echoed[1], "ECHO: true, true");
    EXPECT_EQ(echoed[2], "ECHO: undef");
    std::filesystem::remove(main);
}
