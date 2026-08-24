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
