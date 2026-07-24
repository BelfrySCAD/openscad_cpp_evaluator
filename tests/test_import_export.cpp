#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/export.hpp"

#include "test_helpers.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <manifold/manifold.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::filesystem::path tempPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("oscad_eval_test_" + name);
}

// A unit cube (size 2, centered -> volume 8) exported to `path` in every
// format under test, via the real evaluator pipeline (not a hand-built
// mesh) so export.cpp itself is exercised too.
void writeCubeAs(const std::filesystem::path& path, void (*writer)(const std::string&, const std::vector<ColoredBody>&)) {
    Evaluated e = evalSrc("cube(2, center=true);");
    writer(path.string(), e.bodies);
}

Value asExpr(const std::string& code, Evaluator& ev) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    const oscad::Expression* expr = exprSrc(code, ast);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    return ev.evalExpr(*expr, ctx);
}

} // namespace

// -- Module-context import() (geometry statement) --------------------------

TEST(ImportModuleContext, StlRoundTripPreservesVolume) {
    const auto path = tempPath("cube.stl");
    writeCubeAs(path, &writeStl);
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, ObjRoundTripPreservesVolume) {
    const auto path = tempPath("cube.obj");
    writeCubeAs(path, &writeObj);
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, OffRoundTripPreservesVolume) {
    const auto path = tempPath("cube.off");
    writeCubeAs(path, &writeOff);
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, ThreeMfRoundTripPreservesVolume) {
    const auto path = tempPath("cube.3mf");
    writeCubeAs(path, &writeThreeMf);
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, UnsupportedExtensionErrors) {
    Evaluator ev;
    auto ast = parseSrc("import(\"nope.xyz\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
}

// -- Expression-context import() -------------------------------------------

TEST(ImportExpressionContext, StlReturnsVnfShape) {
    const auto path = tempPath("cube_vnf.stl");
    writeCubeAs(path, &writeStl);
    Evaluator ev;
    Value v = asExpr("import(\"" + path.string() + "\")", ev);
    const auto& outer = std::get<ListPtr>(v)->items;
    ASSERT_EQ(outer.size(), 2u);
    const auto& verts = std::get<ListPtr>(outer[0])->items;
    const auto& faces = std::get<ListPtr>(outer[1])->items;
    EXPECT_EQ(verts.size(), 8u); // a welded cube has 8 corners
    EXPECT_GT(faces.size(), 0u);
    std::filesystem::remove(path);
}

TEST(ImportExpressionContext, JsonReturnsNativeValues) {
    const auto path = tempPath("data.json");
    {
        std::ofstream out(path);
        out << R"({"name": "x", "n": 3, "nested": {"a": 1, "b": 2}, "list": [1, 2, 3]})";
    }
    Evaluator ev;
    Value v = asExpr("import(\"" + path.string() + "\")", ev);
    const auto& obj = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(obj.size(), 4u);
    EXPECT_EQ(obj[0].first, "name");
    EXPECT_EQ(std::get<std::string>(obj[0].second), "x");
    EXPECT_EQ(obj[1].first, "n");
    EXPECT_DOUBLE_EQ(std::get<double>(obj[1].second), 3.0);
    const auto& nested = std::get<ObjectPtr>(obj[2].second)->items;
    ASSERT_EQ(nested.size(), 2u);
    EXPECT_EQ(nested[0].first, "a");
    const auto& list = std::get<ListPtr>(obj[3].second)->items;
    ASSERT_EQ(list.size(), 3u);
    EXPECT_DOUBLE_EQ(std::get<double>(list[2]), 3.0);
    std::filesystem::remove(path);
}

// -- object()/is_object()/has_key() ----------------------------------------

TEST(ObjectBuiltin, ConstructsOrderedMapFromNamedArgs) {
    Evaluator ev;
    Value v = asExpr("object(b=2, a=1)", ev);
    const auto& items = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(items.size(), 2u);
    EXPECT_TRUE(std::get<bool>(asExpr("is_object(object(a=1))", ev)));
}

TEST(ObjectBuiltin, PositionalMergesExistingObject) {
    Evaluator ev;
    Value v = asExpr("object(object(a=1,b=2), c=3)", ev);
    const auto& items = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(items.size(), 3u);
}
