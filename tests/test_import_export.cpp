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

TEST(ImportModuleContext, MissingFileArgumentErrors) {
    Evaluator ev;
    auto ast = parseSrc("import();");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
}

TEST(ImportModuleContext, JsonExtensionErrorsAsGeometryStatement) {
    const auto path = tempPath("data_as_module.json");
    {
        std::ofstream out(path);
        out << R"({"a": 1})";
    }
    Evaluator ev;
    auto ast = parseSrc("import(\"" + path.string() + "\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, MalformedMeshFileErrors) {
    // loadMeshByExt's own exception -> caught and rethrown as ev.error()
    // inside resolveImport's try/catch -- every other STL/OBJ/OFF/3MF test
    // here round-trips a well-formed file written by this project's own
    // exporter.
    const auto path = tempPath("malformed.stl");
    {
        std::ofstream out(path);
        out << "this is not a valid STL file at all";
    }
    Evaluator ev;
    auto ast = parseSrc("import(\"" + path.string() + "\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, NonManifoldMeshWarns) {
    // generateImport's own Manifold::Status() != NoError branch -- a
    // single free-floating triangle (not welded to anything, non-manifold
    // boundary) via a hand-written OFF file, distinct from every other
    // mesh test here which round-trips a valid, closed, watertight cube.
    const auto path = tempPath("nonmanifold.off");
    {
        std::ofstream out(path);
        out << "OFF\n3 1 0\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n";
    }
    std::string lastWarning;
    Evaluated e = evalSrc("import(\"" + path.string() + "\");", [&](const std::string& msg) { lastWarning = msg; });
    EXPECT_NE(lastWarning.find("import: mesh is not a closed solid"), std::string::npos);
    // The triangle is now handed back for display rather than dropped: a
    // file that warns once and then shows nothing gives no way to see what
    // is actually wrong with it. It carries no Manifold, so it still can't
    // take part in a CSG operation.
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_TRUE(e.bodies[0].isDisplayOnly());
    EXPECT_EQ(e.bodies[0].rawMesh->triVerts.size(), 3u);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, EmptyMeshHasNoTrianglesErrors) {
    // generateImport's own "tris.empty()" check -- only reachable at
    // generate time (not resolveTree()'s own try/catch, which only guards
    // loadMeshByExt itself), so this needs the full resolve+generate
    // pipeline, unlike every other error test in this file.
    const auto path = tempPath("empty.off");
    {
        std::ofstream out(path);
        out << "OFF\n0 0 0\n";
    }
    EXPECT_THROW(evalSrc("import(\"" + path.string() + "\");"), EvalError);
    std::filesystem::remove(path);
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

TEST(ImportExpressionContext, JsonNullValueBecomesUndef) {
    // jsonToValue's own final fallback (null, or any other unhandled JSON
    // node type) -- every other JsonReturnsNativeValues field above is a
    // string/number/object/list.
    const auto path = tempPath("data_null.json");
    {
        std::ofstream out(path);
        out << R"({"x": null})";
    }
    Evaluator ev;
    Value v = asExpr("import(\"" + path.string() + "\")", ev);
    const auto& obj = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(obj.size(), 1u);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(obj[0].second));
    std::filesystem::remove(path);
}

TEST(ImportExpressionContext, DxfReturnsRegionContours) {
    const auto path = tempPath("square_expr_ie.dxf");
    {
        std::ofstream out(path);
        out << "0\nSECTION\n2\nENTITIES\n0\nLWPOLYLINE\n8\n0\n90\n4\n70\n1\n"
               "10\n0.0\n20\n0.0\n10\n1.0\n20\n0.0\n10\n1.0\n20\n1.0\n10\n0.0\n20\n1.0\n"
               "0\nENDSEC\n0\nEOF\n";
    }
    Evaluator ev;
    Value v = asExpr("import(\"" + path.string() + "\")", ev);
    const auto& contours = std::get<ListPtr>(v)->items;
    ASSERT_EQ(contours.size(), 1u);
    std::filesystem::remove(path);
}

TEST(ImportExpressionContext, MissingFileArgumentErrors) {
    Evaluator ev;
    EXPECT_THROW(asExpr("import()", ev), EvalError);
}

TEST(ImportExpressionContext, UnsupportedExtensionErrors) {
    Evaluator ev;
    EXPECT_THROW(asExpr("import(\"nope.xyz\")", ev), EvalError);
}

TEST(ImportExpressionContext, MalformedMeshFileErrors) {
    const auto path = tempPath("malformed_expr.stl");
    {
        std::ofstream out(path);
        out << "not a valid STL file";
    }
    Evaluator ev;
    EXPECT_THROW(asExpr("import(\"" + path.string() + "\")", ev), EvalError);
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
