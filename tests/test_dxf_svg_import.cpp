#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::filesystem::path tempPath(const std::string& name) { return std::filesystem::temp_directory_path() / ("oscad_eval_test_" + name); }

void writeFile(const std::filesystem::path& path, const std::string& content) {
    std::ofstream out(path);
    out << content;
}

Value asExpr(const std::string& code, Evaluator& ev) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    const oscad::Expression* expr = exprSrc(code, ast);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    return ev.evalExpr(*expr, ctx);
}

const char* kMinimalDxfSquare =
    "0\nSECTION\n2\nENTITIES\n0\nLWPOLYLINE\n8\n0\n90\n4\n70\n1\n"
    "10\n0.0\n20\n0.0\n10\n4.0\n20\n0.0\n10\n4.0\n20\n3.0\n10\n0.0\n20\n3.0\n"
    "0\nENDSEC\n0\nEOF\n";

} // namespace

// -- DXF import ---------------------------------------------------------

TEST(DxfImport, ClosedLwpolylineProducesExpectedBoundingBox) {
    const auto path = tempPath("square.dxf");
    writeFile(path, kMinimalDxfSquare);
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6); // 4x3 square
    std::filesystem::remove(path);
}

TEST(DxfImport, ExpressionContextReturnsRegion) {
    const auto path = tempPath("square_expr.dxf");
    writeFile(path, kMinimalDxfSquare);
    Evaluator ev;
    Value v = asExpr("import(\"" + path.string() + "\")", ev);
    const auto& contours = std::get<ListPtr>(v)->items;
    ASSERT_EQ(contours.size(), 1u);
    const auto& pts = std::get<ListPtr>(contours[0])->items;
    ASSERT_EQ(pts.size(), 4u);
    const auto& p0 = std::get<ListPtr>(pts[0])->items;
    EXPECT_DOUBLE_EQ(std::get<double>(p0[0]), 0.0);
    EXPECT_DOUBLE_EQ(std::get<double>(p0[1]), 0.0);
    std::filesystem::remove(path);
}

TEST(DxfImport, LayerFilterExcludesOtherLayers) {
    const std::string dxf =
        "0\nSECTION\n2\nENTITIES\n"
        "0\nLWPOLYLINE\n8\nkeep\n90\n4\n70\n1\n10\n0.0\n20\n0.0\n10\n1.0\n20\n0.0\n10\n1.0\n20\n1.0\n10\n0.0\n20\n1.0\n"
        "0\nLWPOLYLINE\n8\ndrop\n90\n4\n70\n1\n10\n5.0\n20\n5.0\n10\n6.0\n20\n5.0\n10\n6.0\n20\n6.0\n10\n5.0\n20\n6.0\n"
        "0\nENDSEC\n0\nEOF\n";
    const auto path = tempPath("layered.dxf");
    writeFile(path, dxf);
    Evaluated e = evalSrc("import(file=\"" + path.string() + "\", layer=\"keep\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 1.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(DxfImport, NoClosedContoursErrors) {
    const std::string dxf = "0\nSECTION\n2\nENTITIES\n0\nENDSEC\n0\nEOF\n";
    const auto path = tempPath("empty.dxf");
    writeFile(path, dxf);
    Evaluator ev;
    auto ast = parseSrc("import(\"" + path.string() + "\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    std::filesystem::remove(path);
}

// -- SVG import -----------------------------------------------------------

TEST(SvgImport, RectProducesExpectedArea) {
    const auto path = tempPath("rect.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><rect x="0" y="0" width="4" height="3"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, PathWithGroupTransformIsTranslatedAndYFlipped) {
    const auto path = tempPath("group.svg");
    writeFile(path, R"svg(<svg xmlns="http://www.w3.org/2000/svg">
        <g transform="translate(10,0)"><path d="M0,0 L4,0 L4,3 L0,3 Z"/></g>
    </svg>)svg");
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.min.x, 10.0, 1e-6);
    EXPECT_NEAR(bounds.max.x, 14.0, 1e-6);
    // SVG's Y axis points down; OpenSCAD's points up, so a shape drawn at
    // y in [0,3] in SVG-source coordinates must land at y in [-3,0].
    EXPECT_NEAR(bounds.min.y, -3.0, 1e-6);
    EXPECT_NEAR(bounds.max.y, 0.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, CircleApproximatesAnalyticArea) {
    const auto path = tempPath("circle.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><circle cx="0" cy="0" r="5"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 3.14159265 * 25.0, 1.0); // 32-segment approximation
    std::filesystem::remove(path);
}

TEST(SvgImport, CubicBezierPathIsWatertight) {
    const auto path = tempPath("bezier.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 C2,5 8,5 10,0 L10,-5 L0,-5 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_GT(e.bodies[0].section->Area(), 0.0);
    std::filesystem::remove(path);
}

TEST(SvgImport, NoShapesErrors) {
    const auto path = tempPath("noshapes.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><defs><rect x="0" y="0" width="1" height="1"/></defs></svg>)");
    Evaluator ev;
    auto ast = parseSrc("import(\"" + path.string() + "\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    std::filesystem::remove(path);
}
