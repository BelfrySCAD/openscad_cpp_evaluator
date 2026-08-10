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
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6); // 4x3 square
    std::filesystem::remove(path);
}

TEST(DxfImport, ExpressionContextReturnsRegion) {
    const auto path = tempPath("square_expr.dxf");
    writeFile(path, kMinimalDxfSquare);
    Evaluator ev;
    Value v = asExpr("import(\"" + path.generic_string() + "\")", ev);
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
    Evaluated e = evalSrc("import(file=\"" + path.generic_string() + "\", layer=\"keep\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 1.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(DxfImport, ClosedPolylineVertexEntityProducesExpectedArea) {
    // The 2D POLYLINE/VERTEX/SEQEND entity shape, distinct from
    // LWPOLYLINE's inline 10/20 pairs -- every other DXF test in this file
    // exercises LWPOLYLINE only, leaving this whole code path untested.
    const std::string dxf =
        "0\nSECTION\n2\nENTITIES\n"
        "0\nPOLYLINE\n8\n0\n70\n1\n"
        "0\nVERTEX\n8\n0\n10\n0.0\n20\n0.0\n"
        "0\nVERTEX\n8\n0\n10\n4.0\n20\n0.0\n"
        "0\nVERTEX\n8\n0\n10\n4.0\n20\n3.0\n"
        "0\nVERTEX\n8\n0\n10\n0.0\n20\n3.0\n"
        "0\nSEQEND\n"
        "0\nENDSEC\n0\nEOF\n";
    const auto path = tempPath("polyline.dxf");
    writeFile(path, dxf);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(DxfImport, OpenPolylineIsIgnored) {
    // closed=false (group code 70 bit 0 unset, or absent entirely) means
    // the entity is dropped -- exercises the "!closed" half of the final
    // guard, distinct from every other DXF fixture here which is closed.
    const std::string dxf =
        "0\nSECTION\n2\nENTITIES\n"
        "0\nPOLYLINE\n8\n0\n"
        "0\nVERTEX\n8\n0\n10\n0.0\n20\n0.0\n"
        "0\nVERTEX\n8\n0\n10\n4.0\n20\n0.0\n"
        "0\nSEQEND\n"
        "0\nENDSEC\n0\nEOF\n";
    const auto path = tempPath("open_polyline.dxf");
    writeFile(path, dxf);
    Evaluator ev;
    auto ast = parseSrc("import(\"" + path.generic_string() + "\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    std::filesystem::remove(path);
}

TEST(DxfImport, UnrecognizedEntityIsSkipped) {
    // The trailing "else { ++i; }" branch in loadDxfContours' top-level
    // loop -- a group-0 entity name that's neither LWPOLYLINE nor
    // POLYLINE (e.g. LINE) must be skipped without disturbing the real
    // closed contour that follows it.
    const std::string dxf =
        "0\nSECTION\n2\nENTITIES\n"
        "0\nLINE\n8\n0\n10\n0.0\n20\n0.0\n11\n1.0\n21\n1.0\n"
        "0\nLWPOLYLINE\n8\n0\n90\n4\n70\n1\n"
        "10\n0.0\n20\n0.0\n10\n4.0\n20\n0.0\n10\n4.0\n20\n3.0\n10\n0.0\n20\n3.0\n"
        "0\nENDSEC\n0\nEOF\n";
    const auto path = tempPath("unrecognized.dxf");
    writeFile(path, dxf);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(DxfImport, NoClosedContoursErrors) {
    const std::string dxf = "0\nSECTION\n2\nENTITIES\n0\nENDSEC\n0\nEOF\n";
    const auto path = tempPath("empty.dxf");
    writeFile(path, dxf);
    Evaluator ev;
    auto ast = parseSrc("import(\"" + path.generic_string() + "\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    std::filesystem::remove(path);
}

// -- SVG import -----------------------------------------------------------

TEST(SvgImport, RectProducesExpectedArea) {
    const auto path = tempPath("rect.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><rect x="0" y="0" width="4" height="3"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
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
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
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
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 3.14159265 * 25.0, 1.0); // 32-segment approximation
    std::filesystem::remove(path);
}

TEST(SvgImport, CubicBezierPathIsWatertight) {
    const auto path = tempPath("bezier.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 C2,5 8,5 10,0 L10,-5 L0,-5 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_GT(e.bodies[0].section->Area(), 0.0);
    std::filesystem::remove(path);
}

TEST(SvgImport, NoShapesErrors) {
    const auto path = tempPath("noshapes.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><defs><rect x="0" y="0" width="1" height="1"/></defs></svg>)");
    Evaluator ev;
    auto ast = parseSrc("import(\"" + path.generic_string() + "\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    std::filesystem::remove(path);
}

TEST(SvgImport, EntityReferencesInAttributeValueDecodedWithoutCorruptingParse) {
    // decodeEntities' 5 named-entity branches are otherwise never exercised
    // (no existing fixture contains a literal '&'). `id` isn't read for
    // geometry, so the only observable effect is that decoding correctly
    // advances past each multi-character escape without leaving stray
    // bytes that would desync the following attributes' quote parsing --
    // if it did, the width/height attributes below would misparse and the
    // area would come out wrong (or parsing would throw).
    const auto path = tempPath("entities.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><rect id="a&amp;b&lt;c&gt;d&quot;e&apos;f" x="0" y="0" width="4" height="3"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, CommentCdataPrologAndDoctypeAreSkipped) {
    // Exercises all 4 of parseElement's non-element skip branches in one
    // fixture: XML prolog (`<?xml...?>`), DOCTYPE (`<!DOCTYPE...>`), a
    // comment, and a CDATA section -- none of which should become a child
    // node or otherwise disturb finding the real <rect>.
    const auto path = tempPath("skips.svg");
    writeFile(path, R"(<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg xmlns="http://www.w3.org/2000/svg">
<!-- a comment -->
<![CDATA[ignored data]]>
<rect x="0" y="0" width="4" height="3"/>
</svg>
)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, TransformMatrixIsApplied) {
    const auto path = tempPath("matrix.svg");
    writeFile(path, R"svg(<svg xmlns="http://www.w3.org/2000/svg">
        <g transform="matrix(2,0,0,2,5,0)"><rect x="0" y="0" width="4" height="3"/></g>
    </svg>)svg");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 48.0, 1e-6); // 2x scale in both axes -> 4x area
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.min.x, 5.0, 1e-6);
    EXPECT_NEAR(bounds.max.x, 13.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, TransformScaleWithSeparateXyIsApplied) {
    const auto path = tempPath("scale.svg");
    writeFile(path, R"svg(<svg xmlns="http://www.w3.org/2000/svg">
        <g transform="scale(2,3)"><rect x="0" y="0" width="4" height="3"/></g>
    </svg>)svg");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 72.0, 1e-6); // 2x * 3x -> 6x area
    std::filesystem::remove(path);
}

TEST(SvgImport, TransformRotateAboutOriginIsApplied) {
    const auto path = tempPath("rotate.svg");
    writeFile(path, R"svg(<svg xmlns="http://www.w3.org/2000/svg">
        <g transform="rotate(90)"><rect x="0" y="0" width="4" height="3"/></g>
    </svg>)svg");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6); // rotation preserves area
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    // Unrotated (just Y-flipped) bounds would be x in [0,4], y in [-3,0];
    // a 90-degree rotation must move the box away from that.
    EXPECT_NEAR(bounds.min.x, -3.0, 1e-6);
    EXPECT_NEAR(bounds.max.x, 0.0, 1e-6);
    EXPECT_NEAR(bounds.min.y, -4.0, 1e-6);
    EXPECT_NEAR(bounds.max.y, 0.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, HorizontalAndVerticalLineCommands) {
    const auto path = tempPath("hv.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 H4 V3 H0 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, SmoothCubicCommandFollowsCubic) {
    const auto path = tempPath("smoothcubic.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 C1,4 3,4 4,0 S7,-4 8,0 L8,-6 L0,-6 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_GT(e.bodies[0].section->Area(), 0.0);
    std::filesystem::remove(path);
}

TEST(SvgImport, SmoothQuadraticCommandFollowsQuadratic) {
    const auto path = tempPath("smoothquad.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 Q2,4 4,0 T8,0 L8,-4 L0,-4 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_GT(e.bodies[0].section->Area(), 0.0);
    std::filesystem::remove(path);
}

TEST(SvgImport, EllipticalArcCommand) {
    const auto path = tempPath("arc.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 A5,5 0 0,1 10,0 L10,-10 L0,-10 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_GT(e.bodies[0].section->Area(), 0.0);
    std::filesystem::remove(path);
}

TEST(SvgImport, PolygonPointsAttribute) {
    const auto path = tempPath("polygon.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><polygon points="0,0 4,0 4,3 0,3"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, PolylinePointsAttribute) {
    const auto path = tempPath("polyline.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><polyline points="0,0 4,0 4,3 0,3"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, StrayCharacterInPointsListIsSkipped) {
    const auto path = tempPath("straypoints.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><polygon points="0,0 x 4,0 4,3 0,3"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, StrayCharacterInPathDataIsSkipped) {
    const auto path = tempPath("straypath.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 L4,0 @ L4,3 L0,3 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, TransformNumberWithExponentIsParsed) {
    // parseNumberList's own scientific-notation branch (used while parsing
    // a transform="translate(...)" argument list), distinct from
    // tokenizePath's separate copy of the same logic for path `d` data.
    const auto path = tempPath("exponent_transform.svg");
    writeFile(path, R"svg(<svg xmlns="http://www.w3.org/2000/svg">
        <g transform="translate(1e1,0)"><rect x="0" y="0" width="4" height="3"/></g>
    </svg>)svg");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.min.x, 10.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, PathNumberWithExponentIsParsed) {
    // tokenizePath's own scientific-notation branch, for numbers inside a
    // path `d` attribute rather than a transform argument list.
    const auto path = tempPath("exponent_path.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 L4e0,0 L4,3 L0,3 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(SvgImport, EllipticalArcWithOutOfRangeRadiiIsScaledUp) {
    // arcPts' own radius-correction branch (SVG spec: if the requested
    // rx/ry are too small to span the chord between the two endpoints at
    // all, both are scaled up by the same factor until they just barely
    // can) -- the other arc test's radii were already large enough to
    // never hit this path.
    const auto path = tempPath("arc_scaled.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 A1,1 0 0,1 10,0 L10,-10 L0,-10 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_GT(e.bodies[0].section->Area(), 0.0);
    std::filesystem::remove(path);
}

TEST(SvgImport, MultipleSubpathsInOnePathProduceMultipleContours) {
    // parsePathD's "M after an already-open contour" branch: a second `M`
    // closes off the first subpath (pushing it onto `contours`) before
    // starting a new one, distinct from an explicit `Z`.
    const auto path = tempPath("multisubpath.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><path d="M0,0 L4,0 L4,3 L0,3 Z M10,0 L14,0 L14,3 L10,3 Z"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 24.0, 1e-6); // two disjoint 4x3 squares
    std::filesystem::remove(path);
}
