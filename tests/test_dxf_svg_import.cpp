#include "openscad_cpp_evaluator/dxf_svg_import.hpp"
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
    std::vector<std::string> log;
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");", [&](const std::string& m) { log.push_back(m); });
    EXPECT_TRUE(e.bodies.empty() || e.bodies[0].section->IsEmpty());
    EXPECT_TRUE(log.empty()) << ::testing::PrintToString(log);
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

// Nothing to fill imports nothing, silently, as in OpenSCAD.
TEST(DxfImport, NoClosedContoursIsSilent) {
    const std::string dxf = "0\nSECTION\n2\nENTITIES\n0\nENDSEC\n0\nEOF\n";
    const auto path = tempPath("empty.dxf");
    writeFile(path, dxf);
    std::vector<std::string> log;
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\"); echo(\"after\");",
                          [&](const std::string& m) { log.push_back(m); });
    EXPECT_EQ(log, std::vector<std::string>{"ECHO: \"after\""});
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

// A character no attribute can start with -- here a '/' not followed by '>'
// -- used to stop the attribute scan without consuming anything, and the
// loop came back to it forever. The rect after it must still be read.
TEST(SvgImport, StrayCharactersInATagDoNotHang) {
    const auto path = tempPath("stray.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><g / =x><rect x="0" y="0" width="4" height="3"/></g></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    std::filesystem::remove(path);
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].section->Area(), 12.0, 1e-6);
}

// A PDF under an .svg name is what hung for good: every "/Name" in it hit
// that loop. It is not an SVG, so it is an error -- but an error, not a hang.
TEST(SvgImport, APdfNamedSvgErrorsInsteadOfHanging) {
    const auto path = tempPath("really_a_pdf.svg");
    writeFile(path, "%PDF-1.4\n1 0 obj\n<< /Type /Catalog /Pages 2 0 R >>\nendobj\n"
                    "2 0 obj\n<< /Type /Pages /Kids [3 0 R] /Count 1 >>\nendobj\n%%EOF\n");
    const std::string err = importFailure("import(\"" + path.generic_string() + "\");");
    EXPECT_NE(err.find("ERROR: Error parsing file '"), std::string::npos) << err;
    std::filesystem::remove(path);
}

// .pdf used to be handed to the SVG parser (and hung). Nothing reads PDF,
// so it is refused like any other format import() does not know.
TEST(SvgImport, PdfIsAnUnsupportedFileType) {
    const auto path = tempPath("drawing.pdf");
    writeFile(path, "%PDF-1.4\n<< /Type /Catalog >>\n%%EOF\n");
    const std::string err = importFailure("import(\"" + path.generic_string() + "\");");
    EXPECT_NE(err.find("Unsupported file format while trying to import file"), std::string::npos) << err;
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
    EXPECT_NEAR(e.bodies[0].section->Area(), 3.14159265 * 25.0, 1.0); // 40-segment approximation
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

TEST(SvgImport, NoShapesIsSilent) {
    const auto path = tempPath("noshapes.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><defs><rect x="0" y="0" width="1" height="1"/></defs></svg>)");
    std::vector<std::string> log;
    evalSrc("import(\"" + path.generic_string() + "\");", [&](const std::string& m) { log.push_back(m); });
    EXPECT_TRUE(log.empty()) << ::testing::PrintToString(log);
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

// A polyline is a line, never filled: OpenSCAD draws its stroke (width 1
// by default, butt caps, mitred joins) -- a 4.5 x 4 C shape, area 11.
TEST(SvgImport, PolylineIsDrawnAsItsStroke) {
    const auto path = tempPath("polyline.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><polyline points="0,0 4,0 4,3 0,3"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 11.0, 1e-6);
    std::filesystem::remove(path);
}

// Not skipped, as OpenSCAD reads it: the stray "x" is the number 0, which
// shifts every pair after it -- (0,0) (0,4) (0,4) (3,0), a triangle of 6.
TEST(SvgImport, StrayCharacterInPointsListReadsAsZero) {
    const auto path = tempPath("straypoints.svg");
    writeFile(path, R"(<svg xmlns="http://www.w3.org/2000/svg"><polygon points="0,0 x 4,0 4,3 0,3"/></svg>)");
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 6.0, 1e-6);
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

// -- dxf_dim() / dxf_cross() ----------------------------------------------
//
// Every expectation below was produced by running the identical script and
// the identical .dxf through OpenSCAD 2026.02.01 -- all sixteen cases,
// warnings included, came back byte-identical.
//
// Paths go into the script via generic_string(), like every other test in
// this file: a Windows path's backslashes are escape characters inside a
// .scad string literal, so path.string() mangles the filename and every one
// of these fails on Windows CI while passing everywhere else.

namespace {

// Group-code pairs, one per line, exactly as a DXF stores them.
const char* kDimDxf =
    "0\nSECTION\n2\nENTITIES\n"
    // type 0 (rotated), angle 0, coords[3]->coords[4] spans 25 in x
    "0\nDIMENSION\n8\ndims\n1\nwidth\n70\n0\n50\n0\n"
    "10\n0\n20\n0\n13\n0\n23\n0\n14\n25\n24\n0\n"
    // type 0 at 90 degrees, spanning 40 in y
    "0\nDIMENSION\n8\ndims\n1\nheight\n70\n0\n50\n90\n"
    "10\n0\n20\n0\n13\n0\n23\n0\n14\n0\n24\n40\n"
    // type 1 (aligned): a 3-4-5 triangle
    "0\nDIMENSION\n8\ndims\n1\ndiag\n70\n1\n13\n0\n23\n0\n14\n3\n24\n4\n"
    // type 4 (radius): coords[5] - coords[0], a 6-8-10 triangle
    "0\nDIMENSION\n8\ndims\n1\nrad\n70\n4\n10\n0\n20\n0\n15\n6\n25\n8\n"
    // type 6 (ordinate), no bit 64 -> the y of coords[3]
    "0\nDIMENSION\n8\ndims\n1\nord\n70\n6\n13\n7\n23\n9\n"
    // ...and with bit 64 set -> the x instead
    "0\nDIMENSION\n8\ndims\n1\nordx\n70\n70\n13\n7\n23\n9\n"
    // type 2 (angular): a quarter turn
    "0\nDIMENSION\n8\ndims\n1\nang\n70\n2\n"
    "10\n1\n20\n0\n15\n0\n25\n0\n13\n0\n23\n0\n14\n0\n24\n1\n"
    // a cross: two LINEs meeting at (10, 20)
    "0\nLINE\n8\ncross\n10\n0\n20\n20\n11\n20\n21\n20\n"
    "0\nLINE\n8\ncross\n10\n10\n20\n0\n11\n10\n21\n40\n"
    // two parallel LINEs, which have no crossing point
    "0\nLINE\n8\npar\n10\n0\n20\n0\n11\n10\n21\n0\n"
    "0\nLINE\n8\npar\n10\n0\n20\n5\n11\n10\n21\n5\n"
    // an outline: two LINEs joined end to end cross mid-way, but form one path
    "0\nLINE\n8\noutline\n10\n0\n20\n0\n11\n10\n21\n10\n"
    "0\nLINE\n8\noutline\n10\n10\n20\n10\n11\n0\n21\n10\n"
    "0\nLINE\n8\noutline\n10\n0\n20\n10\n11\n10\n21\n0\n"
    // a LINE closed by an ARC (centre (0,0), r 10, 0..90 degrees): a path too
    "0\nLINE\n8\narc\n10\n10\n20\n0\n11\n0\n21\n10\n"
    "0\nARC\n8\narc\n10\n0\n20\n0\n40\n10\n50\n0\n51\n90\n"
    "0\nLINE\n8\narc\n10\n0\n20\n0\n11\n10\n21\n10\n"
    "0\nENDSEC\n0\nEOF\n";

double dimOf(const std::string& call, Evaluator& ev) { return std::get<double>(asExpr(call, ev)); }

} // namespace

TEST(DxfDim, ReadsEachDimensionType) {
    const auto path = tempPath("dims.dxf");
    writeFile(path, kDimDxf);
    Evaluator ev;
    const std::string f = "file=\"" + path.generic_string() + "\", layer=\"dims\"";
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"width\")", ev), 25.0, 1e-9);
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"height\")", ev), 40.0, 1e-9);
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"diag\")", ev), 5.0, 1e-9);   // aligned
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"rad\")", ev), 10.0, 1e-9);   // radius
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"ord\")", ev), 9.0, 1e-9);    // ordinate y
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"ordx\")", ev), 7.0, 1e-9);   // ordinate x
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"ang\")", ev), 90.0, 1e-9);   // angular
    std::filesystem::remove(path);
}

TEST(DxfDim, ScaleAndOriginApply) {
    const auto path = tempPath("dims_scale.dxf");
    writeFile(path, kDimDxf);
    Evaluator ev;
    const std::string f = "file=\"" + path.generic_string() + "\", layer=\"dims\"";
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"width\", scale=2)", ev), 50.0, 1e-9);
    // origin shifts positions but not the distance between two of them.
    EXPECT_NEAR(dimOf("dxf_dim(" + f + ", name=\"width\", origin=[5,5])", ev), 25.0, 1e-9);
    std::filesystem::remove(path);
}

TEST(DxfDim, NameAndLayerFilter) {
    const auto path = tempPath("dims_filter.dxf");
    writeFile(path, kDimDxf);
    Evaluator ev;
    // No name given -> the first dimension on that layer.
    EXPECT_NEAR(dimOf("dxf_dim(file=\"" + path.generic_string() + "\", layer=\"dims\")", ev), 25.0, 1e-9);
    // No layer given -> any layer.
    EXPECT_NEAR(dimOf("dxf_dim(file=\"" + path.generic_string() + "\", name=\"diag\")", ev), 5.0, 1e-9);
    std::filesystem::remove(path);
}

TEST(DxfDim, MissingDimensionAndMissingFileWarn) {
    const auto path = tempPath("dims_missing.dxf");
    writeFile(path, kDimDxf);
    std::string last;
    Evaluator ev([&](const std::string& m) { last = m; });
    EXPECT_TRUE(std::holds_alternative<std::monostate>(
        asExpr("dxf_dim(file=\"" + path.generic_string() + "\", layer=\"dims\", name=\"nope\")", ev)));
    EXPECT_NE(last.find("Can't find dimension 'nope'"), std::string::npos) << last;

    EXPECT_TRUE(std::holds_alternative<std::monostate>(
        asExpr("dxf_dim(file=\"definitely_not_here.dxf\", name=\"x\")", ev)));
    EXPECT_NE(last.find("Can't open DXF file"), std::string::npos) << last;
    std::filesystem::remove(path);
}

TEST(DxfCross, FindsTheCrossingPoint) {
    const auto path = tempPath("cross.dxf");
    writeFile(path, kDimDxf);
    Evaluator ev;
    Value v = asExpr("dxf_cross(file=\"" + path.generic_string() + "\", layer=\"cross\")", ev);
    const auto& xy = std::get<ListPtr>(v)->items;
    ASSERT_EQ(xy.size(), 2u);
    EXPECT_NEAR(std::get<double>(xy[0]), 10.0, 1e-9);
    EXPECT_NEAR(std::get<double>(xy[1]), 20.0, 1e-9);

    // origin shifts the answer with the geometry.
    Value o = asExpr("dxf_cross(file=\"" + path.generic_string() + "\", layer=\"cross\", origin=[5,5])", ev);
    const auto& oxy = std::get<ListPtr>(o)->items;
    EXPECT_NEAR(std::get<double>(oxy[0]), 5.0, 1e-9);
    EXPECT_NEAR(std::get<double>(oxy[1]), 15.0, 1e-9);
    std::filesystem::remove(path);
}

// The reference counts only 2-point paths: a LINE sharing an end with
// another LINE or an ARC belongs to an outline and is no stroke of a cross,
// even where two such lines do intersect. Checked against OpenSCAD
// 2026.02.01 on example009.dxf's fan_top layer.
TEST(DxfCross, LinesJoinedIntoAPathAreNoCross) {
    const auto path = tempPath("dxfcross_paths.dxf");
    writeFile(path, kDimDxf);
    for (const char* layer : {"outline", "arc"}) {
        std::string last;
        Evaluator ev([&](const std::string& m) { last = m; });
        EXPECT_TRUE(std::holds_alternative<std::monostate>(asExpr("dxf_cross(file=\"" + path.generic_string() + "\", layer=\"" + layer + "\")", ev)))
            << layer;
        EXPECT_NE(last.find("Can't find cross"), std::string::npos) << layer << ": " << last;
    }
}

TEST(DxfCross, ParallelOrTooFewLinesWarns) {
    const auto path = tempPath("cross_par.dxf");
    writeFile(path, kDimDxf);
    std::string last;
    Evaluator ev([&](const std::string& m) { last = m; });
    EXPECT_TRUE(std::holds_alternative<std::monostate>(
        asExpr("dxf_cross(file=\"" + path.generic_string() + "\", layer=\"par\")", ev)));
    EXPECT_NE(last.find("Can't find cross"), std::string::npos) << last;

    last.clear();
    EXPECT_TRUE(std::holds_alternative<std::monostate>(
        asExpr("dxf_cross(file=\"" + path.generic_string() + "\", layer=\"dims\")", ev)));
    EXPECT_NE(last.find("Can't find cross"), std::string::npos) << last;
    std::filesystem::remove(path);
}

// --- SVG id=/class= filters ------------------------------------------------
//
// Upstream OpenSCAD filters an SVG import by `id`. `class` is this port's own
// addition (`supported_feature("svg-class")`), because a class is how a
// drawing marks "every cut line" without naming each one.
//
// `layer` selects an Inkscape layer, as upstream does; see SvgParity below.

namespace {

constexpr const char* kFilterSvg = R"SVG(<svg xmlns="http://www.w3.org/2000/svg"
     width="100" height="100" viewBox="0 0 100 100">
  <g id="grp" transform="translate(10,0)">
    <rect id="inner" class="cut outline" x="0" y="0" width="20" height="10"/>
  </g>
  <rect id="other" class="outline" x="50" y="50" width="10" height="10"/>
</svg>)SVG";

size_t contourCount(const std::string& file, const std::string& args, Evaluator& ev) {
    Value v = asExpr("import(\"" + file + "\"" + args + ")", ev);
    return std::get<ListPtr>(v)->items.size();
}

} // namespace

TEST(SvgFilter, NoFilterImportsEverything) {
    const auto path = tempPath("filter_all.svg");
    writeFile(path, kFilterSvg);
    Evaluator ev;
    EXPECT_EQ(contourCount(path.generic_string(), "", ev), 2u);
    std::filesystem::remove(path);
}

TEST(SvgFilter, IdSelectsOneElement) {
    const auto path = tempPath("filter_id.svg");
    writeFile(path, kFilterSvg);
    Evaluator ev;
    EXPECT_EQ(contourCount(path.generic_string(), ", id=\"inner\"", ev), 1u);
    EXPECT_EQ(contourCount(path.generic_string(), ", id=\"other\"", ev), 1u);
    std::filesystem::remove(path);
}

TEST(SvgFilter, IdOnAGroupTakesItsContents) {
    const auto path = tempPath("filter_grp.svg");
    writeFile(path, kFilterSvg);
    Evaluator ev;
    EXPECT_EQ(contourCount(path.generic_string(), ", id=\"grp\"", ev), 1u);
    std::filesystem::remove(path);
}

TEST(SvgFilter, ClassIsListMembershipNotEquality) {
    // class="cut outline" is matched by either name on its own.
    const auto path = tempPath("filter_cls.svg");
    writeFile(path, kFilterSvg);
    Evaluator ev;
    EXPECT_EQ(contourCount(path.generic_string(), ", class=\"cut\"", ev), 1u);
    EXPECT_EQ(contourCount(path.generic_string(), ", class=\"outline\"", ev), 2u);
    // A prefix of a real class must not match.
    EXPECT_EQ(contourCount(path.generic_string(), ", class=\"out\"", ev), 0u);
    std::filesystem::remove(path);
}

TEST(SvgFilter, AnEnclosingTransformStillApplies) {
    // The selected rect sits inside translate(10,0). Picking it must not
    // move it back to the origin -- it has to land where it does in the
    // drawing, or a filtered import cannot be composed with an unfiltered one.
    const auto path = tempPath("filter_xf.svg");
    writeFile(path, kFilterSvg);
    Evaluator ev;
    Value v = asExpr("import(\"" + path.generic_string() + "\", id=\"inner\")", ev);
    const auto& region = std::get<ListPtr>(v)->items;
    ASSERT_EQ(region.size(), 1u);
    const auto& pts = std::get<ListPtr>(region[0])->items;
    ASSERT_EQ(pts.size(), 4u);
    double minx = 1e9;
    for (const auto& p : pts) minx = std::min(minx, std::get<double>(std::get<ListPtr>(p)->items[0]));
    // x = 10 on a unitless 100-unit page, which OpenSCAD reads at 72 dpi
    // (to within the fill's 1e-8 grid).
    EXPECT_NEAR(minx, 10.0 * 25.4 / 72.0, 1e-7);
    std::filesystem::remove(path);
}

TEST(SvgFilter, AMissImportsNothingAndWarns) {
    // Emphatically not a fall-back to the whole drawing: a cut layer that
    // quietly became every layer is the worst answer available.
    const auto path = tempPath("filter_miss.svg");
    writeFile(path, kFilterSvg);
    std::string last;
    Evaluator ev([&](const std::string& m) { last = m; });
    EXPECT_EQ(contourCount(path.generic_string(), ", id=\"nosuch\"", ev), 0u);
    EXPECT_NE(last.find("did not match anything"), std::string::npos) << last;
    std::filesystem::remove(path);
}

TEST(SvgFilter, IdAndClassTogetherMatchEither) {
    const auto path = tempPath("filter_both.svg");
    writeFile(path, kFilterSvg);
    Evaluator ev;
    EXPECT_EQ(contourCount(path.generic_string(), ", id=\"other\", class=\"cut\"", ev), 2u);
    std::filesystem::remove(path);
}

namespace oscadeval {
namespace {

// SVG placement as OpenSCAD 2026.02.01 does it: page units to mm, the
// viewBox under preserveAspectRatio, Y flipped about the page height (or
// about the drawing's centre with center=true). Bounds read off the
// reference's own export for each document, with and without center.
struct SvgPlacementCase {
    const char* attrs;
    const char* body;
    bool center;
    double bounds[4];
};

TEST(SvgPlacement, MatchesTheReference) {
    const SvgPlacementCase cases[] = {
        {R"(width="100" height="100" viewBox="0 0 100 100")",
         R"(<rect x="0" y="0" width="10" height="10"/><rect x="20" y="50" width="30" height="5"/>)", false,
         {0.0, 15.875, 17.639, 35.278}},
        {R"(width="100" height="100" viewBox="0 0 100 100")",
         R"(<rect x="0" y="0" width="10" height="10"/><rect x="20" y="50" width="30" height="5"/>)", true,
         {-8.819, -9.701, 8.819, 9.701}},
        {R"(width="50mm" height="20mm" viewBox="10 5 200 40")",
         R"(<rect x="10" y="5" width="20" height="10"/><rect x="92" y="17" width="16" height="16"/>)", false,
         {0.0, 5.5, 24.5, 12.5}},
        {R"(width="4in" height="2in" viewBox="0 0 100 100" preserveAspectRatio="xMaxYMin meet")",
         R"(<rect x="0" y="0" width="100" height="100"/>)", false, {50.8, 0.0, 101.6, 50.8}},
        {R"(width="4in" height="2in" viewBox="0 0 100 100" preserveAspectRatio="xMaxYMin meet")",
         R"(<rect x="0" y="0" width="100" height="100"/>)", true, {-25.4, -25.4, 25.4, 25.4}},
        {R"(width="300px" height="150px")", R"(<rect x="10" y="10" width="30" height="20"/>)", false,
         {10.0, 9.688, 40.0, 29.688}},
        {R"(viewBox="0 0 200 100")", R"(<rect x="0" y="0" width="200" height="100"/>)", false,
         {0.0, 0.0, 70.556, 35.278}},
        {R"(viewBox="0 0 200 100")", R"(<rect x="0" y="0" width="200" height="100"/>)", true,
         {-35.278, -17.639, 35.278, 17.639}},
    };
    int n = 0;
    for (const SvgPlacementCase& c : cases) {
        const auto path = std::filesystem::temp_directory_path() / ("svg_place_" + std::to_string(n++) + ".svg");
        std::ofstream(path) << "<svg xmlns=\"http://www.w3.org/2000/svg\" " << c.attrs << ">" << c.body << "</svg>";
        const std::vector<Contour2d> contours = loadSvgContours(path.string(), {}, nullptr, 72.0, c.center);
        double lo[2] = {1e18, 1e18}, hi[2] = {-1e18, -1e18};
        for (const Contour2d& ct : contours)
            for (const auto& p : ct)
                for (int a = 0; a < 2; ++a) {
                    lo[a] = std::min(lo[a], p[a]);
                    hi[a] = std::max(hi[a], p[a]);
                }
        EXPECT_NEAR(lo[0], c.bounds[0], 1e-3) << c.attrs << " center=" << c.center;
        EXPECT_NEAR(lo[1], c.bounds[1], 1e-3) << c.attrs << " center=" << c.center;
        EXPECT_NEAR(hi[0], c.bounds[2], 1e-3) << c.attrs << " center=" << c.center;
        EXPECT_NEAR(hi[1], c.bounds[3], 1e-3) << c.attrs << " center=" << c.center;
        std::filesystem::remove(path);
    }
}

TEST(SvgPlacement, DpiScalesUnitlessLengths) {
    const auto path = std::filesystem::temp_directory_path() / "svg_place_dpi.svg";
    std::ofstream(path) << R"(<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 100 100">)"
                        << R"(<rect x="0" y="0" width="10" height="10"/><rect x="20" y="50" width="30" height="5"/></svg>)";
    const std::vector<Contour2d> contours = loadSvgContours(path.string(), {}, nullptr, 96.0, false);
    double maxy = -1e18;
    for (const Contour2d& ct : contours)
        for (const auto& p : ct) maxy = std::max(maxy, p[1]);
    EXPECT_NEAR(maxy, 26.458, 1e-3);
    std::filesystem::remove(path);
}

} // namespace
} // namespace oscadeval

// -- parity with OpenSCAD 2026.02.01 -----------------------------------------
//
// Each case is a 100 mm page of 100 user units, imported with `args`; the
// area, vertex count and bounds are the reference binary's own (its SVG
// export of the same import). Every one of these was wrong before the
// libsvg port.
namespace {

struct SvgParityCase {
    const char* name;
    const char* body;
    const char* args;
    double area;
    size_t vertices;
    double bounds[4];
};

constexpr const char* kParityHeader =
    R"svg(<svg xmlns="http://www.w3.org/2000/svg" xmlns:svg="http://www.w3.org/2000/svg" )svg"
    R"svg(xmlns:xlink="http://www.w3.org/1999/xlink" xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape" )svg"
    R"svg(width="100mm" height="100mm" viewBox="0 0 100 100">)svg";

constexpr const char* kLayers =
    R"svg(<g inkscape:groupmode="layer" inkscape:label="outer"><rect id="a" x="0" y="0" width="10" height="10"/>)svg"
    R"svg(<g inkscape:groupmode="layer" inkscape:label="inner" style="display:none">)svg"
    R"svg(<rect id="b" x="20" y="0" width="10" height="5"/></g></g>)svg";

std::string parityImport(const SvgParityCase& c, std::vector<std::string>* log = nullptr) {
    const auto path = tempPath(std::string("parity_") + c.name + ".svg");
    writeFile(path, std::string(kParityHeader) + c.body + "</svg>");
    return "import(\"" + path.generic_string() + "\"" + c.args + ");";
}

} // namespace

TEST(SvgParity, MatchesTheReference) {
    const SvgParityCase cases[] = {
        // Separate elements union; one path's overlapping subpaths cancel.
        {"union", R"svg(<rect x="10" y="10" width="20" height="20"/><rect x="20" y="20" width="20" height="20"/>)svg", "",
         700, 8, {10, 60, 40, 90}},
        {"xor", R"svg(<path d="M10 10 h20 v20 h-20 z M 20 20 h20 v20 h-20 z"/>)svg", "", 600, 12, {10, 60, 40, 90}},
        // A child's transform applies before its parent's.
        {"nested",
         R"svg(<g transform="translate(10,5) scale(2,1)"><g transform="rotate(30, 5, 5)"><rect x="0" y="0" width="10" height="5"/></g></g>)svg",
         "", 100, 4, {11.3397, 87.5, 33.6603, 96.8301}},
        {"display",
         R"svg(<g style="display:none"><rect x="0" y="0" width="10" height="10"/></g><rect display="none" x="40" y="0" width="10" height="10"/>)svg"
         R"svg(<rect style="display : none" x="60" y="0" width="10" height="10"/><rect x="80" y="0" width="10" height="10"/>)svg",
         "", 100, 4, {80, 90, 90, 100}},
        // Only a <defs> shape can be used; x/y apply after its own transform.
        {"use",
         R"svg(<defs><g id="gg" transform="translate(5,0)"><rect x="0" y="0" width="10" height="10"/></g></defs>)svg"
         R"svg(<use xlink:href="#gg" x="50" y="20"/><rect id="r" x="0" y="0" width="5" height="5"/><use xlink:href="#r" x="20" y="20"/>)svg",
         "", 125, 8, {0, 70, 65, 100}},
        {"layer", kLayers, R"svg(, layer="outer")svg", 100, 4, {0, 90, 10, 100}},
        // An id selects even under a hidden layer; the layer restricts it.
        {"layer_id", kLayers, R"svg(, id="b", layer="outer")svg", 50, 4, {20, 95, 30, 100}},
        // Lines are strokes: width, caps and joins.
        {"stroke",
         R"svg(<line x1="10" y1="10" x2="30" y2="10" style="stroke-width:4;stroke-linecap:square"/>)svg"
         R"svg(<polyline points="50,10 70,10 70,30" stroke-width="2" stroke-linejoin="round"/>)svg",
         "", 175.783, 23, {8, 70, 71, 92}},
        // An open subpath is stroked -- unless the path closed one before it.
        {"open_path", R"svg(<path d="M10 10 L 30 10 L 30 30"/><path d="M50 10 L 70 10 L 70 30 Z M 80 10 L 90 10"/>)svg", "",
         240, 9, {10, 70, 70, 90.5}},
        // Curves split by $fn/$fa/$fs: circles at least 40, beziers 20.
        {"circle", R"svg(<circle cx="50" cy="50" r="10"/>)svg", "", 312.871, 40, {40, 40, 60, 60}},
        {"circle_fn", R"svg(<circle cx="50" cy="50" r="10"/>)svg", ", $fn=50", 313.334, 50, {40.0197, 40, 59.9803, 60}},
        {"cubic", R"svg(<path d="M 10 10 C 20 0 40 0 50 10 Z"/>)svg", "", 209.375, 21, {10, 90, 50, 97.5}},
        {"cubic_fn", R"svg(<path d="M 10 10 C 20 0 40 0 50 10 Z"/>)svg", ", $fn=50", 209.9, 51, {10, 90, 50, 97.5}},
        {"arc_fa", R"svg(<path d="M 10 50 A 20 20 0 0 1 50 50 Z"/>)svg", ", $fa=1, $fs=0.1", 628.287, 181, {10, 50, 50, 70}},
        {"rounded_rect", R"svg(<rect x="10" y="10" width="40" height="20" rx="5"/>)svg", "", 778.141, 40, {10, 70, 50, 90}},
        // <svg:rect> is not a rect; <symbol> and <a> are see-through, and
        // <a>'s transform is ignored.
        {"tags",
         R"svg(<rect x="0" y="0" width="10" height="10"/><svg:rect x="20" y="0" width="10" height="10"/>)svg"
         R"svg(<symbol><rect x="40" y="0" width="10" height="10"/></symbol><a transform="translate(50,50)"><rect x="60" y="0" width="10" height="10"/></a>)svg",
         "", 300, 12, {0, 90, 70, 100}},
    };
    for (const SvgParityCase& c : cases) {
        Evaluated e = evalSrc(parityImport(c));
        ASSERT_EQ(e.bodies.size(), 1u) << c.name;
        ASSERT_TRUE(e.bodies[0].section.has_value()) << c.name;
        const manifold::CrossSection& s = *e.bodies[0].section;
        EXPECT_NEAR(s.Area(), c.area, 1e-5 * c.area) << c.name;  // the reference printed 6 digits
        size_t vertices = 0;
        for (const auto& poly : s.ToPolygons()) vertices += poly.size();
        EXPECT_EQ(vertices, c.vertices) << c.name;
        const manifold::Rect b = s.Bounds();
        EXPECT_NEAR(b.min.x, c.bounds[0], 1e-3) << c.name;
        EXPECT_NEAR(b.min.y, c.bounds[1], 1e-3) << c.name;
        EXPECT_NEAR(b.max.x, c.bounds[2], 1e-3) << c.name;
        EXPECT_NEAR(b.max.y, c.bounds[3], 1e-3) << c.name;
        std::filesystem::remove(tempPath(std::string("parity_") + c.name + ".svg"));
    }
}

TEST(SvgParity, IdOutsideTheLayerIsAMiss) {
    const SvgParityCase c{"layer_miss", kLayers, R"svg(, id="a", layer="inner")svg", 0, 0, {}};
    std::vector<std::string> log;
    Evaluated e = evalSrc(parityImport(c), [&](const std::string& m) { log.push_back(m); });
    std::filesystem::remove(tempPath("parity_layer_miss.svg"));
    EXPECT_TRUE(e.bodies.empty() || !e.bodies[0].section || e.bodies[0].section->IsEmpty());
    ASSERT_EQ(log.size(), 1u);
    EXPECT_NE(log[0].find(R"svg(import() filter id = "a", layer = "inner" did not match anything)svg"), std::string::npos)
        << log[0];
}

// Below 0.001 is refused with a warning and read at 72 dpi. (OpenSCAD's own
// message says "giving" and prints its default as "undef"; ours does not.)
TEST(SvgParity, TinyDpiWarnsAndUsesTheDefault) {
    const SvgParityCase c{"dpi", R"svg(<rect x="0" y="0" width="10" height="10"/>)svg", ", dpi=0.0001", 0, 0, {}};
    std::vector<std::string> log;
    Evaluated e = evalSrc(parityImport(c), [&](const std::string& m) { log.push_back(m); });
    std::filesystem::remove(tempPath("parity_dpi.svg"));
    ASSERT_EQ(log.size(), 1u);
    EXPECT_NE(log[0].find("Invalid dpi value given, using default of 72 dpi"), std::string::npos) << log[0];
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].section->Area(), 100, 1e-6);
}
