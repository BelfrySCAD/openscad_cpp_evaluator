#include "openscad_cpp_evaluator/export.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <locale>
#include <gtest/gtest.h>
#include <sstream>
#include <stb_image.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::filesystem::path tempPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("oscad_eval_test_" + name);
}

std::vector<ColoredBody> evalToBodies(const std::string& code) {
    auto ast = parseSrc(code);
    auto scope = oscad::buildScopes(ast);
    Evaluator ev;
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    auto tree = ev.resolveTree(ast, ctx);
    return ev.generateTree(tree);
}

// Minimal binary-STL reader: returns (triangleCount, allVertexXCoords).
struct StlSummary {
    uint32_t triangleCount;
    float minX, maxX;
};

StlSummary readStl(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    char header[80];
    in.read(header, 80);
    uint32_t count = 0;
    in.read(reinterpret_cast<char*>(&count), sizeof(count));
    float minX = 1e30f, maxX = -1e30f;
    for (uint32_t t = 0; t < count; ++t) {
        float floats[12];
        uint16_t attr;
        in.read(reinterpret_cast<char*>(floats), sizeof(floats));
        in.read(reinterpret_cast<char*>(&attr), sizeof(attr));
        for (int v = 1; v <= 3; ++v) { // skip the normal (floats[0..2])
            float x = floats[v * 3];
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
        }
    }
    return {count, minX, maxX};
}

} // namespace

TEST(ExportStl, WritesCorrectTriangleCountAndHeader) {
    std::vector<ColoredBody> bodies = evalToBodies("cube(2);");
    const std::string path = tempPath("cube.stl").string();
    writeStl(path, bodies);

    StlSummary s = readStl(path);
    EXPECT_EQ(s.triangleCount, 12u);
    EXPECT_FLOAT_EQ(s.minX, 0.0f);
    EXPECT_FLOAT_EQ(s.maxX, 2.0f);
    std::remove(path.c_str());
}

TEST(ExportStl, TranslatedCubeBoundsMatch) {
    std::vector<ColoredBody> bodies = evalToBodies("translate([1,0,0]) cube(2);");
    const std::string path = tempPath("translate.stl").string();
    writeStl(path, bodies);

    StlSummary s = readStl(path);
    EXPECT_FLOAT_EQ(s.minX, 1.0f);
    EXPECT_FLOAT_EQ(s.maxX, 3.0f);
    std::remove(path.c_str());
}

TEST(ExportStl, NoGeometryThrows) {
    std::vector<ColoredBody> empty;
    EXPECT_THROW(writeStl(tempPath("empty.stl").string(), empty), std::runtime_error);
}

// -- toRenderableBodies (Phase 6) ------------------------------------------

TEST(ToRenderableBodies, ThinExtrudesABareTopLevel2dShapeSoItCanExportAsStl) {
    // A bare `circle();` (no `body`, only `section`) would make writeStl
    // throw "No geometry to export" -- toRenderableBodies() converts it
    // into a 1-unit-tall Manifold first, mirroring the reference
    // CLI's own to_renderable_bodies() call right before export. Without
    // this step, no top-level 2D-only script could ever export a mesh.
    std::vector<ColoredBody> bodies = evalToBodies("circle(2, $fn=32);");
    ASSERT_EQ(bodies.size(), 1u);
    ASSERT_FALSE(bodies[0].body.has_value());
    ASSERT_TRUE(bodies[0].section.has_value());

    std::vector<ColoredBody> renderable = toRenderableBodies(bodies);
    ASSERT_EQ(renderable.size(), 1u);
    ASSERT_TRUE(renderable[0].body.has_value());
    EXPECT_TRUE(renderable[0].flatPreview);
    manifold::Box bbox = renderable[0].body->BoundingBox();
    // 1 unit, matching what the reference extrudes a 2D preview to --
    // measured against OpenSCAD at three camera tilts.
    EXPECT_NEAR(bbox.max.z - bbox.min.z, 1.0, 1e-9);

    const std::string path = tempPath("flat_preview.stl").string();
    writeStl(path, renderable);
    StlSummary s = readStl(path);
    EXPECT_GT(s.triangleCount, 0u);
    std::remove(path.c_str());
}

TEST(ToRenderableBodies, A3dBodyPassesThroughUnchanged) {
    std::vector<ColoredBody> bodies = evalToBodies("cube(2);");
    std::vector<ColoredBody> renderable = toRenderableBodies(bodies);
    ASSERT_EQ(renderable.size(), 1u);
    ASSERT_TRUE(renderable[0].body.has_value());
    EXPECT_FALSE(renderable[0].flatPreview);
    EXPECT_NEAR(renderable[0].body->Volume(), 8.0, 1e-9);
}

// -- AMF -------------------------------------------------------------------

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

size_t countOf(const std::string& hay, const std::string& needle) {
    size_t n = 0, at = 0;
    while ((at = hay.find(needle, at)) != std::string::npos) { ++n; at += needle.size(); }
    return n;
}

} // namespace

TEST(ExportAmf, IsAnOfferedFormat) {
    const std::vector<std::string>& exts = exportExtensions();
    EXPECT_NE(std::find(exts.begin(), exts.end(), ".amf"), exts.end());
}

TEST(ExportAmf, WritesOneObjectAndOneMaterialPerColour) {
    // Two separated cubes, different colours: two objects, two materials.
    std::vector<ColoredBody> bodies =
        evalToBodies("color(\"red\") cube(2); translate([10,0,0]) color(\"blue\") cube(2);");
    const std::string path = tempPath("two.amf").string();
    exportModel(path, bodies, ExportOptions{});

    const std::string xml = readFile(path);
    EXPECT_EQ(countOf(xml, "<object id="), 2u);
    EXPECT_EQ(countOf(xml, "<material id="), 2u);
    // 12 triangles per cube, and every one lands in some volume.
    EXPECT_EQ(countOf(xml, "<triangle>"), 24u);
    EXPECT_NE(xml.find("<amf unit=\"millimeter\""), std::string::npos);
    std::remove(path.c_str());
}

TEST(ExportAmf, TwoObjectsSharingAColourShareOneMaterial) {
    // splitComponents, because two objects is the premise: same-coloured
    // disjoint pieces are ONE object by default (see the DisjointPieces
    // tests below). What is under test here is that the two of them share a
    // single material entry rather than each declaring their own.
    std::vector<ColoredBody> bodies =
        evalToBodies("color(\"red\") cube(2); translate([10,0,0]) color(\"red\") cube(2);");
    const std::string path = tempPath("shared.amf").string();
    ExportOptions opts;
    opts.splitComponents = true;
    exportModel(path, bodies, opts);

    const std::string xml = readFile(path);
    EXPECT_EQ(countOf(xml, "<object id="), 2u);
    EXPECT_EQ(countOf(xml, "<material id="), 1u);
    std::remove(path.c_str());
}

// Issue #319: a model in several disconnected pieces was written as one
// object PER PIECE, so a slicer listed them individually -- hundreds of
// entries for a multiboard tile, and Prusa Slicer objecting. OpenSCAD writes
// one object, and so does this by default now; the split is still available
// for callers that want the pieces apart.
TEST(ExportSplit, DisjointPiecesAreOneObjectByDefault) {
    std::vector<ColoredBody> bodies =
        evalToBodies("cube(10); translate([20,0,0]) cube(10); translate([40,0,0]) cube(10);");
    const std::string path = tempPath("joined.amf").string();
    exportModel(path, bodies, ExportOptions{});

    const std::string xml = readFile(path);
    EXPECT_EQ(countOf(xml, "<object id="), 1u);
    EXPECT_EQ(countOf(xml, "<volume"), 1u);
    std::remove(path.c_str());
}

TEST(ExportSplit, DisjointPiecesSplitWhenAsked) {
    std::vector<ColoredBody> bodies =
        evalToBodies("cube(10); translate([20,0,0]) cube(10); translate([40,0,0]) cube(10);");
    const std::string path = tempPath("split.amf").string();
    ExportOptions opts;
    opts.splitComponents = true;
    exportModel(path, bodies, opts);

    const std::string xml = readFile(path);
    EXPECT_EQ(countOf(xml, "<object id="), 3u);
    std::remove(path.c_str());
}

// Rule 2 is not what became optional: differently-coloured solids must stay
// separate objects either way, or a multi-material export loses its colours.
TEST(ExportSplit, ColoursStayApartWithTheSplitOff) {
    std::vector<ColoredBody> bodies =
        evalToBodies("color(\"red\") cube(10); translate([20,0,0]) color(\"blue\") cube(10);");
    const std::string path = tempPath("colours.amf").string();
    exportModel(path, bodies, ExportOptions{});

    const std::string xml = readFile(path);
    EXPECT_EQ(countOf(xml, "<object id="), 2u);
    EXPECT_EQ(countOf(xml, "<material id="), 2u);
    std::remove(path.c_str());
}

TEST(ExportAmf, MaterialAndObjectIdsStartAtOne) {
    // id 0 is reserved by the spec; a reader may drop anything using it.
    std::vector<ColoredBody> bodies = evalToBodies("cube(2);");
    const std::string path = tempPath("ids.amf").string();
    exportModel(path, bodies, ExportOptions{});

    const std::string xml = readFile(path);
    EXPECT_NE(xml.find("<material id=\"1\">"), std::string::npos);
    EXPECT_NE(xml.find("<object id=\"1\">"), std::string::npos);
    EXPECT_EQ(xml.find("id=\"0\""), std::string::npos);
    std::remove(path.c_str());
}

TEST(ExportAmf, NoGeometryThrows) {
    std::vector<ColoredBody> empty;
    EXPECT_THROW(exportModel(tempPath("empty.amf").string(), empty, ExportOptions{}), std::runtime_error);
}

TEST(ExportAmf, AMultiColouredObjectBecomesOneVolumePerColour) {
    // A union welds these into ONE solid whose surface carries two colours.
    // AMF has no per-face colour that slicers read reliably, so the object
    // is written as one <volume> per colour -- which is exactly what a
    // multimaterial slicer assigns to separate tools.
    std::vector<ColoredBody> bodies = evalToBodies(
        "union() { color(\"red\") cube(20); color(\"blue\") translate([10,5,5]) cube(20); }");
    const std::string path = tempPath("multi.amf").string();
    exportModel(path, bodies, ExportOptions{});

    const std::string xml = readFile(path);
    EXPECT_EQ(countOf(xml, "<object id="), 1u);
    EXPECT_EQ(countOf(xml, "<volume materialid="), 2u);
    EXPECT_EQ(countOf(xml, "<material id="), 2u);
    std::remove(path.c_str());
}

// -- SVG (2D) --------------------------------------------------------------

namespace {

std::string readText(const std::string& path) {
    std::ifstream in(path);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// Write `code` as SVG and return the file's text, deleting the file.
std::string svgOf(const std::string& code, const std::string& name, const ExportSvgOptions& opts = {}) {
    const std::string path = tempPath(name).string();
    writeSvg(path, evalToBodies(code), opts);
    const std::string text = readText(path);
    std::remove(path.c_str());
    return text;
}

std::string firstLineContaining(const std::string& text, const std::string& needle) {
    const size_t at = text.find(needle);
    if (at == std::string::npos) return "";
    const size_t start = text.rfind('\n', at);
    const size_t end = text.find('\n', at);
    return text.substr(start == std::string::npos ? 0 : start + 1,
                        (end == std::string::npos ? text.size() : end) - (start == std::string::npos ? 0 : start + 1));
}

} // namespace

// The page rule is NOT a fixed 1mm margin -- it is the bounding box padded
// by half the stroke width, then rounded outward to whole millimetres. The
// three cases below are the ones that tell those two rules apart, and each
// header is what real OpenSCAD 2026.02.01 wrote for the same model.
TEST(ExportSvg, PageIsStrokePaddedBoundsRoundedOutward) {
    const std::string text = svgOf("square([70.4, 25.3]);", "page_default.svg");
    EXPECT_EQ(firstLineContaining(text, "<svg "),
              "<svg width=\"72mm\" height=\"27mm\" viewBox=\"-1 -26 72 27\" "
              "xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">");
}

TEST(ExportSvg, NoStrokeMeansNoStrokePadding) {
    ExportSvgOptions opts;
    opts.stroke = false;
    const std::string text = svgOf("square([70.4, 25.3]);", "page_nostroke.svg", opts);
    EXPECT_EQ(firstLineContaining(text, "<svg "),
              "<svg width=\"71mm\" height=\"26mm\" viewBox=\"0 -26 71 26\" "
              "xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">");
    EXPECT_NE(text.find("stroke=\"none\""), std::string::npos);
}

TEST(ExportSvg, WiderStrokeGrowsThePage) {
    ExportSvgOptions opts;
    opts.strokeWidth = 4.0;
    const std::string text = svgOf("square([70.4, 25.3]);", "page_wide.svg", opts);
    EXPECT_EQ(firstLineContaining(text, "<svg "),
              "<svg width=\"75mm\" height=\"30mm\" viewBox=\"-2 -28 75 30\" "
              "xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">");
}

TEST(ExportSvg, NegatesY) {
    // Model Y 10..15 must come out as SVG y -15..-10: SVG's Y grows
    // downward. Getting this backwards mirrors the drawing, and on a
    // symmetric model it is invisible -- hence an asymmetric one.
    const std::string text = svgOf("translate([0,10]) square([5,5]);", "negate_y.svg");
    EXPECT_NE(text.find(",-15"), std::string::npos);
    EXPECT_NE(text.find(",-10"), std::string::npos);
    EXPECT_EQ(text.find(",15"), std::string::npos);
}

TEST(ExportSvg, HoleIsAnotherSubpathOfTheSamePath) {
    const std::string text =
        svgOf("difference() { square([40,25]); translate([20,12]) circle(5, $fn=16); }", "hole.svg");
    EXPECT_EQ(countOf(text, "<path "), 1u);  // one body, one <path>
    EXPECT_EQ(countOf(text, "M "), 2u);      // outline + hole
    EXPECT_EQ(countOf(text, " z"), 2u);
    // fill="none" is why the hole needs no winding rule to show through.
    EXPECT_NE(text.find("fill=\"none\""), std::string::npos);
}

TEST(ExportSvg, DisjointShapesShareOneBodyAndOnePath) {
    const std::string text = svgOf("square(10); translate([20,0]) square(10);", "disjoint.svg");
    EXPECT_EQ(countOf(text, "M "), 2u);
    // 0..30 in x, padded by 0.175 and rounded out: -1 .. 31.
    EXPECT_NE(text.find("<svg width=\"32mm\" height=\"12mm\" viewBox=\"-1 -11 32 12\""), std::string::npos);
}

TEST(ExportSvg, StrokeAndFillOptionsReachTheFile) {
    ExportSvgOptions opts;
    opts.fill = true;
    opts.fillColor = "#ffcc00";
    opts.strokeColor = "red";
    opts.strokeWidth = 0.5;
    const std::string text = svgOf("square(10);", "styled.svg", opts);
    EXPECT_NE(text.find("stroke=\"red\""), std::string::npos);
    EXPECT_NE(text.find("fill=\"#ffcc00\""), std::string::npos);
    EXPECT_NE(text.find("stroke-width=\"0.5\""), std::string::npos);
}

TEST(ExportSvg, SectionTransformIsApplied) {
    // A transform a CrossSection cannot hold -- here a rotation out of the
    // XY plane -- lives in ColoredBody::sectionXform instead. Ignoring it
    // drew the unrotated shape. Tilting a 10mm square 45 degrees about X
    // foreshortens it to 10*cos(45) = 7.07107 on the page.
    const std::string text = svgOf("rotate([45,0,0]) square([10,10]);", "xform.svg");
    EXPECT_NE(text.find("-7.07107"), std::string::npos);
    EXPECT_NE(text.find("<svg width=\"12mm\" height=\"9mm\" viewBox=\"-1 -8 12 9\""), std::string::npos);
}

TEST(ExportSvg, InPlaneTranslationLandsWhereTheModelIs) {
    const std::string text = svgOf("translate([3,4]) square([5,5]);", "moved.svg");
    EXPECT_NE(text.find("M 3,-4"), std::string::npos);
    EXPECT_NE(text.find("L 8,-9"), std::string::npos);
}

TEST(ExportSvg, RefusesA3dModel) {
    std::vector<ColoredBody> bodies = evalToBodies("cube(10);");
    EXPECT_THROW(writeSvg(tempPath("solid.svg").string(), bodies), std::runtime_error);
}

TEST(ExportSvg, RefusesMixed2dAnd3d) {
    std::vector<ColoredBody> bodies = evalToBodies("cube(10); translate([20,0]) square(5);");
    EXPECT_THROW(writeSvg(tempPath("mixed.svg").string(), bodies), std::runtime_error);
}

TEST(ExportSvg, WorksOnRenderableBodiesToo) {
    // The Python binding runs toRenderableBodies() once and hands the SAME
    // list to the renderer and to export, so a 2D script reaches writeSvg
    // as a 1-unit slab that still carries its section. The contours are
    // what gets written -- the slab's own outline would be the same shape
    // but is not the point; the section is the geometry.
    std::vector<ColoredBody> renderable = toRenderableBodies(evalToBodies("square([70.4, 25.3]);"));
    ASSERT_TRUE(renderable[0].flatPreview);
    ASSERT_TRUE(renderable[0].section.has_value());
    const std::string path = tempPath("preview.svg").string();
    writeSvg(path, renderable);
    const std::string text = readText(path);
    std::remove(path.c_str());
    EXPECT_NE(text.find("<svg width=\"72mm\" height=\"27mm\" viewBox=\"-1 -26 72 27\""), std::string::npos);
    EXPECT_EQ(countOf(text, "M "), 1u);
}

TEST(ExportSvg, NoGeometryThrows) {
    std::vector<ColoredBody> empty;
    EXPECT_THROW(writeSvg(tempPath("empty.svg").string(), empty), std::runtime_error);
}

TEST(ExportSvg, ReachableThroughExportModelAndListedAsAnExtension) {
    const std::vector<std::string>& exts = exportExtensions();
    EXPECT_NE(std::find(exts.begin(), exts.end(), ".svg"), exts.end());

    const std::string path = tempPath("via_export_model.svg").string();
    const std::vector<std::string> warnings = exportModel(path, evalToBodies("circle(5, $fn=8);"), ExportOptions{});
    EXPECT_TRUE(warnings.empty());  // none of the mesh checks apply to contours
    const std::string text = readText(path);
    EXPECT_NE(text.find("<svg "), std::string::npos);
    EXPECT_EQ(countOf(text, "M "), 1u);
    std::remove(path.c_str());
}

// Byte-for-byte against real OpenSCAD 2026.02.01's own `-o out.svg` for the
// same script -- header, vertex order, the 6-vertex line wrap and the "-0"
// that a negated zero prints as. Contour ORDER (outline before hole) and
// winding come from Manifold rather than CGAL, so agreeing here is worth
// pinning: it is the part that could drift without anything else noticing.
TEST(ExportSvg, MatchesRealOpenscadByteForByte) {
    const std::string expected =
        R"SVG(<?xml version="1.0" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="42mm" height="27mm" viewBox="-1 -26 42 27" xmlns="http://www.w3.org/2000/svg" version="1.1">
<title>OpenSCAD Model</title>
<path d="
M 40,-25 L 0,-25 L 0,-0 L 40,-0 z
M 18.0866,-7.3806 L 16.4645,-8.46447 L 15.3806,-10.0866 L 15,-12 L 15.3806,-13.9134 L 16.4645,-15.5355
 L 18.0866,-16.6194 L 20,-17 L 21.9134,-16.6194 L 23.5355,-15.5355 L 24.6194,-13.9134 L 25,-12
 L 24.6194,-10.0866 L 23.5355,-8.46447 L 21.9134,-7.3806 L 20,-7 z
" stroke="black" fill="none" stroke-width="0.35"/>
</svg>
)SVG";
    EXPECT_EQ(svgOf("difference() { square([40,25]); translate([20,12]) circle(5, $fn=16); }", "parity.svg"),
              expected);
}

// -- DXF (2D) --------------------------------------------------------------

namespace {

std::string dxfOf(const std::string& code, const std::string& name) {
    const std::string path = tempPath(name).string();
    writeDxf(path, evalToBodies(code));
    const std::string text = readText(path);
    std::remove(path.c_str());
    return text;
}

// The value on the line after the first `code` line following `header`
// (DXF is group-code / value pairs, one per line).
std::string dxfValueAfter(const std::string& text, const std::string& header, const std::string& code) {
    size_t at = text.find("\n" + header + "\n");
    if (at == std::string::npos) return "";
    at = text.find("\n" + code + "\n", at + 1);
    if (at == std::string::npos) return "";
    const size_t start = at + code.size() + 2;
    return text.substr(start, text.find('\n', start) - start);
}

} // namespace

// Byte-for-byte against real OpenSCAD 2026.02.01's `-o out.dxf` for the same
// script: header, tables, entity layout, contour order and vertex order. The
// coordinates are chosen to need no more than 6 significant digits, the only
// place ours deliberately differs (see FullPrecisionNotSixDigits).
TEST(ExportDxf, MatchesRealOpenscadByteForByte) {
    const std::string expected =
        "999\nDXF from OpenSCAD\n  0\nSECTION\n  2\nHEADER\n  9\n$ACADVER\n  1\nAC1006\n"
        "  9\n$INSBASE\n 10\n0.0\n 20\n0.0\n 30\n0.0\n"
        "  9\n$EXTMIN\n 10\n0\n 20\n0\n  9\n$EXTMAX\n 10\n40\n 20\n25\n"
        "  9\n$LINMIN\n 10\n0\n 20\n0\n  9\n$LINMAX\n 10\n40\n 20\n25\n"
        "  0\nENDSEC\n  0\nSECTION\n  2\nTABLES\n  0\nTABLE\n  2\nLTYPE\n 70\n1\n"
        "  0\nLTYPE\n  2\nCONTINUOUS\n 70\n64\n  3\nSolid line\n 72\n65\n 73\n0\n 40\n0.000000\n"
        "  0\nENDTAB\n  0\nTABLE\n  2\nLAYER\n 70\n6\n  0\nLAYER\n  2\n0\n 70\n64\n 62\n7\n  6\nCONTINUOUS\n"
        "  0\nENDTAB\n  0\nTABLE\n  2\nSTYLE\n 70\n0\n  0\nENDTAB\n  0\nENDSEC\n"
        "  0\nSECTION\n  2\nBLOCKS\n  0\nENDSEC\n  0\nSECTION\n  2\nENTITIES\n"
        "  0\nLWPOLYLINE\n100\nAcDbEntity\n  8\n0\n100\nAcDbPolyline\n 90\n4\n 70\n1\n"
        " 10\n40\n 20\n25\n 10\n0\n 20\n25\n 10\n0\n 20\n0\n 10\n40\n 20\n0\n"
        "  0\nLWPOLYLINE\n100\nAcDbEntity\n  8\n0\n100\nAcDbPolyline\n 90\n4\n 70\n1\n"
        " 10\n10\n 20\n5\n 10\n10\n 20\n17.5\n 10\n30\n 20\n17.5\n 10\n30\n 20\n5\n"
        "  0\nENDSEC\n  0\nEOF\n";
    EXPECT_EQ(dxfOf("difference() { square([40,25]); translate([10,5]) square([20,12.5]); }", "parity.dxf"),
              expected);
}

// OpenSCAD streams doubles at the default 6 significant digits, so 1234.5678
// reaches the file as 1234.57 -- a 2 micron error on a cut part, from the
// file format alone. Ours writes the shortest text that reads back exactly.
TEST(ExportDxf, FullPrecisionNotSixDigits) {
    const std::string text = dxfOf("square([1234.5678, 0.1]);", "precise.dxf");
    EXPECT_EQ(dxfValueAfter(text, "$EXTMAX", " 10"), "1234.5678");
    EXPECT_EQ(dxfValueAfter(text, "$EXTMAX", " 20"), "0.1");
    EXPECT_EQ(countOf(text, "\n1234.5678\n"), 4u);  // $EXTMAX, $LINMAX and two vertices
}

// OpenSCAD starts its maxima at numeric_limits<double>::min(), the smallest
// POSITIVE double, so a model entirely below zero reports $EXTMAX of ~0.
TEST(ExportDxf, ExtentsOfAModelBelowZero) {
    const std::string text = dxfOf("translate([-50,-30]) square(10);", "negative.dxf");
    EXPECT_EQ(dxfValueAfter(text, "$EXTMIN", " 10"), "-50");
    EXPECT_EQ(dxfValueAfter(text, "$EXTMIN", " 20"), "-30");
    EXPECT_EQ(dxfValueAfter(text, "$EXTMAX", " 10"), "-40");
    EXPECT_EQ(dxfValueAfter(text, "$EXTMAX", " 20"), "-20");
}

TEST(ExportDxf, EveryContourIsAClosedPolylineHolesIncluded) {
    const std::string text =
        dxfOf("difference() { circle(10, $fn=12); circle(5, $fn=12); } translate([30,0]) square(4);", "multi.dxf");
    EXPECT_EQ(countOf(text, "LWPOLYLINE"), 3u);           // outer, hole, and the separate square
    EXPECT_EQ(countOf(text, " 90\n12\n 70\n1\n"), 2u);    // both circles: 12 vertices, closed
    EXPECT_EQ(countOf(text, " 90\n4\n 70\n1\n"), 1u);
}

namespace {
// A locale that writes 1500.5 as "1.500,5" -- what a German global locale
// would do to any number that reaches a stream without the classic locale.
struct GermanishPunct : std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
    char do_thousands_sep() const override { return '.'; }
    std::string do_grouping() const override { return "\3"; }
};
}  // namespace

TEST(ExportDxf, NumbersIgnoreTheGlobalLocale) {
    const std::locale previous = std::locale::global(std::locale(std::locale::classic(), new GermanishPunct));
    std::string text;
    try {
        text = dxfOf("translate([0.5,0]) circle(1000, $fn=1500);", "locale.dxf");
    } catch (...) {
        std::locale::global(previous);
        throw;
    }
    std::locale::global(previous);
    EXPECT_NE(text.find(" 90\n1500\n"), std::string::npos);
    EXPECT_EQ(dxfValueAfter(text, "$EXTMAX", " 10"), "1000.5");
    EXPECT_EQ(text.find(','), std::string::npos);
}

TEST(ExportDxf, RefusesA3dModel) {
    std::vector<ColoredBody> bodies = evalToBodies("cube(10);");
    EXPECT_THROW(writeDxf(tempPath("solid.dxf").string(), bodies), std::runtime_error);
}

TEST(ExportDxf, NoGeometryThrows) {
    std::vector<ColoredBody> empty;
    EXPECT_THROW(writeDxf(tempPath("empty.dxf").string(), empty), std::runtime_error);
}

TEST(ExportDxf, ReachableThroughExportModelAndListedAsAnExtension) {
    const std::vector<std::string>& exts = exportExtensions();
    EXPECT_NE(std::find(exts.begin(), exts.end(), ".dxf"), exts.end());
    const std::string path = tempPath("via_export_model.dxf").string();
    const std::vector<std::string> warnings = exportModel(path, evalToBodies("square(5);"), ExportOptions{});
    EXPECT_TRUE(warnings.empty());
    const std::string text = readText(path);
    EXPECT_EQ(countOf(text, "LWPOLYLINE"), 1u);
    EXPECT_NE(text.find("  0\nEOF\n"), std::string::npos);
    std::remove(path.c_str());
}

// -- POV-Ray (mesh2) -------------------------------------------------------

namespace {

std::string povOf(const std::string& code, const std::string& name, const ExportOptions& opts = {}) {
    const std::string path = tempPath(name).string();
    ExportOptions o = opts;
    o.format = ".pov";
    exportModel(path, evalToBodies(code), o);
    const std::string text = readText(path);
    std::remove(path.c_str());
    return text;
}

// The block following `header` up to and including its matching "}" line --
// crude, but every block this writer emits closes on a line of its own.
std::string povBlock(const std::string& text, const std::string& header, size_t from = 0) {
    const size_t at = text.find(header, from);
    if (at == std::string::npos) return "";
    const size_t end = text.find("\n  }", at);
    return text.substr(at, end == std::string::npos ? std::string::npos : end - at);
}

} // namespace

// One mesh2 per exported object -- here one per colour -- with a welded
// vertex list (a cube is 8 vertices, not 36), 12 triangles, and the
// inside_vector that makes POV-Ray treat it as a solid rather than a
// surface. OpenSCAD writes the same scene as 12 separate polygon objects.
TEST(ExportPov, OneWeldedSolidMesh2PerObject) {
    const std::string text = povOf(
        "color([1, 0.5, 0.25, 0.125]) translate([-10, 0, 0]) cube(10);\n"
        "color(\"green\") translate([10, 0, 0]) cube(10);", "two.pov");
    EXPECT_EQ(countOf(text, "mesh2 {"), 2u);
    EXPECT_EQ(countOf(text, "polygon {"), 0u);
    EXPECT_EQ(countOf(text, "vertex_vectors { 8,"), 2u);
    EXPECT_EQ(countOf(text, "face_indices { 12,"), 2u);
    EXPECT_EQ(countOf(text, "inside_vector <0, 0, 1>"), 2u);
    // rgbf with filter = 1 - alpha, as OpenSCAD writes colour.
    EXPECT_NE(text.find("color rgbf <1, 0.5, 0.25, 0.875>"), std::string::npos);
    EXPECT_NE(text.find("#version 3.7;"), std::string::npos);
    EXPECT_NE(text.find("#include \"rad_def.inc\""), std::string::npos);
}

// Per-triangle colour -- a CSG merge of two colours -- is a texture_list
// with one entry per colour and a texture index on each triangle. The mesh
// stays welded; PLY, by contrast, has to unweld to colour a triangle.
TEST(ExportPov, PerTriangleColourIsATextureIndexNotAnUnweld) {
    const std::string text =
        povOf("color(\"red\") difference() { cube(20, center=true); color(\"blue\") sphere(13, $fn=24); }",
              "tricolor.pov");
    EXPECT_EQ(countOf(text, "mesh2 {"), 1u);
    const std::string textures = povBlock(text, "texture_list {");
    EXPECT_NE(textures.find("texture_list { 2"), std::string::npos);
    EXPECT_NE(textures.find("rgbf <1, 0, 0, 0>"), std::string::npos);
    EXPECT_NE(textures.find("rgbf <0, 0, 1, 0>"), std::string::npos);
    const std::string faces = povBlock(text, "face_indices {");
    EXPECT_NE(faces.find(">, 0"), std::string::npos);
    EXPECT_NE(faces.find(">, 1"), std::string::npos);
}

// Checked by rendering in POV-Ray 3.7 against BelfrySCAD's own PNG of the
// same camera, which agreed to a pixel. Two departures from OpenSCAD make
// that true: rotate comes BEFORE translate (the eye orbits $vpt), and the
// viewport's VERTICAL fov is turned into POV-Ray's HORIZONTAL angle from
// the image's own aspect ratio.
TEST(ExportPov, CameraOrbitsVptAndKeepsTheVerticalFieldOfView) {
    ExportOptions opts;
    opts.pov.camera = ExportPovCamera{{0, 5, 5}, {55, 0, 25}, 140, 22.5};
    const std::string text = povOf("cube(10);", "camera.pov", opts);
    const size_t cam = text.find("camera {");
    ASSERT_NE(cam, std::string::npos);
    const std::string block = text.substr(cam, text.find("\n}\n", cam) - cam);
    EXPECT_NE(block.find(" location <0, 0, 140>"), std::string::npos);
    EXPECT_NE(block.find("angle degrees(2 * atan(tan(radians(22.5 / 2)) * image_width / image_height))"),
              std::string::npos);
    const size_t rotate = block.find("rotate <55, 0 + clock * 3, 25 + clock>");
    const size_t translate = block.find("translate <0, 5, 5>");
    ASSERT_NE(rotate, std::string::npos);
    ASSERT_NE(translate, std::string::npos);
    EXPECT_LT(rotate, translate);
}

TEST(ExportPov, WithoutACameraFramesTheBoundingBoxAsOpenscadDoes) {
    const std::string text = povOf("translate([-10, 0, 0]) cube([30, 10, 10]);", "nocam.pov");
    EXPECT_NE(text.find("camera { look_at <5, 5, 5> location <50, -20, 20> up <0, 0, 1>"), std::string::npos);
    // 27 lights: below, at and beyond the box on every axis.
    EXPECT_EQ(countOf(text, "light_source {"), 27u);
    EXPECT_NE(text.find("light_source { <-70, -20, -20> color rgb <0.2, 0.2, 0.2> }"), std::string::npos);
}

TEST(ExportPov, NumbersIgnoreTheGlobalLocale) {
    const std::locale previous = std::locale::global(std::locale(std::locale::classic(), new GermanishPunct));
    std::string text;
    try {
        text = povOf("translate([0.5, 0, 0]) sphere(10, $fn=64);", "locale.pov");
    } catch (...) {
        std::locale::global(previous);
        throw;
    }
    std::locale::global(previous);
    const std::string verts = povBlock(text, "vertex_vectors {");
    ASSERT_FALSE(verts.empty());
    const int count = std::stoi(verts.substr(std::string("vertex_vectors { ").size()));
    EXPECT_GT(count, 1000);                                     // so grouping would show
    EXPECT_EQ(verts.find('.', 0) < verts.find('\n') ? 1 : 0, 0); // the count has no separator
    EXPECT_EQ(text.find("0,5"), std::string::npos);              // no decimal commas anywhere
}

TEST(ExportPov, ReachableThroughExportModelAndListedAsAnExtension) {
    const std::vector<std::string>& exts = exportExtensions();
    EXPECT_NE(std::find(exts.begin(), exts.end(), ".pov"), exts.end());
    const std::string path = tempPath("via_export_model.pov").string();
    EXPECT_TRUE(exportModel(path, evalToBodies("cube(5);"), ExportOptions{}).empty());
    EXPECT_EQ(countOf(readText(path), "mesh2 {"), 1u);
    std::remove(path.c_str());
}

// -- PDF (2D) --------------------------------------------------------------

namespace {

std::string readBinary(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// The page's content stream, inflated when it was Flate-compressed.
// Inflating with stb's own decoder doubles as proof that what we wrote is a
// real zlib stream -- which is exactly what /FlateDecode promises a reader.
std::string pdfContentStream(const std::string& file) {
    const size_t at = file.find("stream\n");
    if (at == std::string::npos) return "";
    const size_t start = at + 7;
    const size_t end = file.find("\nendstream", start);
    const std::string raw = file.substr(start, end - start);
    if (file.find("/FlateDecode") == std::string::npos) return raw;
    int outLen = 0;
    char* inflated = stbi_zlib_decode_malloc(raw.data(), static_cast<int>(raw.size()), &outLen);
    if (!inflated) return "";
    std::string out(inflated, inflated + outLen);
    free(inflated);
    return out;
}

struct Pdf {
    std::string file;
    std::string content;
    std::vector<std::string> warnings;
};

Pdf pdfOf(const std::string& code, const std::string& name, const ExportPdfOptions& opts = {}) {
    const std::string path = tempPath(name).string();
    Pdf out;
    out.warnings = writePdf(path, evalToBodies(code), opts);
    out.file = readBinary(path);
    out.content = pdfContentStream(out.file);
    std::remove(path.c_str());
    return out;
}

// Every number that appears before a `m`/`l` operator, as (x, y) pairs.
std::vector<std::pair<double, double>> pathPoints(const std::string& content) {
    std::vector<std::pair<double, double>> pts;
    std::istringstream in(content);
    std::string tok;
    std::vector<std::string> toks;
    while (in >> tok) toks.push_back(tok);
    for (size_t i = 2; i < toks.size(); ++i) {
        if (toks[i] != "m" && toks[i] != "l") continue;
        try {
            pts.emplace_back(std::stod(toks[i - 2]), std::stod(toks[i - 1]));
        } catch (const std::exception&) {
            // A "0 0 m"-shaped false positive is impossible here, but a
            // non-numeric neighbour (an operator) simply is not a point.
        }
    }
    return pts;
}

} // namespace

TEST(ExportPdf, IsAValidLookingPdfWithAnA4PortraitPageByDefault) {
    const Pdf pdf = pdfOf("square([40,25]);", "a4.pdf");
    EXPECT_EQ(pdf.file.substr(0, 8), "%PDF-1.4");
    EXPECT_NE(pdf.file.find("/MediaBox [0 0 595 842]"), std::string::npos);
    EXPECT_NE(pdf.file.find("\nxref\n"), std::string::npos);
    EXPECT_NE(pdf.file.find("startxref"), std::string::npos);
    EXPECT_EQ(pdf.file.substr(pdf.file.size() - 6), "%%EOF\n");
    EXPECT_TRUE(pdf.warnings.empty());
}

TEST(ExportPdf, UsesTheBase14HelveticaSoNothingIsEmbedded) {
    // The whole reason this issue turned out to be small: a standard-14
    // font needs no font file, no subsetting and no metrics.
    const Pdf pdf = pdfOf("square([40,25]);", "font.pdf");
    EXPECT_NE(pdf.file.find("/BaseFont /Helvetica"), std::string::npos);
    EXPECT_EQ(pdf.file.find("/FontFile"), std::string::npos);
}

// The ruler's own lines are `m`/`l` too, so these two read the drawing
// with the ruler turned off -- otherwise every measurement is of the page.
namespace {
ExportPdfOptions drawingOnly() {
    ExportPdfOptions opts;
    opts.showScale = false;
    return opts;
}
} // namespace

TEST(ExportPdf, DrawsAtTrueSizeAndCentresOnThePage) {
    // 40mm x 25mm at 72/25.4 pt per mm, centred on 595x842.
    const Pdf pdf = pdfOf("square([40,25]);", "size.pdf", drawingOnly());
    const std::vector<std::pair<double, double>> pts = pathPoints(pdf.content);
    ASSERT_FALSE(pts.empty());
    double minx = pts[0].first, maxx = pts[0].first, miny = pts[0].second, maxy = pts[0].second;
    for (const auto& p : pts) {
        minx = std::min(minx, p.first);
        maxx = std::max(maxx, p.first);
        miny = std::min(miny, p.second);
        maxy = std::max(maxy, p.second);
    }
    EXPECT_NEAR(maxx - minx, 40.0 * 72.0 / 25.4, 0.01);
    EXPECT_NEAR(maxy - miny, 25.0 * 72.0 / 25.4, 0.01);
    // Exactly centred -- OpenSCAD's own is a fraction of a point off
    // because it truncates the span to an int, which is a bug, not a spec.
    EXPECT_NEAR((minx + maxx) / 2.0, 595.0 / 2.0, 0.01);
    EXPECT_NEAR((miny + maxy) / 2.0, 842.0 / 2.0, 0.01);
}

TEST(ExportPdf, DoesNotNegateYUnlikeSvg) {
    // PDF user space is Y-up, same as the model's, so the placement is a
    // scale and an offset in both axes and nothing more. Copying the SVG
    // writer's Y negation here would print the page upside down, which on
    // a symmetric model is invisible -- hence a triangle with its apex at
    // one known corner.
    const Pdf pdf = pdfOf("polygon([[0,0],[40,0],[40,25]]);", "yup.pdf", drawingOnly());
    const std::vector<std::pair<double, double>> pts = pathPoints(pdf.content);
    ASSERT_GE(pts.size(), 3u);
    double topY = pts[0].second, topX = pts[0].first, leftX = pts[0].first;
    for (const auto& p : pts) {
        if (p.second > topY) {
            topY = p.second;
            topX = p.first;
        }
        leftX = std::min(leftX, p.first);
    }
    // Model (40,25) is both the highest and the rightmost vertex. Negated
    // Y would put the highest point on the LEFT, at model (0,0).
    EXPECT_GT(topX, leftX + 1.0);
}

TEST(ExportPdf, PaperSizeAndOrientation) {
    ExportPdfOptions letter;
    letter.paper = ExportPdfOptions::Paper::Letter;
    EXPECT_NE(pdfOf("square(10);", "letter.pdf", letter).file.find("/MediaBox [0 0 612 792]"),
              std::string::npos);

    ExportPdfOptions land = letter;
    land.orientation = ExportPdfOptions::Orientation::Landscape;
    EXPECT_NE(pdfOf("square(10);", "land.pdf", land).file.find("/MediaBox [0 0 792 612]"), std::string::npos);

    // AUTO follows the model: wider than tall means landscape.
    ExportPdfOptions autoOrient;
    autoOrient.orientation = ExportPdfOptions::Orientation::Auto;
    EXPECT_NE(pdfOf("square([200,20]);", "auto_wide.pdf", autoOrient).file.find("/MediaBox [0 0 842 595]"),
              std::string::npos);
    EXPECT_NE(pdfOf("square([20,200]);", "auto_tall.pdf", autoOrient).file.find("/MediaBox [0 0 595 842]"),
              std::string::npos);
}

TEST(ExportPdf, DrawsTheRulerAndItsCaptionByDefault) {
    const Pdf pdf = pdfOf("square([70,25]);", "ruler.pdf");
    EXPECT_NE(pdf.content.find("Scale is to calibrate actual printed dimension"), std::string::npos);
    // Labels every 20mm, both signs -- the ruler is model space projected
    // across the whole sheet, not a scale bar under the drawing.
    EXPECT_NE(pdf.content.find("(0) Tj"), std::string::npos);
    EXPECT_NE(pdf.content.find("(20) Tj"), std::string::npos);
    EXPECT_NE(pdf.content.find("(-20) Tj"), std::string::npos);
    // ... and NOT on every tick.
    EXPECT_EQ(pdf.content.find("(10) Tj"), std::string::npos);
}

TEST(ExportPdf, TheZeroTickSitsOnTheModelOrigin) {
    // This is what makes the printed page a ruler for the model rather
    // than for the paper.
    const Pdf pdf = pdfOf("square([70,25]);", "origin.pdf");
    const size_t at = pdf.content.find("Tm (0) Tj");
    ASSERT_NE(at, std::string::npos);
    // "1 0 0 1 <x> <y> Tm (0) Tj" -- pull the x back out.
    const size_t numStart = pdf.content.rfind("1 0 0 1 ", at) + 8;
    const double x = std::stod(pdf.content.substr(numStart, at - numStart));
    // Model x=0 is 35mm left of the model centre, which sits on the page
    // centre; the label is drawn 1pt right of its tick.
    const double expected = 595.0 / 2.0 - 35.0 * 72.0 / 25.4 + 1.0;
    EXPECT_NEAR(x, expected, 0.01);
}

TEST(ExportPdf, ScaleOffLeavesJustTheDrawing) {
    ExportPdfOptions opts;
    opts.showScale = false;
    const Pdf pdf = pdfOf("square([70,25]);", "noscale.pdf", opts);
    EXPECT_EQ(pdf.content.find("Scale is to calibrate"), std::string::npos);
    EXPECT_EQ(pdf.content.find(" Tj"), std::string::npos);
}

TEST(ExportPdf, TheCaptionCanBeDroppedWithoutDroppingTheRuler) {
    ExportPdfOptions opts;
    opts.showScaleMsg = false;
    const Pdf pdf = pdfOf("square([70,25]);", "nomsg.pdf", opts);
    EXPECT_EQ(pdf.content.find("Scale is to calibrate"), std::string::npos);
    EXPECT_NE(pdf.content.find("(20) Tj"), std::string::npos);
}

TEST(ExportPdf, GridOnlyWhenAskedAndItIsGridSizeThatDrivesIt) {
    const Pdf without = pdfOf("square([70,25]);", "nogrid.pdf");
    ExportPdfOptions opts;
    opts.showGrid = true;
    opts.gridSize = 5.0;
    const Pdf with = pdfOf("square([70,25]);", "grid.pdf", opts);
    EXPECT_GT(countOf(with.content, " l S"), countOf(without.content, " l S"));

    // A finer grid means more lines; the ruler's own ticks never change,
    // since those are hard-coded at 10mm.
    opts.gridSize = 2.0;
    const Pdf finer = pdfOf("square([70,25]);", "grid2.pdf", opts);
    EXPECT_GT(countOf(finer.content, " l S"), countOf(with.content, " l S"));
}

TEST(ExportPdf, FilenameIsDrawnOnlyWhenAskedForAndSupplied) {
    ExportPdfOptions opts;
    opts.showFilename = true;
    EXPECT_EQ(pdfOf("square(10);", "noname.pdf", opts).content.find("(scale-card.scad) Tj"),
              std::string::npos);
    opts.designFilename = "scale-card.scad";
    EXPECT_NE(pdfOf("square(10);", "named.pdf", opts).content.find("(scale-card.scad) Tj"),
              std::string::npos);
}

TEST(ExportPdf, FillAndStrokeReachTheContentStream) {
    ExportPdfOptions opts;
    opts.fill = true;
    opts.fillColor = "red";
    opts.strokeColor = "blue";
    opts.strokeWidth = 1.0;
    const Pdf pdf = pdfOf("square(10);", "styled.pdf", opts);
    EXPECT_NE(pdf.content.find("1 0 0 rg"), std::string::npos);
    EXPECT_NE(pdf.content.find("0 0 1 RG"), std::string::npos);
    // 1mm of stroke is 2.8346pt, not 1.
    EXPECT_NE(pdf.content.find("2.8346 w"), std::string::npos);
    EXPECT_NE(pdf.content.find("B\n"), std::string::npos);  // fill AND stroke
}

TEST(ExportPdf, AModelTooBigForThePageIsWarnedAboutAndStillWritten) {
    const std::string path = tempPath("toobig.pdf").string();
    const std::vector<std::string> warnings = writePdf(path, evalToBodies("square([500,500]);"));
    ASSERT_EQ(warnings.size(), 1u);
    EXPECT_NE(warnings[0].find("larger than the printable area"), std::string::npos);
    EXPECT_GT(readBinary(path).size(), 500u);  // written anyway
    std::remove(path.c_str());
}

TEST(ExportPdf, MetadataIsOptional) {
    ExportPdfOptions opts;
    opts.metaTitle = "Calibration scale";
    opts.metaAuthor = "A Person";
    const Pdf with = pdfOf("square(10);", "meta.pdf", opts);
    EXPECT_NE(with.file.find("/Title (Calibration scale)"), std::string::npos);
    EXPECT_NE(with.file.find("/Author (A Person)"), std::string::npos);
    EXPECT_NE(with.file.find("/CreationDate (D:"), std::string::npos);

    opts.addMetaData = false;
    const Pdf without = pdfOf("square(10);", "nometa.pdf", opts);
    EXPECT_EQ(without.file.find("/Title"), std::string::npos);
    EXPECT_EQ(without.file.find("/Info"), std::string::npos);
}

TEST(ExportPdf, PaperAndOrientationNames) {
    ExportPdfOptions::Paper paper = ExportPdfOptions::Paper::A4;
    EXPECT_TRUE(paperFromName("Tabloid", paper));
    EXPECT_EQ(paper, ExportPdfOptions::Paper::Tabloid);
    EXPECT_FALSE(paperFromName("a2", paper));
    EXPECT_EQ(paper, ExportPdfOptions::Paper::Tabloid);  // untouched

    ExportPdfOptions::Orientation o = ExportPdfOptions::Orientation::Portrait;
    EXPECT_TRUE(orientationFromName("AUTO", o));
    EXPECT_EQ(o, ExportPdfOptions::Orientation::Auto);
    EXPECT_FALSE(orientationFromName("sideways", o));
}

TEST(ExportPdf, RefusesA3dModelAndEmptyGeometry) {
    std::vector<ColoredBody> solid = evalToBodies("cube(10);");
    EXPECT_THROW(writePdf(tempPath("solid.pdf").string(), solid), std::runtime_error);
    std::vector<ColoredBody> empty;
    EXPECT_THROW(writePdf(tempPath("empty.pdf").string(), empty), std::runtime_error);
}

TEST(ExportPdf, ReachableThroughExportModelAndListedAsAnExtension) {
    const std::vector<std::string>& exts = exportExtensions();
    EXPECT_NE(std::find(exts.begin(), exts.end(), ".pdf"), exts.end());

    const std::string path = tempPath("via_export_model.pdf").string();
    const std::vector<std::string> warnings = exportModel(path, evalToBodies("circle(5, $fn=8);"), ExportOptions{});
    EXPECT_TRUE(warnings.empty());
    EXPECT_EQ(readBinary(path).substr(0, 8), "%PDF-1.4");
    std::remove(path.c_str());
}


// -- single-material vs multi-material -------------------------------------

TEST(SplitColors, MultiMaterialGivesOneObjectPerColour) {
    // The default: each colour becomes something a slicer can assign to a
    // filament.
    std::vector<ColoredBody> bodies =
        evalToBodies("color(\"red\") cube(10); color(\"blue\") translate([12,0,0]) cube(10);");
    const std::vector<ExportObject> objs = splitBodiesForExport(bodies, nullptr, false, /*splitColors=*/true);
    EXPECT_EQ(objs.size(), 2u);
}

TEST(SplitColors, SingleMaterialGivesOneObject) {
    // A single-material print has nothing to do with the colours, and
    // splitting on them only gives the slicer parts to list.
    std::vector<ColoredBody> bodies =
        evalToBodies("color(\"red\") cube(10); color(\"blue\") translate([12,0,0]) cube(10);");
    const std::vector<ExportObject> objs = splitBodiesForExport(bodies, nullptr, false, /*splitColors=*/false);
    ASSERT_EQ(objs.size(), 1u);
    EXPECT_TRUE(objs[0].triColors.empty());  // no per-triangle colour to carry
}

// Each body is cut only by the later bodies that actually reach it, rather
// than by a running union of all of them (#177). These pin the two things
// that rewrite must not change.

TEST(SplitColors, AFarAwayBodyChangesNothing) {
    // A body nowhere near the others cannot take volume from them, so adding
    // one must leave every other object exactly as it was. The old code
    // subtracted the union of ALL later bodies and leaned on a bounding-box
    // test against that union to stay correct -- once the union spanned the
    // model, the test stopped discriminating.
    const std::string pair =
        "color(\"red\") cube(10); color(\"blue\") translate([5,0,0]) cube(10);";
    std::vector<ColoredBody> without = evalToBodies(pair);
    std::vector<ColoredBody> with =
        evalToBodies(pair + " color(\"green\") translate([500,0,0]) cube(10);");

    const std::vector<ExportObject> a = splitBodiesForExport(without, nullptr, false, true);
    const std::vector<ExportObject> b = splitBodiesForExport(with, nullptr, false, true);
    ASSERT_EQ(a.size(), 2u);
    ASSERT_EQ(b.size(), 3u);
    for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].tris.size(), b[i].tris.size()) << "object " << i << " was re-cut";
        EXPECT_EQ(a[i].verts.size(), b[i].verts.size()) << "object " << i << " was re-cut";
    }
}

TEST(SplitColors, LaterWinsAcrossAnInterveningColour) {
    // red, blue, red -- all overlapping, in that order. The later body wins,
    // so the second red takes its volume whole, blue keeps only what that
    // red does not cover, and the first red keeps only what neither does.
    //
    // Worth pinning because the obvious way to make this cheaper -- union
    // the same-coloured bodies first, then subtract per colour -- gets it
    // wrong: it would let the first red beat the blue that comes after it.
    std::vector<ColoredBody> bodies = evalToBodies(
        "color(\"red\")  cube(10);"
        "color(\"blue\") translate([5,0,0]) cube(10);"
        "color(\"red\")  translate([10,0,0]) cube(10);");
    const std::vector<ExportObject> objs = splitBodiesForExport(bodies, nullptr, false, true);
    ASSERT_EQ(objs.size(), 2u);   // one per colour, the two reds merged

    const auto volumeOf = [](const ExportObject& o) {
        double v = 0.0;
        for (size_t t = 0; t + 2 < o.tris.size(); t += 3) {
            const auto at = [&](size_t k) {
                const size_t b = static_cast<size_t>(o.tris[t + k]) * 3;
                return std::array<double, 3>{o.verts[b], o.verts[b + 1], o.verts[b + 2]};
            };
            const std::array<double, 3> p = at(0), q = at(1), r = at(2);
            v += (p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0]) +
                  p[2] * (q[0] * r[1] - q[1] * r[0])) / 6.0;
        }
        return v;
    };

    // Blue spans x 5..15 and the later red takes x 10..20, leaving x 5..10:
    // 5 x 10 x 10. The reds keep everything else: 2000 - 500.
    double red = 0.0, blue = 0.0;
    for (const ExportObject& o : objs) (o.color[2] > o.color[0] ? blue : red) += volumeOf(o);
    EXPECT_NEAR(blue, 500.0, 1e-6);
    EXPECT_NEAR(red, 1500.0, 1e-6);
}

TEST(SplitColors, SingleMaterialWeldsTouchingColourRegions) {
    // Unioned, not concatenated: two touching cubes of different colours
    // must come out as one solid without the coincident interior faces
    // that stacking the per-colour objects would leave behind.
    std::vector<ColoredBody> bodies =
        evalToBodies("color(\"red\") cube(10); color(\"blue\") translate([10,0,0]) cube(10);");
    const std::vector<ExportObject> single = splitBodiesForExport(bodies, nullptr, false, /*splitColors=*/false);
    ASSERT_EQ(single.size(), 1u);

    const std::vector<ExportObject> multi = splitBodiesForExport(bodies, nullptr, false, true);
    ASSERT_EQ(multi.size(), 2u);
    size_t multiTris = 0;
    for (const ExportObject& o : multi) multiTris += o.tris.size() / 3;

    // Fewer triangles than the two objects have between them: the pair of
    // coincident faces along the seam is gone, which is the difference
    // between a welded solid and two boxes stacked in one object. (Not
    // the 12 of a plain 20x10x10 box -- Manifold keeps the seam's
    // vertices, so the faces crossing it stay split.)
    EXPECT_LT(single[0].tris.size() / 3, multiTris);
    EXPECT_EQ(single[0].tris.size() / 3, 20u);
}

TEST(SplitColors, SingleMaterialKeepsTheWholeVolume) {
    std::vector<ColoredBody> bodies =
        evalToBodies("color(\"red\") cube(10); color(\"blue\") translate([12,0,0]) cube(10);");
    const std::vector<ExportObject> multi = splitBodiesForExport(bodies, nullptr, false, true);
    const std::vector<ExportObject> single = splitBodiesForExport(bodies, nullptr, false, false);
    size_t multiTris = 0;
    for (const ExportObject& o : multi) multiTris += o.tris.size();
    EXPECT_EQ(single[0].tris.size(), multiTris);  // same surface, one object
}

TEST(SplitColors, AnUncolouredModelIsUnaffectedEitherWay) {
    std::vector<ColoredBody> bodies = evalToBodies("cube(10); translate([12,0,0]) cube(10);");
    EXPECT_EQ(splitBodiesForExport(bodies, nullptr, false, true).size(), 1u);
    EXPECT_EQ(splitBodiesForExport(bodies, nullptr, false, false).size(), 1u);
}

// A per-triangle-coloured object written after another object indexed its
// faces with faces.size() + base, counting the earlier object twice -- so
// every index pointed past the vertex list. Read the file back and check.
TEST(ExportPly, PerTriangleColourAfterAnotherObjectIndexesItsOwnVertices) {
    ExportObject plain;
    plain.verts = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
    plain.tris = {0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2};
    ExportObject painted = plain;
    for (float& v : painted.verts) v += 5.0f;
    painted.triColors = {{1, 0, 0, 1}, {0, 1, 0, 1}, {0, 0, 1, 1}, {1, 1, 0, 1}};

    const auto path = tempPath("two_objects.ply");
    writePly(path.string(), {plain, painted});

    std::ifstream in(path, std::ios::binary);
    std::string line;
    size_t nVerts = 0, nFaces = 0;
    while (std::getline(in, line) && line != "end_header") {
        std::istringstream ls(line);
        std::string a, b;
        size_t n = 0;
        ls >> a >> b >> n;
        if (a == "element" && b == "vertex") nVerts = n;
        if (a == "element" && b == "face") nFaces = n;
    }
    ASSERT_EQ(nVerts, 4u + 12u);   // the painted object is unwelded: 3 per triangle
    ASSERT_EQ(nFaces, 8u);
    in.seekg(static_cast<std::streamoff>(nVerts * (3 * sizeof(float) + 3)), std::ios::cur);
    std::vector<int32_t> seen;
    for (size_t f = 0; f < nFaces; ++f) {
        char count = 0;
        in.read(&count, 1);
        ASSERT_EQ(count, 3);
        for (int k = 0; k < 3; ++k) {
            int32_t idx = -1;
            in.read(reinterpret_cast<char*>(&idx), sizeof idx);
            EXPECT_GE(idx, 0);
            EXPECT_LT(static_cast<size_t>(idx), nVerts);
            seen.push_back(idx);
        }
    }
    // The painted object's faces are exactly its own twelve vertices, in order.
    for (int k = 0; k < 12; ++k) EXPECT_EQ(seen[12 + k], 4 + k);
    in.close();  // Windows refuses to remove a file that is still open
    std::filesystem::remove(path);
}
