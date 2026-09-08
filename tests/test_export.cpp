#include "openscad_cpp_evaluator/export.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
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

