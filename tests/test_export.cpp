#include "openscad_cpp_evaluator/export.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

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
    // into a thin (1e-3 unit tall) Manifold first, mirroring the reference
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
    EXPECT_NEAR(bbox.max.z - bbox.min.z, 1e-3, 1e-9);

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
    std::vector<ColoredBody> bodies =
        evalToBodies("color(\"red\") cube(2); translate([10,0,0]) color(\"red\") cube(2);");
    const std::string path = tempPath("shared.amf").string();
    exportModel(path, bodies, ExportOptions{});

    const std::string xml = readFile(path);
    EXPECT_EQ(countOf(xml, "<object id="), 2u);
    EXPECT_EQ(countOf(xml, "<material id="), 1u);
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
