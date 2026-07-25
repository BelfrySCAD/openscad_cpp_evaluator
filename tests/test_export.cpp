#include "openscad_cpp_evaluator/export.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

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
    const std::string path = "/tmp/oscad_eval_test_cube.stl";
    writeStl(path, bodies);

    StlSummary s = readStl(path);
    EXPECT_EQ(s.triangleCount, 12u);
    EXPECT_FLOAT_EQ(s.minX, 0.0f);
    EXPECT_FLOAT_EQ(s.maxX, 2.0f);
    std::remove(path.c_str());
}

TEST(ExportStl, TranslatedCubeBoundsMatch) {
    std::vector<ColoredBody> bodies = evalToBodies("translate([1,0,0]) cube(2);");
    const std::string path = "/tmp/oscad_eval_test_translate.stl";
    writeStl(path, bodies);

    StlSummary s = readStl(path);
    EXPECT_FLOAT_EQ(s.minX, 1.0f);
    EXPECT_FLOAT_EQ(s.maxX, 3.0f);
    std::remove(path.c_str());
}

TEST(ExportStl, NoGeometryThrows) {
    std::vector<ColoredBody> empty;
    EXPECT_THROW(writeStl("/tmp/oscad_eval_test_empty.stl", empty), std::runtime_error);
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

    const std::string path = "/tmp/oscad_eval_test_flat_preview.stl";
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
