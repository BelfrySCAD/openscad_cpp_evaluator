#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

// STB_IMAGE_WRITE_IMPLEMENTATION is defined once, in src/zip_stored.cpp
// (part of the library this test binary links against) -- must not be
// redefined here too, or both object files would define the same stb
// symbols (ODR violation).
#include <stb_image_write.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <vector>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::filesystem::path tempPath(const std::string& name) { return std::filesystem::temp_directory_path() / ("oscad_eval_test_" + name); }

void writeDat(const std::filesystem::path& path, const std::vector<std::vector<double>>& rows) {
    std::ofstream out(path);
    for (const auto& row : rows) {
        for (size_t i = 0; i < row.size(); ++i) out << (i ? " " : "") << row[i];
        out << "\n";
    }
}

// Writes a `w`x`h` grayscale PNG where pixel (col, imgRow) has the given
// byte value (imgRow 0 = top scanline, standard raster order).
void writeGrayscalePng(const std::filesystem::path& path, int w, int h, const std::vector<unsigned char>& pixels) {
    stbi_write_png(path.string().c_str(), w, h, 1, pixels.data(), w);
}

} // namespace

// -- .dat loading -----------------------------------------------------------

TEST(Surface, DatFileHeightsMatchExactVertexPositions) {
    // File's first line = highest Y (OpenSCAD convention) -- verified via
    // export.py cross-check in the session, exact match with the Python
    // reference on this shape (12 triangles, bbox [0,0,0]-[2,2,5]).
    const auto path = tempPath("terrain.dat");
    writeDat(path, {{0, 0, 0}, {0, 5, 0}, {0, 0, 0}});
    Evaluated e = evalSrc("surface(file=\"" + path.generic_string() + "\", center=false);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, 0.0, 1e-9);
    EXPECT_NEAR(bbox.max.x, 2.0, 1e-9);
    EXPECT_NEAR(bbox.min.z, 0.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 5.0, 1e-9);
    std::filesystem::remove(path);
}

TEST(Surface, CenterTrueOffsetsXyAroundOrigin) {
    const auto path = tempPath("terrain_center.dat");
    writeDat(path, {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}});
    Evaluated e = evalSrc("surface(file=\"" + path.generic_string() + "\", center=true);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, -1.0, 1e-9); // (cols-1)/2 = 1
    EXPECT_NEAR(bbox.max.x, 1.0, 1e-9);
    std::filesystem::remove(path);
}

TEST(Surface, FlatGridVolumeMatchesFootprintTimesHeight) {
    const auto path = tempPath("flat.dat");
    writeDat(path, {{3, 3, 3}, {3, 3, 3}, {3, 3, 3}});
    Evaluated e = evalSrc("surface(file=\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 2.0 * 2.0 * 3.0, 1e-6); // (cols-1)*(rows-1)*height
    std::filesystem::remove(path);
}

TEST(Surface, EmptyFileErrors) {
    const auto path = tempPath("empty.dat");
    writeDat(path, {});
    Evaluator ev;
    auto ast = parseSrc("surface(file=\"" + path.generic_string() + "\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    std::filesystem::remove(path);
}

// -- image loading (stb_image, via a stb_image_write-generated fixture) ----

TEST(Surface, UniformGrayImageVolumeMatchesLuminanceFormula) {
    const auto path = tempPath("flat.png");
    writeGrayscalePng(path, 3, 3, std::vector<unsigned char>(9, 128));
    Evaluated e = evalSrc("surface(file=\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].body.has_value());
    // gray=128 (r=g=b, so the weighted luminance formula reduces to the
    // input value exactly) -> height = 128/255*100.
    const double height = 128.0 / 255.0 * 100.0;
    EXPECT_NEAR(e.bodies[0].body->Volume(), 2.0 * 2.0 * height, height * 2e-3);
    std::filesystem::remove(path);
}

TEST(Surface, InvertFlipsTheHeightMapping) {
    const auto path = tempPath("flat2.png");
    writeGrayscalePng(path, 3, 3, std::vector<unsigned char>(9, 200));
    Evaluated normal = evalSrc("surface(file=\"" + path.generic_string() + "\", invert=false);");
    Evaluated inverted = evalSrc("surface(file=\"" + path.generic_string() + "\", invert=true);");
    const double heightNormal = 200.0 / 255.0 * 100.0;
    const double heightInverted = (255.0 - 200.0) / 255.0 * 100.0;
    EXPECT_NEAR(normal.bodies[0].body->Volume(), 2.0 * 2.0 * heightNormal, heightNormal * 2e-3);
    EXPECT_NEAR(inverted.bodies[0].body->Volume(), 2.0 * 2.0 * heightInverted, std::max(1e-6, heightInverted * 2e-3));
    std::filesystem::remove(path);
}

TEST(Surface, ImageRowOrderMatchesBottomRowIsYZero) {
    // 2x2 image: top image row (imgRow 0) black, bottom image row (imgRow
    // 1) white. Since a PNG's row 0 is its top scanline and OpenSCAD's Y=0
    // is the *bottom* of the grid, the white (bottom-of-image) row must
    // land at the LOW-Y side of the surface, not the high-Y side.
    const auto path = tempPath("rowcheck.png");
    std::vector<unsigned char> px = {0, 0, 255, 255}; // row0: black,black ; row1: white,white
    writeGrayscalePng(path, 2, 2, px);
    Evaluated e = evalSrc("surface(file=\"" + path.generic_string() + "\");");
    ASSERT_TRUE(e.bodies[0].body.has_value());
    // Slice a thin box at low Y (y in [0,0.5]) and confirm it reaches the
    // white row's full height (100), not the black row's height (0).
    manifold::Manifold slab = manifold::Manifold::Cube(manifold::vec3(2, 0.5, 200), false).Translate(manifold::vec3(-0.5, 0, -50));
    manifold::Manifold lowY = *e.bodies[0].body ^ slab;
    EXPECT_NEAR(lowY.BoundingBox().max.z, 100.0, 1.0);
    std::filesystem::remove(path);
}
