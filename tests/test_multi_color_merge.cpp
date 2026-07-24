#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <set>

// Mirrors the reference's TestMultiColorCSGMerge: a real boolean CSG merge
// (generateCsg) used to collapse every child's color into just the first
// child's -- e.g. union()-ing an opaque cube with a translucent sphere
// dropped the sphere's color and alpha entirely, rendering the whole
// result fully opaque. attachTriColors() (booleans.cpp) recovers this via
// Manifold's own per-triangle runOriginalID/runIndex provenance (already
// used for idToNode/WYSIWYG ray-cast picking) plus the parallel
// Evaluator::idToColor map populated by tagGenerated().

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

bool approxEqual(const std::array<float, 4>& a, const std::array<float, 4>& b, float eps = 1e-4f) {
    for (int i = 0; i < 4; ++i) {
        if (std::fabs(a[i] - b[i]) > eps) return false;
    }
    return true;
}

std::vector<std::array<float, 4>> distinctColors(const std::vector<std::array<float, 4>>& triColors) {
    std::vector<std::array<float, 4>> out;
    for (const auto& c : triColors) {
        if (std::none_of(out.begin(), out.end(), [&](const auto& o) { return approxEqual(o, c, 1e-6f); })) out.push_back(c);
    }
    return out;
}

constexpr std::array<float, 4> kDefaultGeometryColor{0.9f, 0.85f, 0.1f, 1.0f};

} // namespace

TEST(MultiColorCsgMerge, UnionOfMixedColorsSetsTriColors) {
    Evaluated e = evalSrc("union() {"
                          "  color(\"lightgreen\") cube(10);"
                          "  color([0,1,1,0.5]) translate([5,5,10]) sphere(d=10, $fn=16);"
                          "}");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].triColors.has_value());
    const auto distinct = distinctColors(*e.bodies[0].triColors);
    EXPECT_EQ(distinct.size(), 2u);
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, {0.0f, 1.0f, 1.0f, 0.5f}); }));
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(),
                            [](const auto& c) { return c[3] == 1.0f && !approxEqual(c, {0.0f, 1.0f, 1.0f, 0.5f}); }));
}

TEST(MultiColorCsgMerge, DifferenceCutFaceGetsDefaultColor) {
    // The cylinder tool has no explicit color() -- its newly-exposed cut
    // face (a fresh runOriginalID contributed by the subtraction tool)
    // must fall back to the default geometry color, matching what real
    // OpenSCAD shows for an uncolored modifier used only as a cutter.
    Evaluated e = evalSrc("difference() {"
                          "  union() {"
                          "    color(\"lightgreen\") cube(10);"
                          "    color([0,1,1,0.5]) translate([5,5,10]) sphere(d=10, $fn=16);"
                          "  }"
                          "  translate([5,5,-0.01]) cylinder(h=10.02, d=8, $fn=16);"
                          "}");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].triColors.has_value());
    const auto distinct = distinctColors(*e.bodies[0].triColors);
    EXPECT_EQ(distinct.size(), 3u);
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, {0.0f, 1.0f, 1.0f, 0.5f}); }));
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, kDefaultGeometryColor); }));
}

TEST(MultiColorCsgMerge, UnionOfSameExplicitColorLeavesTriColorsUnset) {
    // Cheap-path guarantee: if every contributing color resolves to the
    // same value, triColors must stay unset (same single-buffer,
    // live-theme-following upload path as before this feature existed).
    Evaluated e = evalSrc("union() {"
                          "  color(\"red\") cube(10);"
                          "  color(\"red\") translate([5,5,10]) sphere(d=10, $fn=16);"
                          "}");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_FALSE(e.bodies[0].triColors.has_value());
}

TEST(MultiColorCsgMerge, UnionWithNoExplicitColorLeavesTriColorsUnset) {
    Evaluated e = evalSrc("union() { cube(10); translate([5,5,10]) sphere(d=10, $fn=16); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_FALSE(e.bodies[0].triColors.has_value());
}

TEST(MultiColorCsgMerge, IdToColorPopulatedPerPrimitive) {
    Evaluated e = evalSrc("color(\"red\") cube(1); color([0,0,1,0.4]) translate([3,0,0]) sphere(1, $fn=16);");
    bool foundTranslucentBlue = false;
    for (const auto& [id, color] : e.ev.idToColor) {
        if (color && (*color)[3] == 0.4f) foundTranslucentBlue = true;
    }
    EXPECT_TRUE(foundTranslucentBlue);
}

TEST(MultiColorCsgMerge, SingleChildUnionNoMergeLeavesTriColorsUnset) {
    // No real boolean merge (union of ONE child) -- generateCsg never even
    // reaches the multi-operand branch, so triColors must stay unset
    // regardless of color.
    Evaluated e = evalSrc("union() { color([1,0,0,0.5]) cube(1); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_FALSE(e.bodies[0].triColors.has_value());
}
