#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/manifold_cache.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
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

TEST(MultiColorCsgMerge, DifferenceCutFaceGetsTheCutGreen) {
    // The cylinder tool has no explicit color() -- its newly-exposed cut
    // face (a fresh runOriginalID contributed by the subtraction tool)
    // takes the cut green, which is what OpenSCAD 2026.02.01 paints there
    // in preview and keeps in a colour-preserving render (3MF export
    // measured: #9DCB51). It used to take the default geometry colour,
    // so a cut through an uncoloured part could not be seen as a cut.
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
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, kCutFaceColor); }));
}

TEST(MultiColorCsgMerge, UncolouredMinuendCutByUncolouredToolStillShowsTheCut) {
    // Nothing is coloured, so the operands' own colours agree -- the merge
    // must look anyway, or the cut vanishes into the default colour.
    Evaluated e = evalSrc("difference() { cube(10); translate([5,5,-1]) cylinder(h=12, d=6, $fn=16); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].triColors.has_value());
    const auto distinct = distinctColors(*e.bodies[0].triColors);
    EXPECT_EQ(distinct.size(), 2u);
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, kCutFaceColor); }));
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, kDefaultGeometryColor); }));
}

TEST(MultiColorCsgMerge, ColouredToolStillPaintsItsCut) {
    Evaluated e = evalSrc("difference() { color(\"orange\") cube(10); color(\"cyan\") translate([5,5,-1]) cylinder(h=12, d=6, $fn=16); }");
    const auto distinct = distinctColors(*e.bodies[0].triColors);
    EXPECT_EQ(distinct.size(), 2u);
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, {0.0f, 1.0f, 1.0f, 1.0f}); }));
}

namespace {
Evaluated evalKeepingMinuend(const std::string& code) {
    Evaluated e{parseSrc(code), nullptr, Evaluator(), {}, {}};
    e.ev.keepMinuendColor = true;
    e.scope = oscad::buildScopes(e.ast);
    EvalContext ctx = EvalContext::makeRoot(e.scope.get());
    e.tree = e.ev.resolveTree(e.ast, ctx);
    e.bodies = e.ev.generateTree(e.tree);
    return e;
}
} // namespace

TEST(KeepMinuendColor, SingleColouredMinuendStaysOneColour) {
    // BelfrySCAD #412's enhancement: the pencil sharpened by a cone should
    // still be pencil-coloured at the point.
    Evaluated e = evalKeepingMinuend(
        "difference() { color(\"orange\") cube(20, center=true); cylinder(30, r=8, center=true, $fn=24); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    // Every run resolves to orange, so no per-triangle array is needed.
    EXPECT_FALSE(e.bodies[0].triColors.has_value());
    ASSERT_TRUE(e.bodies[0].color.has_value());
    EXPECT_TRUE(approxEqual(*e.bodies[0].color, {1.0f, 0.647058845f, 0.0f, 1.0f}));
    // And the geometry is the same difference.
    Evaluated plain = evalSrc("difference() { color(\"orange\") cube(20, center=true); cylinder(30, r=8, center=true, $fn=24); }");
    EXPECT_NEAR(e.bodies[0].body->Volume(), plain.bodies[0].body->Volume(), 1e-3);
}

TEST(KeepMinuendColor, EachMinuendPartKeepsItsOwnColourOnItsCutFaces) {
    // A red block at x<0 and a blue one at x>0, a coloured tool through
    // both: the cut faces on the red side are red, on the blue side blue,
    // and the tool's cyan appears nowhere.
    Evaluated e = evalKeepingMinuend(
        "difference() {"
        "  union() { color(\"red\") translate([-10,-5,-5]) cube(10); color(\"blue\") translate([0,-5,-5]) cube(10); }"
        "  color(\"cyan\") rotate([0,90,0]) cylinder(h=30, r=2, center=true, $fn=16);"
        "}");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].triColors.has_value());
    const auto distinct = distinctColors(*e.bodies[0].triColors);
    EXPECT_EQ(distinct.size(), 2u);
    const manifold::MeshGL mesh = e.bodies[0].body->GetMeshGL();
    const auto& tc = *e.bodies[0].triColors;
    ASSERT_EQ(tc.size(), mesh.triVerts.size() / 3);
    for (size_t t = 0; t < tc.size(); ++t) {
        double cx = 0;
        for (int k = 0; k < 3; ++k) cx += mesh.vertProperties[mesh.triVerts[3 * t + k] * mesh.numProp];
        cx /= 3;
        const std::array<float, 4> want = cx < 0 ? std::array<float, 4>{1, 0, 0, 1} : std::array<float, 4>{0, 0, 1, 1};
        ASSERT_TRUE(approxEqual(tc[t], want)) << "triangle " << t << " at x=" << cx;
    }
}

TEST(KeepMinuendColor, PartsFollowTransformsAndColorOverTheUnion) {
    // translate() over the union moves the remembered parts with it, so
    // the per-part cut still lands on the right side; color() over the
    // union recolours every part, so the cut is that colour and nothing
    // else.
    Evaluated moved = evalKeepingMinuend(
        "difference() {"
        "  translate([0,0,20]) union() { color(\"red\") translate([-10,-5,-5]) cube(10); color(\"blue\") translate([0,-5,-5]) cube(10); }"
        "  translate([0,0,20]) color(\"cyan\") rotate([0,90,0]) cylinder(h=30, r=2, center=true, $fn=16);"
        "}");
    ASSERT_TRUE(moved.bodies[0].triColors.has_value());
    EXPECT_EQ(distinctColors(*moved.bodies[0].triColors).size(), 2u);
    const manifold::MeshGL mesh = moved.bodies[0].body->GetMeshGL();
    const auto& tc = *moved.bodies[0].triColors;
    for (size_t t = 0; t < tc.size(); ++t) {
        double cx = 0;
        for (int k = 0; k < 3; ++k) cx += mesh.vertProperties[mesh.triVerts[3 * t + k] * mesh.numProp];
        cx /= 3;
        const std::array<float, 4> want = cx < 0 ? std::array<float, 4>{1, 0, 0, 1} : std::array<float, 4>{0, 0, 1, 1};
        ASSERT_TRUE(approxEqual(tc[t], want)) << "triangle " << t << " at x=" << cx;
    }

    Evaluated recoloured = evalKeepingMinuend(
        "difference() {"
        "  color(\"green\") union() { color(\"red\") translate([-10,-5,-5]) cube(10); color(\"blue\") translate([0,-5,-5]) cube(10); }"
        "  color(\"cyan\") rotate([0,90,0]) cylinder(h=30, r=2, center=true, $fn=16);"
        "}");
    EXPECT_FALSE(recoloured.bodies[0].triColors.has_value());
    ASSERT_TRUE(recoloured.bodies[0].color.has_value());
    EXPECT_TRUE(approxEqual(*recoloured.bodies[0].color, {0.0f, 128.0f / 255.0f, 0.0f, 1.0f}));
}

TEST(KeepMinuendColor, CachedUnionMinuendStillCutsPerPart) {
    // Edit only the tool: the union is a cache hit, restamped to fresh
    // IDs, and its remembered parts must be restamped with it.
    auto cache = std::make_shared<ManifoldCache>();
    auto run = [&](const std::string& tool) {
        Evaluated e{parseSrc("difference() {"
                             "  union() { color(\"red\") translate([-10,-5,-5]) cube(10); color(\"blue\") translate([0,-5,-5]) cube(10); }"
                             "  " + tool + " }"),
                    nullptr, Evaluator({}, nullptr, cache), {}, {}};
        e.ev.keepMinuendColor = true;
        e.scope = oscad::buildScopes(e.ast);
        EvalContext ctx = EvalContext::makeRoot(e.scope.get());
        e.tree = e.ev.resolveTree(e.ast, ctx);
        e.bodies = e.ev.generateTree(e.tree);
        return e;
    };
    run("rotate([0,90,0]) cylinder(h=30, r=2, center=true, $fn=16);");
    Evaluated second = run("rotate([0,90,0]) cylinder(h=30, r=3, center=true, $fn=16);");
    ASSERT_TRUE(second.bodies[0].triColors.has_value());
    const auto distinct = distinctColors(*second.bodies[0].triColors);
    EXPECT_EQ(distinct.size(), 2u);
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, {1, 0, 0, 1}); }));
    EXPECT_TRUE(std::any_of(distinct.begin(), distinct.end(), [](const auto& c) { return approxEqual(c, {0, 0, 1, 1}); }));
}

TEST(KeepMinuendColor, CachedResultsAreKeyedApartPerMode) {
    auto cache = std::make_shared<ManifoldCache>();
    const std::string src = "difference() { color(\"orange\") cube(10); translate([5,5,-1]) cylinder(h=12, d=6, $fn=16); }";
    Evaluated off = evalSrcWithCache(src, cache);
    ASSERT_TRUE(off.bodies[0].triColors.has_value());   // orange + cut green
    Evaluated on{parseSrc(src), nullptr, Evaluator({}, nullptr, cache), {}, {}};
    on.ev.keepMinuendColor = true;
    on.scope = oscad::buildScopes(on.ast);
    EvalContext ctx = EvalContext::makeRoot(on.scope.get());
    on.tree = on.ev.resolveTree(on.ast, ctx);
    on.bodies = on.ev.generateTree(on.tree);
    EXPECT_FALSE(on.bodies[0].triColors.has_value());   // all orange, not the cached green-cut body
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
