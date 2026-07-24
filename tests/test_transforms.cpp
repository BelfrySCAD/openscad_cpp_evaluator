#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

// -- rotate ---------------------------------------------------------------

TEST(Rotate, EulerAnglesRotateCubeAroundZ) {
    // A 1x2x3 cube (uncentered) rotated 90 deg around Z swaps its x/y extent.
    Evaluated e = evalSrc("rotate([0,0,90]) cube([1,2,3]);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x - bbox.min.x, 2.0, 1e-6);
    EXPECT_NEAR(bbox.max.y - bbox.min.y, 1.0, 1e-6);
    EXPECT_NEAR(bbox.max.z - bbox.min.z, 3.0, 1e-6);
}

TEST(Rotate, AxisAngleFormRotatesAroundArbitraryAxis) {
    // 180 deg around the X axis: volume/shape preserved (rigid rotation).
    Evaluated e = evalSrc("rotate(180, [1,0,0]) cube([1,2,3], center=true);");
    EXPECT_NEAR(e.bodies[0].body->Volume(), 6.0, 1e-6);
}

TEST(Rotate, DefaultAxisIsZ) {
    Evaluated withAxis = evalSrc("rotate(45, [0,0,1]) cube([2,4,1], center=true);");
    Evaluated withoutAxis = evalSrc("rotate(45) cube([2,4,1], center=true);");
    manifold::Box a = withAxis.bodies[0].body->BoundingBox();
    manifold::Box b = withoutAxis.bodies[0].body->BoundingBox();
    EXPECT_NEAR(a.max.x, b.max.x, 1e-6);
    EXPECT_NEAR(a.max.y, b.max.y, 1e-6);
}

// -- scale / mirror -----------------------------------------------------

TEST(Scale, ScalesEachAxisIndependently) {
    Evaluated e = evalSrc("scale([2,3,4]) cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 2.0, 1e-9);
    EXPECT_NEAR(bbox.max.y, 3.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 4.0, 1e-9);
}

TEST(Scale, ScalarBroadcastsToAllAxes) {
    Evaluated e = evalSrc("scale(2) cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 2.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 2.0, 1e-9);
}

TEST(Mirror, PreservesVolumeAndFlipsPosition) {
    Evaluated e = evalSrc("mirror([1,0,0]) translate([1,0,0]) cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, -2.0, 1e-9);
    EXPECT_NEAR(bbox.max.x, -1.0, 1e-9);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0, 1e-9);
}

// -- multmatrix -----------------------------------------------------------

TEST(Multmatrix, PlainTranslationMatrixActsLikeTranslate) {
    Evaluated e = evalSrc("multmatrix([[1,0,0,5],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, 5.0, 1e-9);
    EXPECT_NEAR(bbox.max.x, 6.0, 1e-9);
}

// -- resize -----------------------------------------------------------

TEST(Resize, RescalesToExactTargetDimensions) {
    Evaluated e = evalSrc("resize([10,20,30]) cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 10.0, 1e-6);
    EXPECT_NEAR(bbox.max.y, 20.0, 1e-6);
    EXPECT_NEAR(bbox.max.z, 30.0, 1e-6);
}

TEST(Resize, ZeroComponentLeavesThatAxisUnscaled) {
    Evaluated e = evalSrc("resize([0,10,0]) cube([1,1,1]);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 1.0, 1e-9); // unscaled
    EXPECT_NEAR(bbox.max.y, 10.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 1.0, 1e-9); // unscaled
}

// -- 2D transform dispatch (a transform's child is a CrossSection, not a
// Manifold -- resolved per-body at generate time) ---------------------------

TEST(Transform2d, TranslateMovesACircle) {
    Evaluated e = evalSrc("translate([5,0]) circle(r=1, $fn=32);");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.min.x, 4.0, 0.01);
    EXPECT_NEAR(bounds.max.x, 6.0, 0.01);
}

TEST(Transform2d, ScalePreservesAreaRatio) {
    Evaluated e = evalSrc("scale([2,3]) square([1,1]);");
    EXPECT_NEAR(e.bodies[0].section->Area(), 6.0, 1e-9);
}

// -- color ------------------------------------------------------------

TEST(Color, NamedColorSetsRgbaOnDescendant) {
    Evaluated e = evalSrc("color(\"red\") cube(1);");
    ASSERT_TRUE(e.bodies[0].color.has_value());
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[0], 1.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[1], 0.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[2], 0.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[3], 1.0f);
}

TEST(Color, RgbListDefaultsAlphaToOne) {
    Evaluated e = evalSrc("color([0,0.5,1]) cube(1);");
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[3], 1.0f);
}

TEST(Color, RgbaListIgnoresSeparateAlphaArgument) {
    // len(c) == 4 -> alpha comes from c[3], NOT the separate `alpha` arg.
    Evaluated e = evalSrc("color([0,0.5,1,0.25], 0.9) cube(1);");
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[3], 0.25f);
}

TEST(Color, HexColorParses) {
    Evaluated e = evalSrc("color(\"#00ff00\") cube(1);");
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[0], 0.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[1], 1.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[2], 0.0f);
}
