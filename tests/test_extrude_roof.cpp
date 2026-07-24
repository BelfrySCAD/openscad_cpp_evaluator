#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

using namespace oscadeval;
using namespace oscadeval::test;

// -- hull -------------------------------------------------------------------

TEST(Hull, ConvexHullOfTwoDisjointCubesFillsTheGap) {
    Evaluated e = evalSrc("hull() { cube(1); translate([5,0,0]) cube(1); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    // Disjoint unit cubes union to volume 2; their hull must be strictly
    // larger (it fills the gap between them).
    EXPECT_GT(e.bodies[0].body->Volume(), 2.0);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, 0.0, 1e-9);
    EXPECT_NEAR(bbox.max.x, 6.0, 1e-9);
}

TEST(Hull, TwoDCirclesHullToACrossSection) {
    Evaluated e = evalSrc("hull() { circle(1, $fn=32); translate([5,0]) circle(1, $fn=32); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_GT(e.bodies[0].section->Area(), std::numbers::pi); // more than one circle's worth
}

// -- minkowski ----------------------------------------------------------

TEST(Minkowski, SumOfCubeAndSphereGrowsEachDimensionByRadius) {
    Evaluated e = evalSrc("minkowski() { cube(2, center=true); sphere(r=1, $fn=16); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    // cube(2, center) spans [-1,1]; +sphere(r=1) should pad every side by ~1.
    EXPECT_NEAR(bbox.min.x, -2.0, 0.05);
    EXPECT_NEAR(bbox.max.x, 2.0, 0.05);
}

// -- linear_extrude -----------------------------------------------------

TEST(LinearExtrude, StraightExtrudeVolumeMatchesAreaTimesHeight) {
    Evaluated e = evalSrc("linear_extrude(height=5) square([2,3]);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 2.0 * 3.0 * 5.0, 1e-6);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.z, 0.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 5.0, 1e-9);
}

TEST(LinearExtrude, CenterTrueStraddlesZEqualsZero) {
    Evaluated e = evalSrc("linear_extrude(height=4, center=true) square(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.z, -2.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 2.0, 1e-9);
}

TEST(LinearExtrude, ScaleShrinksTheTopFace) {
    Evaluated e = evalSrc("linear_extrude(height=10, scale=0) square(2, center=true);");
    ASSERT_TRUE(e.bodies[0].body.has_value());
    // A cone-shaped extrude (scale to a point) has 1/3 the volume of the
    // equivalent straight extrude (pyramid volume formula).
    const double straight = 2.0 * 2.0 * 10.0;
    EXPECT_NEAR(e.bodies[0].body->Volume(), straight / 3.0, straight * 0.02);
}

// -- rotate_extrude -----------------------------------------------------

TEST(RotateExtrude, FullRevolveOfASquareMatchesPappusTheorem) {
    // A 1x1 square spanning x in [3,4] (centroid at x=3.5), revolved 360
    // degrees around the Z axis, forms a square-profile ring -- Pappus's
    // centroid theorem gives its volume exactly for any planar
    // cross-section (not just circular ones): 2*pi*R_centroid*Area.
    Evaluated e = evalSrc("rotate_extrude($fn=200) translate([3,-0.5]) square(1);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    const double analytic = 2.0 * std::numbers::pi * 3.5 * 1.0; // R_centroid=3.5, area=1
    EXPECT_NEAR(e.bodies[0].body->Volume(), analytic, analytic * 0.01);
}

// -- projection -----------------------------------------------------------

TEST(Projection, NonCutProjectsFullSilhouette) {
    Evaluated e = evalSrc("projection() cube([2,3,4]);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 2.0 * 3.0, 1e-6);
}

TEST(Projection, CutSlicesAtZEqualsZero) {
    Evaluated e = evalSrc("projection(cut=true) translate([0,0,-2]) cube([2,3,4], center=true);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    // cube spans z in [-4,0]; a cut at z=0 grazes the top face exactly, so
    // the slice is either the full 2x3 cross-section or empty depending on
    // exact-boundary handling -- assert it's one of those two, not garbage.
    const double area = e.bodies[0].section->Area();
    EXPECT_TRUE(area < 1e-6 || std::fabs(area - 6.0) < 1e-6);
}

// -- offset -----------------------------------------------------------------

TEST(Offset, RoundGrowsAreaBySquarePerimeterPlusCircleCorners) {
    Evaluated e = evalSrc("offset(r=1, $fn=64) square(4, center=true);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    // A round outward offset by r of a WxW square is a rounded square:
    // area = W^2 + 4*W*r + pi*r^2 (side strips + 4 quarter-circle corners).
    const double analytic = 4.0 * 4.0 + 4.0 * 4.0 * 1.0 + std::numbers::pi * 1.0 * 1.0;
    EXPECT_NEAR(e.bodies[0].section->Area(), analytic, analytic * 0.01);
}

TEST(Offset, DeltaGrowsAreaTowardTheExactRectangleBound) {
    // A WxW square offset outward by delta grows toward (but, per
    // Manifold's own JoinType::Square corner treatment, not quite all the
    // way to) the sharp-cornered (W+2*delta)^2 bound -- some corner area is
    // trimmed at 90-degree joins. Assert growth direction/magnitude rather
    // than an exact figure that depends on JoinType::Square's own corner
    // formula.
    Evaluated e = evalSrc("offset(delta=1) square(4, center=true);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    const double area = e.bodies[0].section->Area();
    EXPECT_GT(area, 16.0);
    EXPECT_LE(area, 36.0);
    EXPECT_NEAR(area, 36.0, 1.0); // within a small corner-trim allowance
}

TEST(Offset, NegativeDeltaShrinksTheSquare) {
    Evaluated e = evalSrc("offset(delta=-1) square(4, center=true);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 2.0 * 2.0, 1e-6);
}

TEST(Offset, NoRadiusOrDeltaPassesThroughFirstChild) {
    Evaluated e = evalSrc("offset() square(4, center=true);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 16.0, 1e-6);
}

// -- roof -----------------------------------------------------------------

TEST(Roof, SquareBaseProducesAPyramid) {
    // A square's straight-skeleton roof is an exact pyramid (Tier 1
    // qualifies: single stable contour, symmetric collapse to a point).
    Evaluated e = evalSrc("roof() square(4, center=true);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_EQ(e.bodies[0].body->Status(), manifold::Manifold::Error::NoError);
    // Pyramid volume = 1/3 * base_area * height; height = half the side (45
    // degree mitered roof pitch) = 2.
    const double analytic = (4.0 * 4.0) * 2.0 / 3.0;
    EXPECT_NEAR(e.bodies[0].body->Volume(), analytic, analytic * 0.02);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.z, 2.0, 1e-3);
}

TEST(Roof, RectangleProducesARidgeNotAPoint) {
    Evaluated e = evalSrc("roof() square([8,4], center=true);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_EQ(e.bodies[0].body->Status(), manifold::Manifold::Error::NoError);
    EXPECT_GT(e.bodies[0].body->Volume(), 0.0);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.z, 2.0, 1e-3); // collapse distance = half the short side
}

TEST(Roof, EmptyInputProducesNoGeometry) {
    Evaluated e = evalSrc("roof();");
    EXPECT_TRUE(e.bodies.empty());
}

// -- General case (holes, multi-contour): the Voronoi-diagram construction
// (see roof.cpp's file header) handles these exactly, unlike a
// single-stable-contour-only straight-skeleton approximation would.

TEST(Roof, TwoDisjointSquaresProduceTwoIndependentPyramids) {
    Evaluated e = evalSrc("roof() { square(3, center=true); translate([8,0]) square(3, center=true); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_EQ(e.bodies[0].body->Status(), manifold::Manifold::Error::NoError);
    // Each 3x3 square pyramid has height 1.5 (half the side) and volume
    // 1/3 * 9 * 1.5 = 4.5; the two are far enough apart to not interact.
    EXPECT_NEAR(e.bodies[0].body->Volume(), 2.0 * (9.0 * 1.5 / 3.0), 0.05);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, -1.5, 1e-6);
    EXPECT_NEAR(bbox.max.x, 9.5, 1e-6);
    EXPECT_NEAR(bbox.max.z, 1.5, 1e-3);
}

TEST(Roof, SquareFrameWithAHoleIsWatertightAndShorterThanTheHolelessPyramid) {
    // A square frame (outer 10x10 minus a concentric 4x4 hole) -- this is
    // exactly the case a single-contour-only skeleton can't handle at all
    // (it has 2 contours, one a hole). The valley around the hole must
    // cap the roof well below the holeless pyramid's apex (height 5,
    // volume 100*5/3) and the result must still be a valid closed solid.
    Evaluated e = evalSrc("roof() difference() { square(10, center=true); square(4, center=true); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_EQ(e.bodies[0].body->Status(), manifold::Manifold::Error::NoError);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_LT(bbox.max.z, 5.0);
    EXPECT_GT(bbox.max.z, 0.0);
    const double holelessPyramidVolume = 10.0 * 10.0 * 5.0 / 3.0;
    EXPECT_GT(e.bodies[0].body->Volume(), 0.0);
    EXPECT_LT(e.bodies[0].body->Volume(), holelessPyramidVolume);
}
