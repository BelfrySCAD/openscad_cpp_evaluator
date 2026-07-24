#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

using namespace oscadeval;
using namespace oscadeval::test;

// -- sphere ---------------------------------------------------------------

TEST(Sphere, IsWatertightAndApproximatesAnalyticVolume) {
    Evaluated e = evalSrc("sphere(r=2, $fn=32);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_EQ(e.bodies[0].body->Status(), manifold::Manifold::Error::NoError);
    const double analytic = 4.0 / 3.0 * std::numbers::pi * 2.0 * 2.0 * 2.0; // r=2
    EXPECT_NEAR(e.bodies[0].body->Volume(), analytic, analytic * 0.05);     // polygon approximation, 5% tolerance
}

TEST(Sphere, DiameterArgumentHalvesToRadius) {
    Evaluated withR = evalSrc("sphere(r=3, $fn=16);");
    Evaluated withD = evalSrc("sphere(d=6, $fn=16);");
    EXPECT_NEAR(withR.bodies[0].body->Volume(), withD.bodies[0].body->Volume(), 1e-9);
}

TEST(Sphere, DefaultRadiusIsOne) {
    // The poles are flat polygon caps (not sharp points, matching real
    // OpenSCAD -- see resolveSphere's doc comment), and ring latitudes are
    // offset by half a step from the true pole, so bbox.max/min.z converge
    // to +-r only as $fn grows -- a high segment count is needed for a
    // tight tolerance here, not a bug in the tolerance itself.
    Evaluated e = evalSrc("sphere($fn=256);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.z, 1.0, 1e-3);
    EXPECT_NEAR(bbox.min.z, -1.0, 1e-3);
}

// -- cylinder ---------------------------------------------------------------

TEST(Cylinder, StraightCylinderVolumeAndBoundsWhenCentered) {
    Evaluated e = evalSrc("cylinder(h=4, r=2, center=true, $fn=64);");
    ASSERT_TRUE(e.bodies[0].body.has_value());
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.z, -2.0, 1e-6);
    EXPECT_NEAR(bbox.max.z, 2.0, 1e-6);
    const double analytic = std::numbers::pi * 2.0 * 2.0 * 4.0; // pi r^2 h
    EXPECT_NEAR(e.bodies[0].body->Volume(), analytic, analytic * 0.02);
}

TEST(Cylinder, UncenteredSitsOnZEqualsZero) {
    Evaluated e = evalSrc("cylinder(h=5, r=1, $fn=16);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.z, 0.0, 1e-6);
    EXPECT_NEAR(bbox.max.z, 5.0, 1e-6);
}

TEST(Cylinder, ConeFrustumTopWiderThanBottom) {
    Evaluated e = evalSrc("cylinder(h=2, r1=1, r2=3, $fn=32);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    // Top (z=2) has radius 3, bottom (z=0) has radius 1 -- bbox x/y span
    // reflects the wider end.
    EXPECT_NEAR(bbox.max.x, 3.0, 0.05);
}

TEST(Cylinder, DiameterArgumentsHalveToRadii) {
    Evaluated withR = evalSrc("cylinder(h=1, r1=1, r2=2, $fn=16);");
    Evaluated withD = evalSrc("cylinder(h=1, d1=2, d2=4, $fn=16);");
    EXPECT_NEAR(withR.bodies[0].body->Volume(), withD.bodies[0].body->Volume(), 1e-9);
}

TEST(Cylinder, NoRadiusArgumentAtAllDefaultsToRadiusOne) {
    Evaluated withNoArgs = evalSrc("cylinder(h=5, $fn=16);");
    Evaluated withExplicitR = evalSrc("cylinder(h=5, r=1, $fn=16);");
    EXPECT_NEAR(withNoArgs.bodies[0].body->Volume(), withExplicitR.bodies[0].body->Volume(), 1e-9);
}

TEST(Cylinder, R1OnlyDefaultsR2ToR1) {
    Evaluated withR1Only = evalSrc("cylinder(h=5, r1=3, $fn=16);");
    Evaluated withBoth = evalSrc("cylinder(h=5, r1=3, r2=3, $fn=16);");
    EXPECT_NEAR(withR1Only.bodies[0].body->Volume(), withBoth.bodies[0].body->Volume(), 1e-9);
}

// -- polyhedron -----------------------------------------------------------

TEST(Polyhedron, TetrahedronVolumeMatchesAnalyticFormula) {
    // Right-angle tetrahedron at the origin with legs along x/y/z of length
    // 2: volume = (1/6) * |x * y * z| = (1/6)*2*2*2 = 4/3. Face winding
    // (CW-from-outside, OpenSCAD convention) matches the Python reference's
    // own test_polyhedron_tetrahedron fixture exactly.
    Evaluated e = evalSrc("polyhedron("
                          "points=[[0,0,0],[2,0,0],[0,2,0],[0,0,2]], "
                          "faces=[[0,1,2],[0,3,1],[0,2,3],[1,3,2]]);");
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_EQ(e.bodies[0].body->Status(), manifold::Manifold::Error::NoError);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 4.0 / 3.0, 1e-6);
}

TEST(Polyhedron, MissingPointsOrFacesRaisesEvalError) {
    EXPECT_THROW(evalSrc("polyhedron(points=[[0,0,0]]);"), EvalError);
}

// -- 2D primitives ------------------------------------------------------

TEST(Primitives2d, CircleArea) {
    Evaluated e = evalSrc("circle(r=2, $fn=64);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    const double analytic = std::numbers::pi * 2.0 * 2.0;
    EXPECT_NEAR(e.bodies[0].section->Area(), analytic, analytic * 0.01);
}

TEST(Primitives2d, SquareAreaAndCenteredBounds) {
    Evaluated uncentered = evalSrc("square([3,4]);");
    EXPECT_NEAR(uncentered.bodies[0].section->Area(), 12.0, 1e-9);
    manifold::Rect bounds = uncentered.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.min.x, 0.0, 1e-9);

    Evaluated centered = evalSrc("square([3,4], center=true);");
    manifold::Rect cbounds = centered.bodies[0].section->Bounds();
    EXPECT_NEAR(cbounds.min.x, -1.5, 1e-9);
    EXPECT_NEAR(cbounds.min.y, -2.0, 1e-9);
}

TEST(Primitives2d, PolygonSingleContourArea) {
    // Unit right triangle: area = 0.5.
    Evaluated e = evalSrc("polygon(points=[[0,0],[1,0],[0,1]]);");
    EXPECT_NEAR(e.bodies[0].section->Area(), 0.5, 1e-9);
}

TEST(Primitives2d, PolygonEvenOddFillsClockwiseWinding) {
    // A clockwise-wound square (reverse of the usual CCW order) must still
    // fill under EvenOdd -- the default Positive fill rule would silently
    // produce an empty CrossSection for this winding, which is exactly the
    // BOSL2 teardrop2d() bug the reference's doc calls out.
    Evaluated e = evalSrc("polygon(points=[[0,0],[0,1],[1,1],[1,0]]);");
    EXPECT_NEAR(e.bodies[0].section->Area(), 1.0, 1e-9);
}

TEST(Primitives2d, PolygonWithPathsSelectsIndexedContour) {
    Evaluated e = evalSrc("polygon(points=[[0,0],[2,0],[2,2],[0,2]], paths=[[0,1,2,3]]);");
    EXPECT_NEAR(e.bodies[0].section->Area(), 4.0, 1e-9);
}
