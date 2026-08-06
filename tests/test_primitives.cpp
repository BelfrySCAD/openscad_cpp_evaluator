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

// r2 does NOT fall back to r1 -- each end defaults to 1 on its own, so
// `cylinder(h, r1=3)` is a cone tapering 3 -> 1, not a straight r=3
// cylinder. Verified against OpenSCAD 2022.08.22 (rbot 3, rtop 1).
TEST(Cylinder, R1OnlyLeavesR2AtItsOwnDefaultOfOne) {
    Evaluated withR1Only = evalSrc("cylinder(h=5, r1=3, $fn=16);");
    Evaluated asCone = evalSrc("cylinder(h=5, r1=3, r2=1, $fn=16);");
    Evaluated asStraight = evalSrc("cylinder(h=5, r1=3, r2=3, $fn=16);");
    EXPECT_NEAR(withR1Only.bodies[0].body->Volume(), asCone.bodies[0].body->Volume(), 1e-9);
    EXPECT_GT(std::abs(withR1Only.bodies[0].body->Volume() - asStraight.bodies[0].body->Volume()), 1.0);
}

// Positional order is (h, r1, r2, center) -- position 1 is r1, not r, so a
// third positional argument really does make a cone and a fourth really
// does centre it. This is the arg-order bug that made `cylinder(10, 5, 2)`
// silently render as a straight r=5 cylinder.
TEST(Cylinder, PositionalArgsAreHR1R2Center) {
    Evaluated positional = evalSrc("cylinder(10, 5, 2, true, $fn=16);");
    Evaluated named = evalSrc("cylinder(h=10, r1=5, r2=2, center=true, $fn=16);");
    EXPECT_NEAR(positional.bodies[0].body->Volume(), named.bodies[0].body->Volume(), 1e-9);
    const manifold::Box bb = positional.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bb.min.z, -5.0, 1e-9);
    EXPECT_NEAR(bb.max.z, 5.0, 1e-9);
}

// Application order: r, then d (which overrides r outright), then r1/r2,
// then d1/d2. Each verified against OpenSCAD 2022.08.22.
TEST(Cylinder, DiameterAndRadiusOverrideOrderMatchesReference) {
    // d=8 sets both ends to 4; the explicit r1=5 then overrides the bottom.
    Evaluated dThenR1 = evalSrc("cylinder(h=10, d=8, r1=5, $fn=16);");
    Evaluated expected1 = evalSrc("cylinder(h=10, r1=5, r2=4, $fn=16);");
    EXPECT_NEAR(dThenR1.bodies[0].body->Volume(), expected1.bodies[0].body->Volume(), 1e-9);

    // r=5 sets both ends; the explicit r2=2 then overrides the top.
    Evaluated rThenR2 = evalSrc("cylinder(h=10, r=5, r2=2, $fn=16);");
    Evaluated expected2 = evalSrc("cylinder(h=10, r1=5, r2=2, $fn=16);");
    EXPECT_NEAR(rThenR2.bodies[0].body->Volume(), expected2.bodies[0].body->Volume(), 1e-9);

    // d wins over r outright rather than deferring to it.
    Evaluated dWinsOverR = evalSrc("cylinder(h=10, r=5, d=8, $fn=16);");
    Evaluated expected3 = evalSrc("cylinder(h=10, r=4, $fn=16);");
    EXPECT_NEAR(dWinsOverR.bodies[0].body->Volume(), expected3.bodies[0].body->Volume(), 1e-9);
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

// -- open (non-closed) polyhedron: display-only ---------------------------

namespace {
// A cube with the -x face omitted: five quads, so four boundary edges.
constexpr const char* kOpenCube =
    "polyhedron(points=[[0,0,0],[10,0,0],[10,10,0],[0,10,0],"
    "                   [0,0,10],[10,0,10],[10,10,10],[0,10,10]],"
    "           faces=[[0,1,2,3],[4,7,6,5],[0,4,5,1],[1,5,6,2],[2,6,7,3]]);";
} // namespace

// Manifold cannot represent an open surface, but silently rendering nothing
// left no way to tell a broken polyhedron from a missing one. The triangles
// are kept for display instead, tagged so nothing tries to CSG them.
TEST(Polyhedron, OpenMeshIsKeptForDisplayAndWarns) {
    std::string warning;
    Evaluated e = evalSrc(kOpenCube, [&](const std::string& m) { warning = m; });

    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_TRUE(e.bodies[0].isDisplayOnly());
    EXPECT_TRUE(e.bodies[0].body->IsEmpty());       // no solid, by definition
    EXPECT_EQ(e.bodies[0].rawMesh->triVerts.size(), 10u * 3u);  // 5 quads -> 10 tris

    EXPECT_NE(warning.find("not closed"), std::string::npos) << warning;
    EXPECT_NE(warning.find("4 boundary edge"), std::string::npos) << warning;
}

// The originalID is what maps a picked triangle back to its source line, so
// a display-only body has to carry one or clicking it would select nothing
// -- precisely when a user most wants to be shown the offending code.
TEST(Polyhedron, OpenMeshCarriesAnOriginalIdForPicking) {
    Evaluated e = evalSrc(kOpenCube);
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::MeshGL& mesh = *e.bodies[0].rawMesh;
    ASSERT_EQ(mesh.runOriginalID.size(), 1u);
    EXPECT_EQ(e.ev.idToNode.count(mesh.runOriginalID[0]), 1u);
    ASSERT_EQ(mesh.runIndex.size(), 2u);
    EXPECT_EQ(mesh.runIndex[1], mesh.triVerts.size());
}

// A closed mesh is still a solid however badly it's wound or built -- only
// an OPEN one takes the display-only path. Manifold's own status name for
// that case ("NotManifold") reads as though it covers these too; it doesn't.
TEST(Polyhedron, ClosedButUglyMeshesStillBuildSolids) {
    // All faces reversed (inside-out).
    Evaluated inverted = evalSrc(
        "polyhedron(points=[[0,0,0],[10,0,0],[10,10,0],[0,10,0],"
        "                   [0,0,10],[10,0,10],[10,10,10],[0,10,10]],"
        "           faces=[[3,2,1,0],[5,6,7,4],[1,5,4,0],[2,6,5,1],[3,7,6,2],[0,4,7,3]]);");
    ASSERT_EQ(inverted.bodies.size(), 1u);
    EXPECT_FALSE(inverted.bodies[0].isDisplayOnly());

    // Two tetrahedra meeting at a single (non-manifold) vertex.
    Evaluated pinched = evalSrc(
        "polyhedron(points=[[0,0,0],[10,0,0],[0,10,0],[0,0,10],"
        "                   [0,0,0],[-10,0,0],[0,-10,0],[0,0,-10]],"
        "           faces=[[0,2,1],[0,1,3],[1,2,3],[0,3,2],"
        "                  [4,6,5],[4,5,7],[5,6,7],[4,7,6]]);");
    ASSERT_EQ(pinched.bodies.size(), 1u);
    EXPECT_FALSE(pinched.bodies[0].isDisplayOnly());
}

// A NaN coordinate is NOT drawable: it would poison the scene bounding box
// and send a renderer's camera auto-fit to infinity. Such a mesh keeps the
// old drop-it behaviour -- but now says so instead of vanishing mutely.
TEST(Polyhedron, NonFiniteVertexIsDiscardedNotDisplayed) {
    std::string warning;
    Evaluated e = evalSrc("polyhedron(points=[[0,0,0],[2,0,0],[0,2,0],[0/0,0,2]],"
                          "           faces=[[0,1,2],[0,3,1],[0,2,3],[1,3,2]]);",
                          [&](const std::string& m) { warning = m; });
    for (const ColoredBody& b : e.bodies) EXPECT_FALSE(b.isDisplayOnly());
    EXPECT_NE(warning.find("NonFiniteVertex"), std::string::npos) << warning;
}

// It has no Manifold, so a boolean would have nothing to operate on. It is
// pulled aside like a `%` body and re-joined afterwards, leaving its valid
// siblings to merge normally.
TEST(Polyhedron, OpenMeshIsExcludedFromCsgButSurvivesIt) {
    Evaluated e = evalSrc(std::string("union() { translate([50,0,0]) cube(4); ") + kOpenCube + " }");
    ASSERT_EQ(e.bodies.size(), 2u);

    int solids = 0, displayOnly = 0;
    for (const ColoredBody& b : e.bodies) {
        if (b.isDisplayOnly()) ++displayOnly;
        else if (b.body && !b.body->IsEmpty()) {
            ++solids;
            EXPECT_NEAR(b.body->Volume(), 64.0, 1e-6);  // the cube, unharmed
        }
    }
    EXPECT_EQ(solids, 1);
    EXPECT_EQ(displayOnly, 1);
}
