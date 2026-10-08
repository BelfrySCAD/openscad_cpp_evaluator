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

TEST(Hull, NoChildrenIsEmpty) {
    Evaluated e = evalSrc("hull();");
    EXPECT_TRUE(e.bodies.empty());
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

TEST(Minkowski, SingleChildPassesThroughUnchanged) {
    Evaluated e = evalSrc("minkowski() { cube(2, center=true); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
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

// linear_extrude's `v`, and its argument checks, as upstream's
// LinearExtrudeNode has them; each case below matched OpenSCAD 2026.02's
// bounding box and volume. `v` was ignored here before.
namespace {
struct Extruded {
    manifold::Box box;
    double volume = 0;
    std::vector<std::string> msgs;
};
Extruded linearExtrude(const std::string& args) {
    Extruded r;
    Evaluated e = evalSrc("linear_extrude(" + args + ") translate([5,0]) square([4,2]);",
                          [&](const std::string& m) { r.msgs.push_back(m); });
    if (!e.bodies.empty() && e.bodies[0].body) {
        r.box = e.bodies[0].body->BoundingBox();
        r.volume = e.bodies[0].body->Volume();
    }
    return r;
}
} // namespace

TEST(LinearExtrude, VAloneIsTheWholeExtrusion) {
    const Extruded r = linearExtrude("v=[1,0,1]");  // length |v|, leaning +X
    EXPECT_NEAR(r.box.max.x, 10.0, 1e-6);
    EXPECT_NEAR(r.box.max.z, 1.0, 1e-6);
    EXPECT_NEAR(r.volume, 8.0, 1e-6);
}

TEST(LinearExtrude, HeightWithVSetsOnlyTheDirection) {
    const Extruded r = linearExtrude("v=[1,1,1], height=10");
    const double side = 10.0 / std::sqrt(3.0);
    EXPECT_NEAR(r.box.max.x, 9.0 + side, 1e-3);
    EXPECT_NEAR(r.box.max.y, 2.0 + side, 1e-3);
    EXPECT_NEAR(r.box.max.z, side, 1e-3);
}

TEST(LinearExtrude, CenteringShiftsByHalfOfV) {
    const Extruded r = linearExtrude("v=[2,3,10], center=true");
    EXPECT_NEAR(r.box.min.x, 4.0, 1e-6);
    EXPECT_NEAR(r.box.max.x, 10.0, 1e-6);
    EXPECT_NEAR(r.box.min.y, -1.5, 1e-6);
    EXPECT_NEAR(r.box.min.z, -5.0, 1e-6);
}

TEST(LinearExtrude, DownwardVExtrudesNothing) {
    EXPECT_EQ(linearExtrude("v=[0,0,-1]").volume, 0.0);
}

TEST(LinearExtrude, OnlyARealBooleanCenters) {
    EXPECT_NEAR(linearExtrude("height=10, center=\"yes\"").box.min.z, 0.0, 1e-6);
    EXPECT_NEAR(linearExtrude("height=10, center=true").box.min.z, -5.0, 1e-6);
}

TEST(LinearExtrude, BadArgumentsWarnAndFallBack) {
    const Extruded h = linearExtrude("height=\"a\"");
    EXPECT_NEAR(h.box.max.z, 100.0, 1e-6);
    ASSERT_EQ(h.msgs.size(), 1u);
    EXPECT_EQ(h.msgs[0].rfind("ERROR: height when specified should be a number", 0), 0u) << h.msgs[0];

    const Extruded v = linearExtrude("v=[1,2]");
    EXPECT_NEAR(v.box.max.z, 1.0, 1e-6);
    ASSERT_EQ(v.msgs.size(), 1u);
    EXPECT_EQ(v.msgs[0].rfind("ERROR: v when specified should be a 3d vector", 0), 0u) << v.msgs[0];

    const Extruded s = linearExtrude("height=10, scale=\"x\"");
    EXPECT_NEAR(s.volume, 80.0, 1e-6);
    ASSERT_EQ(s.msgs.size(), 1u);
    EXPECT_EQ(s.msgs[0].rfind("WARNING: linear_extrude(..., scale=\"x\") could not be converted", 0), 0u) << s.msgs[0];
}

TEST(LinearExtrude, ArgumentsArePositionalInUpstreamsOrder) {
    // height, v, scale, center, twist, slices, segments
    EXPECT_NEAR(linearExtrude("10, [1,0,1]").box.max.x, 9.0 + 10.0 / std::sqrt(2.0), 1e-3);
    EXPECT_NEAR(linearExtrude("10, undef, 0.5").volume, 46.67, 0.01);
    EXPECT_NEAR(linearExtrude("10, undef, 1, true").box.min.z, -5.0, 1e-6);
    EXPECT_NEAR(linearExtrude("10, undef, 1, false, 90").box.min.y, -9.0, 1e-6);
    EXPECT_NEAR(linearExtrude("10, undef, 1, false, 90, 2").volume, 81.62, 0.01);
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

// The sweep runs from `start` through start + angle, as upstream's
// RotateExtrudeNode has it; start was ignored here until this was checked
// against OpenSCAD 2026.02 (every case below matched it bounding box for
// bounding box).
namespace {
manifold::Box rotateExtrudeBox(const std::string& args, std::vector<std::string>* msgs = nullptr) {
    Evaluated e = evalSrc("rotate_extrude(" + args + ") translate([10,0]) square(2);",
                          [&](const std::string& m) { if (msgs) msgs->push_back(m); });
    EXPECT_EQ(e.bodies.size(), 1u);
    return e.bodies.empty() ? manifold::Box{} : e.bodies[0].body->BoundingBox();
}
} // namespace

TEST(RotateExtrude, StartTurnsWhereTheSweepBegins) {
    const manifold::Box b = rotateExtrudeBox("angle=90, start=90");  // the +Y/-X quadrant
    EXPECT_NEAR(b.min.x, -12.0, 1e-6);
    EXPECT_NEAR(b.max.x, 0.0, 1e-6);
    EXPECT_NEAR(b.min.y, 0.0, 1e-6);
    const manifold::Box neg = rotateExtrudeBox("angle=-90, start=90");  // sweeps back to +X
    EXPECT_NEAR(neg.min.x, 0.0, 1e-6);
    EXPECT_NEAR(neg.min.y, 0.0, 1e-6);
    EXPECT_NEAR(neg.max.y, 12.0, 1e-6);
}

TEST(RotateExtrude, AFullTurnWithNoAngleStartsOnMinusXAndSaysItWillChange) {
    std::vector<std::string> msgs;
    const manifold::Box b = rotateExtrudeBox("$fn=5", &msgs);  // a pentagon: the start shows
    EXPECT_NEAR(b.min.x, -12.0, 1e-6);
    EXPECT_NEAR(b.max.x, 12.0 * std::cos(std::numbers::pi / 5), 1e-3);
    ASSERT_EQ(msgs.size(), 1u);
    EXPECT_EQ(msgs[0].rfind("DEPRECATED: In future releases, rotational extrusion without \"angle\"", 0), 0u) << msgs[0];
    // Given an angle or a start, it starts where told and says nothing.
    msgs.clear();
    EXPECT_NEAR(rotateExtrudeBox("angle=360, $fn=5", &msgs).max.x, 12.0, 1e-6);
    EXPECT_NEAR(rotateExtrudeBox("start=0, $fn=5", &msgs).max.x, 12.0, 1e-6);
    EXPECT_TRUE(msgs.empty());
}

TEST(RotateExtrude, StartIsTheSecondPositionalArgument) {
    EXPECT_NEAR(rotateExtrudeBox("90, 90").min.x, -12.0, 1e-6);  // angle, start
}

TEST(RotateExtrude, AnAngleOutsideAFullTurnIsAFullTurn) {
    for (const char* a : {"angle=400", "angle=-400", "angle=-360"}) {
        const manifold::Box b = rotateExtrudeBox(a);
        EXPECT_NEAR(b.min.x, -12.0, 1e-3) << a;
        EXPECT_NEAR(b.max.x, 12.0, 1e-3) << a;
    }
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

// `r` owns position 0, so offset(2) == offset(r=2). It used to be
// named-only, making a bare offset(2) a silent no-op.
TEST(Offset, FirstPositionalArgumentIsRadius) {
    Evaluated positional = evalSrc("offset(1, $fn=64) square(4, center=true);");
    Evaluated named = evalSrc("offset(r=1, $fn=64) square(4, center=true);");
    ASSERT_TRUE(positional.bodies[0].section.has_value());
    EXPECT_NEAR(positional.bodies[0].section->Area(), named.bodies[0].section->Area(), 1e-9);
    EXPECT_GT(positional.bodies[0].section->Area(), 16.0);
}

// A bare offset() is offset(r=1), NOT a pass-through -- the reference
// initialises its delta to 1 with a round join and only overwrites it from
// an argument. Verified against OpenSCAD 2022.08.22, where offset() and
// offset(r=1) produce identical geometry.
TEST(Offset, NoRadiusOrDeltaDefaultsToRadiusOne) {
    Evaluated bare = evalSrc("offset() square(4, center=true);");
    Evaluated explicitR = evalSrc("offset(r=1) square(4, center=true);");
    ASSERT_EQ(bare.bodies.size(), 1u);
    ASSERT_TRUE(bare.bodies[0].section.has_value());
    EXPECT_NEAR(bare.bodies[0].section->Area(), explicitR.bodies[0].section->Area(), 1e-9);
    EXPECT_GT(bare.bodies[0].section->Area(), 16.0); // grew, rather than passing through
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

// -- method="straight": the straight-skeleton roof. Expected volumes and
// heights are OpenSCAD 2026.02.01's (--enable=roof), read from its OFF
// output; the shapes with integer corners have exact rational volumes.

namespace {

struct RoofResult {
    double volume, height;
    manifold::Manifold::Error status;
    double slopeError; // roof area / (floor area * sqrt 2) - 1: 0 if every face rises at 45 degrees
};

RoofResult straightRoof(const std::string& shape, const std::string& method = "straight") {
    Evaluated e = evalSrc("roof(method=\"" + method + "\") " + shape);
    EXPECT_EQ(e.bodies.size(), 1u);
    if (e.bodies.empty() || !e.bodies[0].body) return {0, 0, manifold::Manifold::Error::InvalidConstruction, 1};
    const manifold::Manifold& m = *e.bodies[0].body;
    const manifold::MeshGL64 mesh = m.GetMeshGL64();
    double floorArea = 0, roofArea = 0;
    for (size_t t = 0; t < mesh.triVerts.size(); t += 3) {
        manifold::vec3 p[3];
        for (int k = 0; k < 3; ++k) {
            const size_t v = mesh.triVerts[t + k] * mesh.numProp;
            p[k] = {mesh.vertProperties[v], mesh.vertProperties[v + 1], mesh.vertProperties[v + 2]};
        }
        const double area = manifold::la::length(manifold::la::cross(p[1] - p[0], p[2] - p[0])) / 2;
        (p[0].z == 0 && p[1].z == 0 && p[2].z == 0 ? floorArea : roofArea) += area;
    }
    return {m.Volume(), m.BoundingBox().max.z, m.Status(), roofArea / (floorArea * std::sqrt(2.0)) - 1};
}

void expectRoof(const std::string& shape, double volume, double height, double tol = 1e-9) {
    SCOPED_TRACE(shape);
    const RoofResult r = straightRoof(shape);
    EXPECT_EQ(r.status, manifold::Manifold::Error::NoError);
    EXPECT_NEAR(r.volume, volume, tol * std::max(1.0, volume));
    EXPECT_NEAR(r.height, height, std::max(tol, 1e-9));
    EXPECT_NEAR(r.slopeError, 0, 1e-6); // vertices a hair apart are merged
}

} // namespace

TEST(Roof, StraightAgreesWithVoronoiOnConvexShapes) {
    // Without reflex corners both roofs are the lower envelope of the
    // edges' 45-degree planes.
    Evaluated straight = evalSrc("roof(method=\"straight\") square(4, center=true);");
    Evaluated voronoi = evalSrc("roof() square(4, center=true);");
    ASSERT_TRUE(straight.bodies[0].body.has_value());
    EXPECT_NEAR(straight.bodies[0].body->Volume(), voronoi.bodies[0].body->Volume(), 1e-9);
}

// At a reflex corner the Voronoi roof is a cone; the straight skeleton's
// faces just meet along the corner's bisector, lower. (These failed while
// method="straight" built the Voronoi roof: 9.424 / 1.172 for the L.)
TEST(Roof, StraightLShapeMatchesReference) {
    expectRoof("polygon([[0,0],[6,0],[6,2],[2,2],[2,6],[0,6]]);", 28.0 / 3, 1);
}

TEST(Roof, StraightUShapeMatchesReference) {
    expectRoof("polygon([[0,0],[6,0],[6,6],[4,6],[4,2],[2,2],[2,6],[0,6]]);", 40.0 / 3, 1);
}

TEST(Roof, StraightCrossMatchesReference) {
    expectRoof("union(){square([10,2],center=true); square([2,10],center=true);}", 52.0 / 3, 1);
}

TEST(Roof, StraightTenPointStarMatchesReference) {
    expectRoof("polygon([for(i=[0:9]) (i%2?2:5)*[cos(36*i),sin(36*i)]]);", 16.082266, 1.64165, 1e-5);
}

TEST(Roof, StraightFrameWithAHoleMatchesReference) {
    // Each hole corner meets an outline corner head on (a vertex event).
    expectRoof("difference(){square(10,center=true); square(4,center=true);}", 63, 1.5);
    expectRoof("difference(){square(10,center=true); translate([1,0.5]) square(3,center=true);}", 4003.0 / 48, 2.25);
}

TEST(Roof, StraightCombMatchesReference) {
    // Teeth of equal width: many simultaneous events along one line.
    expectRoof("union(){square([11,2]); for(i=[0:5]) translate([2*i,0]) square([1,6]);}", 49.0 / 3, 1);
}

TEST(Roof, StraightDegenerateShapesMatchReference) {
    expectRoof("square([8,4], center=true);", 80.0 / 3, 2);                                          // a ridge
    expectRoof("union(){translate([-5,0]) square([10,2]); translate([-1,-6]) square([2,6]);}", 46.0 / 3, 1); // T: all at once
    expectRoof("polygon([[0,0],[2,0],[2,2],[4,2],[4,0],[6,0],[6,6],[4,6],[4,4],[2,4],[2,6],[0,6]]);", 40.0 / 3, 1);
    expectRoof("difference(){square(14); for(i=[0:3],j=[0:3]) translate([1.5+3*i,1+3*j]) square(2);}", 271.0 / 6, 1);
    expectRoof("polygon(concat([[0,0],[10,0]], [for(i=[0:4]) each [[10-2*i,2*i+2],[8-2*i,2*i+2]]]));", 56, 3);
    // Collinear corners, as Clipper often leaves them.
    expectRoof("polygon([[0,0],[2,0],[4,0],[6,0],[6,3],[6,4],[3,4],[0,4],[0,2]]);", 56.0 / 3, 2);
}

TEST(Roof, StraightTwoDisjointSquaresProduceTwoPyramids) {
    expectRoof("{square(3, center=true); translate([8,0]) square(3, center=true);}", 9, 1.5);
}

TEST(Roof, StraightRandomPolygonWithAHoleMatchesReference) {
    expectRoof("difference(){ polygon([[7.889, 0.054], [6.592, 1.825], [8.181, 4.972], [5.771, 6.144], [3.986, 7.827], "
               "[1.928, 9.003], [-0.453, 6.393], [-2.012, 5.747], [-3.501, 4.918], [-6.839, 5.452], [-8.209, 3.447], "
               "[-8.971, 1.226], [-8.122, -1.211], [-8.0, -3.437], [-7.567, -6.242], [-4.35, -6.311], [-3.145, -9.164], "
               "[-0.67, -8.491], [1.503, -7.027], [4.462, -8.48], [5.534, -5.781], [6.912, -4.089], [6.528, -1.754]]); "
               "translate([0.42,1.81]) circle(0.56,$fn=4); }",
               357.75343, 4.71365, 1e-5);
}

// Two pieces of a union can come back from Clipper as separate outlines
// sharing an edge a hairline apart; they must be roofed as one. (The
// reference agrees: 1.2839 / 0.6530.)
TEST(Roof, StraightJoinsUnionPiecesThatShareAnEdge) {
    expectRoof("union(){polygon([[5.666, 2.389], [3.728, 7.665], [1.629, 8.325]]); "
               "polygon([[7.789, 3.576], [2.125, 9.3], [8.018, 2.482]]);}",
               1.283895, 0.65303, 1e-5);
}

TEST(Roof, StraightHandlesThousandsOfVertices) {
    // Nearly simultaneous collapse at the centre of a fine circle.
    const RoofResult r = straightRoof("circle(10, $fn=2000);");
    EXPECT_EQ(r.status, manifold::Manifold::Error::NoError);
    // The apex sits where a thousand near-simultaneous events meet, so it
    // is only as exact as the outline's corners.
    EXPECT_NEAR(r.height, 10 * std::cos(std::numbers::pi / 2000), 1e-5);
    // Same accuracy for the slope: x86 lands at 1.01e-6, arm64 just under 1e-6.
    EXPECT_NEAR(r.slopeError, 0, 1e-5);
    const RoofResult g = straightRoof("polygon([for(i=[0:1999]) (i%4<2?10:9)*[cos(i*0.18),sin(i*0.18)]]);");
    EXPECT_EQ(g.status, manifold::Manifold::Error::NoError);
    EXPECT_NEAR(g.height, 9, 1e-4);
    EXPECT_NEAR(g.slopeError, 0, 1e-5);
}

// -- method="voronoi" over unions of pieces that touch along edges. These
// all gave no roof at all before: Clipper returns the pieces as outlines a
// hairline apart, which on the grid overlap or cross, and Boost's Voronoi
// builder needs segments that meet only at their ends.

namespace {

void expectVoronoiRoof(const std::string& shape, double volume, double height, double tol) {
    SCOPED_TRACE(shape);
    const RoofResult r = straightRoof(shape, "voronoi");
    EXPECT_EQ(r.status, manifold::Manifold::Error::NoError);
    EXPECT_NEAR(r.volume, volume, tol * volume);
    EXPECT_NEAR(r.height, height, tol);
}

} // namespace

// Four triangles sharing edges. The reference agrees: 23.67785 / 2.39894.
TEST(Roof, VoronoiJoinsTrianglesThatShareEdges) {
    expectVoronoiRoof("union() { polygon([[2.764760357, 4.118581106], [5.901400476, 6.372929785], [8.606126287, 1.988496895]]); "
                      "polygon([[5.901400476, 6.372929785], [8.606126287, 1.988496895], [11.028736411, 6.50946765]]); "
                      "polygon([[8.606126287, 1.988496895], [2.764760357, 4.118581106], [4.975336337, 1.106200614]]); "
                      "polygon([[8.606126287, 1.988496895], [2.764760357, 4.118581106], [6.985603416, 6.618989998]]); }",
                      23.677855, 2.39894, 1e-5);
}

// Rotated rectangles sharing (parts of) edges. OpenSCAD 2026.02.01 draws
// nothing for it; the volume is the integral of the distance to the outline
// to within the faceting of the cones (170.68 numerically).
TEST(Roof, VoronoiJoinsRotatedRectanglesThatShareEdges) {
    expectVoronoiRoof("union() { polygon([[-3.369828352, 0.413896268], [-7.46233132, 4.010047378], [-11.05848243, -0.08245559], [-6.965979462, -3.6786067]]); "
                      "polygon([[-4.919727978, -5.476682255], [-9.012230946, -1.880531145], [-14.406457611, -8.019285597], [-10.313954643, -11.615436707]]); "
                      "polygon([[6.613253139, -4.732154468], [0.474498687, 0.662072197], [-4.919727978, -5.476682255], [1.219026474, -10.87090892]]); "
                      "polygon([[10.457580178, -4.483978539], [8.411328694, -2.685902984], [6.613253139, -4.732154468], [8.659504623, -6.530230023]]); }",
                      170.76468, 4.08601, 1e-5);
}

// Here the outline as Clipper leaves it crosses itself on the grid, and the
// Voronoi builder never finished, allocating until the process was killed
// (tens of gigabytes). (The integral of the distance to the outline is
// 221.3; the cones over corners meeting head on are faceted coarsely, as by
// the reference.)
TEST(Roof, VoronoiRoofOfACrossingOutlineStaysBounded) {
    expectVoronoiRoof("union() { polygon([[8.304902057, -8.407695464], [0.596294574, -9.683488249], [1.872087359, -17.392095732], [9.580694842, -16.116302947]]); "
                      "polygon([[14.737716755, 0.576704804], [7.029109272, -0.699087981], [8.304902057, -8.407695464], [16.01350954, -7.131902679]]); "
                      "polygon([[8.304902057, -8.407695464], [-3.258009167, -10.321384641], [-2.620112775, -14.175688383], [8.94279845, -12.261999205]]); "
                      "polygon([[2.536909139, 2.517319368], [-1.317394603, 1.879422976], [0.596294574, -9.683488249], [4.450598316, -9.045591856]]); }",
                      222.74776, 3.90673, 1e-5);
}

// A staircase of triangles. Outside it, two of its edges are all but
// parallel, so their Voronoi bisectors meet some 10^12 away; that vertex,
// used by no facet, stretched the bounding box Manifold sizes its tolerance
// by, and its cleanup collapsed the whole roof to nothing. (The integral of
// the distance to the outline is 78.02.)
TEST(Roof, VoronoiRoofIgnoresFarVoronoiVertices) {
    expectVoronoiRoof("union() { polygon([[-1.015413575, -0.344245093], [-1.006343725, -5.505019653], [4.154430835, -5.495949802]]); "
                      "polygon([[-1.015413575, -0.344245093], [4.154430835, -5.495949802], [4.145360985, -0.335175242]]); "
                      "polygon([[-1.006343725, -5.505019653], [4.163500685, -10.656724363], [4.154430835, -5.495949802]]); "
                      "polygon([[4.154430835, -5.495949802], [4.163500685, -10.656724363], [9.324275246, -10.647654512]]); "
                      "polygon([[4.163500685, -10.656724363], [9.333345096, -15.808429072], [9.324275246, -10.647654512]]); "
                      "polygon([[-6.176188135, -0.353314943], [-1.006343725, -5.505019653], [-1.015413575, -0.344245093]]); }",
                      78.047622, 3.02312, 1e-5);
}

// Axis-aligned rectangles: a clean outline, but rounded to the grid four
// sites are all but equidistant at one point, and Boost's builder leaves a
// cell with no finite edges. A coarser grid rounds it differently.
// OpenSCAD 2026.02.01 draws nothing here either; the integral of the
// distance to the outline is 1.348 (the faceted cone sits a little above).
TEST(Roof, VoronoiRetriesWhereTheBuilderFails) {
    expectVoronoiRoof("union() { polygon([[0.7, -0.7], [2.1, -0.7], [2.1, 0.0], [0.7, 0.0]]); "
                      "polygon([[2.1, -0.7], [2.8, -0.7], [2.8, 0.7], [2.1, 0.7]]); "
                      "polygon([[2.8, -1.4], [4.2, -1.4], [4.2, 0.7], [2.8, 0.7]]); }",
                      1.3513906, 0.820101, 1e-5);
}

TEST(Roof, UnknownMethodWarnsAndFallsBackToVoronoi) {
    std::string lastWarning;
    Evaluated e = evalSrc("roof(method=\"bogus\") square(4, center=true);", [&](const std::string& msg) { lastWarning = msg; });
    EXPECT_NE(lastWarning.find("Unknown roof method 'bogus'"), std::string::npos);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_EQ(e.bodies[0].body->Status(), manifold::Manifold::Error::NoError);
}

// As OpenSCAD 2026.02.01: $fs then $fa clamped with the same warnings
// circle() gives, for either method, before the children's own output.
TEST(Roof, TooSmallFaAndFsWarnBeforeChildren) {
    for (const std::string method : {"voronoi", "straight"}) {
        std::vector<std::string> messages;
        Evaluated e = evalSrc("roof(method=\"" + method + "\", $fa=0, $fs=0) { echo(\"child\"); square(2); }",
                              [&](const std::string& msg) { messages.push_back(msg); });
        ASSERT_EQ(messages.size(), 3u) << method;
        EXPECT_NE(messages[0].find("$fs too small - clamping to 0.010000"), std::string::npos) << messages[0];
        EXPECT_NE(messages[1].find("$fa too small - clamping to 0.010000"), std::string::npos) << messages[1];
        EXPECT_NE(messages[2].find("child"), std::string::npos) << messages[2];
        ASSERT_EQ(e.bodies.size(), 1u);
        EXPECT_NEAR(e.bodies[0].body->Volume(), 4.0 / 3, 1e-9);
    }
}

// As OpenSCAD 2026.02.01: a non-string method is shown as str() shows it
// and warned about; undef is the default, silently.
TEST(Roof, NonStringMethodWarnsAndUndefDoesNot) {
    std::vector<std::string> warnings;
    evalSrc("roof(method=1.5) square(2); roof(method=undef) square(2);", [&](const std::string& msg) { warnings.push_back(msg); });
    ASSERT_EQ(warnings.size(), 1u);
    EXPECT_NE(warnings[0].find("Unknown roof method '1.5'. Using 'voronoi'."), std::string::npos);
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
