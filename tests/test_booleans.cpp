#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

// -- difference / intersection -------------------------------------------

TEST(Difference, SubtractsInnerCubeVolume) {
    // Outer [0,2]^3 (vol 8) minus an inner cube fully inside it, [0.5,1.5]^3
    // (vol 1) -> 7.
    Evaluated e = evalSrc("difference() { cube(2); translate([0.5,0.5,0.5]) cube(1); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 7.0, 1e-6);
}

TEST(Difference, MultipleBodiesInFirstStatementFormOnePositiveOperand) {
    // Two disjoint cubes as the FIRST statement's combined bodies (unioned
    // together, vol 16) minus a third cube overlapping only the first one
    // by 1 -- a flat (non-grouped) evaluation would instead treat the
    // second cube as a subtractor of the first, giving the wrong volume.
    Evaluated e = evalSrc("difference() {"
                          "  union() { cube(2); translate([10,0,0]) cube(2); }"
                          "  translate([1,0,0]) cube(2);"
                          "}");
    ASSERT_EQ(e.bodies.size(), 1u);
    // 16 - overlap(1*2*2=4) = 12
    EXPECT_NEAR(e.bodies[0].body->Volume(), 12.0, 1e-6);
}

TEST(Intersection, KeepsOnlyOverlappingVolume) {
    Evaluated e = evalSrc("intersection() { cube(2); translate([1,1,1]) cube(2); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0, 1e-6);
}

TEST(Intersection, DisabledStatementIsTrulyEmptyAndDiscardsResult) {
    Evaluated e = evalSrc("intersection() { cube(2); *cube(1); }");
    // *cube(1) produces no CSGNode at all -> its "statement" contributes a
    // group of size 0 -- flattening an empty group yields no bodies, which
    // intersection() treats as "intersection with nothing" -> empty result.
    EXPECT_TRUE(e.bodies.empty());
}

TEST(Union, NoChildrenIsEmpty) {
    Evaluated e = evalSrc("union();");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(Union, SingleChildPassesThroughUnchanged) {
    Evaluated e = evalSrc("union() { cube(2); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
}

TEST(Union, MultipleBodiesInFirstStatementAllSurvive) {
    // Same "a statement can itself contribute multiple bodies" shape as
    // Difference's own regression test above, but for union() -- both
    // disjoint cubes from the first statement's own union() must appear in
    // the outer union's result, not just the first one.
    Evaluated e = evalSrc("union() {"
                          "  union() { cube(2); translate([10,0,0]) cube(2); }"
                          "  translate([1,0,0]) cube(2);"
                          "}");
    ASSERT_EQ(e.bodies.size(), 1u);
    // [0,2]+[1,3] unioned = [0,3] (vol 3*2*2=12), plus disjoint [10,12] (vol 8) -> 20.
    EXPECT_NEAR(e.bodies[0].body->Volume(), 20.0, 1e-6);
}

TEST(Intersection, EmptyFirstOperandGivesEmpty) {
    Evaluated e = evalSrc("intersection() { *cube(10); cube(2); }");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(Difference, EmptyFirstOperandGivesEmpty) {
    Evaluated e = evalSrc("difference() { *cube(10); cube(2); }");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(Difference, EmptySubtractorLeavesBaseUnchanged) {
    Evaluated e = evalSrc("difference() { cube(4); *cube(10); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 64.0, 1e-6);
}

TEST(Union, OneInvalidOperandDoesNotDiscardValidSiblings) {
    // Regression: Manifold's own `+` propagates a non-NoError Status()
    // (e.g. NonFiniteVertex, from a degenerate NaN-vertex polyhedron here --
    // in the wild, from an accumulated transform deep in an unrelated
    // ancestor's own positioning math) onto the WHOLE combined result
    // instead of treating the bad operand as a no-op contributor. Before
    // filtering invalid operands out in generateCsg, a single bad part
    // anywhere in a many-operand union() silently discarded every valid
    // sibling too -- found via a real user project (snappy-reprap) where
    // this zeroed out an entire ~65-part extruder assembly with no warning.
    Evaluated e = evalSrc("union() {"
                          "  polyhedron(points=[[0,0,0],[2,0,0],[0,2,0],[0/0,0,2]],"
                          "             faces=[[0,1,2],[0,3,1],[0,2,3],[1,3,2]]);"
                          "  translate([10,0,0]) cube(2);"
                          "}");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6); // just the valid cube -- the NaN polyhedron is dropped, not fatal
}

// -- 2D-native boolean CSG (union/difference/intersection applied directly
// to 2D children, not mediated through linear_extrude()) -----------------

TEST(TwoDeeBoolean, UnionOfTwoOverlappingCircles) {
    Evaluated e = evalSrc("union() { circle(2, $fn=64); translate([1,0]) circle(2, $fn=64); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    const double singleCircle = 3.14159265358979 * 2.0 * 2.0;
    // Union of two overlapping same-size circles is strictly less than the
    // sum of their areas but strictly more than either alone.
    EXPECT_GT(e.bodies[0].section->Area(), singleCircle);
    EXPECT_LT(e.bodies[0].section->Area(), singleCircle * 2.0);
}

TEST(TwoDeeBoolean, DifferenceSubtractsCircleFromSquare) {
    // square(4, center=true) spans [-2,2]^2; circle(r=1) at the origin is
    // fully inside it, so the whole disc is removed.
    Evaluated e = evalSrc("difference() { square(4, center=true); circle(1, $fn=64); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    const double circleArea = 3.14159265358979 * 1.0 * 1.0;
    EXPECT_NEAR(e.bodies[0].section->Area(), 16.0 - circleArea, 0.05);
}

TEST(TwoDeeBoolean, IntersectionOfSquareAndCircle) {
    // square(4, center=true) spans [-2,2]^2; a circle(r=2) at the origin is
    // fully inside it too, so the intersection is the whole disc.
    Evaluated e = evalSrc("intersection() { square(4, center=true); circle(2, $fn=64); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    const double circleArea = 3.14159265358979 * 2.0 * 2.0;
    EXPECT_NEAR(e.bodies[0].section->Area(), circleArea, 0.05);
}

TEST(TwoDeeBoolean, ResultCanBeExtrudedAfterward) {
    Evaluated e = evalSrc("linear_extrude(height=3) union() { circle(2, $fn=32); translate([1,0]) circle(2, $fn=32); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.z - bbox.min.z, 3.0, 1e-6);
}

// -- modifiers (#/%/!/*) --------------------------------------------------

TEST(Modifiers, DisableProducesNoGeometryAndNoCsgNode) {
    Evaluated e = evalSrc("*cube(1); cube(2);");
    ASSERT_EQ(e.tree.size(), 1u); // only cube(2) got a tree node at all
    EXPECT_EQ(e.tree[0]->kind, "cube");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-9); // cube(2)
}

TEST(Modifiers, HighlightTagsRoleButKeepsGeometry) {
    Evaluated e = evalSrc("#cube(2);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(e.bodies[0].role, BodyRole::Highlight);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-9);
}

TEST(Modifiers, BackgroundIsExcludedFromBooleanMergeButStillReturned) {
    Evaluated e = evalSrc("difference() { cube(2); %translate([0.5,0.5,0.5]) cube(1); }");
    ASSERT_EQ(e.bodies.size(), 2u);
    const bool hasUnalteredCube =
        std::any_of(e.bodies.begin(), e.bodies.end(),
                    [](const ColoredBody& b) { return b.role == BodyRole::Normal && std::abs(b.body->Volume() - 8.0) < 1e-6; });
    const bool hasBackground = std::any_of(e.bodies.begin(), e.bodies.end(),
                                            [](const ColoredBody& b) { return b.role == BodyRole::Background; });
    EXPECT_TRUE(hasUnalteredCube); // the %-tagged cube never subtracted -- difference() saw an empty positive-op group
    EXPECT_TRUE(hasBackground);
}

TEST(Modifiers, TopLevelShowOnlyLeavesNothingButItsOwnSubtree) {
    Evaluated e = evaluateSrc("!cube(1); cube(2); #cube(3);");
    // `!` makes its subtree the whole model, so both siblings go -- the
    // highlighted one included. This test previously expected cube(3) to
    // survive on the strength of its role; the reference implementation
    // renders only cube(1) here (checked against 2021.01).
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(e.bodies[0].role, BodyRole::ShowOnly);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0, 1e-9);
}

// The reason a body-role filter could not do this: by the time a body is
// tagged, every operation wrapped around it has already been applied.
TEST(Modifiers, ShowOnlyDiscardsTheOperationsWrappedAroundIt) {
    Evaluated moved = evaluateSrc("translate([50,0,0]) !cube(5);");
    ASSERT_EQ(moved.bodies.size(), 1u);
    const manifold::Box box = moved.bodies[0].body->BoundingBox();
    EXPECT_NEAR(box.min.x, 0.0, 1e-9) << "the translate is an ancestor, so it does not apply";
    EXPECT_NEAR(box.max.x, 5.0, 1e-9);
}

// The case that prompted this: an extrude is an ancestor like any other,
// so what is left is the 2D circle rather than the cylinder.
TEST(Modifiers, ShowOnlyInsideAnExtrudeLeavesTheProfile) {
    Evaluated e = evaluateSrc("linear_extrude(height=10) !circle(10);");
    ASSERT_EQ(e.bodies.size(), 1u);
    // What survives is 2D: a section, not a solid. Extruding it is what
    // the discarded ancestor would have done.
    ASSERT_TRUE(e.bodies[0].section.has_value()) << "extruded into a solid";
    const manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.max.x, 10.0, 0.1);
    EXPECT_NEAR(bounds.min.x, -10.0, 0.1);
}

TEST(Modifiers, MoreThanOneShowOnlyTakesTheFirstAndWarns) {
    std::vector<std::string> echoed;
    Evaluated e = evaluateSrc("translate([50,0,0]) !cube(5);\n!sphere(3);",
                              [&echoed](const std::string& m) { echoed.push_back(m); });
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::Box box = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(box.max.x, 5.0, 1e-9) << "the cube, not the sphere";
    EXPECT_TRUE(std::any_of(echoed.begin(), echoed.end(), [](const std::string& m) {
        return m.find("More than one Root Modifier") != std::string::npos;
    })) << "no warning in: " << (echoed.empty() ? "(nothing)" : echoed.front());
}

TEST(Modifiers, NoShowOnlyMeansAllRolesPassThrough) {
    Evaluated e = evaluateSrc("%cube(1); cube(2);");
    EXPECT_EQ(e.bodies.size(), 2u);
}

// -- polyhedron face triangulation ---------------------------------------

// An L-shaped prism: its top and bottom are 6-gons with one reflex corner.
// Fan-triangulating a concave face lays triangles outside the polygon and
// winds some of them inside out, so the solid comes out the wrong size --
// which is how BOSL2's nurbs_sheet() end caps ended up covering 281% of
// their own area with 15 of 32 triangles reversed.
// A U: vertex 0 cannot see the far arm, so a fan anchored there lays
// triangles across the notch, outside the solid. (An L is not enough on
// its own -- it happens to be star-shaped from its first vertex, so even
// a fan triangulates it correctly, which is why the volume check below
// needs this shape too.)
static const char* kUPrism = R"(
pts = [[0,0],[3,0],[3,3],[2,3],[2,1],[1,1],[1,3],[0,3]];
polyhedron(
  points = concat([for (p=pts) [p[0],p[1],0]], [for (p=pts) [p[0],p[1],1]]),
  faces = concat(
    [[for (i=[0:len(pts)-1]) i]],
    [[for (i=[len(pts)-1:-1:0]) i+len(pts)]],
    [for (i=[0:len(pts)-1]) [i, i+len(pts), (i+1)%len(pts)+len(pts), (i+1)%len(pts)]]
  )
);
)";

static const char* kLPrism = R"(
polyhedron(
    points = [
        [0,0,0], [2,0,0], [2,1,0], [1,1,0], [1,2,0], [0,2,0],
        [0,0,1], [2,0,1], [2,1,1], [1,1,1], [1,2,1], [0,2,1]
    ],
    faces = [
        [0,1,2,3,4,5],
        [11,10,9,8,7,6],
        [0,6,7,1], [1,7,8,2], [2,8,9,3],
        [3,9,10,4], [4,10,11,5], [5,11,6,0]
    ]
);
)";

TEST(PolyhedronFaces, AConcaveFaceGivesTheRightVolume) {
    Evaluated e = evaluateSrc(kLPrism);
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    // The L is 3 square units, extruded 1 high.
    EXPECT_NEAR(e.bodies[0].body->Volume(), 3.0, 1e-9);
}

TEST(PolyhedronFaces, AConcaveFaceGivesTheRightSurfaceArea) {
    Evaluated e = evaluateSrc(kLPrism);
    ASSERT_EQ(e.bodies.size(), 1u);
    // 2 x 3 for the ends, plus a perimeter of 8 x height 1.
    EXPECT_NEAR(e.bodies[0].body->SurfaceArea(), 14.0, 1e-9);
}

TEST(PolyhedronFaces, EachFaceBecomesExactlyNMinusTwoTriangles) {
    Evaluated e = evaluateSrc(kLPrism);
    ASSERT_EQ(e.bodies.size(), 1u);
    // Two 6-gons (4 each) and six quads (2 each).
    const manifold::MeshGL mesh = e.bodies[0].body->GetMeshGL();
    EXPECT_EQ(mesh.triVerts.size() / 3, 4u + 4u + 6u * 2u);
}

// A convex face has to keep working, and a triangle must not be disturbed.
TEST(PolyhedronFaces, AFaceWithAnUnseeableCornerGivesTheRightVolume) {
    Evaluated e = evaluateSrc(kUPrism);
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    // 3x3 less the 1x2 notch, one unit high.
    EXPECT_NEAR(e.bodies[0].body->Volume(), 7.0, 1e-9);
    // Two ends of 7, plus a 16-long perimeter one unit high.
    EXPECT_NEAR(e.bodies[0].body->SurfaceArea(), 30.0, 1e-9);
}

TEST(PolyhedronFaces, AConvexFaceIsUnchanged) {
    Evaluated e = evaluateSrc("cube(2);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-9);
    EXPECT_NEAR(e.bodies[0].body->SurfaceArea(), 24.0, 1e-9);
}

TEST(PolyhedronFaces, ADegenerateFaceDoesNotThrowOrHang) {
    // All points collinear: no plane to project onto. It must fall through
    // rather than spin in the ear-clipping loop.
    Evaluated e = evaluateSrc(R"(
        polyhedron(points=[[0,0,0],[1,0,0],[2,0,0],[0,0,1]],
                   faces=[[0,1,2],[0,2,3],[0,3,1],[1,3,2]]);
    )");
    EXPECT_EQ(e.bodies.size(), 1u);
}

// -- 2D minkowski ---------------------------------------------------------

// minkowski() used to drop 2D sections on the floor, silently: this
// produced no geometry at all where the reference produces the rounded
// square asked for. Manifold has no 2D Minkowski, so it is built from
// convex pieces and hulls -- see minkowski2d().
TEST(Minkowski2D, RoundsAConvexOutline) {
    Evaluated e = evaluateSrc(
        "linear_extrude(6) minkowski() { square([30,20], center=true); circle(4, $fn=24); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    // A 30x20 rectangle grown by 4: the rectangle, four 4-wide sides, and
    // the corners adding up to one circle. Times 6 high.
    const double expected = (30.0 * 20.0 + 2 * 4 * (30 + 20) + M_PI * 16.0) * 6.0;
    EXPECT_NEAR(e.bodies[0].body->Volume(), expected, expected * 0.01);
    const manifold::Box box = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(box.max.x - box.min.x, 38.0, 0.01);
    EXPECT_NEAR(box.max.y - box.min.y, 28.0, 0.01);
}

// The case a hull cannot fake: a concave outline has to stay concave.
TEST(Minkowski2D, KeepsAConcaveOutlineConcave) {
    Evaluated e = evaluateSrc(R"(
        linear_extrude(4) minkowski() {
            polygon([[0,0],[40,0],[40,25],[22,25],[22,12],[0,12]]);
            circle(3, $fn=16);
        }
    )");
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::Box box = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(box.max.x - box.min.x, 46.0, 0.2);
    EXPECT_NEAR(box.max.y - box.min.y, 31.0, 0.2);
    // The notch has to still be a notch. A volume bound is too blunt to
    // say so -- hulling the pieces rather than unioning them lands within
    // 1% of the right answer here -- so probe the hole itself: the L is
    // missing the region x 0..22, y 12..25, and rounding by 3 does not
    // reach anywhere near the middle of it.
    // The probe has to sit inside what a hull WOULD cover and outside the
    // real shape, or it proves nothing: the hull runs from (0,12) to
    // (22,25), and the rounded L stops at y=15 for any x below 22.
    const manifold::Manifold probe =
        manifold::Manifold::Cube({1.0, 1.0, 8.0}).Translate({17.0, 17.0, -2.0});
    EXPECT_TRUE((*e.bodies[0].body ^ probe).IsEmpty()) << "the notch was filled in";
    // ...while somewhere solid is genuinely solid.
    const manifold::Manifold inside =
        manifold::Manifold::Cube({2.0, 2.0, 8.0}).Translate({30.0, 5.0, -2.0});
    EXPECT_FALSE((*e.bodies[0].body ^ inside).IsEmpty());
}

TEST(Minkowski2D, OneSectionIsHandedBackUnchanged) {
    Evaluated e = evaluateSrc("linear_extrude(3) minkowski() { square([10,10]); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 300.0, 1e-6);
}

TEST(Minkowski2D, ThreeDStillWorksAndStillWins) {
    Evaluated a = evaluateSrc("minkowski() { cube([30,20,6], center=true); sphere(3, $fn=12); }");
    ASSERT_EQ(a.bodies.size(), 1u);
    EXPECT_GT(a.bodies[0].body->Volume(), 30.0 * 20.0 * 6.0);
}

// The edge sweep is only valid when the shape being swept is convex and
// contains the origin, and when a third operand is folded rather than
// unioned in. Each of these fails, by a measurable amount, if the
// corresponding step is skipped -- so each is here on its own.

TEST(Minkowski2D, SweepsAConcaveSweeperByItsConvexPieces) {
    // Hulling the sweeper instead of decomposing it computes A (+) hull(B)
    // and comes out at 5092 against the reference's 4992.
    Evaluated e = evaluateSrc(R"(
        linear_extrude(4) minkowski() {
            polygon([[0,0],[40,0],[40,25],[22,25],[22,12],[0,12]]);
            polygon([[0,0],[8,0],[8,8],[5,8],[5,3],[0,3]]);
        }
    )");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 4992.0, 5.0);
}

TEST(Minkowski2D, ASweeperAwayFromTheOriginStillLandsInTheRightPlace) {
    // The identity unions A itself, which only holds when the sweeper
    // contains the origin. Without shifting it there and back, the result
    // keeps an untranslated copy of A and starts at x=0 instead of x=16.
    Evaluated e = evaluateSrc(R"(
        linear_extrude(4) minkowski() {
            polygon([[0,0],[40,0],[40,25],[22,25],[22,12],[0,12]]);
            translate([20,0]) circle(4, $fn=24);
        }
    )");
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::Box box = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(box.min.x, 16.0, 0.2);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 5120.47, 5.0);
}

TEST(Minkowski2D, ThreeOperandsAreSummedInTurnNotUnioned) {
    // A (+) B (+) C, not A (+) (B union C) -- which would give 3711.
    Evaluated e = evaluateSrc(
        "linear_extrude(4) minkowski() { square([30,20], center=true);"
        " circle(3, $fn=16); square([6,2], center=true); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 4670.21, 5.0);
    const manifold::Box box = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(box.max.x - box.min.x, 42.0, 0.2);
}

TEST(Minkowski2D, AShapeWithAHoleHasBothItsContoursSwept) {
    // Only the boundary of the shape being swept along is walked, so a
    // hole is not a special case -- it is another contour.
    Evaluated e = evaluateSrc(R"(
        linear_extrude(3) minkowski() {
            difference() { square([40,30], center=true); circle(8, $fn=24); }
            circle(2, $fn=16);
        }
    )");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 4141.98, 5.0);
    // The hole shrinks by the sweep rather than filling in.
    const manifold::Manifold probe =
        manifold::Manifold::Cube({1.0, 1.0, 6.0}).Translate({-0.5, -0.5, -1.0});
    EXPECT_TRUE((*e.bodies[0].body ^ probe).IsEmpty()) << "the hole was filled in";
}
