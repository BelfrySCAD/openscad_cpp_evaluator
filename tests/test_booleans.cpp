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

// -- fill() ---------------------------------------------------------------
//
// 2D only: union the children, then drop every hole. Each area below was
// checked against OpenSCAD 2026.02.01 by extruding the same shape and
// comparing STL volumes -- the filled cases matched to 0.0000.

TEST(Fill, ClosesAHole) {
    // A 10x10 square with a disc punched out fills back to the solid square.
    Evaluated e = evalSrc(
        "fill() difference() { square(10, center=true); circle(3, $fn=64); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 100.0, 1e-6);
}

TEST(Fill, MergesOverlappingOutlinesIntoOne) {
    // Two holed squares 4 apart. Filling drops both holes and the outer
    // boundaries then overlap, so the result must be re-unioned into a
    // single 14x10 region rather than left as two loops.
    Evaluated e = evalSrc(
        "fill() { difference() { square(10, center=true); circle(3, $fn=64); }"
        "         translate([4,0]) difference() { square(10, center=true); circle(3, $fn=64); } }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 140.0, 1e-6);
}

TEST(Fill, NestedOutlinesCollapseToTheOutermost) {
    // A ring inside another ring's hole: only the outermost boundary is
    // positive, so everything inside it fills solid.
    Evaluated e = evalSrc(
        "fill() { difference() { circle(10, $fn=64); circle(8, $fn=64); }"
        "         difference() { circle(6, $fn=64); circle(4, $fn=64); } }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    // The 64-segment polygon approximating r=10, not pi*100 exactly.
    EXPECT_NEAR(e.bodies[0].section->Area(), 313.6548, 0.01);
}

TEST(Fill, LeavesADisjointRegionDisjoint) {
    // Nothing to fill and nothing to merge -- two separate squares stay two
    // separate squares, total area unchanged.
    Evaluated e = evalSrc("fill() { square(4); translate([10,0]) square(4); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 32.0, 1e-6);
}

TEST(Fill, HoleLeftOpenToTheOutsideIsNotAHole) {
    // Notching the ring so its inner void connects to the outside leaves one
    // positive outline, which fills to the full disc -- same as the closed
    // ring above, and the same as OpenSCAD gives.
    Evaluated e = evalSrc(
        "fill() difference() { difference() { circle(10, $fn=64); circle(8, $fn=64); } square(3); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 313.6548, 0.01);
}

TEST(Fill, SolidShapeIsUnchanged) {
    Evaluated e = evalSrc("fill() square(4, center=true);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 16.0, 1e-9);
}

TEST(Fill, ThreeDeeChildWarnsAndContributesNothing) {
    std::string lastWarning;
    Evaluated e = evalSrc("fill() cube(1);", [&](const std::string& m) { lastWarning = m; });
    EXPECT_NE(lastWarning.find("fill() not yet implemented for 3D"), std::string::npos);
    EXPECT_TRUE(e.bodies.empty());
}

TEST(Fill, NoChildrenIsEmptyNotAnError) {
    Evaluated e = evalSrc("fill();");
    EXPECT_TRUE(e.bodies.empty());
}

// -- Mixed 2D/3D children -------------------------------------------------
//
// A node's dimension is fixed by its first real child; children of the other
// dimension are dropped, with the reference's own two warnings. All 25
// module/shape combinations were diffed against OpenSCAD 2026.02.01 warning
// for warning, and the surviving geometry compared by STL volume.
//
// This is not only a diagnostics fix. generateCsg builds one result and
// switches on whether it holds a body or a section, so before this a group
// that changed dimension part-way dereferenced the empty optional --
// `union() { cube(1); square(4); }` took the process down with "mutex lock
// failed: Invalid argument".

namespace {
std::vector<std::string> dimWarnings(const std::string& code) {
    std::vector<std::string> out;
    evalSrc(code, [&](const std::string& m) {
        if (m.find("Mixing 2D and 3D") != std::string::npos ||
            m.find("child object for") != std::string::npos) {
            out.push_back(m.substr(0, m.find(" in file")));
        }
    });
    std::sort(out.begin(), out.end());
    return out;
}
} // namespace

TEST(MixedDimensions, ThreeDeeFirstUnionDoesNotCrash) {
    // The regression that started this: a body-then-section group.
    Evaluated e = evalSrc("union() { cube(1); square(4); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0, 1e-9);
}

TEST(MixedDimensions, TwoDeeFirstKeepsTheTwoDeeChild) {
    Evaluated e = evalSrc("union() { square(4); cube(1); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 16.0, 1e-9);
}

TEST(MixedDimensions, GroupWarnsTwice) {
    EXPECT_EQ(dimWarnings("union() { square(4); cube(1); }"),
              (std::vector<std::string>{"WARNING: Ignoring 3D child object for 2D operation",
                                        "WARNING: Mixing 2D and 3D objects is not supported"}));
    EXPECT_EQ(dimWarnings("union() { cube(1); square(4); }"),
              (std::vector<std::string>{"WARNING: Ignoring 2D child object for 3D operation",
                                        "WARNING: Mixing 2D and 3D objects is not supported"}));
}

TEST(MixedDimensions, TransformsAndControlFlowAreGroupsToo) {
    const std::vector<std::string> expected{"WARNING: Ignoring 3D child object for 2D operation",
                                            "WARNING: Mixing 2D and 3D objects is not supported"};
    EXPECT_EQ(dimWarnings("translate([0,0,0]) { square(4); cube(1); }"), expected);
    EXPECT_EQ(dimWarnings("scale(2) { square(4); cube(1); }"), expected);
    EXPECT_EQ(dimWarnings("color(\"red\") { square(4); cube(1); }"), expected);
    EXPECT_EQ(dimWarnings("for (i = [0:0]) { square(4); cube(1); }"), expected);
    EXPECT_EQ(dimWarnings("if (true) { square(4); cube(1); }"), expected);
    EXPECT_EQ(dimWarnings("hull() { square(4); cube(1); }"), expected);
}

TEST(MixedDimensions, FixedDimensionOperationsWarnOnlyOnce) {
    // These are always 2D input, so there is no "mixing" to report -- just
    // the one dropped child. Same for projection, which is always 3D input.
    EXPECT_EQ(dimWarnings("linear_extrude(1) { square(4); cube(1); }"),
              (std::vector<std::string>{"WARNING: Ignoring 3D child object for 2D operation"}));
    EXPECT_EQ(dimWarnings("offset(1) cube(1);"),
              (std::vector<std::string>{"WARNING: Ignoring 3D child object for 2D operation"}));
    EXPECT_EQ(dimWarnings("projection() { square(4); }"),
              (std::vector<std::string>{"WARNING: Ignoring 2D child object for 3D operation"}));
}

TEST(MixedDimensions, BackgroundChildIsExempt) {
    // A % ghost of the other dimension is not an error -- matching
    // isValidDim's own isBackground() skip.
    EXPECT_TRUE(dimWarnings("union() { square(4); %cube(1); }").empty());
}

TEST(MixedDimensions, RoofWarnsForNeither) {
    // Deliberate: the reference reports nothing here either.
    EXPECT_TRUE(dimWarnings("roof() { square(4); cube(1); }").empty());
}

TEST(MixedDimensions, TopLevelIsAGroupAsWell) {
    // The top level is an implicit union, and gets the same treatment.
    EXPECT_EQ(dimWarnings("square(4); cube(1);"),
              (std::vector<std::string>{"WARNING: Ignoring 3D child object for 2D operation",
                                        "WARNING: Mixing 2D and 3D objects is not supported"}));
    Evaluated e = evalSrc("square(4); cube(1);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_TRUE(e.bodies[0].section.has_value());
}

TEST(MixedDimensions, UniformChildrenAreUntouched) {
    EXPECT_TRUE(dimWarnings("union() { cube(1); cube(2); }").empty());
    EXPECT_TRUE(dimWarnings("union() { square(1); square(2); }").empty());
    Evaluated e = evalSrc("union() { cube(1); translate([5,0,0]) cube(1); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 2.0, 1e-9);
}

// -- minkowski_difference() -----------------------------------------------
//
// Erosion: the operation minkowski() has no inverse for. A BelfrySCAD
// extension, because OpenSCAD has no equivalent module and no way to spell
// it in the language -- minkowski() only ever sums.
//
// Eroding a cube by a cube is exact (no curved tool to tessellate), so those
// expectations are equalities rather than tolerances.

TEST(MinkowskiDifference, ErodesACubeExactly) {
    // A 10-cube eroded by a 2-cube is an 8-cube: every face moves in by the
    // tool's half-width.
    Evaluated e = evalSrc("minkowski_difference() { cube(10, center=true); cube(2, center=true); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 512.0, 1e-6);
    const manifold::Box bb = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bb.min.x, -4.0, 1e-9);
    EXPECT_NEAR(bb.max.x, 4.0, 1e-9);
}

TEST(MinkowskiDifference, ErodesByASphereToWithinTessellation) {
    // Same 8-cube, but the tool is a polyhedral sphere whose inradius is a
    // touch under 1, so the result is a touch larger. That gap IS the
    // tessellation and shrinks with $fn -- it is not an error in the erosion.
    Evaluated e = evalSrc("minkowski_difference() { cube(10, center=true); sphere(1, $fn=64); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 512.0, 1.0);
    EXPECT_GT(e.bodies[0].body->Volume(), 512.0);
}

TEST(MinkowskiDifference, ShrinksANonConvexBodyOnEverySide) {
    // The case that motivates the module: a plate with holes, shrunk by a
    // clearance. The bounding box must come in by the tool's radius all
    // round -- 20x20x4 becomes 18x18x2.
    Evaluated e = evalSrc(
        "minkowski_difference() {"
        "  difference() { cube([20,20,4], center=true);"
        "                 for (x=[-6,0,6]) translate([x,0,0]) cylinder(h=6,r=2.5,center=true,$fn=24); }"
        "  sphere(1, $fn=24); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    const manifold::Box bb = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bb.max.x, 9.0, 0.05);
    EXPECT_NEAR(bb.max.y, 9.0, 0.05);
    EXPECT_NEAR(bb.max.z, 1.0, 0.05);
}

TEST(MinkowskiDifference, SingleChildIsANoOp) {
    // Nothing to erode with, so nothing happens -- the same shape
    // minkowski() gives a lone child back.
    Evaluated e = evalSrc("minkowski_difference() { cube(10, center=true); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1000.0, 1e-9);
}

TEST(MinkowskiDifference, NoChildrenIsEmpty) {
    Evaluated e = evalSrc("minkowski_difference();");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(MinkowskiDifference, ErodesSuccessivelyByEveryToolAfterTheFirst) {
    // Two 2-cube tools take 1 off each side twice: 10 -> 8 -> 6.
    Evaluated e = evalSrc(
        "minkowski_difference() { cube(10, center=true); cube(2, center=true); cube(2, center=true); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 216.0, 1e-6);
}

TEST(MinkowskiDifference, TwoDeeChildrenAreIgnoredLikeMinkowskis) {
    // 3D only, matching minkowski()'s own 3D-wins dispatch. 2D erosion
    // already exists as offset(r=-N).
    Evaluated e = evalSrc("minkowski_difference() { square(10); square(2); }");
    EXPECT_TRUE(e.bodies.empty());
}

// -- simplify() -----------------------------------------------------------
//
// Mesh decimation within a tolerance. A BelfrySCAD extension: nothing in the
// OpenSCAD language can reduce a triangle count.

namespace {
const char* kSimplifyModel =
    "difference() { sphere(r=20, $fn=64); "
    "  for (a=[0:60:359]) rotate([0,0,a]) translate([12,0,0]) cylinder(h=60,r=4,center=true,$fn=32); }";

const manifold::Manifold& soleBody(const Evaluated& e) { return *e.bodies[0].body; }
} // namespace

TEST(Simplify, ReducesTriangleCountWithinTolerance) {
    Evaluated plain = evalSrc(std::string(kSimplifyModel) + ";");
    Evaluated cut = evalSrc(std::string("simplify(0.1) ") + kSimplifyModel + ";");
    ASSERT_EQ(cut.bodies.size(), 1u);
    EXPECT_LT(soleBody(cut).NumTri(), soleBody(plain).NumTri());
    // ...without moving the surface much: 0.1 costs well under 1% of volume.
    EXPECT_NEAR(soleBody(cut).Volume(), soleBody(plain).Volume(),
                0.01 * soleBody(plain).Volume());
}

TEST(Simplify, DefaultTolerandeScalesWithTheModel) {
    // No argument means 0.1% of the bounding-box diagonal. Manifold's own
    // default of 0 falls back to the mesh epsilon and does nothing at all,
    // so a bare simplify() must NOT pass zero through -- silently doing
    // nothing is the worst outcome.
    Evaluated plain = evalSrc(std::string(kSimplifyModel) + ";");
    Evaluated def = evalSrc(std::string("simplify() ") + kSimplifyModel + ";");
    EXPECT_LT(soleBody(def).NumTri(), soleBody(plain).NumTri());

    // Scale-independence is the point: the same model 1000x larger must lose
    // the same proportion of its triangles, which an absolute default could
    // never manage.
    Evaluated big = evalSrc(std::string("scale(1000) simplify() ") + kSimplifyModel + ";");
    Evaluated bigPlain = evalSrc(std::string("scale(1000) ") + kSimplifyModel + ";");
    const double ratioSmall = double(soleBody(def).NumTri()) / soleBody(plain).NumTri();
    const double ratioBig = double(soleBody(big).NumTri()) / soleBody(bigPlain).NumTri();
    EXPECT_NEAR(ratioSmall, ratioBig, 0.02);
}

TEST(Simplify, KeepsOriginalIdProvenance) {
    // Selection and drag-to-edit map an AST node to geometry through the
    // original-ID runs, so a simplify() in the chain must not drop them.
    Evaluated plain = evalSrc(std::string(kSimplifyModel) + ";");
    Evaluated cut = evalSrc(std::string("simplify(0.1) ") + kSimplifyModel + ";");
    const manifold::MeshGL a = soleBody(plain).GetMeshGL();
    const manifold::MeshGL b = soleBody(cut).GetMeshGL();
    // Compare the SHAPE of the provenance, not the ID values: tagGenerated
    // stamps fresh ids on every evaluation, so the two runs here carry
    // different numbers for the same parts (1 vs 27 in practice). What must
    // survive simplification is that the runs are still there and still
    // distinguish the same number of contributing parts.
    EXPECT_EQ(a.runOriginalID.size(), b.runOriginalID.size());
    ASSERT_FALSE(b.runOriginalID.empty());
    const std::set<uint32_t> idsA(a.runOriginalID.begin(), a.runOriginalID.end());
    const std::set<uint32_t> idsB(b.runOriginalID.begin(), b.runOriginalID.end());
    EXPECT_EQ(idsA.size(), idsB.size());
}

TEST(Simplify, KeepsTopology) {
    // Six drilled holes must still be six holes -- a decimation that welds
    // one shut has changed the part, not just its mesh.
    Evaluated plain = evalSrc(std::string(kSimplifyModel) + ";");
    Evaluated cut = evalSrc(std::string("simplify(0.1) ") + kSimplifyModel + ";");
    EXPECT_EQ(soleBody(cut).Genus(), soleBody(plain).Genus());
}

TEST(Simplify, WorksOnTwoDeeSections) {
    // The tolerance is in model units here too: on an r=20 circle, 0.1
    // costs 0.5% of the area and 0.5 costs 2.5%.
    Evaluated plain = evalSrc("circle(r=20, $fn=256);");
    Evaluated cut = evalSrc("simplify(0.1) circle(r=20, $fn=256);");
    ASSERT_TRUE(cut.bodies[0].section.has_value());
    EXPECT_NEAR(cut.bodies[0].section->Area(), plain.bodies[0].section->Area(),
                0.01 * plain.bodies[0].section->Area());
    EXPECT_LT(cut.bodies[0].section->Area(), plain.bodies[0].section->Area());
}

TEST(Simplify, NegativeToleranceWarnsAndPassesThrough) {
    std::string last;
    Evaluated e = evalSrc(std::string("simplify(-1) ") + kSimplifyModel + ";",
                           [&](const std::string& m) { last = m; });
    EXPECT_NE(last.find("must not be negative"), std::string::npos) << last;
    Evaluated plain = evalSrc(std::string(kSimplifyModel) + ";");
    EXPECT_EQ(soleBody(e).NumTri(), soleBody(plain).NumTri());
}

TEST(Simplify, NoChildrenIsEmpty) {
    Evaluated e = evalSrc("simplify();");
    EXPECT_TRUE(e.bodies.empty());
}

// -- levelset -------------------------------------------------------------
//
// A solid from a grid of sampled values. Takes a grid rather than an SDF
// callback: measured, building the field in script costs 0.41 us/sample
// against 0.96 us for a closure call, and a grid lets Manifold mesh in
// parallel because nothing re-enters the evaluator.
//
// Accuracy is bounded by the grid, not by Manifold -- it samples on a
// body-centred cubic lattice and the lambda interpolates -- so every
// tolerance here is a percentage, not an epsilon.

namespace {

// field[i][j][k] of the distance from the origin, over [-half, half]^3.
std::string sphereFieldSrc(int n, double half) {
    return "N = " + std::to_string(n) + "; H = " + std::to_string(half) +
            ";\nfunction co(t) = -H + 2*H*t/(N-1);\n"
            "f = [for (i=[0:N-1]) [for (j=[0:N-1]) [for (k=[0:N-1])\n"
            "      sqrt(co(i)*co(i) + co(j)*co(j) + co(k)*co(k)) ]]];\n";
}

std::vector<std::string> levelsetWarnings(const std::string& code) {
    std::vector<std::string> out;
    evalSrc(code, [&](const std::string& m) {
        if (m.rfind("WARNING", 0) == 0) out.push_back(m);
    });
    return out;
}

} // namespace

TEST(LevelSet, ASphereFieldGivesASphere) {
    Evaluated e = evalSrc(sphereFieldSrc(50, 30) +
                           "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=20);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const double analytic = 4.0 / 3.0 * 3.14159265358979 * 20 * 20 * 20;
    EXPECT_NEAR(soleBody(e).Volume(), analytic, 0.01 * analytic);  // within 1%
    EXPECT_EQ(soleBody(e).Genus(), 0);
}

TEST(LevelSet, AccuracyImprovesWithGridResolution) {
    // The grid is the accuracy limit, so more samples must mean less error.
    // Anything else means the resolution argument is not doing its job.
    const double analytic = 4.0 / 3.0 * 3.14159265358979 * 20 * 20 * 20;
    double prevErr = 1e30;
    for (int n : {25, 40, 60}) {
        Evaluated e = evalSrc(sphereFieldSrc(n, 30) +
                               "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=20);");
        ASSERT_EQ(e.bodies.size(), 1u) << "n=" << n;
        const double err = std::abs(soleBody(e).Volume() - analytic);
        EXPECT_LT(err, prevErr) << "n=" << n << " was no better than the coarser grid";
        prevErr = err;
    }
}

TEST(LevelSet, ATorusFieldHasGenusOne) {
    // The test that catches an implementation producing plausible-looking
    // but topologically wrong output -- volume alone would not.
    Evaluated e = evalSrc(
        "N = 60; H = 30;\nfunction co(t) = -H + 2*H*t/(N-1);\n"
        "f = [for (i=[0:N-1]) [for (j=[0:N-1]) [for (k=[0:N-1])\n"
        "      let(q = sqrt(co(i)*co(i) + co(j)*co(j)) - 15)\n"
        "      sqrt(q*q + co(k)*co(k)) ]]];\n"
        "levelset(f, bounds=[[-H,-H,-H],[H,H,H]], isovalue=6);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(soleBody(e).Genus(), 1);
}

TEST(LevelSet, DisjointBlobsComeOutAsSeparateComponents) {
    // Two spheres: Euler characteristic 4, so genus 1 - 4/2 = -1.
    Evaluated e = evalSrc(
        "N = 60; H = 30;\nfunction co(t) = -H + 2*H*t/(N-1);\n"
        "f = [for (i=[0:N-1]) [for (j=[0:N-1]) [for (k=[0:N-1])\n"
        "      min(sqrt((co(i)-12)*(co(i)-12) + co(j)*co(j) + co(k)*co(k)),\n"
        "          sqrt((co(i)+12)*(co(i)+12) + co(j)*co(j) + co(k)*co(k))) ]]];\n"
        "levelset(f, bounds=[[-H,-H,-H],[H,H,H]], isovalue=8);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(soleBody(e).Genus(), -1);
}

TEST(LevelSet, IndexOrderIsXThenYThenZ) {
    // Deliberately asymmetric: a symmetric field passes even if the axes are
    // transposed, which is the classic marching-cubes bug.
    Evaluated e = evalSrc(
        "N = 60; H = 30;\nfunction co(t) = -H + 2*H*t/(N-1);\n"
        "f = [for (i=[0:N-1]) [for (j=[0:N-1]) [for (k=[0:N-1])\n"
        "      max(abs(co(i))/5, abs(co(j))/10, abs(co(k))/20) ]]];\n"
        "levelset(f, bounds=[[-H,-H,-H],[H,H,H]], isovalue=1);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::Box b = soleBody(e).BoundingBox();
    EXPECT_NEAR(b.max.x, 5.0, 0.6);
    EXPECT_NEAR(b.max.y, 10.0, 0.6);
    EXPECT_NEAR(b.max.z, 20.0, 0.6);
}

TEST(LevelSet, IsovalueSelectsTheSurface) {
    Evaluated small = evalSrc(sphereFieldSrc(40, 30) +
                               "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=10);");
    Evaluated big = evalSrc(sphereFieldSrc(40, 30) +
                             "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=20);");
    EXPECT_LT(soleBody(small).Volume(), soleBody(big).Volume());
}

TEST(LevelSet, BoundsPlaceTheResult) {
    Evaluated e = evalSrc(sphereFieldSrc(40, 30) +
                           "levelset(f, bounds=[[100,-30,-30],[160,30,30]], isovalue=20);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::Box b = soleBody(e).BoundingBox();
    EXPECT_NEAR((b.min.x + b.max.x) / 2, 130.0, 1.0);
}

TEST(LevelSet, AnIsovalueNothingReachesIsEmpty) {
    // A distance field is never negative, so nothing is inside. Not an
    // error: asking is a fair question.
    Evaluated e = evalSrc(sphereFieldSrc(20, 30) +
                           "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=-1);");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(LevelSet, AnIsovalueEverythingSatisfiesFillsTheBounds) {
    // The mirror case, and it is NOT empty: if every sample is inside, the
    // solid is the whole box, clipped at the bounds. Worth pinning, because
    // "nothing crossed the surface" and "everything is inside" look alike
    // from the outside and mean opposite things.
    Evaluated e = evalSrc(sphereFieldSrc(20, 30) +
                           "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=1000);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::Box b = soleBody(e).BoundingBox();
    EXPECT_NEAR(b.max.x - b.min.x, 60.0, 1.0);
    EXPECT_NEAR(soleBody(e).Volume(), 60.0 * 60.0 * 60.0, 0.02 * 60.0 * 60.0 * 60.0);
}

TEST(LevelSet, MalformedFieldsWarn) {
    for (const char* code : {
             "levelset([[[1,2],[3,4]],[[5,6],[7]]], bounds=[[0,0,0],[1,1,1]]);",   // ragged
             "levelset([[[1,2],[3,\"x\"]],[[5,6],[7,8]]], bounds=[[0,0,0],[1,1,1]]);", // not numeric
             "levelset(5, bounds=[[0,0,0],[1,1,1]]);",                              // not a field
             "levelset([[[1,2],[3,4]],[[5,6],[7,8]]], bounds=[[0,0,0],[0,1,1]]);",  // empty extent
         }) {
        EXPECT_FALSE(levelsetWarnings(code).empty()) << "silent on: " << code;
    }
}

// -- levelset, function form ----------------------------------------------
//
// Same builtin, field given as function(x,y,z) instead of a grid. Slower --
// a closure call per sample, and canParallel must be false because the
// evaluator would otherwise be called back from Manifold's worker threads --
// but MORE accurate, because Manifold picks its own sample points and can
// snap toward the true surface instead of interpolating a fixed lattice.
//
// Measured at matched resolution, sphere against analytic:
//    50^3   grid 0.23s (-0.244%)   function 0.32s (-0.103%)
//   100^3   grid 1.06s (-0.059%)   function 1.91s (-0.025%)

TEST(LevelSetFn, AFunctionFieldGivesTheSameSphere) {
    Evaluated e = evalSrc(
        "levelset(function(x,y,z) sqrt(x*x+y*y+z*z), "
        "bounds=[[-30,-30,-30],[30,30,30]], isovalue=20, edge=1.5);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const double analytic = 4.0 / 3.0 * 3.14159265358979 * 20 * 20 * 20;
    EXPECT_NEAR(soleBody(e).Volume(), analytic, 0.01 * analytic);
    EXPECT_EQ(soleBody(e).Genus(), 0);
}

TEST(LevelSetFn, AgreesWithTheGridFormOnTheSameField) {
    // The two intakes must describe the same solid; they differ only in how
    // finely they can resolve it.
    Evaluated grid = evalSrc(sphereFieldSrc(50, 30) +
                              "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=20);");
    Evaluated fn = evalSrc(
        "levelset(function(x,y,z) sqrt(x*x+y*y+z*z), "
        "bounds=[[-30,-30,-30],[30,30,30]], isovalue=20, edge=" +
        std::to_string(60.0 / 49.0) + ");");
    ASSERT_EQ(grid.bodies.size(), 1u);
    ASSERT_EQ(fn.bodies.size(), 1u);
    EXPECT_EQ(soleBody(fn).Genus(), soleBody(grid).Genus());
    EXPECT_NEAR(soleBody(fn).Volume(), soleBody(grid).Volume(),
                0.01 * soleBody(grid).Volume());
}

TEST(LevelSetFn, TopologyIsFoundFromAFunctionToo) {
    Evaluated e = evalSrc(
        "levelset(function(x,y,z) let(q = sqrt(x*x+y*y) - 15) sqrt(q*q + z*z), "
        "bounds=[[-30,-30,-30],[30,30,30]], isovalue=6, edge=1.2);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(soleBody(e).Genus(), 1);
}

TEST(LevelSetFn, AFunctionFieldRequiresEdge) {
    // There is no grid to infer spacing from, and the cost is CUBIC in it --
    // guessing would silently produce either a useless mesh or a ten-minute
    // one. Making the caller say is the safer failure.
    const std::vector<std::string> w = levelsetWarnings(
        "levelset(function(x,y,z) sqrt(x*x+y*y+z*z), bounds=[[-1,-1,-1],[1,1,1]]);");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("edge="), std::string::npos) << w[0];
}

TEST(LevelSetFn, AFunctionOfTheWrongArityWarns) {
    // One parameter is wrong for either dimension, so the message names both.
    const std::vector<std::string> one = levelsetWarnings(
        "levelset(function(p) 1, bounds=[[-1,-1,-1],[1,1,1]], edge=0.5);");
    ASSERT_EQ(one.size(), 1u);
    EXPECT_NE(one[0].find("function(x,y)"), std::string::npos) << one[0];

    // Two parameters is right for 2D and wrong for 3D, which is only
    // knowable once `bounds` has been read -- hence a second, later check.
    const std::vector<std::string> two = levelsetWarnings(
        "levelset(function(x,y) x, bounds=[[-1,-1,-1],[1,1,1]], edge=0.5);");
    ASSERT_EQ(two.size(), 1u);
    EXPECT_NE(two[0].find("three parameters"), std::string::npos) << two[0];
}

TEST(LevelSetFn, AFieldFunctionCanCaptureOuterVariables) {
    Evaluated e = evalSrc(
        "r = 20;\nlevelset(function(x,y,z) sqrt(x*x+y*y+z*z), "
        "bounds=[[-30,-30,-30],[30,30,30]], isovalue=r, edge=1.5);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(soleBody(e).Volume(), 4.0 / 3.0 * 3.14159265358979 * 8000, 400.0);
}

TEST(LevelSet, AnExplicitlyUndefIsovalueFallsBackToTheDefault) {
    // Same rule as linear_solve's b, and for the same reason: a wrapper
    // with a fixed signature forwards every parameter, undef included.
    Evaluated withUndef = evalSrc(sphereFieldSrc(30, 30) +
                                   "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=undef);");
    Evaluated withZero = evalSrc(sphereFieldSrc(30, 30) +
                                  "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=0);");
    EXPECT_EQ(withUndef.bodies.size(), withZero.bodies.size());
    EXPECT_TRUE(levelsetWarnings(sphereFieldSrc(30, 30) +
                                  "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=undef);")
                    .empty());
}

// -- levelset: bounded isovalue ranges -------------------------------------
//
// isovalue is unified as a band: a scalar v means [-INF, v] ("at or below",
// the distance-field reading), a range means BETWEEN, and [lo, INF] is
// BOSL2's "at or above" idiom. One LevelSet pass either way -- a bounded
// range could be had as difference(levelset(hi), levelset(lo)), but that
// meshes twice and then booleans.

TEST(LevelSetRange, ABoundedRangeGivesAShell) {
    Evaluated e = evalSrc(
        "levelset(function(x,y,z) sqrt(x*x+y*y+z*z), bounds=[[-30,-30,-30],[30,30,30]], "
        "isovalue=[15,20], edge=1);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const double analytic = 4.0 / 3.0 * 3.14159265358979 * (8000 - 3375);
    EXPECT_NEAR(soleBody(e).Volume(), analytic, 0.01 * analytic);
    EXPECT_EQ(soleBody(e).Genus(), -1);   // hollow: a void inside
}

TEST(LevelSetRange, AnOpenEndedRangeIsBosl2sIdiom) {
    // BOSL2 passes [isovalue, INF] to mean "at or above", the opposite of
    // our scalar default. A caller forwarding BOSL2's own argument gets
    // BOSL2's own semantics with no translation and no invert.
    Evaluated e = evalSrc(
        "levelset(function(x,y,z) sqrt(x*x+y*y+z*z), bounds=[[-30,-30,-30],[30,30,30]], "
        "isovalue=[20,1e18], edge=1);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const double expect = 60.0 * 60.0 * 60.0 - 4.0 / 3.0 * 3.14159265358979 * 8000;
    EXPECT_NEAR(soleBody(e).Volume(), expect, 0.01 * expect);
}

TEST(LevelSetRange, ABackwardsRangeWarns) {
    EXPECT_FALSE(levelsetWarnings("levelset(function(x,y,z) x, bounds=[[-1,-1,-1],[1,1,1]], "
                                   "isovalue=[5,1], edge=0.5);")
                     .empty());
}

// -- levelset: 2D ----------------------------------------------------------
//
// CrossSection has no contour extraction, so the contours come from marching
// squares here and are handed to CrossSection(Polygons, FillRule). 2D or 3D
// is decided by `bounds`, not guessed from the field.

TEST(LevelSet2d, AFunctionFieldGivesADisc) {
    Evaluated e = evalSrc(
        "levelset(function(x,y) sqrt(x*x+y*y), bounds=[[-30,-30],[30,30]], isovalue=20, edge=0.5);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 3.14159265358979 * 400, 1.0);
}

TEST(LevelSet2d, AGridFieldGivesTheSameDisc) {
    Evaluated e = evalSrc(
        "N = 121;\nfunction co(t) = -30 + 60*t/(N-1);\n"
        "plane = [for (i=[0:N-1]) [for (j=[0:N-1]) sqrt(co(i)*co(i) + co(j)*co(j)) ]];\n"
        "levelset(plane, bounds=[[-30,-30],[30,30]], isovalue=20);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 3.14159265358979 * 400, 1.0);
}

TEST(LevelSet2d, ARangeGivesAnAnnulusWithARealHole) {
    Evaluated e = evalSrc(
        "levelset(function(x,y) sqrt(x*x+y*y), bounds=[[-30,-30],[30,30]], "
        "isovalue=[15,20], edge=0.5);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_NEAR(e.bodies[0].section->Area(), 3.14159265358979 * 175, 1.0);
    EXPECT_EQ(e.bodies[0].section->NumContour(), 2u);   // outer + hole
}

TEST(LevelSet2d, AContourRunningOffTheBoxIsClippedExactly) {
    // The padded ring sits a spacing outside the box, so a contour that meets
    // the edge lands out in the padding. Measured before clipping: a
    // half-plane came out 0.9 * spacing too big along every side it touched,
    // an O(h) error where a closed contour is O(h^2). Clipping to `bounds`
    // removes exactly that, and the answer becomes resolution-independent.
    for (const char* edge : {"1", "0.5", "0.25"}) {
        Evaluated e = evalSrc(std::string("levelset(function(x,y) x, bounds=[[-20,-20],[20,20]], "
                                           "isovalue=[-1e18,0], edge=") +
                               edge + ");");
        ASSERT_EQ(e.bodies.size(), 1u) << "edge=" << edge;
        EXPECT_NEAR(e.bodies[0].section->Area(), 800.0, 1e-6) << "edge=" << edge;
    }
}

TEST(LevelSet2d, IndexOrderIsXThenY) {
    // Asymmetric on purpose: a symmetric field passes even transposed.
    Evaluated e = evalSrc(
        "levelset(function(x,y) max(abs(x)/5, abs(y)/15), bounds=[[-30,-30],[30,30]], "
        "isovalue=1, edge=0.5);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::Rect r = e.bodies[0].section->Bounds();
    EXPECT_NEAR(r.max.x, 5.0, 0.6);
    EXPECT_NEAR(r.max.y, 15.0, 0.6);
}

TEST(LevelSet2d, SeparateBlobsStaySeparate) {
    Evaluated e = evalSrc(
        "levelset(function(x,y) min(norm([x-12,y]), norm([x+12,y])), "
        "bounds=[[-30,-30],[30,30]], isovalue=8, edge=0.5);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(e.bodies[0].section->NumContour(), 2u);
    EXPECT_NEAR(e.bodies[0].section->Area(), 2 * 3.14159265358979 * 64, 1.5);
}

TEST(LevelSet2d, A2dFunctionWithA3dBoundsWarns) {
    EXPECT_FALSE(levelsetWarnings("levelset(function(x,y) x, bounds=[[-1,-1,-1],[1,1,1]], edge=0.5);")
                     .empty());
}

TEST(LevelSet2d, A3dArrayWithA2dBoundsWarns) {
    EXPECT_FALSE(levelsetWarnings("levelset([[[1,2],[3,4]],[[5,6],[7,8]]], bounds=[[0,0],[1,1]]);")
                     .empty());
}

TEST(LevelSet, ASurfaceMeetingTheBoxIsCutCleanly) {
    // Manifold closes the mesh where the surface reaches the edge of the box
    // it is given, and that closure follows the SAMPLE LATTICE -- a visible
    // staircase on anything that touches the boundary. A gyroid touches it
    // everywhere, which is how this was found: the shape looked ragged next
    // to BOSL2's isosurface() of the same field.
    //
    // Sampling a padded box and cutting back makes the cut a plane. The
    // numeric form of that: a half-space is EXACTLY half the box, at every
    // resolution. Before the fix it was resolution-dependent.
    //
    // The 2D path had this fixed already (clipToBounds); 3D did not, because
    // 2D was the case that got measured at the time.
    for (const char* edge : {"2", "1", "0.5"}) {
        Evaluated e = evalSrc(std::string("levelset(function(x,y,z) x, "
                                           "bounds=[[-20,-20,-20],[20,20,20]], "
                                           "isovalue=[-1e18,0], edge=") +
                               edge + ");");
        ASSERT_EQ(e.bodies.size(), 1u) << "edge=" << edge;
        EXPECT_NEAR(soleBody(e).Volume(), 32000.0, 1e-6) << "edge=" << edge;
    }
}

TEST(LevelSet, PaddingDoesNotDisturbAShapeInsideTheBox) {
    // The padding is sampled outside the requested bounds, so a shape that
    // never reaches the boundary must come out exactly as before.
    Evaluated e = evalSrc(sphereFieldSrc(50, 30) +
                           "levelset(f, bounds=[[-30,-30,-30],[30,30,30]], isovalue=20);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const double analytic = 4.0 / 3.0 * 3.14159265358979 * 8000;
    EXPECT_NEAR(soleBody(e).Volume(), analytic, 0.01 * analytic);
    EXPECT_EQ(soleBody(e).Genus(), 0);
}

TEST(LevelSet, AGridFieldIsAlsoCutCleanlyAtTheBox) {
    // The grid intake cannot sample beyond its data, so the sampler reads
    // "outside" past the block rather than clamping and smearing the edge
    // values outward. Same clean cut as the function form.
    Evaluated e = evalSrc(
        "N = 41;\nfunction co(t) = -20 + 40*t/(N-1);\n"
        "f = [for (i=[0:N-1]) [for (j=[0:N-1]) [for (k=[0:N-1]) co(i) ]]];\n"
        "levelset(f, bounds=[[-20,-20,-20],[20,20,20]], isovalue=[-1e18,0]);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(soleBody(e).Volume(), 32000.0, 40.0);   // one sample layer of slack
}
