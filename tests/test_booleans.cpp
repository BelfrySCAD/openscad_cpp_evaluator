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
