// Curve discretization: $fn/$fe/$fa/$fs -> segments, slices and outline
// splits, as OpenSCAD's CurveDiscretizer computes them. Every expected
// number below is the OpenSCAD 2026.02.01 binary's own (run with
// --enable=discretization-by-error for the $fe cases): vertex counts of the
// 2D output, triangle counts and volumes of the 3D. Several were wrong here
// before the port -- see each test.

#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"

#include "test_helpers.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

size_t vertices2d(const std::string& src) {
    Evaluated e = evalSrc(src);
    EXPECT_EQ(e.bodies.size(), 1u);
    return e.bodies.empty() || !e.bodies[0].section ? 0 : e.bodies[0].section->NumVert();
}

const manifold::Manifold& solid(const Evaluated& e) {
    EXPECT_EQ(e.bodies.size(), 1u);
    return *e.bodies.at(0).body;
}

} // namespace

// -- $fe -------------------------------------------------------------------

TEST(Discretizer, FeSetsTheCircleSegmentCount) {
    EXPECT_EQ(vertices2d("circle(r=10, $fe=0.1);"), 23u);      // pi / acos(1 - 0.01) = 22.2, ceiled
    EXPECT_EQ(vertices2d("circle(r=10, $fe=3);"), 5u);         // fe/r past 0.19098: the 5 floor
    EXPECT_EQ(vertices2d("circle(r=10, $fe=0.1, $fn=7);"), 7u); // $fn wins
}

TEST(Discretizer, FeIgnoresFaAndFs) {
    EXPECT_EQ(vertices2d("circle(r=10, $fe=0.1, $fa=1, $fs=0.1);"), 23u);
}

TEST(Discretizer, ANegativeOrZeroFeFallsBackToFaFs) {
    EXPECT_EQ(vertices2d("circle(r=10, $fe=0);"), vertices2d("circle(r=10);"));
    EXPECT_EQ(vertices2d("circle(r=10, $fe=-1);"), vertices2d("circle(r=10);"));
}

TEST(Discretizer, FeIsAdvertised) {
    // A failed assert() throws, failing the test.
    evalSrc("assert(supported_feature(\"discretization-by-error\") == 1);");
}

TEST(Discretizer, FeArcsScaleTheUnceiledCount) {
    // n = 5.2 for fe/r = 0.18: a 270-degree arc is ceil(5.2 * 0.75) = 4 here,
    // where the $fa/$fs rule would ceil n first and give 5.
    const Discretizer d{0.0, 1.8, 12.0, 2.0};
    const double n = std::numbers::pi / std::acos(1 - 0.18);
    ASSERT_GT(n, 5.0);
    ASSERT_LT(n, 6.0);
    EXPECT_EQ(d.circular(10.0, 270.0), static_cast<int>(std::ceil(n * 0.75)));
}

// -- $fn -------------------------------------------------------------------

TEST(Discretizer, AFractionalFnRoundsUp) {
    // Was truncated: 5.5 gave 5, OpenSCAD gives 6.
    EXPECT_EQ(vertices2d("circle(r=10, $fn=5.5);"), 6u);
}

// -- rotate_extrude ----------------------------------------------------------

TEST(Discretizer, PartialRotateExtrudeUsesItsShareOfTheCircle) {
    // Used to pack the whole circle's 30 segments into the 90 degrees (244
    // triangles).
    Evaluated e = evalSrc("rotate_extrude(angle=90) translate([10,0]) square(2);");
    EXPECT_EQ(solid(e).NumTri(), 68u);
    EXPECT_NEAR(solid(e).Volume(), 68.6718, 1e-3);
}

TEST(Discretizer, AShortArcCanHaveFewerThanThreeSections) {
    // ceil(10 * 45/360) = 2, which Manifold's Revolve cannot draw.
    Evaluated e = evalSrc("rotate_extrude(angle=45, $fn=10) translate([10,0]) square(2);");
    EXPECT_EQ(solid(e).NumTri(), 20u);
    EXPECT_NEAR(solid(e).Volume(), 33.6761, 1e-3);
    Evaluated neg = evalSrc("rotate_extrude(angle=-45, $fn=10) translate([10,0]) square(2);");
    EXPECT_NEAR(solid(neg).Volume(), 33.6761, 1e-3); // not inside out
}

TEST(Discretizer, FeSizesRotateExtrudeArcs) {
    Evaluated e = evalSrc("rotate_extrude(angle=270, $fe=0.3) translate([10,0]) square(2);");
    EXPECT_EQ(solid(e).NumTri(), 92u);
    EXPECT_NEAR(solid(e).Volume(), 201.0609, 1e-3);
}

// -- linear_extrude ----------------------------------------------------------

TEST(Discretizer, TwistWithoutSlicesIsSlicedByTheHelix) {
    // Was ONE slice: the 180-degree turn collapsed to a straight prism.
    // Volume also pins the side-quad diagonals (shorter one, as upstream).
    Evaluated e = evalSrc("linear_extrude(height=10, twist=180) square(5);");
    EXPECT_EQ(solid(e).NumTri(), 332u);
    EXPECT_NEAR(solid(e).Volume(), 254.2261, 1e-3);
}

TEST(Discretizer, SlicesMeansThatManySlices) {
    // Was one more: slices went to Manifold as nDivisions.
    Evaluated e = evalSrc("linear_extrude(height=10, twist=90, slices=4, $fn=4) square(5);");
    EXPECT_EQ(solid(e).NumTri(), 36u);
    EXPECT_NEAR(solid(e).Volume(), 275.5469, 1e-3);
}

TEST(Discretizer, FeSlicesATwist) {
    Evaluated e = evalSrc("linear_extrude(height=10, twist=360, $fe=0.1) square(5);");
    EXPECT_EQ(solid(e).NumTri(), 476u);
    EXPECT_NEAR(solid(e).Volume(), 254.5042, 1e-3);
}

TEST(Discretizer, NonUniformScaleWithTwistSplitsTheOutline) {
    Evaluated e = evalSrc("linear_extrude(height=10, scale=[2,0.5], twist=90) square(5);");
    EXPECT_EQ(solid(e).NumTri(), 320u);
    EXPECT_NEAR(solid(e).Volume(), 271.6205, 1e-3);
}

TEST(Discretizer, SegmentsZeroTurnsOutlineSplittingOff) {
    Evaluated split = evalSrc("linear_extrude(height=10, twist=90) square(5);");
    Evaluated plain = evalSrc("linear_extrude(height=10, twist=90, segments=0) square(5);");
    EXPECT_LT(solid(plain).NumTri(), solid(split).NumTri());
}
