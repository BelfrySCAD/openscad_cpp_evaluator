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

TEST(Rotate, DegenerateZeroLengthAxisIsIdentity) {
    // axisAngleMatrix's own zero-length-axis guard (division by the axis'
    // own length would otherwise be a divide-by-zero) -- falls back to the
    // identity transform, matching the reference's own behavior for this
    // degenerate input.
    Evaluated e = evalSrc("rotate(45, [0,0,0]) cube([1,2,3], center=true);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 0.5, 1e-9);
    EXPECT_NEAR(bbox.max.y, 1.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 1.5, 1e-9);
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

TEST(Multmatrix, UndefArgumentIsANoOp) {
    Evaluated e = evalSrc("multmatrix() cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 1.0, 1e-9);
}

TEST(Multmatrix, ThreeByThreeMatrixIsPaddedWithZeroTranslation) {
    // toMat3x4's own rowAt() bounds-check fallback (0.0) for the missing
    // 4th column of a 3x3 input -- every other multmatrix test here uses a
    // full 4x4/4x3 matrix.
    Evaluated e = evalSrc("multmatrix([[1,0,0],[0,1,0],[0,0,1]]) translate([2,0,0]) cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, 2.0, 1e-9); // pure identity rotation, no added translation
    EXPECT_NEAR(bbox.max.x, 3.0, 1e-9);
}

// -- no children ------------------------------------------------------------

TEST(Transform3d, NoChildrenIsEmpty) {
    Evaluated e = evalSrc("translate([1,0,0]);");
    EXPECT_TRUE(e.bodies.empty());
}

// -- undef/omitted argument fallbacks -------------------------------------

TEST(Transform3d, TranslateWithNoArgumentDefaultsToOrigin) {
    // toVec3's own final fallback (v is neither a number nor a list, e.g.
    // the monostate default an omitted "v" argument gets from getArg) --
    // every other translate() test in this suite passes a real vector.
    Evaluated e = evalSrc("translate() cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, 0.0, 1e-9);
    EXPECT_NEAR(bbox.max.x, 1.0, 1e-9);
}

TEST(Transform3d, ScalarTranslateBecomesXOnlyVector) {
    // toVec3's own scalar (bare double) branch -- translate(5) is
    // equivalent to translate([5,0,0]), not an error or a uniform offset.
    Evaluated e = evalSrc("translate(5) cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, 5.0, 1e-9);
    EXPECT_NEAR(bbox.min.y, 0.0, 1e-9);
    EXPECT_NEAR(bbox.min.z, 0.0, 1e-9);
}

TEST(Transform3d, TwoElementVectorZPadsToZero) {
    Evaluated e = evalSrc("translate([3,4]) cube(1);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, 3.0, 1e-9);
    EXPECT_NEAR(bbox.min.y, 4.0, 1e-9);
    EXPECT_NEAR(bbox.min.z, 0.0, 1e-9);
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

// `auto` scales the zero-valued axes by the factor of whichever axis asked
// for the LARGEST new size. Every expectation below was confirmed against
// real OpenSCAD 2022.08.22; before this, `auto` was accepted and ignored.
TEST(Resize, AutoScalesZeroAxesByTheLargestRequestedAxisFactor) {
    // Bare `true` covers all three axes: 20/5 = 4x on X, so Y and Z too.
    Evaluated allAxes = evalSrc("resize([20,0,0], true) cube(5);");
    manifold::Box bbox = allAxes.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 20.0, 1e-9);
    EXPECT_NEAR(bbox.max.y, 20.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 20.0, 1e-9);

    // Named form is identical to the positional one.
    Evaluated named = evalSrc("resize([20,0,0], auto=true) cube(5);");
    manifold::Box nbox = named.bodies[0].body->BoundingBox();
    EXPECT_NEAR(nbox.max.y, 20.0, 1e-9);

    // A per-axis list only auto-scales the axes it names.
    Evaluated perAxis = evalSrc("resize([20,0,0], [false,true,false]) cube(5);");
    manifold::Box pbox = perAxis.bodies[0].body->BoundingBox();
    EXPECT_NEAR(pbox.max.x, 20.0, 1e-9);
    EXPECT_NEAR(pbox.max.y, 20.0, 1e-9);
    EXPECT_NEAR(pbox.max.z, 5.0, 1e-9); // not auto -- left alone
}

TEST(Resize, AutoFactorComesFromTheLargestNewsizeNotTheFirst) {
    // span (5,10,20); newsize (20,30,0). The largest requested size is 30
    // on Y, so the auto Z axis scales by 30/10 = 3 -> 60, NOT by X's 20/5.
    Evaluated e = evalSrc("resize([20,30,0], true) cube([5,10,20]);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 20.0, 1e-9);
    EXPECT_NEAR(bbox.max.y, 30.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 60.0, 1e-9);
}

TEST(Resize, AutoWithNoRequestedSizeIsANoOp) {
    Evaluated e = evalSrc("resize([0,0,0], true) cube(5);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 5.0, 1e-9);
    EXPECT_NEAR(bbox.max.y, 5.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 5.0, 1e-9);
}

TEST(Resize, AutoFalseLeavesZeroAxesUnscaled) {
    Evaluated e = evalSrc("resize([20,0,0], false) cube(5);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 20.0, 1e-9);
    EXPECT_NEAR(bbox.max.y, 5.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 5.0, 1e-9);
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

TEST(Transform2d, ScalarRotateAppliesAngleDirectly) {
    Evaluated a = evalSrc("rotate(90) square([4,2]);");
    manifold::Rect bounds = a.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.max.x - bounds.min.x, 2.0, 1e-6);
    EXPECT_NEAR(bounds.max.y - bounds.min.y, 4.0, 1e-6);
}

TEST(Transform2d, VectorRotateUsesThirdComponentAsAngle) {
    // applyTransform2d's own "a" argument can also be a [x,y,z] vector
    // (matching the 3D rotate() call shape) -- only the z component is
    // actually used as the 2D rotation angle.
    Evaluated withVec = evalSrc("rotate([0,0,90]) square([4,2]);");
    Evaluated withScalar = evalSrc("rotate(90) square([4,2]);");
    manifold::Rect a = withVec.bodies[0].section->Bounds();
    manifold::Rect b = withScalar.bodies[0].section->Bounds();
    EXPECT_NEAR(a.max.x - a.min.x, b.max.x - b.min.x, 1e-6);
    EXPECT_NEAR(a.max.y - a.min.y, b.max.y - b.min.y, 1e-6);
}

TEST(Transform2d, VectorRotateWithNoZComponentDefaultsToZeroAngle) {
    Evaluated e = evalSrc("rotate([0,0]) square([4,2]);");
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.max.x - bounds.min.x, 4.0, 1e-6);
    EXPECT_NEAR(bounds.max.y - bounds.min.y, 2.0, 1e-6);
}

TEST(Transform2d, ScalarScaleBroadcastsToBothAxes) {
    Evaluated e = evalSrc("scale(2) square([1,1]);");
    EXPECT_NEAR(e.bodies[0].section->Area(), 4.0, 1e-9);
}

TEST(Transform2d, MirrorFlipsAcrossAxis) {
    Evaluated e = evalSrc("mirror([1,0]) translate([2,0]) square([1,1]);");
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.min.x, -3.0, 1e-9);
    EXPECT_NEAR(bounds.max.x, -2.0, 1e-9);
}

TEST(Transform2d, MultmatrixActsLikeTranslate) {
    Evaluated e = evalSrc("multmatrix([[1,0,0,5],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) square([1,1]);");
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.min.x, 5.0, 1e-9);
    EXPECT_NEAR(bounds.max.x, 6.0, 1e-9);
}

TEST(Transform2d, MultmatrixUndefArgumentIsANoOp) {
    Evaluated e = evalSrc("multmatrix() square([1,1]);");
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.max.x, 1.0, 1e-9);
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

TEST(Color, HexColorUppercaseDigitsParse) {
    Evaluated e = evalSrc("color(\"#00FF00\") cube(1);");
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[0], 0.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[1], 1.0f);
}

TEST(Color, ShortHexColorParses) {
    // hexDigit's 3-char short form (#RGB, each digit doubled) -- every
    // other hex test here uses the full 6-digit form.
    Evaluated e = evalSrc("color(\"#0f0\") cube(1);");
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[0], 0.0f);
    EXPECT_NEAR((*e.bodies[0].color)[1], 1.0f, 1e-6);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[2], 0.0f);
}

TEST(Color, UnrecognizedColorNameFallsBackToWhite) {
    Evaluated e = evalSrc("color(\"not_a_real_color\") cube(1);");
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[0], 1.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[1], 1.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[2], 1.0f);
}

TEST(Color, MalformedHexLengthFallsBackToWhite) {
    // cssColor's own "neither 3 nor 6 hex digits" fallback -- distinct
    // from an unrecognized NAME's fallback (same result, different guard).
    Evaluated e = evalSrc("color(\"#12345\") cube(1);");
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[0], 1.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[1], 1.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[2], 1.0f);
}

TEST(Color, NonStringNonListArgumentFallsBackToWhite) {
    // resolveColor's own default-rgba fallback -- a plain number for `c`
    // matches neither the string branch (cssColor) nor the list branch at
    // all, distinct from an unrecognized color NAME (which still goes
    // through cssColor's own fallback).
    Evaluated e = evalSrc("color(42) cube(1);");
    ASSERT_TRUE(e.bodies[0].color.has_value());
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[0], 1.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[1], 1.0f);
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[2], 1.0f);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0, 1e-9); // geometry still produced
}

TEST(Color, NoChildrenIsEmpty) {
    Evaluated e = evalSrc("color(\"red\");");
    EXPECT_TRUE(e.bodies.empty());
}
