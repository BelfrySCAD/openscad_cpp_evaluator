// Behaviour checked case by case against OpenSCAD 2026.02.01 (and upstream
// master's source where newer) in the 2026-10 builtin parity audit. Each
// expectation here is what the reference binary did; the comment names the
// way this evaluator used to differ.

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <string>
#include <vector>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

struct Outcome {
    std::vector<std::string> log;
    double area = 0;    // total 2D area across sections
    double volume = 0;  // total 3D volume across solids
    size_t bodies = 0;
};

Outcome run(const std::string& src) {
    Outcome r;
    Evaluated e = evaluateSrc(src, [&](const std::string& m) { r.log.push_back(m); });
    for (const ColoredBody& b : e.bodies) {
        // Only what an export would write: not `%` scenery, not `#` ghosts.
        if (b.role == BodyRole::Background || b.role == BodyRole::HighlightGhost) continue;
        if (b.section) r.area += b.section->Area();
        else if (b.body && !b.body->IsEmpty()) r.volume += b.body->Volume();
        if ((b.section && !b.section->IsEmpty()) || (b.body && !b.body->IsEmpty())) ++r.bodies;
    }
    return r;
}

bool logged(const Outcome& r, const std::string& text) {
    for (const std::string& l : r.log)
        if (l.find(text) != std::string::npos) return true;
    return false;
}

}  // namespace

// -- crashes -----------------------------------------------------------------

// An index past the end of `points` read out of bounds: SIGBUS at 1e8, and
// silent undefined behaviour at smaller values.
TEST(Parity, PolygonOutOfRangeIndexIsDroppedWithAWarning) {
    const Outcome r = run("polygon([[0,0],[10,0],[10,10],[0,10]], [[0,1,2,100000000]]);");
    EXPECT_TRUE(logged(r, "Point index 100000000 is out of bounds (from paths[0][3])"));
    EXPECT_NEAR(r.area, 50.0, 1e-9);
}

// rands() with an infinite count saturated to INT_MAX and hung.
TEST(Parity, RandsInfiniteCountResetsToOne) {
    const Outcome r = run("echo(rands(0,1,1/0,1));");
    EXPECT_TRUE(logged(r, "rands() cannot create an infinite number of results"));
    EXPECT_TRUE(logged(r, "resetting number of results to 1"));
    EXPECT_TRUE(logged(r, "ECHO: [0.997185]"));
}

// A point at infinity reached Clipper2 and its range exception aborted the
// whole render; OpenSCAD makes the point [0,0] and carries on.
TEST(Parity, PolygonNonFinitePointBecomesOriginNotAnAbort) {
    const Outcome r = run("polygon([[0,0],[10,0],[10,10],[0,10],[5,1/0]]); translate([20,0]) square(1);");
    EXPECT_TRUE(logged(r, "Unable to convert points[4] = [5, inf] to a vec2 of numbers"));
    EXPECT_NEAR(r.area, 100.0 + 1.0, 1e-9);
}

// Any 2D boolean or offset over coordinates beyond Clipper2's range threw
// out of the whole render. The node is now dropped with a warning.
TEST(Parity, Oversized2DOperationIsSkippedNotFatal) {
    const Outcome r = run("union(){ square(1e20); square(1); } translate([20,0,0]) cube(1);");
    EXPECT_TRUE(logged(r, "coordinates too large for 2D geometry"));
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
}

// -- polygon/square/circle argument handling ----------------------------------

TEST(Parity, PolygonBadInputIsRepairedAsUpstreamDoes) {
    // A malformed point becomes [0,0]: the square loses a corner.
    Outcome r = run("polygon([[0,0],[10,\"a\"],[10,10],[0,10]]);");
    EXPECT_TRUE(logged(r, "Unable to convert points[1] = [10, \"a\"] to a vec2 of numbers"));
    EXPECT_NEAR(r.area, 50.0, 1e-9);
    // paths that is not a list of lists: every entry is reported, and with
    // no path left the points draw as one outline.
    r = run("polygon([[0,0],[10,0],[10,10],[0,10]], [0,1,2]);");
    EXPECT_TRUE(logged(r, "Unable to convert paths[0] = 0 to a vector of numbers"));
    EXPECT_NEAR(r.area, 100.0, 1e-9);
    r = run("polygon([[0,0],[10,0],[10,10],[0,10]], \"abc\");");
    EXPECT_TRUE(logged(r, "Unable to convert paths = \"abc\""));
    EXPECT_NEAR(r.area, 100.0, 1e-9);
    // A string index is skipped; a negative one is point 0, as built.
    r = run("polygon([[0,0],[10,0],[10,10],[0,10]], [[0,1,\"a\",3]]);");
    EXPECT_TRUE(logged(r, "Unable to convert paths[0][2] = \"a\" to a number"));
    EXPECT_NEAR(r.area, 50.0, 1e-9);
    // A missing points list no longer stops the script.
    r = run("polygon(); square(1);");
    EXPECT_TRUE(logged(r, "Unable to convert points = undef to a vector of coordinates"));
    EXPECT_NEAR(r.area, 1.0, 1e-9);
}

TEST(Parity, SquareAndCircleKeepTheirDefaultsForBadSizes) {
    Outcome r = run("square([3]);");
    EXPECT_TRUE(logged(r, "Unable to convert square(size=[3], ...) parameter to a number or a vec2 of numbers"));
    EXPECT_NEAR(r.area, 1.0, 1e-9);
    r = run("square([3,4,5]);");  // a vec2 is exactly two numbers
    EXPECT_NEAR(r.area, 1.0, 1e-9);
    r = run("circle(\"a\", $fn=4);");  // r stays 1, not 0
    EXPECT_NEAR(r.area, 2.0, 1e-9);
    // circle(r, d): both positional, d wins with upstream's warning.
    r = run("circle(3, 5, $fn=4);");
    EXPECT_TRUE(logged(r, "Ignoring radius variable \"r\" as diameter \"d\" is defined too."));
    EXPECT_NEAR(r.area, 2.0 * 2.5 * 2.5, 1e-9);
}

TEST(Parity, SquareCentersOnlyForARealBool) {
    for (const char* c : {"1", "\"yes\"", "[1]"}) {
        Evaluated e = evaluateSrc(std::string("square(2, center=") + c + ");");
        ASSERT_EQ(e.bodies.size(), 1u) << c;
        EXPECT_NEAR(e.bodies[0].section->Bounds().min.x, 0.0, 1e-12) << c;
    }
    Evaluated e = evaluateSrc("square(2, center=true);");
    EXPECT_NEAR(e.bodies[0].section->Bounds().min.x, -1.0, 1e-12);
}

// -- rands ---------------------------------------------------------------------

TEST(Parity, RandsMatchesUpstreamForOddArguments) {
    // Reversed bounds are swapped, not fed backwards to the distribution.
    EXPECT_TRUE(logged(run("echo(rands(1,0,3,7));"), "ECHO: [0.227339, 0.318972, 0.978223]"));
    // The count is |count|, truncated.
    EXPECT_TRUE(logged(run("echo(rands(0,1,-2,7));"), "ECHO: [0.227339, 0.318972]"));
    // A non-integer seed is hashed as upstream does, not truncated to 1.
    EXPECT_FALSE(logged(run("echo(rands(0,1,1,1.5) == rands(0,1,1,1));"), "ECHO: true"));
    // An infinite bound is reset with upstream's two warnings.
    const Outcome r = run("echo(rands(0,1/0,1,1));");
    EXPECT_TRUE(logged(r, "rands() range max cannot be infinite"));
    EXPECT_TRUE(logged(r, "resetting to 89884656743115785407263711865852178399035283762922498299458738401578630390014"));
}

// -- transforms ----------------------------------------------------------------

namespace {
manifold::Rect sectionBounds(const std::string& src) {
    Evaluated e = evaluateSrc(src);
    manifold::Rect r;
    for (const ColoredBody& b : e.bodies)
        if (b.section) r = r.Union(b.section->Bounds());
    return r;
}
manifold::Box solidBounds(const std::string& src) {
    Evaluated e = evaluateSrc(src);
    manifold::Box r;
    for (const ColoredBody& b : e.bodies)
        if (b.body && !b.body->IsEmpty()) r = r.Union(b.body->BoundingBox());
    return r;
}
}  // namespace

// A transform out of the XY plane rides on a 2D shape for display, but every
// 2D operation sees it projected, as OpenSCAD's F6 does. union, offset, hull
// and linear_extrude used the shape as if the transform had not happened.
TEST(Parity, OperationsSeeA2DShapesOutOfPlaneTransformProjected) {
    EXPECT_NEAR(sectionBounds("union(){ multmatrix([[1,0,0,1],[0,1,0,0],[0,0,1,3]]) square(1); "
                              "translate([5,0]) square(1); }").min.x,
                1.0, 1e-9);
    const manifold::Box b =
        solidBounds("linear_extrude(1) multmatrix([[1,0.5,0,1],[0,1,0,2],[0,0,1,3]]) square([1,2]);");
    EXPECT_NEAR(b.min.x, 1.0, 1e-9);
    EXPECT_NEAR(b.max.x, 3.0, 1e-9);
    EXPECT_NEAR(b.max.y, 4.0, 1e-9);
    // A quarter turn out of the plane is singular: the shape is removed.
    const Outcome r = run("linear_extrude(1) rotate([90,0,0]) square(1); cube(0.5);");
    EXPECT_TRUE(logged(r, "Scaling a 2D object with 0 - removing object"));
    EXPECT_NEAR(r.volume, 0.125, 1e-9);
}

TEST(Parity, TwoDRotateHonoursTheAxisAndResizeWorks) {
    // v=[0,0,-1] turns the other way (it used to be ignored for 2D).
    EXPECT_NEAR(sectionBounds("rotate(a=30, v=[0,0,-1]) square([1,2]);").min.y, -0.5, 1e-9);
    const manifold::Rect r = sectionBounds("resize([10,20]) square([1,2]);");
    EXPECT_NEAR(r.max.x, 10.0, 1e-9);
    EXPECT_NEAR(r.max.y, 20.0, 1e-9);
    // A negative newsize leaves its axis alone.
    EXPECT_NEAR(solidBounds("resize([-10,20,30]) cube([1,2,3]);").max.x, 1.0, 1e-9);
}

TEST(Parity, TransformArgumentsFollowUpstream) {
    // mirror with a zero normal is the identity, not a deletion.
    EXPECT_NEAR(run("mirror([0,0,0]) cube([1,2,3]);").volume, 6.0, 1e-9);
    // multmatrix fills from the identity and divides by [3][3].
    EXPECT_NEAR(run("multmatrix([[2,0],[0,3]]) cube([1,2,3]);").volume, 36.0, 1e-9);
    EXPECT_NEAR(run("multmatrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,2]]) cube([1,2,3]);").volume, 0.75, 1e-9);
    // Arguments a transform cannot use are ignored with upstream's warning.
    Outcome r = run("scale([2]) cube(1);");
    EXPECT_TRUE(logged(r, "Unable to convert scale([2]) parameter to a number, a vec3 or vec2 of numbers or a number"));
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
    r = run("rotate(a=[10,20,30], v=[1,0,0]) cube(1);");
    EXPECT_TRUE(logged(r, "When parameter a is supplied as vector, v is ignored"));
    // A NaN in the final matrix removes the object, with upstream's warning.
    r = run("scale(0/0) cube(1); translate([3,0,0]) cube(1);");
    EXPECT_TRUE(logged(r, "Transformation matrix contains Not-a-Number and/or Infinity - removing object."));
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
}

// -- modifiers and operands ------------------------------------------------------

TEST(Parity, BackgroundStatementIsNotAnOperand) {
    EXPECT_NEAR(run("difference(){ %cube(2); translate([1,1,1]) cube(2); }").volume, 8.0, 1e-9);
    EXPECT_NEAR(run("intersection(){ cube(2); %translate([5,0,0]) cube(1); }").volume, 8.0, 1e-9);
    // ...but a module or union() holding only `%` is an empty operand.
    EXPECT_NEAR(run("module m(){ %cube(3); } intersection(){ cube(2); m(); }").volume, 0.0, 1e-9);
}

// A `#` operand is merged AND drawn as a ghost; the ghost used to be
// exported (and re-used by an outer boolean) as real geometry.
TEST(Parity, HighlightedOperandIsNotExportedTwice) {
    Evaluated e = evaluateSrc("difference(){ cube(2); #translate([1,1,1]) cube(2); }");
    double exported = 0;
    bool ghost = false;
    for (const ColoredBody& b : e.bodies) {
        if (b.role == BodyRole::HighlightGhost) ghost = true;
        else if (b.body) exported += b.body->Volume();
    }
    EXPECT_TRUE(ghost);
    EXPECT_NEAR(exported, 7.0, 1e-9);
    // A `#` with nothing merging it is ordinary, exported geometry.
    Evaluated top = evaluateSrc("#cube(2);");
    ASSERT_EQ(top.bodies.size(), 1u);
    EXPECT_EQ(top.bodies[0].role, BodyRole::Highlight);
}

// Manifold::MinkowskiSum unions its first operand into the result.
TEST(Parity, MinkowskiIsExactForOperandsAwayFromTheOrigin) {
    const Outcome r = run("minkowski(){ cube(1); translate([5,0,0]) cube(1); }");
    EXPECT_NEAR(r.volume, 8.0, 1e-9);
    EXPECT_NEAR(solidBounds("minkowski(){ cube(1); translate([5,0,0]) cube(1); }").min.x, 5.0, 1e-9);
}

TEST(Parity, IntersectionForIntersectsEveryStatementFlat) {
    EXPECT_NEAR(run("intersection_for(i=[0:1]){ translate([i*0.5,0,0]) cube(1); translate([3,0,0]) cube(1); }").volume,
                0.0, 1e-9);
}

// -- 3D primitives -------------------------------------------------------------

// Non-positive or non-finite sizes draw nothing, as each upstream
// createGeometry does. sphere(-1) was an inside-out sphere that added
// material to any difference(); cylinder(r2=-2) drew a prism.
TEST(Parity, InvalidPrimitiveSizesDrawNothing) {
    EXPECT_NEAR(run("difference(){ cube(4, center=true); sphere(-1, $fn=12); }").volume, 64.0, 1e-9);
    for (const char* src : {"sphere(-1);", "sphere(0);", "sphere(1/0);", "cylinder(h=2, r1=1, r2=-2);",
                            "cylinder(h=-2, r=1);", "cylinder(h=2, r1=0, r2=0);", "cube([1,1,0]);", "cube(-1);",
                            "hull() cube([1,1,0]);"}) {
        const Outcome r = run(src);
        EXPECT_EQ(r.bodies, 0u) << src;
    }
}

// Arguments that are not numbers keep the defaults rather than becoming 0.
TEST(Parity, PrimitiveArgumentsFollowUpstream) {
    EXPECT_GT(run("sphere(\"a\", $fn=8);").volume, 1.0);  // r = 1, not 0
    EXPECT_NEAR(run("cylinder(h=\"a\", r=1, $fn=4);").volume, 2.0, 1e-9);  // h = 1
    // center= counts only as a real bool.
    EXPECT_NEAR(solidBounds("cube(2, center=1);").min.x, 0.0, 1e-12);
    EXPECT_NEAR(solidBounds("cylinder(h=2, r=1, center=1);").min.z, 0.0, 1e-12);
    // d beats r, with upstream's warnings.
    Outcome r = run("sphere(r=2, d=10, $fn=8);");
    EXPECT_TRUE(logged(r, "Ignoring radius variable \"r\" as diameter \"d\" is defined too."));
    r = run("cylinder(h=1, r=1, r1=2, $fn=8);");
    EXPECT_TRUE(logged(r, "Cylinder parameters ambiguous"));
}

// $fa/$fs below 0.01 are clamped to 0.01 with upstream's warning; they were
// reset to the 12/2 defaults (30 segments here, not OpenSCAD's 32).
TEST(Parity, FragmentAngleAndSizeAreClampedNotReset) {
    Evaluated e = evaluateSrc("cylinder(h=1, r=10, $fa=0);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(e.bodies[0].body->NumVert(), 64u);  // 32 segments, top and bottom
    const Outcome r = run("cylinder(h=1, r=10, $fa=0);");
    EXPECT_TRUE(logged(r, "$fa too small - clamping to 0.010000"));
}

TEST(Parity, PolyhedronBadInputIsReportedAndDroppedNotGuessed) {
    // An out-of-range index is dropped with upstream's warning; it was
    // silently remapped to point 0.
    Outcome r = run("polyhedron([[0,0,0],[10,0,0],[0,10,0],[0,0,10]], [[0,1,2],[0,3,1],[0,2,3],[1,3,9]]);");
    EXPECT_TRUE(logged(r, "Point index 9 is out of bounds (from faces[3][2])"));
    // Missing faces warns and the script carries on.
    r = run("polyhedron([[0,0,0],[1,0,0],[0,1,0]]); cube(1);");
    EXPECT_TRUE(logged(r, "Unable to convert faces = undef to a vector of vector of point indices"));
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
    // The legacy triangles= alias still works, with 2026.02.01's notice.
    r = run("polyhedron(points=[[0,0,0],[10,0,0],[0,10,0],[0,0,10]], triangles=[[0,1,2],[0,3,1],[0,2,3],[1,3,2]]);");
    EXPECT_TRUE(logged(r, "DEPRECATED: polyhedron(triangles=[]) will be removed in future releases."));
    EXPECT_GT(r.volume, 100.0);
}

// -- offset, extrusions, import, surface ------------------------------------------

// delta= is a sharp (miter) offset and chamfer=true a cut corner; the two
// were swapped. The miter limit is upstream's 1e6, not Manifold's 2.
TEST(Parity, OffsetDeltaIsSharpAndChamferCuts) {
    EXPECT_NEAR(run("offset(delta=2) square(10);").area, 196.0, 1e-9);
    // OpenSCAD's square join cuts each corner: 193.2548 (checked, 2026.02.01).
    EXPECT_NEAR(run("offset(delta=2, chamfer=true) square(10);").area, 193.2548, 1e-3);
    EXPECT_NEAR(sectionBounds("offset(delta=1) polygon([[0,0],[10,0],[0,1]]);").max.x, 30.05, 0.01);
    const Outcome both = run("offset(r=1, delta=3, $fn=8) square(4);");
    EXPECT_TRUE(logged(both, "Ignoring \"delta\" argument as \"r\" is defined too."));
}

TEST(Parity, RotateExtrudeSignsAndSides) {
    // A negative angle is a sweep the other way, not an inside-out solid.
    EXPECT_NEAR(run("rotate_extrude(angle=-90) translate([3,0]) square([1,2]);").volume, 10.7151, 1e-3);
    EXPECT_NEAR(run("difference(){ cube([10,10,1], center=true); "
                    "rotate_extrude(angle=-90) translate([3,0]) square([1,2]); }").volume,
                97.3215, 1e-3);
    // A profile left of the axis is swept where it lies.
    EXPECT_NEAR(run("rotate_extrude($fn=16) translate([-4,0]) square([1,2]);").volume, 42.8604, 1e-3);
    // Across the axis: upstream's error, nothing drawn, the script carries on.
    const Outcome r = run("rotate_extrude() translate([-1,0]) square([3,2]); cube(1);");
    EXPECT_TRUE(logged(r, "ERROR: Children of rotate_extrude() may not lie across the Y axis (Range of X coords for "
                          "all children [-1.00 : 2.00])"));
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
}

// Zero scale with twist or a non-uniform scale: upstream's diagonal choice
// (reversed on the collapsed top slice), not Manifold::Extrude's.
TEST(Parity, LinearExtrudeToZeroScaleMatchesUpstream) {
    EXPECT_NEAR(run("linear_extrude(10, scale=0, twist=180) square(4, center=true);").volume, 55.3393, 1e-3);
    EXPECT_NEAR(run("linear_extrude(10, scale=[0,1], twist=60) square(4, center=true);").volume, 81.7820, 1e-3);
    EXPECT_NEAR(run("linear_extrude(10, scale=[2,0], slices=4) square(4, center=true);").volume, 106.6667, 1e-3);
}

TEST(Parity, ImportCenterAppliesToEveryFormat) {
    const auto path = std::filesystem::temp_directory_path() / "parity_center.off";
    {
        std::ofstream f(path);
        f << "OFF\n4 4 0\n1 1 1\n4 1 1\n1 5 1\n1 1 7\n3 0 2 1\n3 0 1 3\n3 0 3 2\n3 1 2 3\n";
    }
    const manifold::Box b = solidBounds("import(\"" + path.generic_string() + "\", center=true);");
    EXPECT_NEAR(b.min.x, -1.5, 1e-9);
    EXPECT_NEAR(b.max.z, 3.0, 1e-9);
    // Only a real bool centres.
    EXPECT_NEAR(solidBounds("import(\"" + path.generic_string() + "\", center=1);").min.x, 1.0, 1e-9);
    std::filesystem::remove(path);
}

// The first line of a .dat is y = 0 (it was mirrored), and the base sits
// below the lowest height (it was at z = 0, so negative heights inverted).
TEST(Parity, SurfaceDatOrientationAndBase) {
    const auto path = std::filesystem::temp_directory_path() / "parity_surface.dat";
    {
        std::ofstream f(path);
        f << "-5 -5 -5\n9 9 9\n";
    }
    const Outcome r = run("surface(\"" + path.generic_string() + "\");");
    const manifold::Box b = solidBounds("surface(\"" + path.generic_string() + "\");");
    EXPECT_NEAR(b.min.z, -6.0, 1e-9);
    EXPECT_GT(r.volume, 0.0);
    std::filesystem::remove(path);
}

// -- values ----------------------------------------------------------------------

// Builtin functions read their arguments in order, names ignored; the names
// were dropped instead, so sin(a=90) was sin(undef).
TEST(Parity, BuiltinFunctionsTakeNamedArgumentsInOrder) {
    EXPECT_TRUE(logged(run("echo(sin(a=90), pow(y=3,x=2), concat(a=[1],b=[2]), len(v=[1,2]), str(a=1,b=2), max(a=1,b=5));"),
                       "ECHO: 1, 9, [1, 2], 2, \"12\", 5"));
    EXPECT_TRUE(logged(run("echo(rands(0,10,2,seed_value=5) == rands(0,10,2,seed_value=5));"), "ECHO: true"));
}

TEST(Parity, LogTakesABase) {
    EXPECT_TRUE(logged(run("echo(log(2,8), log(0.5,4), log(100));"), "ECHO: 3, -2, 2"));
}

// Printed through double rounding: 123456.5 printed 123456, and a subnormal
// printed "infe-323".
TEST(Parity, NumbersPrintToSixSignificantDigits) {
    EXPECT_TRUE(logged(run("echo(123456.5, 99.99995, 1e-300/1e23, 1234567, 0.0001, 0.00001);"),
                       "ECHO: 123457, 99.9999, 9.88131e-324, 1.23457e+6, 0.0001, 0.00001"));
}

TEST(Parity, RoundIsExactNotFloorOfXPlusHalf) {
    EXPECT_TRUE(logged(run("echo(round(0.49999999999999994), round(2.5), round(-2.5));"), "ECHO: 0, 3, -3"));
}

TEST(Parity, CrossWarnsAboutBadElements) {
    const Outcome r = run("echo(cross([1,0,1/0],[0,1,0]));");
    EXPECT_TRUE(logged(r, "Invalid value (INF) in parameter vector for cross()"));
    EXPECT_TRUE(logged(r, "ECHO: undef"));
    EXPECT_TRUE(logged(run("echo(cross([1,2],[3,4]));"), "ECHO: -2"));
}

TEST(Parity, LookupRejectsANonFiniteKey) {
    const Outcome r = run("echo(lookup(1/0,[[0,0],[1,10]]));");
    EXPECT_TRUE(logged(r, "lookup(inf, ...) first argument is not a number"));
    EXPECT_TRUE(logged(r, "ECHO: undef"));
}

// \U was not an escape at all, and a bad escape passed silently.
TEST(Parity, StringEscapes) {
    EXPECT_TRUE(logged(run("echo(len(\"\\U01F600\"), ord(\"\\U01F600\"));"), "ECHO: 1, 128512"));
    EXPECT_TRUE(logged(run("echo(\"\\q\");"), "Undefined escape sequence"));
}

TEST(Parity, ParentModulesCountsTheModuleItself) {
    EXPECT_TRUE(logged(run("module m() echo($parent_modules); m();"), "ECHO: 1"));
}

// [0:"a"] was [0:0]; OpenSCAD 2026.02 makes it undef, and master warns.
TEST(Parity, RangeBoundsMustBeNumbers) {
    const Outcome r = run("echo([0:\"a\"], [undef:1], [0:\"x\":3], [for (i=[0:\"a\"]) i]);");
    EXPECT_TRUE(logged(r, "ECHO: undef, undef, undef, []"));
    EXPECT_TRUE(logged(r, "Unable to convert [0:...:\"a\"] to a range"));
    EXPECT_TRUE(logged(r, "Unable to convert [...:\"x\":...] to a step value"));
}

// The later binding won; OpenSCAD keeps the first and warns with the value.
TEST(Parity, LetKeepsTheFirstOfARepeatedName) {
    for (const char* src : {"echo(let(a=1, a=a+3) a);", "let(a=1, a=a+3) echo(a);", "echo([let(a=1, a=a+3) a][0]);",
                            "function f(a) = let(a=1, a=a+3) a; echo(f(0));"}) {
        const Outcome r = run(src);
        EXPECT_TRUE(logged(r, "Ignoring duplicate variable assignment \"a\" = 4")) << src;
        EXPECT_TRUE(logged(r, "ECHO: 1")) << src;
    }
}

namespace {
std::string assertError(const std::string& src) {
    try {
        run(src);
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}
}  // namespace

// assert() passed; a message was always quoted; an undef message vanished.
// The message is only evaluated on failure -- which OpenSCAD does NOT do: it
// evaluates every argument, pass or fail. That divergence is deliberate (see
// Evaluator::evalAssert).
TEST(Parity, AssertFollowsUpstream) {
    EXPECT_NE(assertError("assert();").find("Assertion failed"), std::string::npos);
    EXPECT_NE(assertError("function f() = assert() 1; x = f();").find("Assertion failed"), std::string::npos);
    EXPECT_NE(assertError("assert(false, 42);").find("Assertion 'false' failed: 42"), std::string::npos);
    EXPECT_NE(assertError("x = assert(false, \"m\") 1;").find("Assertion 'false' failed: \"m\""), std::string::npos);
    EXPECT_NE(assertError("assert(undef, undef);").find("failed: undef"), std::string::npos);
    EXPECT_NE(assertError("assert(message=\"m\");").find("Assertion failed: \"m\""), std::string::npos);
    EXPECT_FALSE(logged(run("x = assert(true, echo(\"lazy\") 1) 2;"), "ECHO: \"lazy\""));
    EXPECT_TRUE(logged(run("assert(1, 2, 3);"), "Too many unnamed arguments supplied"));
}

TEST(Parity, ForWithNoVariablesRunsZeroTimes) {
    EXPECT_FALSE(logged(run("for() echo(\"in\");"), "ECHO"));
    EXPECT_FALSE(logged(run("module q() for() echo(\"in\"); q();"), "ECHO"));
    EXPECT_FALSE(logged(run("intersection_for() echo(\"in\");"), "ECHO"));
}

// children(undef) drew every child; a bad vector element aborted the rest.
TEST(Parity, ChildrenIndexMustBeGiven) {
    const Outcome r = run("module m() children(undef); m() { echo(\"a\"); }");
    EXPECT_TRUE(logged(r, "Bad parameter type (undef) for children"));
    EXPECT_FALSE(logged(r, "ECHO"));
    const Outcome v = run("module m() children([\"x\", 0]); m() { echo(\"a\"); }");
    EXPECT_TRUE(logged(v, "Bad parameter type (x) for children"));
    EXPECT_TRUE(logged(v, "ECHO: \"a\""));
    EXPECT_TRUE(logged(run("module m() children(0); m();"), "Children index (0) out of bounds (0 children)"));
}

// The box was the exact outline; OpenSCAD's is hinted and grid-fitted.
TEST(Parity, TextMetricsBoxIsGridFitted) {
    EXPECT_TRUE(logged(run("echo(textmetrics(\"Hello\"));"),
                       "position = [1.1392, -0.1408]; size = [29.929, 10.208]; ascent = 10.0672; descent = -0.1408;"));
}

// -- messages (tier 5) -------------------------------------------------------------

// The condition was quoted as written; the reference parenthesises every
// binary operator and ternary.
TEST(Parity, AssertQuotesItsConditionAsTheReferenceSpellsIt) {
    EXPECT_NE(assertError("a = 1; assert(a + 1 * 2 > 3);").find("Assertion '((a + (1 * 2)) > 3)' failed"),
              std::string::npos);
    EXPECT_NE(assertError("assert(1 > 2 ? false : true == false);").find("Assertion '((1 > 2) ? false : (true == false))' failed"),
              std::string::npos);
}

TEST(Parity, UnknownVariableIsDoubleQuoted) {
    EXPECT_TRUE(logged(run("echo(nosuch);"), "Ignoring unknown variable \"nosuch\""));
}

TEST(Parity, TextWarnsAboutANonStringText) {
    EXPECT_TRUE(logged(run("text(42);"), "text(..., text=42) Invalid type: expected string, found number"));
    EXPECT_FALSE(logged(run("text(undef);"), "Invalid type"));
}

// A zero step was a silently empty range; the reference counts it as its
// uint32 ceiling and warns wherever it is iterated.
TEST(Parity, ZeroStepRangeIsTooMany) {
    const char* kFor = "Bad range parameter in for statement: too many elements (4294967295)";
    EXPECT_TRUE(logged(run("for (i = [0:0:5]) echo(i);"), kFor));
    EXPECT_TRUE(logged(run("echo([for (i = [0:0:5]) i]);"), kFor));
    EXPECT_TRUE(logged(run("module m() children([0:0:1]); m() cube(1);"),
                       "Bad range parameter for children: too many elements (4294967295)"));
    EXPECT_FALSE(logged(run("echo([for (i = [0:0/0:5]) i]);"), "too many"));  // NaN stays silent
}
