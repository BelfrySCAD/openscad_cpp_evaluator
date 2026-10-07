// Behaviour checked case by case against OpenSCAD 2026.02.01 (and upstream
// master's source where newer) in the 2026-10 builtin parity audit. Each
// expectation here is what the reference binary did; the comment names the
// way this evaluator used to differ.

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

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
