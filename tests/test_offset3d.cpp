// offset() of a solid: r=, delta= and delta= chamfer=true, outward and inward
// (src/builtins/offset3d.cpp). Exact cases only: every one below has an
// analytic volume or an exact Minkowski reference (a cube or octahedron tool
// has no curve to tessellate).

#include <gtest/gtest.h>

#include <cmath>

#include "test_helpers.hpp"

using namespace oscadeval;
using namespace oscadeval::test;
using manifold::Manifold;

namespace {

// An L: a 20x10x10 block with an 11x12x6 step cut out of one end.
const char* kL = "difference() { cube([20, 10, 10]); translate([10, -1, 5]) cube([11, 12, 6]); }";

// An orthogonal part with saddle vertices everywhere: a block with a through
// hole, a boss on top and a notch out of one edge.
const char* kPart =
    "union() {"
    "  difference() { cube([20, 20, 10]); translate([7, 7, -1]) cube([6, 6, 12]);"
    "                 translate([16, -1, 7]) cube([4, 22, 3]); }"
    "  translate([2, 2, 10]) cube([5, 5, 4]);"
    "}";

const char* kOcta = "polyhedron([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]],"
                    "[[0,2,4],[2,1,4],[1,3,4],[3,0,4],[2,0,5],[1,2,5],[3,1,5],[0,3,5]]);";

Manifold solid(const std::string& src) {
    Evaluated e = evalSrc(src);
    std::vector<Manifold> ms;
    for (const ColoredBody& b : e.bodies)
        if (b.body) ms.push_back(*b.body);
    return ms.empty() ? Manifold() : Manifold::BatchBoolean(ms, manifold::OpType::Add);
}

double vol(const std::string& src) { return solid(src).Volume(); }

// Relative: every piece is grown by 1e-9 of the part's size, so that two
// offsets meeting exactly overlap rather than touch.
#define EXPECT_VOL(src, want) EXPECT_NEAR(vol(src), (want), 1e-6 * std::max(1.0, std::fabs(want)))

double symdiff(const Manifold& a, const Manifold& b) { return (a - b).Volume() + (b - a).Volume(); }

} // namespace

TEST(Offset3D, CubeEveryModeBothWays) {
    EXPECT_VOL("offset(delta=1) cube(10);", 1728);
    EXPECT_VOL("offset(delta=1, chamfer=true) cube(10);", 1000 + 600 + 60 + 8.0 / 6);
    // Inward every mode is the same cube: a convex solid has no corner to round.
    EXPECT_VOL("offset(delta=-1) cube(10);", 512);
    EXPECT_VOL("offset(delta=-1, chamfer=true) cube(10);", 512);
    EXPECT_VOL("offset(r=-1, $fn=24) cube(10);", 512);
    // r= outward is the Minkowski sum with an inscribed ball: a little under
    // the true rounded cube. ($fn as for 2D: the default gives r=1 five sides.)
    const double exact = 1600 + 120 * M_PI / 4 + 4.0 / 3 * M_PI;
    const double r = vol("offset(r=1, $fn=24) cube(10);");
    EXPECT_LT(r, exact);
    EXPECT_GT(r, exact * 0.99);
}

TEST(Offset3D, ConvexMiteredOffsetIsExact) {
    // A convex solid's delta= offset is built as one hull: a 64-sided prism
    // grows to the 64-gon whose apothem is one more, extruded 2 taller.
    const double n = 64, a = 5 * std::cos(M_PI / n) + 1;
    EXPECT_VOL("offset(delta=1) cylinder(h=10, r=5, $fn=64);", 12 * n * a * a * std::tan(M_PI / n));
}

TEST(Offset3D, ThinPlateVanishesInward) {
    EXPECT_NEAR(vol("offset(delta=-1) cube([10, 10, 1.5]);"), 0, 1e-9);
    EXPECT_NEAR(vol("offset(r=-1) cube([10, 10, 1.5]);"), 0, 1e-9);
}

TEST(Offset3D, LShapeMatchesAnalyticVolumes) {
    const std::string L = kL;
    EXPECT_VOL("offset(delta=1) " + L, 2568);
    EXPECT_VOL("offset(delta=-1) " + L, 752);
    EXPECT_VOL("offset(delta=-1, chamfer=true) " + L, 756);
    // r= inward rounds the one concave edge (8 long) with the faceted ball.
    EXPECT_NEAR(vol("offset(r=-1, $fn=24) " + L), (94 + 1 - M_PI / 4) * 8, 2e-3 * 752);
}

TEST(Offset3D, OrthogonalPartMatchesMinkowski) {
    const std::string P = kPart;
    for (double d : {1.0, 2.5}) {
        const std::string ds = std::to_string(d);
        const std::string cube = "cube(" + std::to_string(2 * d) + ", center=true);";
        const std::string octa = "scale(" + ds + ") " + kOcta;
        const Manifold ref = solid("minkowski() { " + P + cube + " }");
        EXPECT_LT(symdiff(solid("offset(delta=" + ds + ") " + P), ref), 1e-6 * ref.Volume()) << d;
        const Manifold refC = solid("minkowski() { " + P + octa + " }");
        EXPECT_LT(symdiff(solid("offset(delta=" + ds + ", chamfer=true) " + P), refC), 1e-6 * refC.Volume()) << d;
        const Manifold er = solid("minkowski_difference() { " + P + cube + " }");
        EXPECT_LT(symdiff(solid("offset(delta=-" + ds + ") " + P), er), 1e-6 * er.Volume()) << d;
        const Manifold erC = solid("minkowski_difference() { " + P + octa + " }");
        EXPECT_LT(symdiff(solid("offset(delta=-" + ds + ", chamfer=true) " + P), erC), 1e-6 * erC.Volume()) << d;
    }
}

TEST(Offset3D, TopologyChangesAreKept) {
    // A ring with a 4-wide hole: kept at delta=1, closed at delta=2.5.
    const std::string ring = "difference() { cube([12, 12, 4]); translate([4, 4, -1]) cube([4, 4, 6]); }";
    EXPECT_EQ(solid("offset(delta=1) " + ring).Genus(), 1);
    const Manifold closed = solid("offset(delta=2.5) " + ring);
    EXPECT_EQ(closed.Genus(), 0);
    EXPECT_EQ(closed.Decompose().size(), 1u);
    // A U whose 3-thick base vanishes at delta=-2: the two arms come apart.
    const std::string U = "difference() { cube([20, 10, 15]); translate([5, -1, 3]) cube([10, 12, 20]); }";
    EXPECT_EQ(solid("offset(delta=-2) " + U).Decompose().size(), 2u);
    EXPECT_EQ(solid("offset(delta=-1) " + U).Decompose().size(), 1u);
}

TEST(Offset3D, TouchingAtExactlyTheDistanceJoinsEveryContact) {
    // Grown by 0.5 with chamfers, two parts of this block meet in two places at
    // once: genus -1 (two shells) just short of 0.5, genus 1 just past it. At
    // exactly 0.5 touching counts as joined, at both contacts -- Manifold's own
    // Minkowski sum joins only one of them there (genus 0).
    const std::string P =
        "difference() { difference() { difference() { union() { union() { union() { cube(8);"
        " translate([2, -2, -2]) cube([5, 1, 4]); } translate([3, 3, -2]) cube([5, 4, 6]); }"
        " translate([6, 6, -1]) cube([2, 6, 4]); } translate([2, 7, -1]) cube([5, 6, 6]); }"
        " translate([4, 2, 5]) cube([3, 1, 3]); } translate([2, -1, -2]) cube([2, 6, 6]); }";
    const Manifold o = solid("offset(delta=0.5, chamfer=true) " + P);
    EXPECT_EQ(o.Decompose().size(), 1u);
    EXPECT_EQ(o.Genus(), 1);
    EXPECT_NEAR(o.Volume(), 818.75, 1e-6 * 818.75);
}

TEST(Offset3D, SplitEdgesGiveTheSameOffset) {
    // A 10-cube whose faces are each a 2x2 grid: every edge is two triangle
    // edges and every face eight triangles. Its offset is the plain cube's.
    // (The halves of a split edge once got a wedge each, meeting face to face;
    // on t41 seed 18's refine(2) that left a zero-width crack, genus 1, that
    // cost 0.27 of volume. This cube came out right even then: it guards the
    // one-wedge-per-straight-edge path, it does not reproduce that crack.)
    const std::string grid =
    "polyhedron([[0,0,0],[0,0,5],[0,0,10],[0,5,0],[0,5,5],[0,5,10],[0,10,0],[0,10,5],[0,10,10],[5,0,0"
    "],[5,0,5],[5,0,10],[5,5,0],[5,5,10],[5,10,0],[5,10,5],[5,10,10],[10,0,0],[10,0,5],[10,0,10],[10,"
    "5,0],[10,5,5],[10,5,10],[10,10,0],[10,10,5],[10,10,10]], [[0,3,4],[0,4,1],[1,4,5],[1,5,2],[3,6,7"
    "],[3,7,4],[4,7,8],[4,8,5],[18,21,20],[18,20,17],[19,22,21],[19,21,18],[21,24,23],[21,23,20],[22,"
    "25,24],[22,24,21],[1,10,9],[1,9,0],[2,11,10],[2,10,1],[10,18,17],[10,17,9],[11,19,18],[11,18,10]"
    ",[6,14,15],[6,15,7],[7,15,16],[7,16,8],[14,23,24],[14,24,15],[15,24,25],[15,25,16],[0,9,12],[0,1"
    "2,3],[3,12,14],[3,14,6],[9,17,20],[9,20,12],[12,20,23],[12,23,14],[5,13,11],[5,11,2],[8,16,13],["
    "8,13,5],[13,22,19],[13,19,11],[16,25,22],[16,22,13]]);";
    ASSERT_NEAR(vol(grid), 1000, 1e-9);
    for (const std::string& mode : {"delta=1", "delta=1, chamfer=true", "delta=-1", "delta=-1, chamfer=true"}) {
        const Manifold a = solid("offset(" + mode + ") cube(10);"), b = solid("offset(" + mode + ") " + grid);
        EXPECT_NEAR(b.Volume(), a.Volume(), 1e-6 * a.Volume()) << mode;
        EXPECT_EQ(b.Genus(), 0) << mode;
        EXPECT_EQ(b.Decompose().size(), 1u) << mode;
    }
}

TEST(Offset3D, FarFromTheOriginIsTheSame) {
    // The offset runs at the origin: Manifold's union lost 0.35 of this
    // volume at 1e5 from it.
    EXPECT_VOL("offset(delta=1, chamfer=true) translate([1e5, 0, 0]) cube(10);", 1000 + 600 + 60 + 8.0 / 6);
}

TEST(Offset3D, FeatureLevel) {
    std::vector<std::string> out;
    evalSrc("echo(supported_feature(\"offset-3d\"));", [&](const std::string& m) { out.push_back(m); });
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0], "ECHO: 1");
}

TEST(Offset3D, TwoDimensionalOffsetIsUnchanged) {
    // A 2D child still takes Clipper's polygon offset, not the solid one.
    Evaluated e = evalSrc("linear_extrude(1) offset(delta=1) square(10);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 144, 1e-9);
}
