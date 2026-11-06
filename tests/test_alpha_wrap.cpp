// alpha_wrap(alpha, offset) -- shrink-wrap the children (BelfrySCAD #941).

#include <gtest/gtest.h>

#include <cmath>

#include "test_helpers.hpp"

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

// An L bracket with a 4-wide hole and a 4-wide slot: genus 2.
const char* kBracket =
    "difference() {"
    "  union() { cube([40, 10, 4]); cube([4, 10, 30]); }"
    "  translate([20, 5, -1]) cylinder(d=4, h=10, $fn=48);"
    "  translate([-1, 3, 12]) cube([10, 4, 10]);"
    "}";

std::vector<std::string> warningsOf(const std::string& src) {
    std::vector<std::string> w;
    evalSrc(src, [&](const std::string& m) { if (m.rfind("WARNING", 0) == 0) w.push_back(m); });
    return w;
}

const manifold::Manifold& soleBody(const Evaluated& e) { return *e.bodies.at(0).body; }

} // namespace

TEST(AlphaWrap, BridgesGapsNarrowerThanAlpha) {
    Evaluated plain = evalSrc(std::string(kBracket) + ";");
    ASSERT_EQ(soleBody(plain).Genus(), 2);
    // A ball of radius 3 cannot enter the 4-wide hole or slot: both capped.
    Evaluated wide = evalSrc(std::string("alpha_wrap(3, 0.5) ") + kBracket + ";");
    ASSERT_EQ(wide.bodies.size(), 1u);
    EXPECT_EQ(soleBody(wide).Status(), manifold::Manifold::Error::NoError);
    EXPECT_EQ(soleBody(wide).Genus(), 0);
    // One of radius 0.5 can: both kept.
    Evaluated narrow = evalSrc(std::string("alpha_wrap(0.5, 0.5) ") + kBracket + ";");
    EXPECT_EQ(soleBody(narrow).Genus(), 2);
}

TEST(AlphaWrap, FlatSidesAreMerged) {
    // Two cubes: without the merge the skin was 7,282 triangles; merged at
    // offset/100 it is ~2.2k, with the volume unchanged (#941).
    const std::string two = "{ cube(10); translate([11,0,0]) cube(5); }";
    Evaluated w = evalSrc("alpha_wrap(0.5, 0.01) " + two);
    EXPECT_EQ(soleBody(w).Status(), manifold::Manifold::Error::NoError);
    EXPECT_LT(soleBody(w).NumTri(), 3000u);
    EXPECT_NEAR(soleBody(w).Volume(), 1132.51, 0.01);
}

TEST(AlphaWrap, SkinSitsOffsetOutsideTheInput) {
    Evaluated w = evalSrc(std::string("alpha_wrap(0.5, 0.1) ") + kBracket + ";");
    const manifold::Box bb = soleBody(w).BoundingBox();
    EXPECT_NEAR(bb.min.x, -0.1, 1e-4);
    EXPECT_NEAR(bb.min.y, -0.1, 1e-4);
    EXPECT_NEAR(bb.min.z, -0.1, 1e-4);
    EXPECT_NEAR(bb.max.x, 40.1, 1e-4);
    EXPECT_NEAR(bb.max.y, 10.1, 1e-4);
    EXPECT_NEAR(bb.max.z, 30.1, 1e-4);
    // It encloses the input: wrapping only ever adds volume.
    Evaluated plain = evalSrc(std::string(kBracket) + ";");
    EXPECT_GT(soleBody(w).Volume(), soleBody(plain).Volume());
}

TEST(AlphaWrap, DefaultsScaleWithTheModel) {
    // No arguments: alpha 1/20 of the bounding diagonal, offset alpha/30, so
    // the same model 100x larger wraps to the same shape, scaled.
    Evaluated small = evalSrc(std::string("alpha_wrap() ") + kBracket + ";");
    Evaluated big = evalSrc(std::string("alpha_wrap() scale(100) ") + kBracket + ";");
    // Close, not identical: the scaled input's float coordinates are not
    // exactly 100x the small one's, so near-ties in the Delaunay
    // triangulation can break the other way. Both are valid wraps.
    EXPECT_NEAR(double(soleBody(big).NumTri()), double(soleBody(small).NumTri()), 0.05 * soleBody(small).NumTri());
    EXPECT_NEAR(soleBody(big).Volume(), soleBody(small).Volume() * 1e6, soleBody(big).Volume() * 0.01);
}

TEST(AlphaWrap, WrapsAnOpenPolyhedron) {
    // A box with no top: Manifold refuses it, so polyhedron() keeps the raw
    // triangles. alpha_wrap() wraps those like any other.
    const std::string open =
        "polyhedron([[0,0,0],[10,0,0],[10,10,0],[0,10,0],[0,0,10],[10,0,10],[10,10,10],[0,10,10]],"
        " [[0,3,2,1],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])";
    std::vector<std::string> w;
    Evaluated e = evalSrc("alpha_wrap(2, 0.2) " + open + ";",
                          [&](const std::string& m) { if (m.find("alpha_wrap") != std::string::npos) w.push_back(m); });
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(soleBody(e).Status(), manifold::Manifold::Error::NoError);
    EXPECT_GT(soleBody(e).Volume(), 0);
    EXPECT_TRUE(w.empty()) << w[0];
}

TEST(AlphaWrap, OverlappingChildrenWrapAsOne) {
    Evaluated e = evalSrc("alpha_wrap(1, 0.1) { cube(20, center=true); translate([12,0,0]) sphere(12, $fn=48); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(soleBody(e).Genus(), 0);
    EXPECT_EQ(soleBody(e).Status(), manifold::Manifold::Error::NoError);
}

TEST(AlphaWrap, RejectsANonPositiveAlphaOrOffset) {
    for (const char* call : {"alpha_wrap(0) cube(1);", "alpha_wrap(-1) cube(1);", "alpha_wrap(1, 0) cube(1);",
                             "alpha_wrap(\"big\") cube(1);"}) {
        const std::vector<std::string> w = warningsOf(call);
        ASSERT_EQ(w.size(), 1u) << call;
        EXPECT_NE(w[0].find("must be a positive number"), std::string::npos) << w[0];
        // ...and the child is left as it was rather than dropped.
        Evaluated e = evalSrc(call);
        ASSERT_EQ(e.bodies.size(), 1u) << call;
        EXPECT_NEAR(soleBody(e).Volume(), 1.0, 1e-9) << call;
    }
}

TEST(AlphaWrap, LeavesTwoDChildrenAlone) {
    const std::vector<std::string> w = warningsOf("alpha_wrap() square(5);");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("2D children are not wrapped"), std::string::npos) << w[0];
}

TEST(AlphaWrap, StopsWhenCancelled) {
    auto ast = parseSrc(std::string("alpha_wrap(0.05, 0.01) ") + kBracket + ";");
    auto scope = oscad::buildScopes(ast);
    Evaluator ev;
    auto flag = std::make_shared<std::atomic<bool>>(false);
    ev.setCancelFlag(flag);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    auto tree = ev.resolveTree(ast, ctx);
    flag->store(true);
    EXPECT_THROW(ev.generateTree(tree), EvalError);
}

namespace {

// 400 points spread evenly over a sphere of radius 10 (a Fibonacci lattice):
// about 1.8 apart, no faces at all.
const char* kSphereCloud =
    "pts = [for (i = [0:399]) let(t = i * 137.508, z = 1 - 2 * (i + 0.5) / 400, r = sqrt(1 - z * z))"
    " 10 * [r * cos(t), r * sin(t), z]];";

} // namespace

TEST(AlphaWrap, PointsWrapACloudIntoASolid) {
    // #1037: a point cloud becomes a closed solid. alpha 3 bridges the ~1.8
    // spacing; the skin sits the default offset, alpha/30, out.
    Evaluated e = evalSrc(std::string(kSphereCloud) + "alpha_wrap(alpha = 3, points = pts);");
    ASSERT_EQ(e.bodies.size(), 1u);
    const manifold::Manifold& m = soleBody(e);
    EXPECT_EQ(m.Status(), manifold::Manifold::Error::NoError);
    EXPECT_EQ(m.Genus(), 0);
    EXPECT_EQ(m.Decompose().size(), 1u);
    const double ball = 4.0 / 3 * M_PI;
    EXPECT_GT(m.Volume(), 0.95 * ball * 1000);            // the sphere the points sample
    EXPECT_LT(m.Volume(), ball * std::pow(10.1, 3));      // no bigger than one offset out
    EXPECT_NEAR(m.BoundingBox().max.z, 10 * (1 - 1.0 / 400) + 0.1, 0.05);
}

TEST(AlphaWrap, ACloudsDefaultAlphaBridgesItsSpacing) {
    // 40 points ~5.3 apart: at the solids' default, 1/20 of the diagonal
    // (1.7), the ball passed between them and each point was wrapped alone,
    // 40 specks. A cloud's default is at least 1.5x its median spacing.
    Evaluated e = evalSrc(
        "pts = [for (i = [0:39]) let(t = i * 137.508, z = 1 - 2 * (i + 0.5) / 40, r = sqrt(1 - z * z))"
        " 10 * [r * cos(t), r * sin(t), z]]; alpha_wrap(points = pts);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(soleBody(e).Genus(), 0);
    EXPECT_EQ(soleBody(e).Decompose().size(), 1u);
    EXPECT_GT(soleBody(e).Volume(), 3000);
    // A dense cloud closes at the default too.
    Evaluated dense = evalSrc(std::string(kSphereCloud) + "alpha_wrap(points = pts);");
    EXPECT_EQ(soleBody(dense).Genus(), 0);
    EXPECT_EQ(soleBody(dense).Decompose().size(), 1u);
}

TEST(AlphaWrap, PointsAndChildrenTogetherIsAnError) {
    // One input or the other: allowing both later would break no script.
    EXPECT_THROW(evalSrc("alpha_wrap(points = [[0,0,0], [1,0,0], [0,1,0], [0,0,1]]) cube(1);"), EvalError);
}

TEST(AlphaWrap, PointsThatAreNotThreeDWarnAndWrapNothing) {
    for (const char* call : {"alpha_wrap(points = [[0, 0], [1, 0]]);", "alpha_wrap(points = 5);",
                             "alpha_wrap(points = [[0, 0, \"z\"]]);"}) {
        const std::vector<std::string> w = warningsOf(call);
        ASSERT_EQ(w.size(), 1u) << call;
        EXPECT_NE(w[0].find("points must be a list of [x, y, z] points"), std::string::npos) << w[0];
        EXPECT_TRUE(evalSrc(call).bodies.empty()) << call;
    }
    EXPECT_TRUE(evalSrc("alpha_wrap(points = []);").bodies.empty());
}

TEST(AlphaWrap, FeatureLevelTwoMeansPoints) {
    std::vector<std::string> out;
    evalSrc("echo(supported_feature(\"alpha-wrap\"));", [&](const std::string& m) { out.push_back(m); });
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0], "ECHO: 2");
}
