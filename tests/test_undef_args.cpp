// A builtin argument given as undef means "not given" (#680), and the
// height/h and angle/a aliases, as upstream OpenSCAD -- each checked
// against OpenSCAD 2026.02 before being written down here.
//
// Wrappers forward their own optional parameters: BOSL2's rotate_extrude
// override passes angle=angle, undef when the user gave none, and taking
// that undef as 0 degrees flattened every plain rotate_extrude() into its
// profile.

#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

struct Shape {
    double volume;
    manifold::Box box;
};

Shape shapeOf(const std::string& code) {
    Evaluated e = evalSrc(code);
    EXPECT_EQ(e.bodies.size(), 1u) << code;
    if (e.bodies.empty() || !e.bodies[0].body.has_value()) return {0.0, {}};
    return {e.bodies[0].body->Volume(), e.bodies[0].body->BoundingBox()};
}

void expectSameShape(const std::string& a, const std::string& b) {
    const Shape x = shapeOf(a), y = shapeOf(b);
    EXPECT_GT(x.volume, 0.0) << a;
    EXPECT_NEAR(x.volume, y.volume, 1e-6) << a << " vs " << b;
    EXPECT_NEAR(x.box.max.x, y.box.max.x, 1e-6) << a;
    EXPECT_NEAR(x.box.max.y, y.box.max.y, 1e-6) << a;
    EXPECT_NEAR(x.box.max.z, y.box.max.z, 1e-6) << a;
    EXPECT_NEAR(x.box.min.x, y.box.min.x, 1e-6) << a;
}

std::vector<std::string> warningsFor(const std::string& code) {
    std::vector<std::string> out;
    Evaluated e = evalSrc(code, [&](const std::string& msg) {
        if (msg.rfind("WARNING: ", 0) == 0) out.push_back(msg);
    });
    (void)e;
    return out;
}

} // namespace

// Every argument that took an explicit undef as a value rather than as absent.
TEST(UndefArgs, AnUndefArgumentIsTheDefault) {
    expectSameShape("rotate_extrude(angle=undef) translate([10,0]) square(5);",
                    "rotate_extrude() translate([10,0]) square(5);");
    expectSameShape("linear_extrude(height=undef) square(5);", "linear_extrude() square(5);");
    expectSameShape("cylinder(h=undef, r=2);", "cylinder(r=2);");
    expectSameShape("linear_extrude(1) square(size=undef);", "linear_extrude(1) square();");
    expectSameShape("linear_extrude(1) text(\"A\", size=undef);", "linear_extrude(1) text(\"A\");");
    expectSameShape("linear_extrude(1) text(\"AB\", spacing=undef);", "linear_extrude(1) text(\"AB\");");
    // Positional undef too.
    expectSameShape("rotate_extrude(undef) translate([10,0]) square(5);",
                    "rotate_extrude() translate([10,0]) square(5);");
}

TEST(UndefArgs, AWrapperForwardingItsOwnOptionalAngleStillRevolvesFully) {
    expectSameShape("module re(angle) rotate_extrude(angle=angle) children();\n"
                    "re() translate([10,0]) square(5);",
                    "rotate_extrude() translate([10,0]) square(5);");
}

TEST(UndefArgs, LinearExtrudeDefaultsToAHeightOfOneHundred) {
    EXPECT_NEAR(shapeOf("linear_extrude() square(5);").box.max.z, 100.0, 1e-9);
    const Shape centred = shapeOf("linear_extrude(center=true) square(5);");
    EXPECT_NEAR(centred.box.min.z, -50.0, 1e-9);
    EXPECT_NEAR(centred.box.max.z, 50.0, 1e-9);
}

TEST(UndefArgs, HIsHeightAndAIsAngle) {
    EXPECT_NEAR(shapeOf("linear_extrude(h=7) square(5);").box.max.z, 7.0, 1e-9);
    expectSameShape("rotate_extrude(a=90) translate([10,0]) square(5);",
                    "rotate_extrude(angle=90) translate([10,0]) square(5);");
    EXPECT_TRUE(warningsFor("linear_extrude(h=7) square(5);").empty());
    EXPECT_TRUE(warningsFor("rotate_extrude(a=90) translate([10,0]) square(5);").empty());
}

TEST(UndefArgs, BothNamesGivenWarnsAndThePrimaryWins) {
    EXPECT_NEAR(shapeOf("linear_extrude(height=3, h=7) square(5);").box.max.z, 3.0, 1e-9);
    const auto w = warningsFor("linear_extrude(height=3, h=7) square(5);");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("Specified both \"height\" and \"h\""), std::string::npos) << w[0];
    expectSameShape("rotate_extrude(angle=45, a=90) translate([10,0]) square(5);",
                    "rotate_extrude(angle=45) translate([10,0]) square(5);");
    const auto wa = warningsFor("rotate_extrude(angle=45, a=90) translate([10,0]) square(5);");
    ASSERT_EQ(wa.size(), 1u);
    EXPECT_NE(wa[0].find("Specified both \"angle\" and \"a\""), std::string::npos) << wa[0];
}
