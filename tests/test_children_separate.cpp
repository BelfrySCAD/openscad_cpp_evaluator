// children(separate=true) -- hand the forwarded children to the enclosing
// union/difference/intersection as SEPARATE operands instead of as the one
// grouped operand a statement normally contributes.
//
// Every test runs under BOTH engines. That is the standing rule in this
// repo, and here it is load-bearing rather than ceremonial: "group_sizes"
// is built in completely separate code per engine (resolveCsg in
// booleans.cpp vs Op::CsgGroupEnd in bytecode_vm.cpp), and the splice
// itself has three call sites across the two.

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

// Same reasoning as test_csg_tree.cpp's own copy: the OSCAD_BYTECODE_VM env
// var is cached after its first read, so a test that must exercise a
// specific engine has to force it.
class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

double totalVolume(const std::vector<ColoredBody>& bodies) {
    double v = 0.0;
    for (const ColoredBody& b : bodies) {
        if (b.body) v += b.body->Volume();
    }
    return v;
}

// One big cube with two small cubes fully INSIDE it, disjoint from each
// other. Deliberate: the two subtrahends contribute nothing to a union, so
// the union answer (125000) and each partially-wrong grouping (124000) are
// all numerically distinct from the right one (123000). Overlapping shapes
// would have made several wrong answers land on the same number.
constexpr const char* kThreeChildren =
    "{ cube(50, center=true);"
    "  translate([-15,0,0]) cube(10, center=true);"
    "  translate([ 15,0,0]) cube(10, center=true); }";

constexpr double kCube = 125000.0;   // 50^3
constexpr double kSmall = 1000.0;    // 10^3

} // namespace

// -- The feature ----------------------------------------------------------

TEST(ChildrenSeparate, DifferenceSubtractsEachForwardedChild) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc(std::string("module frame() { difference() children(separate=true); }\n"
                                          "frame() ") + kThreeChildren);
        EXPECT_NEAR(totalVolume(e.bodies), kCube - 2 * kSmall, 1e-6) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, MatchesTheHandWrittenDifference) {
    // The oracle: the same shape spelled out. These must agree exactly --
    // that is the whole claim of the feature.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated sep = evalSrc(std::string("module frame() { difference() children(separate=true); }\n"
                                            "frame() ") + kThreeChildren);
        Evaluated hand = evalSrc(std::string("difference() ") + kThreeChildren);
        EXPECT_NEAR(totalVolume(sep.bodies), totalVolume(hand.bodies), 1e-6) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, WithoutTheFlagNothingChanges) {
    // Pins the default. A regression here means every existing script that
    // forwards children through a difference() silently changed meaning.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc(std::string("module frame() { difference() children(); }\n"
                                          "frame() ") + kThreeChildren);
        EXPECT_NEAR(totalVolume(e.bodies), kCube, 1e-6) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, IntersectionIntersectsEachForwardedChild) {
    // The case with no workaround at all: for difference, A-(B|C) == A-B-C,
    // so a union of the subtrahends happens to be right. For intersection
    // it is not -- A&(B|C) != A&B&C.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const char* kids = "{ cube([10,10,10]); translate([5,0,0]) cube([10,10,10]); }";
        Evaluated sep = evalSrc(std::string("module m() { intersection() children(separate=true); }\nm() ") + kids);
        Evaluated off = evalSrc(std::string("module m() { intersection() children(); }\nm() ") + kids);
        EXPECT_NEAR(totalVolume(sep.bodies), 500.0, 1e-6) << "vm=" << vm;   // the overlap
        EXPECT_NEAR(totalVolume(off.bodies), 1500.0, 1e-6) << "vm=" << vm;  // the union
    }
}

TEST(ChildrenSeparate, HonorsAnIndexSelection) {
    // The 4th child would change the volume visibly if the selection leaked.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc("module frame() { difference() children([0:2], separate=true); }\n"
                              "frame() { cube(50, center=true);"
                              "          translate([-15,0,0]) cube(10, center=true);"
                              "          translate([ 15,0,0]) cube(10, center=true);"
                              "          translate([  0,0,0]) cube(20, center=true); }");
        EXPECT_NEAR(totalVolume(e.bodies), kCube - 2 * kSmall, 1e-6) << "vm=" << vm;
    }
}

// -- Tree shape: the mechanism itself -------------------------------------

TEST(ChildrenSeparate, SplicesInsteadOfWrappingAndSplitsGroupSizes) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc(std::string("module frame() { difference() children(separate=true); }\n"
                                          "frame() ") + kThreeChildren);
        ASSERT_EQ(e.tree.size(), 1u) << "vm=" << vm;
        const CSGNode& diff = *e.tree[0];
        EXPECT_EQ(diff.kind, "difference") << "vm=" << vm;
        // The grouping wrapper would hide the marks from the group walk, so
        // it must not be there: three children, three groups of one.
        ASSERT_EQ(diff.children.size(), 3u) << "vm=" << vm;
        for (const auto& c : diff.children) {
            EXPECT_NE(c->kind, "union") << "vm=" << vm;
        }
        const ListPtr* sizes = std::get_if<ListPtr>(&diff.params.at("group_sizes"));
        ASSERT_NE(sizes, nullptr) << "vm=" << vm;
        ASSERT_EQ((*sizes)->items.size(), 3u) << "vm=" << vm;
        for (const Value& s : (*sizes)->items) EXPECT_EQ(std::get<double>(s), 1.0) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, WithoutTheFlagStillWrapsIntoOneGroup) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc(std::string("module frame() { difference() children(); }\n"
                                          "frame() ") + kThreeChildren);
        ASSERT_EQ(e.tree.size(), 1u) << "vm=" << vm;
        const CSGNode& diff = *e.tree[0];
        ASSERT_EQ(diff.children.size(), 1u) << "vm=" << vm;
        EXPECT_EQ(diff.children[0]->kind, "union") << "vm=" << vm;
        EXPECT_FALSE(diff.children[0]->isBuiltin) << "vm=" << vm;
        const ListPtr* sizes = std::get_if<ListPtr>(&diff.params.at("group_sizes"));
        ASSERT_NE(sizes, nullptr) << "vm=" << vm;
        ASSERT_EQ((*sizes)->items.size(), 1u) << "vm=" << vm;
        EXPECT_EQ(std::get<double>((*sizes)->items[0]), 1.0) << "vm=" << vm;
    }
}

// -- Degradation: it must not leak ----------------------------------------

TEST(ChildrenSeparate, UnderATransformDegradesToOneOperand) {
    // The children land in translate()'s own frame, so difference() sees the
    // single translate node. Nothing to separate -- the union is correct.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc(std::string("module frame() { difference() translate([0,0,0]) children(separate=true); }\n"
                                          "frame() ") + kThreeChildren);
        EXPECT_NEAR(totalVolume(e.bodies), kCube, 1e-6) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, DoesNotAffectALaterUnrelatedDifference) {
    // A separated splice under a non-CSG parent must not colour the grouping
    // of a difference() elsewhere in the same evaluation -- including one
    // whose first statement happens to produce the same number of nodes.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc("module m() { translate([200,0,0]) children(separate=true); }\n"
                              "m() { cube(1); cube(2); cube(3); }\n"
                              "difference() { cube(50, center=true); translate([-15,0,0]) cube(10, center=true); }");
        // 1 + 8 + 27 from the spliced group, plus a normal difference.
        EXPECT_NEAR(totalVolume(e.bodies), 36.0 + kCube - kSmall, 1e-6) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, StopsAtAUserModuleBoundary) {
    // separate=true applies to the operator enclosing THAT children() call,
    // not to whatever encloses the module the call sits in. pass() is its
    // own shape; that it forwards its children separately is its business,
    // and difference() outside it still sees one operand.
    //
    // Mechanically: a user module's splice wraps its children as usual,
    // which hides the marks from the group walk. Only the children() call's
    // own splice honours them (spliceModuleChildren's honorSeparateMarks).
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc(std::string("module pass() { children(separate=true); }\n"
                                          "difference() pass() ") + kThreeChildren);
        EXPECT_NEAR(totalVolume(e.bodies), kCube, 1e-6) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, StillWorksInsideTheModuleThatAsksForIt) {
    // The other half of the boundary rule: a module that puts the operator
    // and the children(separate=true) call together still works, however
    // deeply that module is itself nested.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc(std::string("module frame() { difference() children(separate=true); }\n"
                                          "module outer() { frame() { cube(50, center=true);"
                                          "                           translate([-15,0,0]) cube(10, center=true);"
                                          "                           translate([ 15,0,0]) cube(10, center=true); } }\n"
                                          "outer();"));
        EXPECT_NEAR(totalVolume(e.bodies), kCube - 2 * kSmall, 1e-6) << "vm=" << vm;
    }
}

// -- Degenerate cases -----------------------------------------------------

TEST(ChildrenSeparate, OneChildIsASilentNoOp) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        std::vector<std::string> warnings;
        Evaluated e = evalSrc("module frame() { difference() children(separate=true); }\n"
                              "frame() { cube(3); }",
                              [&](const std::string& m) { warnings.push_back(m); });
        EXPECT_NEAR(totalVolume(e.bodies), 27.0, 1e-6) << "vm=" << vm;
        EXPECT_TRUE(warnings.empty()) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, NoChildrenIsASilentNoOp) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        std::vector<std::string> warnings;
        Evaluated e = evalSrc("module frame() { difference() children(separate=true); }\nframe();",
                              [&](const std::string& m) { warnings.push_back(m); });
        EXPECT_TRUE(e.bodies.empty()) << "vm=" << vm;
        EXPECT_TRUE(warnings.empty()) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, LeavesUnionAndHullUnchanged) {
    // union() gives the same geometry either way, and hull()/minkowski()
    // read bodies rather than groups -- the wrapper they used to receive was
    // plain concatenation, so they already saw the children separately.
    // This is the executable form of that claim.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        for (const char* op : {"union", "hull"}) {
            const std::string on = std::string("module m() { ") + op + "() children(separate=true); }\nm() ";
            const std::string off = std::string("module m() { ") + op + "() children(); }\nm() ";
            Evaluated a = evalSrc(on + kThreeChildren);
            Evaluated b = evalSrc(off + kThreeChildren);
            EXPECT_NEAR(totalVolume(a.bodies), totalVolume(b.bodies), 1e-6) << "vm=" << vm << " op=" << op;
        }
    }
}

// -- The argument itself --------------------------------------------------

TEST(ChildrenSeparate, TheArgumentIsAccepted) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        std::vector<std::string> warnings;
        evalSrc("module m() { children(separate=true); }\nm() { cube(1); cube(2); }",
                [&](const std::string& m) { warnings.push_back(m); });
        EXPECT_TRUE(warnings.empty()) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, AMisspelledArgumentStillWarnsUnderBothEngines) {
    // Op::CallChildren did not call warnUnexpectedBuiltinArgs, so a typo was
    // silent on the compiled path only. That gap is why this test can exist.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        std::vector<std::string> warnings;
        evalSrc("module m() { children(seperate=true); }\nm() { cube(1); cube(2); }",
                [&](const std::string& m) { warnings.push_back(m); });
        bool found = false;
        for (const std::string& w : warnings) {
            if (w.find("seperate") != std::string::npos) found = true;
        }
        EXPECT_TRUE(found) << "vm=" << vm << ", warnings=" << warnings.size();
    }
}

TEST(ChildrenSeparate, AcceptsThePositionalSecondArgument) {
    // Adding "separate" to the builtin's parameter list already silences the
    // "Too many unnamed arguments" warning for this shape, so it must be
    // read rather than silently ignored.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc("module frame() { difference() children([0:2], true); }\n"
                              "frame() { cube(50, center=true);"
                              "          translate([-15,0,0]) cube(10, center=true);"
                              "          translate([ 15,0,0]) cube(10, center=true); }");
        EXPECT_NEAR(totalVolume(e.bodies), kCube - 2 * kSmall, 1e-6) << "vm=" << vm;
    }
}

// -- Statement-level splicing ---------------------------------------------
//
// separate=true expands the forward into one real `children(k)` statement
// per child at the call site. These pin the consequences of that, which is
// everything the feature actually is.

namespace {

std::vector<std::string> echoesFrom(const std::string& src) {
    std::vector<std::string> out;
    evalSrc(src, [&](const std::string& m) { out.push_back(m); });
    return out;
}

} // namespace

TEST(ChildrenSeparate, ForwardedChildrenBecomeCountableStatements) {
    // The divergence from OpenSCAD, stated as a test: $children counts child
    // STATEMENTS, and a separating forward now IS several of them. Every
    // other shape keeps counting exactly as OpenSCAD does -- verified
    // against the reference binary, which reports 1/1/2/1 here.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e = echoesFrom(
            "module inner() { echo($children); }\n"
            "module plain() { inner() { children([0:2]); }; }\n"
            "module sep()   { inner() { children([0:2], separate=true); }; }\n"
            "module mixed() { inner() { cube(1); children([0:1], separate=true); }; }\n"
            "plain() { cube(1); cube(2); cube(3); }\n"
            "sep()   { cube(1); cube(2); cube(3); }\n"
            "mixed() { cube(1); cube(2); cube(3); }\n"
            "inner() { for (i=[0:9]) cube(1); }\n");
        ASSERT_EQ(e.size(), 4u) << "vm=" << vm;
        EXPECT_NE(e[0].find("1"), std::string::npos) << "plain, vm=" << vm;   // a forward is one statement
        EXPECT_NE(e[1].find("3"), std::string::npos) << "sep, vm=" << vm;     // <- the divergence
        EXPECT_NE(e[2].find("3"), std::string::npos) << "mixed, vm=" << vm;   // 1 written + 2 spliced
        EXPECT_NE(e[3].find("1"), std::string::npos) << "for-loop, vm=" << vm;
    }
}

TEST(ChildrenSeparate, SplicedChildrenAreIndexableIndividually) {
    // If they count as statements they must also index as statements.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc("module pick1() { children(1); }\n"
                              "module fwd()   { pick1() { children([0:2], separate=true); }; }\n"
                              "fwd() { cube(1); cube(10); cube(100); }");
        EXPECT_NEAR(totalVolume(e.bodies), 1000.0, 1e-6) << "vm=" << vm;  // the middle one
    }
}

TEST(ChildrenSeparate, OneStatementStaysOneOperandHoweverManyBodiesItMakes) {
    // The bug that motivated the redesign. A `for` child is ONE statement, so
    // it is ONE operand -- the same "statements, not bodies" rule that lets
    // BOSL2's attachable() return parent+attachments as a single operand.
    // The old per-body marking made it three, so the first cube alone became
    // the positive operand and the volume came out 875 instead of 2875.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const char* kids = "{ for (i=[0:2]) translate([i*20,0,0]) cube(10); cube(5); }";
        Evaluated sep = evalSrc(std::string("module m() { difference() children(separate=true); }\nm() ") + kids);
        Evaluated hand = evalSrc(std::string("difference() ") + kids);
        EXPECT_NEAR(totalVolume(sep.bodies), 2875.0, 1e-6) << "vm=" << vm;
        EXPECT_NEAR(totalVolume(sep.bodies), totalVolume(hand.bodies), 1e-6) << "vm=" << vm;
    }
}

TEST(ChildrenSeparate, SelectingNothingContributesNoStatements) {
    // A forward that picks nothing expands to zero statements, so the block
    // can hold FEWER statements than it has lines. Deliberate: the count has
    // to describe what is really there for indexing to stay honest.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        // A parent with no children at all, so the forward selects nothing
        // without also warning about anything -- echoesFrom sees warnings
        // too, and a backwards range would have put one first.
        const std::vector<std::string> e = echoesFrom(
            "module inner() { echo($children); }\n"
            "module outer() { inner() { cube(1); children(separate=true); }; }\n"
            "outer();\n");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm << " (unexpected warning?)";
        EXPECT_NE(e[0].find("1"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}

TEST(ChildrenSeparate, SurvivesARecursiveForward) {
    // The shape the whole redesign exists for: a module forwarding "all the
    // rest" to itself. It only terminates correctly because $children sees
    // the spliced members, so each level really does peel one child off.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e = echoesFrom(
            "module peel() {\n"
            "    echo($children);\n"
            "    if ($children > 1) peel() { children([1:$children-1], separate=true); }\n"
            "}\n"
            "peel() { cube(1); cube(2); cube(3); cube(4); }\n");
        ASSERT_EQ(e.size(), 4u) << "vm=" << vm;   // 4, 3, 2, 1 -- one per level
        for (size_t i = 0; i < 4; ++i) {
            EXPECT_NE(e[i].find(std::to_string(4 - i)), std::string::npos)
                << "level " << i << ", vm=" << vm << ", got " << e[i];
        }
    }
}

TEST(ChildrenSeparate, ASeparatingForwardInsideALoopStillGroupsPerStatement) {
    // Exercises Op::CsgGroupChildren's own loop against the interpreter's:
    // the expansion happens per evaluation, inside a construct that is
    // itself transparent in the CSG tree.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc("module m() { difference() { children(0); children([1:2], separate=true); } }\n"
                              "m() { cube(50, center=true);"
                              "      translate([-15,0,0]) cube(10, center=true);"
                              "      translate([ 15,0,0]) cube(10, center=true); }");
        EXPECT_NEAR(totalVolume(e.bodies), 125000.0 - 2000.0, 1e-6) << "vm=" << vm;
    }
}
