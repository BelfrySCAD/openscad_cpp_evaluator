#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

// -- Tree shape -----------------------------------------------------------

TEST(CsgTree, OneModularCallProducesOneTreeNode) {
    Evaluated e = evalSrc("cube(1);");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "cube");
    EXPECT_TRUE(e.tree[0]->isBuiltin);
    EXPECT_TRUE(e.tree[0]->children.empty());
}

TEST(CsgTree, TransformNestsItsChildUnderneath) {
    Evaluated e = evalSrc("translate([1,0,0]) cube(2);");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "translate");
    ASSERT_EQ(e.tree[0]->children.size(), 1u);
    EXPECT_EQ(e.tree[0]->children[0]->kind, "cube");
}

TEST(CsgTree, UnionNestsBothChildren) {
    Evaluated e = evalSrc("union() { cube(1); translate([2,0,0]) cube(1); }");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "union");
    ASSERT_EQ(e.tree[0]->children.size(), 2u);
    EXPECT_EQ(e.tree[0]->children[0]->kind, "cube");
    EXPECT_EQ(e.tree[0]->children[1]->kind, "translate");
}

// -- Generated geometry -----------------------------------------------------

TEST(CsgTree, CubeDefaultSizeAndPosition) {
    Evaluated e = evalSrc("cube(2);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_EQ(e.bodies[0].body->NumTri(), 12u); // 2 triangles/face * 6 faces
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_DOUBLE_EQ(bbox.min.x, 0.0);
    EXPECT_DOUBLE_EQ(bbox.max.x, 2.0);
}

TEST(CsgTree, CubeCenteredStraddlesOrigin) {
    Evaluated e = evalSrc("cube(2, center=true);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_DOUBLE_EQ(bbox.min.x, -1.0);
    EXPECT_DOUBLE_EQ(bbox.max.x, 1.0);
}

TEST(CsgTree, CubeNonUniformSizeVector) {
    Evaluated e = evalSrc("cube([1,2,3]);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_DOUBLE_EQ(bbox.max.x, 1.0);
    EXPECT_DOUBLE_EQ(bbox.max.y, 2.0);
    EXPECT_DOUBLE_EQ(bbox.max.z, 3.0);
}

TEST(CsgTree, TranslateMovesTheCube) {
    // Phase 2's own exit-criterion fixture.
    Evaluated e = evalSrc("translate([1,0,0]) cube(2);");
    ASSERT_EQ(e.bodies.size(), 1u);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_DOUBLE_EQ(bbox.min.x, 1.0);
    EXPECT_DOUBLE_EQ(bbox.max.x, 3.0);
    EXPECT_DOUBLE_EQ(bbox.min.y, 0.0);
    EXPECT_DOUBLE_EQ(bbox.max.y, 2.0);
}

TEST(CsgTree, UnionOfDisjointCubesMergesIntoOneBodyCoveringBoth) {
    Evaluated e = evalSrc("union() { cube(2); translate([3,0,0]) cube(2); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_EQ(e.bodies[0].body->NumTri(), 24u); // disjoint, so no faces merge away
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_DOUBLE_EQ(bbox.min.x, 0.0);
    EXPECT_DOUBLE_EQ(bbox.max.x, 5.0);
}

TEST(CsgTree, UnionOfOverlappingCubesMergesVolumeCorrectly) {
    // Overlapping cubes: a real boolean union's volume is |A| + |B| -
    // |A ∩ B|, not |A| + |B| (which a naive concat/compose would give) --
    // the most reliable way to confirm this is a genuine Manifold boolean,
    // not just two solids placed side by side. Both cubes have volume 8;
    // A = [0,2]^3, B = [1,3]x[0,2]x[0,2], overlap = [1,2]x[0,2]x[0,2] = 4,
    // so the union should be 8+8-4 = 12.
    Evaluated e = evalSrc("union() { cube(2); translate([1,0,0]) cube(2); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 12.0, 1e-6);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_DOUBLE_EQ(bbox.min.x, 0.0);
    EXPECT_DOUBLE_EQ(bbox.max.x, 3.0);
}

// -- Provenance (id_to_node) --------------------------------------------

TEST(CsgTree, TagGeneratedRecordsProvenanceForACube) {
    Evaluated e = evalSrc("cube(1);");
    EXPECT_FALSE(e.ev.idToNode.empty());
    for (const auto& [id, node] : e.ev.idToNode) {
        EXPECT_EQ(node->kind(), oscad::NodeKind::ModularCall);
    }
}

// -- Not-yet-implemented builtins/ops fail loudly ----------------------------

TEST(CsgTree, UnknownModuleWarnsAndContinues) {
    // Real OpenSCAD issues a WARNING for an unknown module name and simply
    // produces no geometry for that call, continuing to evaluate everything
    // else -- verified directly against the Python reference (returns 1
    // body for this exact script, no TRACE lines at top level since the
    // call stack is empty there). Previously this port threw and aborted
    // the whole evaluation instead.
    std::vector<std::string> echoed;
    Evaluated e = evalSrc("sphere_typo(1); cube(2);", [&](const std::string& m) { echoed.push_back(m); });
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "cube");
    ASSERT_EQ(echoed.size(), 1u);
    EXPECT_NE(echoed[0].find("WARNING: Ignoring unknown module 'sphere_typo'"), std::string::npos);
}

TEST(CsgTree, UnknownModuleInsideUserModuleWarnsWithTrace) {
    // Same fallback, but from inside an active user-module call -- TRACE
    // lines walk the call stack exactly like an error's would (verified
    // against the Python reference: "TRACE: call of 'm()'..." + "TRACE:
    // called by 'm'...").
    std::vector<std::string> echoed;
    Evaluated e = evalSrc("module m() { sphere_typo(1); } m(); cube(2);", [&](const std::string& m) { echoed.push_back(m); });
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "cube");
    ASSERT_EQ(echoed.size(), 1u);
    EXPECT_NE(echoed[0].find("WARNING: Ignoring unknown module 'sphere_typo'"), std::string::npos);
    EXPECT_NE(echoed[0].find("TRACE: call of 'm()'"), std::string::npos);
    EXPECT_NE(echoed[0].find("TRACE: called by 'm'"), std::string::npos);
}

TEST(CsgTree, HullNestsBothChildrenLikeUnion) {
    // hull() splices its children transparently, same tree shape as
    // union/difference/intersection -- see test_extrude_roof.cpp for its
    // actual geometric-correctness coverage (Phase 6).
    Evaluated e = evalSrc("hull() { cube(2); cube(1); }");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "hull");
    EXPECT_EQ(e.tree[0]->children.size(), 2u);
}

TEST(CsgTree, LinearExtrudeNestsItsChildUnderneath) {
    Evaluated e = evalSrc("linear_extrude(height=1) circle(1);");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "linear_extrude");
    ASSERT_EQ(e.tree[0]->children.size(), 1u);
    EXPECT_EQ(e.tree[0]->children[0]->kind, "circle");
}
