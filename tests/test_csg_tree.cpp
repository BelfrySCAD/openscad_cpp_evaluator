#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {
// Mirrors test_bytecode_compiler.cpp's/test_tail_calls.cpp's own ScopedVm --
// the plain OSCAD_BYTECODE_VM env var is cached forever after its first
// read, so a test whose own correctness depends on the compiled path
// actually running (see BareModuleRecursionStaysTreeDepthOneRegardlessOf
// CallDepth, below -- 20,000-deep recursion needs Op::CallModule, not
// kMaxUserCallDepth=30's native recursion) must force it explicitly.
class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};
} // namespace

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

// -- CSGNode::treeDepth / Evaluator::kMaxCsgTreeDepth ----------------------
//
// A recursive user-module call that wraps only a SINGLE spliced child
// (evalModularCall's own splice branch collapses it away, no extra
// CSGNode) can recurse arbitrarily deep (bounded only by kMaxUserCallDepth/
// kMaxVmCallStackDepth, the CALL-depth guards) without the resulting TREE
// ever getting any deeper. A recursive module that instead adds its OWN
// incremental geometry each level -- an ordinary "chain/spiral of shapes"
// pattern, not exotic -- makes each level's own splice see >1 child, so it
// gets wrapped in a synthetic "union" CSGNode instead of collapsing away,
// and the TREE genuinely grows one level per call. Before this guard, a
// long enough chain resolved fine (the call-depth guards don't bound tree
// depth at all) and then segfaulted the instant the resulting CSGNode
// tree -- built, never walked -- went out of scope: std::unique_ptr<
// CSGNode>'s own default recursive destructor is itself an un-guardable
// native-stack risk once such a tree exists at all, so this has to be
// caught DURING resolve, at tree-construction time, not by a later walk.

TEST(CsgTree, BareModuleRecursionStaysTreeDepthOneRegardlessOfCallDepth) {
    ScopedVm vm(true); // 20,000-deep recursion needs the compiled path
    // Splicing collapses every level away -- 20,000 calls deep, but the
    // FINAL tree is still just the one leaf cube. Proves the new guard
    // (kMaxCsgTreeDepth = 2000) doesn't false-positive on the shape Stage
    // 2's own deep-recursion tests already rely on.
    Evaluated e = evalSrc("module recur(n) { if (n>0) recur(n-1); else cube(1); }\nrecur(20000);");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "cube");
    EXPECT_EQ(e.tree[0]->treeDepth, 1);
}

TEST(CsgTree, IncrementalGeometryChainHitsTheDepthGuardCleanlyInsteadOfCrashing) {
    // Forced compiled specifically so this is unambiguously testing THIS
    // guard (kMaxCsgTreeDepth=2000) and not incidentally passing because
    // interpreted recursion would ALSO throw first, at kMaxUserCallDepth
    // (30) -- a much shallower, unrelated guard.
    ScopedVm vm(true);
    // Each level adds its own cube(0.1) alongside the recursive call, so
    // the tree grows one level per call -- 5000 is comfortably past
    // kMaxCsgTreeDepth (2000). Before this guard, a script shaped exactly
    // like this segfaulted (confirmed directly: resolveTree() itself
    // returned successfully and printed its own result; the crash was
    // purely from destroying the resulting tree afterward, generateTree()
    // never even reached).
    try {
        evalSrc("module recur(n) { if (n>0) { cube(0.1); recur(n-1); } else { cube(1); } }\nrecur(5000);");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        EXPECT_NE(std::string(e.what()).find("Recursion too deep"), std::string::npos);
    }
}

TEST(CsgTree, DeepNonTailChainErrorTraceStaysBoundedInsteadOfMegabytes) {
    // Same shape as IncrementalGeometryChainHitsTheDepthGuardCleanlyInstead
    // OfCrashing, above, but asserting on the TRACE itself: before
    // traceLines() (eval_error.cpp) bounded it, this exact error carried
    // one "TRACE:" pair per real (non-tail) call stack frame -- ~6000
    // lines, several megabytes, for a script that isn't doing anything
    // unusual by this session's own new standards (Stage 1/2 make exactly
    // this depth of real recursion routine). Checks the marker line
    // appears and the total TRACE-line count stays small regardless of
    // how deep the actual chain was.
    ScopedVm vm(true);
    try {
        evalSrc("module recur(n) { if (n>0) { cube(0.1); recur(n-1); } else { cube(1); } }\nrecur(5000);");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("TRACE: ... "), std::string::npos);
        EXPECT_NE(msg.find(" more frames ..."), std::string::npos);
        size_t count = 0;
        for (size_t pos = msg.find("TRACE:"); pos != std::string::npos; pos = msg.find("TRACE:", pos + 1)) ++count;
        EXPECT_LE(count, 41u); // 40 shown (20 each end, 2 lines/Module frame) + 1 marker
    }
}

// -- Warning call-site attribution ----------------------------------------

// A warning raised inside a module reports where it happened AND which
// top-level line reached it. Without the latter, a warning from library
// code names only the library's own line, which the reader can neither act
// on nor trace back to their own script.
//
// Deliberately richer than real OpenSCAD, which prints the raising location
// alone and no trace (verified against 2022.08.22).
TEST(WarningTrace, EvalTimeWarningNamesTheTopLevelCallSite) {
    std::string warning;
    evalSrc("function helper(n) = n + nope;\n"      // line 1: warning raised here
            "module inner(v) { x = helper(v); cube(1); }\n"
            "module outer(q) { inner(q); }\n"
            "outer(2);\n",                           // line 4: the user's own line
            [&](const std::string& m) { if (warning.empty()) warning = m; });

    EXPECT_NE(warning.find("Ignoring unknown variable 'nope'"), std::string::npos) << warning;
    EXPECT_NE(warning.find("line 1"), std::string::npos) << warning;      // where it happened
    EXPECT_NE(warning.find("from <string>, line 4"), std::string::npos) << warning; // who caused it
    // Full chain follows, same shape as an error's trace.
    EXPECT_NE(warning.find("TRACE:"), std::string::npos) << warning;
}

// A GenerateFn runs after resolve has unwound the call stack, so this case
// has no live frames to walk -- the entry position is captured onto the
// CSGNode at resolve time instead (CSGNode::warnEntry). This is the case
// that motivated the whole change: an open polyhedron() inside BOSL2 used
// to report only "BOSL2/vnf.scad, line 1624".
TEST(WarningTrace, GenerateTimeWarningNamesTheTopLevelCallSite) {
    std::string warning;
    evalSrc("module openbox() {\n"
            "  polyhedron(points=[[0,0,0],[10,0,0],[10,10,0],[0,10,0],\n"
            "                     [0,0,10],[10,0,10],[10,10,10],[0,10,10]],\n"
            "             faces=[[0,1,2,3],[4,7,6,5],[0,4,5,1],[1,5,6,2],[2,6,7,3]]);\n"
            "}\n"
            "openbox();\n",                          // line 6: the user's own line
            [&](const std::string& m) { if (warning.empty()) warning = m; });

    EXPECT_NE(warning.find("mesh is not closed"), std::string::npos) << warning;
    EXPECT_NE(warning.find("from <string>, line 6"), std::string::npos) << warning;
}

// A warning with nothing above it stays exactly one line. The overwhelmingly
// common case, and the reason the "from" clause and TRACE are conditional --
// ordinary warnings must not sprout a trace nobody needs.
TEST(WarningTrace, TopLevelWarningStaysASingleLineWithNoTrace) {
    std::string warning;
    evalSrc("x = nope;\n", [&](const std::string& m) { if (warning.empty()) warning = m; });

    EXPECT_NE(warning.find("Ignoring unknown variable 'nope'"), std::string::npos) << warning;
    EXPECT_EQ(warning.find("TRACE:"), std::string::npos) << warning;
    EXPECT_EQ(warning.find(", from "), std::string::npos) << warning;
    EXPECT_EQ(warning.find('\n'), std::string::npos) << warning;
}

// The clause names the OUTERMOST frame, not the immediate caller: the
// question being answered is "which line of mine started this", and the
// intermediate frames are already in the TRACE below.
TEST(WarningTrace, FromClauseNamesOutermostFrameNotImmediateCaller) {
    std::string warning;
    evalSrc("module a() { y = nope; cube(1); }\n"    // line 1: raised here
            "module b() { a(); }\n"                   // line 2: immediate caller of a()
            "b();\n",                                 // line 3: the top-level entry
            [&](const std::string& m) { if (warning.empty()) warning = m; });

    EXPECT_NE(warning.find("from <string>, line 3"), std::string::npos) << warning;
    EXPECT_EQ(warning.find("from <string>, line 2"), std::string::npos) << warning;
}
