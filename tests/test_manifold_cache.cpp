#include "openscad_cpp_evaluator/manifold_cache.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <iterator>
#include <numbers>
#include <set>

using namespace oscadeval;
using namespace oscadeval::test;

// -- cacheKey() / canonValue() ---------------------------------------------

TEST(ManifoldCacheKey, BoolAndNumberParamsProduceDifferentCacheKeys) {
    // Named regression test for the exact collision the reference's own
    // _canon docstring calls out: Python's `False == 0`/`True == 1` (and
    // matching hashes) silently colliding two differently-typed cached
    // values. canonValue() tags bool distinctly from double so this can't
    // happen here -- verified directly at the cache-key level, not just
    // indirectly through geometry output.
    CSGNode a;
    a.kind = "x";
    a.isBuiltin = true;
    a.params["p"] = Value{false};

    CSGNode b;
    b.kind = "x";
    b.isBuiltin = true;
    b.params["p"] = Value{0.0};

    EXPECT_NE(cacheKey(a), cacheKey(b));
}

TEST(ManifoldCacheKey, IdenticalStructureProducesIdenticalKeyRegardlessOfParamInsertionOrder) {
    CSGNode a;
    a.kind = "cube";
    a.isBuiltin = true;
    a.params["size"] = Value{2.0};
    a.params["center"] = Value{true};

    CSGNode b;
    b.kind = "cube";
    b.isBuiltin = true;
    b.params["center"] = Value{true};
    b.params["size"] = Value{2.0};

    EXPECT_EQ(cacheKey(a), cacheKey(b));
}

TEST(ManifoldCacheKey, DifferentKindOrIsBuiltinChangesTheKey) {
    CSGNode a;
    a.kind = "cube";
    a.isBuiltin = true;
    CSGNode b;
    b.kind = "sphere";
    b.isBuiltin = true;
    CSGNode c;
    c.kind = "cube";
    c.isBuiltin = false;

    EXPECT_NE(cacheKey(a), cacheKey(b));
    EXPECT_NE(cacheKey(a), cacheKey(c));
}

TEST(ManifoldCacheKey, DifferentChildrenChangeTheKey) {
    auto child1 = std::make_unique<CSGNode>();
    child1->kind = "cube";
    child1->isBuiltin = true;
    auto child2 = std::make_unique<CSGNode>();
    child2->kind = "sphere";
    child2->isBuiltin = true;

    CSGNode a;
    a.kind = "union";
    a.isBuiltin = true;
    a.children.push_back(std::move(child1));

    CSGNode b;
    b.kind = "union";
    b.isBuiltin = true;
    b.children.push_back(std::move(child2));

    EXPECT_NE(cacheKey(a), cacheKey(b));
}

// -- rands() taint -----------------------------------------------------

TEST(ManifoldCacheUncacheable, PlainNodeIsCacheable) {
    Evaluated e = evalSrc("cube(5);");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_FALSE(e.tree[0]->uncacheable);
}

TEST(ManifoldCacheUncacheable, RandsInOwnArgumentsTaintsTheNode) {
    Evaluated e = evalSrc("cube(rands(1,10,1)[0]);");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_TRUE(e.tree[0]->uncacheable);
}

TEST(ManifoldCacheUncacheable, TaintPropagatesUpThroughUnion) {
    Evaluated e = evalSrc("union() { cube(rands(1,10,1)[0]); cube(2); }");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "union");
    EXPECT_TRUE(e.tree[0]->uncacheable);
}

TEST(ManifoldCacheUncacheable, RandsBeforeGeometryInModuleBodyTaintsSplicedSibling) {
    // rands() fires during the module's OWN resolve (the assignment,
    // before any geometry statement runs) rather than inside cube(2)'s
    // own resolve -- this exercises evalModularCall's splice-branch taint
    // propagation (the "assignment before geometry" case the reference's
    // own comment calls out), not the simpler "taint self-detected inside
    // a child's own resolve" path the previous test covers.
    Evaluated e = evalSrc("module m() { x = rands(1,10,1)[0]; cube(2); } m();");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "cube");
    EXPECT_TRUE(e.tree[0]->uncacheable);
}

// -- cache reuse -----------------------------------------------------------

TEST(ManifoldCache, CacheHitSkipsRegeneratingTheSubtree) {
    // Read through the children: a hit serves the whole subtree at once, so
    // the child is never generated and its own .bodies stays empty while
    // the parent's is populated. (idToNode is no longer a proxy for this --
    // restampCachedIds() populates it on a hit too, on purpose, or nothing
    // served from cache would be selectable.)
    const char* src = "translate([1,0,0]) cube(5, center=true);";
    auto cache = std::make_shared<ManifoldCache>();

    Evaluated first = evalSrcWithCache(src, cache);
    ASSERT_EQ(first.tree.size(), 1u);
    ASSERT_EQ(first.tree[0]->children.size(), 1u);
    EXPECT_FALSE(first.tree[0]->children[0]->bodies.empty()) << "the cube really was generated";

    Evaluated second = evalSrcWithCache(src, cache);
    ASSERT_EQ(second.tree.size(), 1u);
    ASSERT_EQ(second.tree[0]->children.size(), 1u);
    EXPECT_TRUE(second.tree[0]->children[0]->bodies.empty())
        << "a cache hit regenerated the subtree it was supposed to replace";

    ASSERT_EQ(second.bodies.size(), 1u);
    ASSERT_TRUE(second.bodies[0].body.has_value());
    EXPECT_NEAR(second.bodies[0].body->Volume(), 125.0, 1e-9);
}

// Geometry served from the cache must still map back to source. Before
// this, a re-render of an unchanged file was all cache hits against a
// fresh Evaluator, so no ID carried a node and nothing in the viewport
// could be picked at all.
TEST(ManifoldCache, GeometryServedFromAnEarlierEvaluateStillMapsToItsSource) {
    auto cache = std::make_shared<ManifoldCache>();
    evalSrcWithCache("cube(5, center=true);", cache);

    Evaluated second = evalSrcWithCache("cube(5, center=true);", cache);
    ASSERT_EQ(second.bodies.size(), 1u);
    const manifold::MeshGL mesh = second.bodies[0].body->GetMeshGL();
    ASSERT_FALSE(mesh.runOriginalID.empty());
    for (uint32_t id : mesh.runOriginalID) {
        ASSERT_EQ(second.ev.idToNode.count(id), 1u) << "cached geometry has no source node";
        EXPECT_NE(second.ev.idToNode.at(id), nullptr);
    }
}

TEST(ManifoldCache, DifferentParamsAreNotServedEachOthersCachedResult) {
    auto cache = std::make_shared<ManifoldCache>();
    Evaluated a = evalSrcWithCache("cube(2, center=true);", cache);
    Evaluated b = evalSrcWithCache("cube(2, center=false);", cache);
    manifold::Box boxA = a.bodies[0].body->BoundingBox();
    manifold::Box boxB = b.bodies[0].body->BoundingBox();
    EXPECT_NEAR(boxA.min.x, -1.0, 1e-9);
    EXPECT_NEAR(boxB.min.x, 0.0, 1e-9);
}

TEST(ManifoldCache, UncacheableNodeAlwaysRegeneratesNeverHitsOrPopulatesCache) {
    auto cache = std::make_shared<ManifoldCache>();
    Evaluated first = evalSrcWithCache("cube(rands(5,5,1)[0]);", cache);
    ASSERT_FALSE(first.ev.idToNode.empty());

    Evaluated second = evalSrcWithCache("cube(rands(5,5,1)[0]);", cache);
    // Tainted -> never a cache hit, so real generate work (and thus
    // tagGenerated()) runs again on the second evaluate too.
    EXPECT_FALSE(second.ev.idToNode.empty());
}

// A cache hit must not hand back the originalIDs of whichever call site
// first produced the geometry. Those IDs are provenance, and sharing them
// made two identical shapes one thing to a UI: selecting either picked
// both. Common in CAD, where arrays of identical parts are the norm.
TEST(ManifoldCache, IdenticalShapesAtDifferentCallSitesGetDistinctOriginalIds) {
    auto cache = std::make_shared<ManifoldCache>();
    Evaluated e = evalSrcWithCache(
        "translate([0,0,0]) cylinder(h=20,r=4,$fn=32);\n"
        "translate([25,0,0]) cylinder(h=20,r=4,$fn=32);\n", cache);
    ASSERT_EQ(e.bodies.size(), 2u);

    // Held in locals: GetMeshGL() returns by value, so taking begin() and
    // end() from two separate calls walks between unrelated buffers.
    const manifold::MeshGL meshA = e.bodies[0].body->GetMeshGL();
    const manifold::MeshGL meshB = e.bodies[1].body->GetMeshGL();
    const std::set<uint32_t> first(meshA.runOriginalID.begin(), meshA.runOriginalID.end());
    const std::set<uint32_t> second(meshB.runOriginalID.begin(), meshB.runOriginalID.end());
    ASSERT_FALSE(first.empty());
    ASSERT_FALSE(second.empty());
    std::vector<uint32_t> shared;
    std::set_intersection(first.begin(), first.end(), second.begin(), second.end(),
                           std::back_inserter(shared));
    EXPECT_TRUE(shared.empty()) << "the two cylinders share an originalID";

    // Both still resolve to a node, so re-stamping does not lose the
    // mapping selection needs.
    for (uint32_t id : first) EXPECT_EQ(e.ev.idToNode.count(id), 1u);
    for (uint32_t id : second) EXPECT_EQ(e.ev.idToNode.count(id), 1u);

    // ...and to *different* nodes. Distinct IDs alone only separated them
    // in the viewport: inheriting the first call site's node left the
    // editor highlighting one line whichever cylinder was picked.
    for (uint32_t a : first)
        for (uint32_t b : second)
            EXPECT_NE(e.ev.idToNode.at(a), e.ev.idToNode.at(b))
                << "both cylinders point at one source node";
}

// The other half of that rule. A cached ID standing for something *inside*
// the reused node keeps its own node: both calls of a module really do
// come from the one body that spells the parts out. Re-pointing those at
// the call site would collapse every part of the call onto a single line
// -- the same defect as above, one level down.
TEST(ManifoldCache, PartsOfAReusedModuleKeepTheirOwnSourceNodes) {
    auto cache = std::make_shared<ManifoldCache>();
    Evaluated e = evalSrcWithCache(
        "module pair() { cube(2); translate([5,0,0]) sphere(1,$fn=8); }\n"
        "pair();\n"
        "translate([0,20,0]) pair();\n", cache);
    ASSERT_EQ(e.bodies.size(), 4u); // cube, sphere, cube, sphere

    std::vector<const oscad::ASTNode*> nodes;
    for (const ColoredBody& b : e.bodies) {
        const manifold::MeshGL mesh = b.body->GetMeshGL();
        ASSERT_EQ(mesh.runOriginalID.size(), 1u);
        const uint32_t id = mesh.runOriginalID[0];
        ASSERT_EQ(e.ev.idToNode.count(id), 1u);
        nodes.push_back(e.ev.idToNode.at(id));
    }
    EXPECT_NE(nodes[0], nodes[1]) << "cube and sphere are not the same source";
    EXPECT_EQ(nodes[0], nodes[2]) << "the second call's cube lost the cube() that wrote it";
    EXPECT_EQ(nodes[1], nodes[3]) << "the second call's sphere lost the sphere() that wrote it";
}

// Re-stamping is per run, not per body, so a cached subtree made of
// several parts stays selectable part by part rather than collapsing into
// one -- which is what Manifold's own AsOriginal() would have done.
TEST(ManifoldCache, ACachedMultiPartSubtreeKeepsOneIdPerPart) {
    auto cache = std::make_shared<ManifoldCache>();
    Evaluated e = evalSrcWithCache(
        "module pair() { cube(2); translate([5,0,0]) sphere(1,$fn=8); }\n"
        "pair();\n"
        "translate([0,20,0]) pair();\n", cache);
    ASSERT_EQ(e.bodies.size(), 4u);
    std::set<uint32_t> all;
    for (const ColoredBody& b : e.bodies) {
        const manifold::MeshGL mesh = b.body->GetMeshGL();
        for (uint32_t id : mesh.runOriginalID) all.insert(id);
    }
    EXPECT_EQ(all.size(), 4u) << "parts of the reused subtree share IDs";
}

TEST(ManifoldCache, OutputIsIdenticalCacheOnVsCacheOffAcrossBuiltinCategories) {
    const std::vector<std::string> scripts = {
        "cube(3);",
        "sphere(r=2, $fn=24);",
        "union() { cube(2); translate([1,1,1]) cube(2); }",
        "difference() { cube(4, center=true); sphere(r=2, $fn=16); }",
        "hull() { cube(1); translate([4,0,0]) sphere(r=1,$fn=16); }",
        "linear_extrude(height=3) circle(2, $fn=16);",
        "roof() square(4, center=true);",
        "text(\"Hi\", size=8, $fn=16);",
    };
    for (const std::string& script : scripts) {
        Evaluated withoutCache = evalSrc(script);
        auto cache = std::make_shared<ManifoldCache>();
        Evaluated withCache = evalSrcWithCache(script, cache);

        ASSERT_EQ(withoutCache.bodies.size(), withCache.bodies.size()) << script;
        for (size_t i = 0; i < withoutCache.bodies.size(); ++i) {
            const ColoredBody& a = withoutCache.bodies[i];
            const ColoredBody& b = withCache.bodies[i];
            ASSERT_EQ(a.body.has_value(), b.body.has_value()) << script;
            ASSERT_EQ(a.section.has_value(), b.section.has_value()) << script;
            if (a.body) {
                EXPECT_NEAR(a.body->Volume(), b.body->Volume(), 1e-6) << script;
                manifold::Box boxA = a.body->BoundingBox(), boxB = b.body->BoundingBox();
                EXPECT_NEAR(boxA.min.x, boxB.min.x, 1e-6) << script;
                EXPECT_NEAR(boxA.max.x, boxB.max.x, 1e-6) << script;
            }
            if (a.section) {
                EXPECT_NEAR(a.section->Area(), b.section->Area(), 1e-6) << script;
            }
        }
    }
}

TEST(ManifoldCache, ClearRemovesEntriesForcingRegeneration) {
    auto cache = std::make_shared<ManifoldCache>();
    Evaluated first = evalSrcWithCache("cube(5);", cache);
    ASSERT_FALSE(first.ev.idToNode.empty());
    cache->clear();
    Evaluated second = evalSrcWithCache("cube(5);", cache);
    EXPECT_FALSE(second.ev.idToNode.empty()); // no longer a cache hit after clear()
}
