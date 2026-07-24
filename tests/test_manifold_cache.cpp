#include "openscad_cpp_evaluator/manifold_cache.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>
#include <numbers>

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
    auto cache = std::make_shared<ManifoldCache>();
    Evaluated first = evalSrcWithCache("cube(5, center=true);", cache);
    ASSERT_FALSE(first.ev.idToNode.empty()); // real generate work happened, tagGenerated() ran

    Evaluated second = evalSrcWithCache("cube(5, center=true);", cache);
    // A cache hit means generateCube() (and thus tagGenerated()) never ran
    // for the second Evaluator's own node -- its idToNode stays empty even
    // though its bodies are populated (served from the cache).
    EXPECT_TRUE(second.ev.idToNode.empty());
    ASSERT_EQ(second.bodies.size(), 1u);
    ASSERT_TRUE(second.bodies[0].body.has_value());
    EXPECT_NEAR(second.bodies[0].body->Volume(), 125.0, 1e-9);
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
