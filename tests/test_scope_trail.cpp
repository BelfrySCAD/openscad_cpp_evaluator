#include "openscad_cpp_evaluator/scope_trail.hpp"

#include <gtest/gtest.h>

namespace oscadeval {
namespace {

TEST(ScopeTrailStorage, SetAtSameLevelOverwritesLatestValue) {
    ScopeTrailStorage<int> storage;
    const int level = storage.openLevel(0);
    storage.set("i", 1, level);
    storage.set("i", 2, level);
    storage.set("i", 3, level);
    ASSERT_NE(storage.lookup("i", level), nullptr);
    EXPECT_EQ(*storage.lookup("i", level), 3);
}

TEST(ScopeTrailStorage, SetAtSameLevelDoesNotAccumulateEntries) {
    // The actual optimization this session added: repeated set() calls at
    // the SAME level (a for-loop variable rebound every iteration without
    // opening a new level, or sibling same-block statements reassigning
    // the same name) must overwrite in place, not grow the underlying
    // stack -- verified via the test-only accessor, since correctness
    // alone (SetAtSameLevelOverwritesLatestValue above) can't distinguish
    // "overwrote in place" from "pushed and lookup() happens to find the
    // newest one anyway."
    ScopeTrailStorage<int> storage;
    const int level = storage.openLevel(0);
    for (int i = 0; i < 1000; ++i) storage.set("i", i, level);
    EXPECT_EQ(storage.debugEntryCountForTesting("i"), 1u);
}

TEST(ScopeTrailStorage, SetAtANewLevelStillPushes) {
    // The optimization must not collapse GENUINE nested bindings -- a
    // child level's own set() for the same name is a real new binding
    // (must be poppable back to the parent's value), not a same-level
    // rebind.
    ScopeTrailStorage<int> storage;
    const int parentLevel = storage.openLevel(0);
    storage.set("x", 1, parentLevel);
    const int childLevel = storage.openLevel(parentLevel);
    storage.set("x", 2, childLevel);
    EXPECT_EQ(storage.debugEntryCountForTesting("x"), 2u);
    ASSERT_NE(storage.lookup("x", childLevel), nullptr);
    EXPECT_EQ(*storage.lookup("x", childLevel), 2);
    storage.popLevel(childLevel);
    ASSERT_NE(storage.lookup("x", parentLevel), nullptr);
    EXPECT_EQ(*storage.lookup("x", parentLevel), 1);
}

TEST(ScopeTrailStorage, PopLevelAfterManySameLevelSetsRestoresParentValue) {
    // The end-to-end correctness case a for-loop actually exercises: many
    // same-level rebinds of the loop variable, then the level closes
    // (loop exits) -- the parent's own binding for that name (if any) must
    // resurface untouched, and the now-closed level's value must not leak.
    ScopeTrailStorage<int> storage;
    const int parentLevel = storage.openLevel(0);
    storage.set("i", -1, parentLevel);
    const int loopLevel = storage.openLevel(parentLevel);
    for (int i = 0; i < 999; ++i) storage.set("i", i, loopLevel);
    ASSERT_NE(storage.lookup("i", loopLevel), nullptr);
    EXPECT_EQ(*storage.lookup("i", loopLevel), 998);
    storage.popLevel(loopLevel);
    ASSERT_NE(storage.lookup("i", parentLevel), nullptr);
    EXPECT_EQ(*storage.lookup("i", parentLevel), -1);
}

TEST(IndexedScopeTrailStorage, SetAtSameLevelDoesNotAccumulateEntries) {
    auto intern = std::make_shared<DynNameIntern>();
    IndexedScopeTrailStorage<int> storage(intern);
    const int level = storage.openLevel(0);
    for (int i = 0; i < 1000; ++i) storage.set("$fn", i, level);
    EXPECT_EQ(storage.debugEntryCountForTesting("$fn"), 1u);
    ASSERT_NE(storage.lookup("$fn", level), nullptr);
    EXPECT_EQ(*storage.lookup("$fn", level), 999);
}

} // namespace
} // namespace oscadeval
