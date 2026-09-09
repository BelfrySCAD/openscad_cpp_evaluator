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

TEST(ScopeTrailStorage, PoppedLevelTerminatesADescendantsAncestryWalk) {
    // Pins the two invariants the level->parent table has to keep, whatever
    // it is implemented with (it is a vector indexed by level, not a map, for
    // allocation reasons -- see ScopeTrailStorage::parent_): a popped level
    // stops an ancestry walk dead rather than forwarding it to the
    // grandparent, and level numbers are never reused afterwards.
    ScopeTrailStorage<int> storage;
    const int grandParent = storage.openLevel(0);
    storage.set("x", 7, grandParent);
    const int parent = storage.openLevel(grandParent);
    const int child = storage.openLevel(parent);
    ASSERT_NE(storage.lookup("x", child), nullptr);
    EXPECT_EQ(*storage.lookup("x", child), 7);

    storage.popLevel(parent); // child is still live (an escaping closure)
    EXPECT_EQ(storage.lookup("x", child), nullptr);
    EXPECT_NE(storage.openLevel(0), parent); // levels are never renumbered

    // The grandparent's own binding is untouched -- only the walk THROUGH
    // the popped level is cut.
    ASSERT_NE(storage.lookup("x", grandParent), nullptr);
    EXPECT_EQ(*storage.lookup("x", grandParent), 7);
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

// popLevel must remove the entry belonging to THE LEVEL BEING POPPED, not
// blindly pop_back() whatever happens to be physically last -- the exact
// out-of-order-pop bug class ScopeTrailStorage::popLevel's own doc comment
// already describes (and fixed) for the non-indexed twin; this indexed
// variant kept the blind pop until Op::CallChildren's forwarding frame
// (bytecode_vm.cpp) became the first real caller to violate LIFO view
// destruction on the dyn trail. Level shape mirrors that real scenario
// exactly: E (a children() call's own effCtx, opened first) and L (the
// forwarding evalCtx, opened later, moved into a VmFrame) both set the
// same name; E pops FIRST while L is still live. The blind pop removed
// L's entry -- the still-live forwarded value -- leaving a lookup from L
// falling through to the root default (caught for real: children($fn=9)
// read back as 0 inside the forwarded child).
TEST(IndexedScopeTrailStorage, PopLevelRemovesItsOwnEntryNotThePhysicallyLastOne) {
    auto intern = std::make_shared<DynNameIntern>();
    IndexedScopeTrailStorage<int> storage(intern);
    const int root = storage.openLevel(0);
    storage.set("$fn", 0, root);
    const int e = storage.openLevel(root); // effCtx's own level
    storage.set("$fn", 9, e);
    const int l = storage.openLevel(root); // forwarding evalCtx's level, opened AFTER e
    storage.set("$fn", 9, l);
    storage.popLevel(e); // e dies while l is still live -- non-LIFO
    ASSERT_NE(storage.lookup("$fn", l), nullptr);
    EXPECT_EQ(*storage.lookup("$fn", l), 9);
    storage.popLevel(l);
    ASSERT_NE(storage.lookup("$fn", root), nullptr);
    EXPECT_EQ(*storage.lookup("$fn", root), 0);
}

// A captured (closure) level's own ancestor must stay alive even after
// EVERY other TrailView referencing that ancestor has gone out of scope --
// see TrailView::openChild's own doc comment (parentView_) for why. This
// reproduces the exact level shape a real escaping-closure scenario builds
// (traced from `function make(x,table)=let(table=...) x!=undef?
// make(table=table)(v=x) : function(v) v!=undef?
// let(nt=concat(table,[v])) make(table=nt) : table;`), with each C++ scope
// block standing in for one step of Evaluator::evalFunctionBodyTrampoline's
// own `chain` vector going out of scope once its owning call returns:
//   1) M1_CALL (isolated call level for the outer make(10))
//   2) M1_LET  (its own let(table=...) child level)
//   3) M2_CALL (a NESTED, non-tail recursive make(table=table) call --
//      isolated, since callCtxFor's sameSpan check correctly refuses to
//      treat same-function recursion as "still nested")
//   4) M2_LET  (that call's own let(table=...))
//   5) C1_CALL (capturedLet-rooted: invoking the closure captured at
//      M2_LET, continuing ITS ancestry rather than the caller's)
//   6) C1_LET  (that invocation's own let(newtable=...))
//   7) M3_CALL (a SECOND tail-hop into the NAMED function make(table=
//      newtable) -- also isolated, not capturedLet-based)
//   8) M3_LET  (that hop's own let(table=...) -- this is the level the
//      SECOND closure, C2, actually captures)
// After every one of 1-8 goes out of scope except the shared_ptr a
// "Closure" would hold (mirroring M3_LET, i.e. what capturedLet points
// to), invoking that closure must still resolve `table` -- bound all the
// way back at M3_LET itself, so this specific case doesn't even need to
// walk past one level, but the SURROUNDING chain (steps 1-7) must not have
// corrupted or blocked that walk on its own way out.
TEST(TrailView, EscapingClosureThroughDoubleTailHopStillSeesAncestorAfterChainUnwinds) {
    std::shared_ptr<TrailView<int>> capturedLet;
    {
        auto root = TrailView<int>::makeRoot();
        auto m1Call = root->openChild(/*isolate=*/true);
        m1Call->set("x", 10);
        auto m1Let = m1Call->openChild(/*isolate=*/false);
        m1Let->set("table", 123);
        auto m2Call = m1Let->openChild(/*isolate=*/true);
        m2Call->set("table", 123);
        auto m2Let = m2Call->openChild(/*isolate=*/false);
        m2Let->set("table", 123);
        auto c1Call = m2Let->openChild(/*isolate=*/false);
        c1Call->set("v", 10);
        auto c1Let = c1Call->openChild(/*isolate=*/false);
        c1Let->set("newtable", 1230);
        auto m3Call = c1Let->openChild(/*isolate=*/true);
        m3Call->set("table", 1230);
        auto m3Let = m3Call->openChild(/*isolate=*/false);
        m3Let->set("table", 1230);
        capturedLet = m3Let;
    } // every level above except capturedLet's own (m3Let) goes out of scope here

    auto c2Call = capturedLet->openChild(/*isolate=*/false);
    ASSERT_NE(c2Call->find("table"), nullptr);
    EXPECT_EQ(*c2Call->find("table"), 1230);
}

TEST(TrailView, MinimalTwoLevelKeepAlive) {
    std::shared_ptr<TrailView<int>> captured;
    {
        auto root = TrailView<int>::makeRoot();
        auto parent = root->openChild(/*isolate=*/true);
        parent->set("x", 42);
        auto child = parent->openChild(/*isolate=*/false);
        captured = child;
    }
    auto grandchild = captured->openChild(/*isolate=*/false);
    ASSERT_NE(grandchild->find("x"), nullptr);
    EXPECT_EQ(*grandchild->find("x"), 42);
}


// parent_ (level -> parent level) used to grow by one int per level ever
// opened and never shrink: 3 x 64MB vectors on a 13M-call script, plus the
// copy at every doubling. A popped level is now marked dead, and whenever the
// NEWEST level dies the vector shrinks back past every dead level above the
// newest live one, whose numbers are then handed out again.
TEST(ScopeTrailStorage, PoppingInCallOrderShrinksTheLevelTable) {
    ScopeTrailStorage<int> storage;
    const int root = storage.openLevel(0);
    const size_t base = storage.debugLevelSlotsForTesting();
    for (int call = 0; call < 1000; ++call) {
        const int a = storage.openLevel(root);
        const int b = storage.openLevel(a);
        storage.set("x", call, b);
        ASSERT_EQ(*storage.lookup("x", b), call);
        storage.popLevel(b);
        storage.popLevel(a);
        EXPECT_EQ(storage.debugLevelSlotsForTesting(), base) << "call " << call;
        EXPECT_EQ(a, root + 1) << "level numbers are reused, not consumed";
    }
    EXPECT_EQ(storage.debugEntryCountForTesting("x"), 0u);
}

TEST(ScopeTrailStorage, AStillLiveLevelStopsTheShrinkAndKeepsItsBindings) {
    // An escaping closure keeps one level alive past its creator's return.
    ScopeTrailStorage<int> storage;
    const int root = storage.openLevel(0);
    storage.set("g", 1, root);
    const int call = storage.openLevel(root);
    const int captured = storage.openLevel(call); // the closure holds this one
    storage.set("c", 42, captured);
    const int later = storage.openLevel(captured);
    storage.popLevel(later);
    storage.popLevel(call); // out of order: `captured` outlives `call`
    const size_t slots = storage.debugLevelSlotsForTesting();
    EXPECT_EQ(slots, static_cast<size_t>(captured) + 1) << "shrunk past `later`, stopped at the live level";
    // New levels take numbers ABOVE the live one, so its entries stay sorted
    // and reachable exactly as before.
    const int fresh = storage.openLevel(root);
    EXPECT_GT(fresh, captured);
    storage.set("c", 7, fresh);
    EXPECT_EQ(*storage.lookup("c", captured), 42);
    EXPECT_EQ(*storage.lookup("c", fresh), 7);
    // (Reads THROUGH the popped `call` level stop there, as they always did
    // -- a real escaping closure keeps its whole ancestor chain alive via
    // TrailView::parentView_, so this storage-level walk never sees that.)
    EXPECT_EQ(storage.lookup("g", captured), nullptr);
    storage.popLevel(fresh);
    storage.popLevel(captured);
    EXPECT_EQ(storage.debugLevelSlotsForTesting(), static_cast<size_t>(root) + 1);
}

TEST(IndexedScopeTrailStorage, PoppingInCallOrderShrinksTheLevelTable) {
    IndexedScopeTrailStorage<int> storage(std::make_shared<DynNameIntern>());
    const int root = storage.openLevel(0);
    const size_t base = storage.debugLevelSlotsForTesting();
    for (int call = 0; call < 1000; ++call) {
        const int a = storage.openLevel(root);
        storage.set("$fn", call, a);
        ASSERT_EQ(*storage.lookup("$fn", a), call);
        storage.popLevel(a);
        ASSERT_EQ(storage.debugLevelSlotsForTesting(), base);
    }
}
} // namespace
} // namespace oscadeval


