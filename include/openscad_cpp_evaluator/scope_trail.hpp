#pragma once

#include "openscad_cpp_evaluator/value.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace oscadeval {

// A "binding trail" (the classic Prolog/WAM undo-log pattern): each
// variable name owns a stack of (value, level) entries. Setting a name
// pushes; entering a new scope opens a fresh `level`; exiting a scope pops
// every entry tagged with that level, in O(bindings actually made at that
// level) rather than O(whole-map-size) -- replacing EvalContext's previous
// copy-the-whole-map-on-every-derivation design. Safe here specifically
// because this codebase has no escaping closures: a FunctionLiteral value
// is a bare AST pointer with no captured environment, and callCtxFor()
// re-derives every call's context from the LIVE call stack -- nothing ever
// reads a binding after the scope that pushed it has already exited.
//
// Visibility is by TRUE ANCESTRY, not by level-number comparison: each
// level records its own PARENT level (the level that was current when it
// was opened, or 0 -- meaning "no parent" -- for an isolating derivation,
// see TrailView::openChild). A view at level L can see an entry pushed at
// level E iff E appears on L's own parent chain (L, parent(L), parent(
// parent(L)), ... down to 0). This was NOT the original design here: an
// earlier version used a single numeric "ceiling" (visible iff E <= L),
// which is necessary but not sufficient -- it incorrectly treated any
// chronologically-earlier-but-still-open level as an ancestor, even an
// unrelated sibling/isolated scope that happens to still be on the call
// stack. Caught on a real script (BOSL2's attachable()/trapezoid(), which
// deep-forwards children() through several more calls while attachable()
// itself -- an ISOLATED scope with its own same-named "path"/"h"
// parameters, still open throughout -- sits on the stack): the ceiling-
// only check found attachable()'s own unbound "path"/"h" (undef) instead
// of trapezoid()'s real ones, since level-number order alone can't tell
// "ancestor" from "unrelated open branch." Parent-chain walking fixes this
// by construction. See eval_context.hpp for how each derivation method
// picks its own level and (via `isolate`) whether the parent chain
// continues to the caller or terminates there.
template <typename T>
class ScopeTrailStorage {
public:
    // `parentLevel`: the calling view's own level, or 0 to terminate the
    // ancestry chain here (an isolating derivation -- see openChild).
    int openLevel(int parentLevel) {
        int level = ++nextLevel_;
        parent_[level] = parentLevel;
        return level;
    }

    void set(const std::string& name, T value, int level) {
        stacks_[name].push_back(Entry{std::move(value), level});
        dirty_[level].push_back(name);
    }

    // Pops every entry pushed at `level`, across every name that was
    // touched at that level (dirty_[level], recorded by set() above) --
    // O(bindings made at this level), not O(total names). Left-in-place,
    // possibly-now-empty per-name vectors are NOT erased from `stacks_`:
    // ponytail: real scripts rebind the same small name set repeatedly
    // (loop vars, $fn, ...), so paying one erase+reinsert per pop is pure
    // waste; revisit only if memory footprint (not speed) ever measurably
    // matters for a script with a huge, non-repeating variable vocabulary.
    void popLevel(int level) {
        auto it = dirty_.find(level);
        if (it != dirty_.end()) {
            for (const std::string& name : it->second) {
                auto sit = stacks_.find(name);
                if (sit != stacks_.end() && !sit->second.empty()) sit->second.pop_back();
            }
            dirty_.erase(it);
        }
        parent_.erase(level);
    }

    // nullptr if `name` has no binding on `myLevel`'s own ancestry chain.
    // Both `it->second` (a name's own pushes, chronological -- and since
    // level numbers are assigned in one global monotonically increasing
    // sequence, chronological order IS ascending-by-level order) and
    // myLevel's own ancestry chain (myLevel, parent(myLevel), ...,
    // descending) are sorted -- so this is a standard sorted-merge search
    // for the largest value common to both: whichever side currently
    // points at the larger level advances (skip a too-deep/unrelated
    // push, or walk one step further up my own ancestry); equal levels
    // mean this push IS on my ancestry chain, and being the largest such
    // common value, it's also the innermost (most specific) applicable
    // one. O(pushes to this name that postdate my nearest common
    // ancestor's own binding, plus my own ancestry depth to that point) --
    // in the common case (read shortly after write, or the name's own
    // pushes are all real ancestors) this is a handful of steps, not a
    // full O(map-size) scan.
    const T* lookup(const std::string& name, int myLevel) const {
        auto it = stacks_.find(name);
        if (it == stacks_.end()) return nullptr;
        const std::vector<Entry>& vec = it->second;
        auto idx = static_cast<long>(vec.size()) - 1;
        int ancestor = myLevel;
        while (ancestor != 0 && idx >= 0) {
            const int entryLevel = vec[static_cast<size_t>(idx)].level;
            if (entryLevel > ancestor) {
                --idx;
            } else if (entryLevel == ancestor) {
                return &vec[static_cast<size_t>(idx)].value;
            } else {
                auto pit = parent_.find(ancestor);
                ancestor = pit != parent_.end() ? pit->second : 0;
            }
        }
        return nullptr;
    }

    bool has(const std::string& name, int myLevel) const { return lookup(name, myLevel) != nullptr; }

    // Ancestry-filtered snapshot -- O(total distinct names ever bound so
    // far in the whole evaluation), not O(names visible now). Only used
    // by debug-REPL/children()-splice enumeration (never a hot path); see
    // this class's own module comment for why this asymmetry with
    // lookup()'s much cheaper cost is accepted, not optimized.
    std::vector<std::pair<std::string, T>> items(int myLevel) const {
        std::vector<std::pair<std::string, T>> result;
        for (const auto& [name, stack] : stacks_) {
            if (const T* v = lookup(name, myLevel)) result.emplace_back(name, *v);
        }
        return result;
    }

private:
    struct Entry {
        T value;
        int level;
    };
    std::unordered_map<std::string, std::vector<Entry>> stacks_;
    std::unordered_map<int, std::vector<std::string>> dirty_;
    std::unordered_map<int, int> parent_; // level -> parent level (0 = chain terminates here)
    int nextLevel_ = 0;
};

// Per-EvalContext handle onto a shared ScopeTrailStorage<T>: this view's
// own `level_` is both what new writes via set() get tagged with AND the
// starting point for a read's ancestry walk (see ScopeTrailStorage::
// lookup) -- see eval_context.cpp for how each of withScope()/childCtx()/
// callCtx()/letChildCtx() derives it. Scope-exit is automatic via this
// class's own destructor, which calls storage_->popLevel(level_) (normal
// return, an early `return`, or exception unwind all trigger it
// identically, no try/catch needed at any call site) -- safe specifically
// because a TrailView is only ever reached through shared_ptr<TrailView<T>>
// (every field that holds one, e.g. EvalContext::let_, is that shared_ptr,
// never the object itself), so the *outer* shared_ptr's own refcounting
// already guarantees the destructor fires exactly once, on the last live
// reference. Copy/move are deleted to keep that invariant enforced rather
// than merely documented -- this used to be a separate `shared_ptr<void>
// guard_` member with its own custom-deleter control-block allocation
// (belt-and-suspenders against a direct-copy hazard that can't actually
// happen given the invariant above); folding popLevel() into ~TrailView()
// directly halves childCtx()/callCtx()'s allocation count (4 trails * 2
// allocations each down to 4 trails * 1) with no change to the
// ancestry-walk/isolation logic itself.
template <typename T>
class TrailView {
public:
    static std::shared_ptr<TrailView<T>> makeRoot() {
        auto storage = std::make_shared<ScopeTrailStorage<T>>();
        int level = storage->openLevel(0);
        return std::make_shared<TrailView<T>>(storage, level);
    }

    // Open a fresh level for writes. `isolate=true` terminates the new
    // view's ancestry chain at itself (callCtx()'s isolation of
    // let_/dynPositions -- nothing from the caller is visible, matching
    // an isolated call's own fresh scope); `isolate=false` continues the
    // chain through this view's own level (childCtx()/letChildCtx() --
    // reads see through to the caller's own bindings, while only this
    // scope's own writes get popped away on exit).
    std::shared_ptr<TrailView<T>> openChild(bool isolate) const {
        int level = storage_->openLevel(isolate ? 0 : level_);
        return std::make_shared<TrailView<T>>(storage_, level);
    }

    TrailView(std::shared_ptr<ScopeTrailStorage<T>> storage, int level)
        : storage_(std::move(storage)), level_(level) {}
    ~TrailView() { storage_->popLevel(level_); }
    TrailView(const TrailView&) = delete;
    TrailView& operator=(const TrailView&) = delete;
    TrailView(TrailView&&) = delete;
    TrailView& operator=(TrailView&&) = delete;

    void set(const std::string& name, T value) { storage_->set(name, std::move(value), level_); }
    const T* find(const std::string& name) const { return storage_->lookup(name, level_); }
    bool count(const std::string& name) const { return storage_->has(name, level_) ? 1 : 0; }
    bool empty() const { return storage_->items(level_).empty(); }
    const T& at(const std::string& name) const {
        const T* v = find(name);
        if (!v) throw std::out_of_range("TrailView::at: key not found");
        return *v;
    }
    std::vector<std::pair<std::string, T>> items() const { return storage_->items(level_); }

private:
    std::shared_ptr<ScopeTrailStorage<T>> storage_;
    int level_;
};

// Shared bidirectional name<->small-int interning table for $-prefixed
// dynamic variables. Owned jointly by dyn's and dynExplicit's storage (see
// EvalContext::makeRoot()) so a name like "$fn" interns to the SAME id in
// both -- required since assigning a $-var writes dyn and marks
// dynExplicit using that one id. The distinct-name vocabulary of any real
// script is tiny (a handful of builtins plus whatever the script defines),
// so this table stays small and is effectively populated once.
class DynNameIntern {
public:
    int idFor(const std::string& name) {
        auto it = ids_.find(name);
        if (it != ids_.end()) return it->second;
        int id = static_cast<int>(names_.size());
        ids_.emplace(name, id);
        names_.push_back(name);
        return id;
    }
    const std::string& nameFor(int id) const { return names_[static_cast<size_t>(id)]; }

private:
    std::unordered_map<std::string, int> ids_;
    std::vector<std::string> names_;
};

// The int-keyed twin of ScopeTrailStorage above, used only for $-dynamic
// variables (dyn/dynExplicit): the identical ancestry-chain visibility
// algorithm, but keyed by a small interned integer (via DynNameIntern)
// instead of a raw std::string, so each name's own push-stack lives in a
// directly-indexed std::vector rather than an unordered_map<string,...> --
// no per-access string hashing/comparison once a name has been interned.
// The public API still takes plain std::string names (interning happens
// internally, transparent to every existing call site) -- this alone
// removes string-vs-string comparison during hashmap collision resolution
// and improves cache locality; the larger win (skipping the interning
// lookup entirely) is realized once the bytecode compiler (a later phase)
// resolves a name to its id ONCE at compile time and threads the id
// through as an instruction immediate instead of re-interning on every
// execution.
template <typename T>
class IndexedScopeTrailStorage {
public:
    explicit IndexedScopeTrailStorage(std::shared_ptr<DynNameIntern> intern) : intern_(std::move(intern)) {}

    int openLevel(int parentLevel) {
        int level = ++nextLevel_;
        parent_[level] = parentLevel;
        return level;
    }

    void set(const std::string& name, T value, int level) {
        int id = intern_->idFor(name);
        ensureSize(id);
        stacks_[static_cast<size_t>(id)].push_back(Entry{std::move(value), level});
        dirty_[level].push_back(id);
    }

    void popLevel(int level) {
        auto it = dirty_.find(level);
        if (it != dirty_.end()) {
            for (int id : it->second) {
                auto& stack = stacks_[static_cast<size_t>(id)];
                if (!stack.empty()) stack.pop_back();
            }
            dirty_.erase(it);
        }
        parent_.erase(level);
    }

    // Same sorted-merge ancestry walk as ScopeTrailStorage::lookup -- see
    // its own doc comment for the full algorithm rationale.
    const T* lookup(const std::string& name, int myLevel) const { return lookupById(intern_->idFor(name), myLevel); }

    const T* lookupById(int id, int myLevel) const {
        if (id < 0 || static_cast<size_t>(id) >= stacks_.size()) return nullptr;
        const std::vector<Entry>& vec = stacks_[static_cast<size_t>(id)];
        auto idx = static_cast<long>(vec.size()) - 1;
        int ancestor = myLevel;
        while (ancestor != 0 && idx >= 0) {
            const int entryLevel = vec[static_cast<size_t>(idx)].level;
            if (entryLevel > ancestor) {
                --idx;
            } else if (entryLevel == ancestor) {
                return &vec[static_cast<size_t>(idx)].value;
            } else {
                auto pit = parent_.find(ancestor);
                ancestor = pit != parent_.end() ? pit->second : 0;
            }
        }
        return nullptr;
    }

    bool has(const std::string& name, int myLevel) const { return lookup(name, myLevel) != nullptr; }

    std::vector<std::pair<std::string, T>> items(int myLevel) const {
        std::vector<std::pair<std::string, T>> result;
        for (size_t id = 0; id < stacks_.size(); ++id) {
            if (const T* v = lookupById(static_cast<int>(id), myLevel)) {
                result.emplace_back(intern_->nameFor(static_cast<int>(id)), *v);
            }
        }
        return result;
    }

private:
    struct Entry {
        T value;
        int level;
    };
    void ensureSize(int id) {
        if (static_cast<size_t>(id) >= stacks_.size()) stacks_.resize(static_cast<size_t>(id) + 1);
    }
    std::shared_ptr<DynNameIntern> intern_;
    std::vector<std::vector<Entry>> stacks_;
    std::unordered_map<int, std::vector<int>> dirty_;
    std::unordered_map<int, int> parent_;
    int nextLevel_ = 0;
};

// Per-EvalContext handle onto a shared IndexedScopeTrailStorage<T> -- the
// int-keyed twin of TrailView<T> above; see that class's doc comment for
// the destructor-based scope-exit mechanism and why copy/move are deleted,
// unchanged here.
template <typename T>
class IndexedTrailView {
public:
    static std::shared_ptr<IndexedTrailView<T>> makeRoot(std::shared_ptr<DynNameIntern> intern) {
        auto storage = std::make_shared<IndexedScopeTrailStorage<T>>(std::move(intern));
        int level = storage->openLevel(0);
        return std::make_shared<IndexedTrailView<T>>(storage, level);
    }

    std::shared_ptr<IndexedTrailView<T>> openChild(bool isolate) const {
        int level = storage_->openLevel(isolate ? 0 : level_);
        return std::make_shared<IndexedTrailView<T>>(storage_, level);
    }

    IndexedTrailView(std::shared_ptr<IndexedScopeTrailStorage<T>> storage, int level)
        : storage_(std::move(storage)), level_(level) {}
    ~IndexedTrailView() { storage_->popLevel(level_); }
    IndexedTrailView(const IndexedTrailView&) = delete;
    IndexedTrailView& operator=(const IndexedTrailView&) = delete;
    IndexedTrailView(IndexedTrailView&&) = delete;
    IndexedTrailView& operator=(IndexedTrailView&&) = delete;

    void set(const std::string& name, T value) { storage_->set(name, std::move(value), level_); }
    const T* find(const std::string& name) const { return storage_->lookup(name, level_); }
    bool count(const std::string& name) const { return storage_->has(name, level_) ? 1 : 0; }
    bool empty() const { return storage_->items(level_).empty(); }
    const T& at(const std::string& name) const {
        const T* v = find(name);
        if (!v) throw std::out_of_range("IndexedTrailView::at: key not found");
        return *v;
    }
    std::vector<std::pair<std::string, T>> items() const { return storage_->items(level_); }

private:
    std::shared_ptr<IndexedScopeTrailStorage<T>> storage_;
    int level_;
};

// dyn and dynExplicit ("which $-names the SCRIPT itself explicitly
// assigned", as opposed to merely present via an ambient seed/default)
// used to be two fully independent IndexedTrailViews, always opened
// together at every derivation site (see eval_context.cpp) but never
// actually sharing storage -- two heap allocations per childCtx()/
// callCtx()/letChildCtx() call where one would do. DynEntry/DynValueView/
// DynExplicitView below combine them into ONE underlying
// IndexedTrailView<DynEntry>, cutting that to one allocation; the two
// view classes are thin, distinctly-typed PROJECTIONS of the very same
// shared_ptr<IndexedTrailView<DynEntry>> (shared via a plain shared_ptr
// copy between them -- a refcount bump, not a second heap object), so
// every existing dyn/dynExplicit call site keeps reading/writing through
// the exact same method names/shapes it always did (find/set/count/
// items/empty), via each view's operator-> returning `this` (so
// `ctx.dyn->find(...)` keeps compiling unchanged even though `dyn` is now
// a plain value member, not a shared_ptr).
//
// dyn and dynExplicit are set INDEPENDENTLY at many call sites -- e.g. a
// parameter default binds only the value (dynExplicit untouched), while
// an explicit `$fn=64;` assignment binds both. A write through EITHER
// view must not clobber the OTHER field's own ancestry-visible value for
// that name at this exact level, so both views' set() read the name's
// CURRENTLY VISIBLE combined entry first (via the shared ancestry walk)
// and carry the other field forward unchanged, rather than resetting it.
struct DynEntry {
    Value value;
    bool explicitlySet = false;
};

class DynValueView {
public:
    DynValueView() = default;
    explicit DynValueView(std::shared_ptr<IndexedTrailView<DynEntry>> trail) : trail_(std::move(trail)) {}

    DynValueView* operator->() { return this; }
    const DynValueView* operator->() const { return this; }

    // The raw shared trail, so EvalContext's derivation methods can open
    // ONE new level and wrap it into both a DynValueView and a
    // DynExplicitView, rather than each view opening its own (which would
    // silently re-fork them into two independent trails again).
    const std::shared_ptr<IndexedTrailView<DynEntry>>& trail() const { return trail_; }

    void set(const std::string& name, Value value) {
        bool explicitlySet = false;
        if (const DynEntry* existing = trail_->find(name)) explicitlySet = existing->explicitlySet;
        trail_->set(name, DynEntry{std::move(value), explicitlySet});
    }
    const Value* find(const std::string& name) const {
        const DynEntry* e = trail_->find(name);
        return e ? &e->value : nullptr;
    }
    bool count(const std::string& name) const { return find(name) != nullptr; }
    bool empty() const { return trail_->empty(); }
    const Value& at(const std::string& name) const {
        const Value* v = find(name);
        if (!v) throw std::out_of_range("DynValueView::at: key not found");
        return *v;
    }
    std::vector<std::pair<std::string, Value>> items() const {
        std::vector<std::pair<std::string, Value>> result;
        for (auto& [name, entry] : trail_->items()) result.emplace_back(name, entry.value);
        return result;
    }

private:
    std::shared_ptr<IndexedTrailView<DynEntry>> trail_;
};

class DynExplicitView {
public:
    DynExplicitView() = default;
    explicit DynExplicitView(std::shared_ptr<IndexedTrailView<DynEntry>> trail) : trail_(std::move(trail)) {}

    DynExplicitView* operator->() { return this; }
    const DynExplicitView* operator->() const { return this; }

    void set(const std::string& name, bool explicitlySet) {
        Value value;
        if (const DynEntry* existing = trail_->find(name)) value = existing->value;
        trail_->set(name, DynEntry{std::move(value), explicitlySet});
    }
    bool find(const std::string& name) const {
        const DynEntry* e = trail_->find(name);
        return e && e->explicitlySet;
    }
    bool count(const std::string& name) const { return find(name); }
    // NOT trail_->empty() -- the shared trail can hold plenty of
    // value-only entries (a seeded default, viewportParams, ...) with
    // explicitlySet=false. dynExplicit's own "empty" means "no NAME has
    // ever been explicitly assigned", which only items() (already
    // filtering on that flag) can answer correctly.
    bool empty() const {
        for (auto& [name, entry] : trail_->items()) {
            if (entry.explicitlySet) return false;
        }
        return true;
    }
    std::vector<std::pair<std::string, bool>> items() const {
        std::vector<std::pair<std::string, bool>> result;
        for (auto& [name, entry] : trail_->items()) result.emplace_back(name, entry.explicitlySet);
        return result;
    }

private:
    std::shared_ptr<IndexedTrailView<DynEntry>> trail_;
};

using NameSet = std::unordered_set<std::string>;

inline NameSet explicitSnapshot(const DynExplicitView& trail) {
    NameSet result;
    for (auto& [name, v] : trail.items()) {
        if (v) result.insert(name);
    }
    return result;
}

} // namespace oscadeval
