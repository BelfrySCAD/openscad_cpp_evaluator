#pragma once

#include "openscad_cpp_evaluator/scope_trail.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast/ast_node.hpp"
#include "openscad_cpp_parser/position.hpp"
#include "openscad_cpp_parser/scope.hpp"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace oscadeval {

using ChildrenNodeList = std::vector<const oscad::ASTNode*>;

// Mutable evaluation state threaded through recursive evaluation. Mirrors
// the Python reference's EvalContext dataclass field-for-field.
// `let_`/`dynPositions` are each a `shared_ptr` onto a TrailView
// (scope_trail.hpp) -- a binding-trail handle, not an owned map/set;
// `dyn`/`dynExplicit` are each a thin value-type VIEW (DynValueView/
// DynExplicitView) wrapping a `shared_ptr` onto ONE shared underlying
// trail between them (see scope_trail.hpp's own doc comment on those two
// classes) -- copying either view is just a shared_ptr copy underneath,
// same aliasing behavior as the plain shared_ptr fields. This replaced an
// earlier design where each field was a `shared_ptr<unordered_map>`
// copied wholesale on every derivation (childCtx()/callCtx()/
// letChildCtx()): profiling a real BOSL2-heavy script found that copying
// was a dominant cost (hundreds of thousands of user function/module
// calls, each paying for a full map copy). The trail makes every
// derivation an O(1) "open a new level" instead, since this codebase has
// no escaping closures to worry about (see scope_trail.hpp's own module
// comment) -- popLevel() on scope-exit only pops what that scope
// actually pushed, not the whole map's contents.
//
// `withScope()` (below) still relies on shared_ptr's reference semantics
// exactly like the old design did: an Assignment statement mutates the
// trail in place (via `set()`), and every sibling EvalContext built for
// the same block (one per statement, see Evaluator::evalChildren) must
// see that mutation immediately -- achieved by aliasing the SAME
// TrailView (same underlying storage, same level, same floor), not by
// opening a new one.
struct EvalContext {
    const oscad::Scope* scope = nullptr;
    // dyn/dynExplicit share ONE underlying trail (DynValueView/
    // DynExplicitView, scope_trail.hpp) -- two distinctly-typed
    // projections of the same shared_ptr<IndexedTrailView<DynEntry>>, not
    // two independent trails, so a derivation only opens one new level for
    // both. Each still reads/writes exactly like its own former
    // independent trail (find/set/count/items/empty via operator->).
    DynValueView dyn;            // $-prefixed only, interned-int-keyed
    std::shared_ptr<TrailView<Value>> let_;                      // plain-named bindings
    std::shared_ptr<TrailView<const oscad::Position*>> dynPositions;
    DynExplicitView dynExplicit; // which $-names the script itself assigned
    std::optional<std::array<double, 4>> color;
    std::shared_ptr<const ChildrenNodeList> childrenNodes;
    const EvalContext* childrenCallerCtx = nullptr;

    // The one genuinely fresh construction: seeds `dyn` with OpenSCAD's
    // built-in $-variable defaults ($fn=0, $fa=12, $fs=2, $t=0,
    // $parent_modules=0). Every other EvalContext in a run is derived
    // from this one via the methods below.
    static EvalContext makeRoot(const oscad::Scope* rootScope);

    // Mirrors _eval_children's direct EvalContext(...) construction: swaps
    // only `scope` (to a sibling statement's own lexical scope from
    // buildScopes()), aliasing every other field with `this` -- NOT a
    // snapshot, and NOT a new trail level either. This is what makes
    // `a = 1; cube(a); a = 2;`'s second assignment visible to a statement
    // evaluated after it.
    EvalContext withScope(const oscad::Scope* newScope) const;

    // Opens a new level for dyn/let_/dynExplicit (reads still see through
    // to whatever this call's own ancestors already had bound -- a
    // closure "inherits" the enclosing call's locals), but ISOLATES
    // dynPositions (raises its floor to the new level -- fresh/empty from
    // this context's own perspective), matching the reference's
    // asymmetric child_ctx() rule exactly. `newScope`/`newColor`/
    // `newChildrenNodes`/`newChildrenCallerCtx` default (null/nullopt) to
    // INHERITING from `this`, not resetting -- getting this backwards is a
    // real shipped-bug class in the reference (a children() forwarding
    // chain silently swallowed under `color(c) children();`-shaped user
    // modules) -- see docs/evaluator.md's child_ctx() entry.
    EvalContext childCtx(const oscad::Scope* newScope = nullptr,
                          std::optional<std::array<double, 4>> newColor = std::nullopt,
                          std::shared_ptr<const ChildrenNodeList> newChildrenNodes = nullptr,
                          const EvalContext* newChildrenCallerCtx = nullptr) const;

    // Isolated call scope for entering a module/function body: opens a new
    // level for dyn/dynExplicit but leaves their floor unchanged (so
    // $-vars remain dynamically scoped into the call -- reads see through
    // to the caller's own bindings), while ISOLATING let_/dynPositions
    // (new level AND new floor -- the callee has its own variable scope;
    // inheriting the caller's plain-named bindings would trigger spurious
    // double-assignment warnings). Unlike childCtx(), childrenNodes/
    // childrenCallerCtx do NOT inherit when omitted -- they reset to
    // empty/null, since entering a real call means childrenNodes should be
    // that call's own children (passed explicitly by the caller), not
    // whatever the calling context happened to have queued.
    EvalContext callCtx(const oscad::Scope* newScope = nullptr,
                         std::optional<std::array<double, 4>> newColor = std::nullopt,
                         std::shared_ptr<const ChildrenNodeList> newChildrenNodes = nullptr,
                         const EvalContext* newChildrenCallerCtx = nullptr) const;

    // let(...)/list-comprehension let() scoping: opens a new level for all
    // four trails, floor unchanged on every one (reads see through to the
    // parent; this let-body's own writes -- including to dyn/dynPositions/
    // dynExplicit, e.g. a `let($fn=99)` override -- pop away cleanly once
    // the let() returns, never leaking back to the parent).
    EvalContext letChildCtx() const;
};

} // namespace oscadeval
