#pragma once

#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast/ast_node.hpp"
#include "openscad_cpp_parser/position.hpp"
#include "openscad_cpp_parser/scope.hpp"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace oscadeval {

using DynMap = std::unordered_map<std::string, Value>;
using LetMap = std::unordered_map<std::string, Value>;
using DynPositionMap = std::unordered_map<std::string, const oscad::Position*>;
using NameSet = std::unordered_set<std::string>;
using ChildrenNodeList = std::vector<const oscad::ASTNode*>;

// Mutable evaluation state threaded through recursive evaluation. Mirrors
// the Python reference's EvalContext dataclass field-for-field, with one
// structural difference forced by the language: `dyn`/`let_`/`dynPositions`
// /`dynExplicit` are shared_ptr-wrapped, not plain value members, because
// one real call site (withScope(), below) relies on Python's dicts being
// *reference* types -- an Assignment statement mutates the map in place,
// and every sibling EvalContext built for the same block (one per
// statement, see Evaluator::evalChildren) must see that mutation
// immediately. Plain std::unordered_map members copied on every derivation
// would silently break the "a later statement sees an earlier sibling's
// assignment" rule the whole evaluator depends on (doc: openscad_evaluator/
// docs/evaluator.md, "Assignment execution order").
struct EvalContext {
    const oscad::Scope* scope = nullptr;
    std::shared_ptr<DynMap> dyn;       // $-prefixed only
    std::shared_ptr<LetMap> let_;      // plain-named bindings
    std::shared_ptr<DynPositionMap> dynPositions;
    std::shared_ptr<NameSet> dynExplicit;
    std::optional<std::array<double, 4>> color;
    std::shared_ptr<const ChildrenNodeList> childrenNodes;
    const EvalContext* childrenCallerCtx = nullptr;

    // The one genuinely fresh construction: seeds `dyn` with OpenSCAD's
    // built-in $-variable defaults ($fn=0, $fa=12, $fs=2, $t=0,
    // $parent_modules=0), matching Evaluator::evaluate()'s (later phase)
    // root EvalContext(scope=root_scope). Every other EvalContext in a run
    // is derived from this one via the methods below.
    static EvalContext makeRoot(const oscad::Scope* rootScope);

    // Mirrors _eval_children's direct EvalContext(...) construction: swaps
    // only `scope` (to a sibling statement's own lexical scope from
    // buildScopes()), aliasing every other field with `this` by reference
    // -- NOT a snapshot copy. This is what makes `a = 1; cube(a); a = 2;`'s
    // second assignment visible to a statement evaluated after it.
    EvalContext withScope(const oscad::Scope* newScope) const;

    // Full copy of dyn/let_/dynExplicit (breaks aliasing with `this`);
    // dynPositions resets to a fresh empty map -- a direct, deliberate port
    // of the Python reference's child_ctx(), including this asymmetry
    // (dyn_positions does not copy alongside dyn). `newScope`/`newColor`/
    // `newChildrenNodes`/`newChildrenCallerCtx` default (null/nullopt) to
    // INHERITING from `this`, not resetting -- getting this backwards is a
    // real shipped-bug class in the reference (a children() forwarding
    // chain silently swallowed under `color(c) children();`-shaped user
    // modules) -- see docs/evaluator.md's child_ctx() entry.
    //
    // `newDyn`, when given (e.g. by call_args.hpp's resolveCallArgs(), for
    // a call site like `sphere(r=2, $fn=64)`), is used as-is for the new
    // context's `dyn` -- and, matching the reference's own asymmetric rule
    // exactly, `dynPositions` is then INHERITED from `this` rather than
    // reset fresh (the reset only happens when `dyn` is left null, i.e.
    // ordinary copy-and-derive). This asymmetry is a direct port, not a
    // guess -- see the reference's child_ctx() docstring.
    EvalContext childCtx(const oscad::Scope* newScope = nullptr,
                          std::optional<std::array<double, 4>> newColor = std::nullopt,
                          std::shared_ptr<const ChildrenNodeList> newChildrenNodes = nullptr,
                          const EvalContext* newChildrenCallerCtx = nullptr,
                          std::shared_ptr<DynMap> newDyn = nullptr) const;

    // Isolated call scope for entering a module/function body: copies
    // dyn/dynExplicit (so $-vars remain dynamically scoped into the call),
    // resets let_/dynPositions to fresh empty maps (the callee has its own
    // variable scope -- inheriting the caller's plain-named bindings would
    // trigger spurious double-assignment warnings). Unlike childCtx(),
    // childrenNodes/childrenCallerCtx do NOT inherit when omitted -- they
    // reset to empty/null, since entering a real call means childrenNodes
    // should be that call's own children (passed explicitly by the
    // caller), not whatever the calling context happened to have queued.
    EvalContext callCtx(const oscad::Scope* newScope = nullptr,
                         std::optional<std::array<double, 4>> newColor = std::nullopt,
                         std::shared_ptr<const ChildrenNodeList> newChildrenNodes = nullptr,
                         const EvalContext* newChildrenCallerCtx = nullptr) const;

    // let(...)/list-comprehension let() scoping: copies `let_` (the whole
    // point -- isolates the let-body's bindings from leaking back to the
    // parent once it returns).
    //
    // ponytail: the Python reference shares dyn/dynPositions/dynExplicit by
    // reference here, with copy-on-first-$-write for dyn/dynExplicit (so a
    // `let($fn=99)` override doesn't leak back out once the let() returns,
    // but avoids a map copy on every ordinary non-$ let()) -- dynPositions
    // is shared with no copy-on-write at all. This port always copies all
    // three immediately instead: avoiding an unordered_map copy isn't worth
    // the aliasing-bug risk in C++, where the copy itself is already cheap.
    // Behaviorally identical for dyn/dynExplicit (mutations still never
    // leak back to the parent). For dynPositions specifically this is a
    // deliberate, low-value divergence -- Python's reference-sharing lets a
    // double-assignment warning's position bookkeeping leak across a
    // let-block boundary in one specific edge case (a plain Assignment
    // *statement* inside a `let(...) { ... }` block); revisit only if a
    // real test surfaces a warning-format mismatch there.
    EvalContext letChildCtx() const;
};

} // namespace oscadeval
