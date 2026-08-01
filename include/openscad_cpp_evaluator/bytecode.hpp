#pragma once

#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast/ast_node.hpp"

#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace oscad {
class FunctionDeclaration;
class FunctionLiteral;
class ModuleDeclaration;
class ModularCall;
class ModularAssert;
class Expression;
} // namespace oscad

namespace oscadeval {

// Phase 1 bytecode: a stack-based instruction set for the "pure computation"
// subset of FunctionDeclaration bodies -- literals, identifiers, arithmetic/
// logical/bitwise/comparison operators, ternary, indexing/member access, and
// let(). See bytecode_compiler.cpp's own header comment for the exact
// compilability rule and why calls/closures/list-comprehensions/echo/assert
// are excluded from THIS phase (a real correctness hazard with the existing
// bindArgs/ctx.let_-based call machinery, not an arbitrary cut).
//
// UNARY_OP/BINARY_OP/INDEX/MEMBER dispatch to Evaluator::applyUnaryOp/
// applyBinaryOp/applyIndexAccess/applyMemberAccess (expr_eval.cpp) -- the
// exact same functions evalExpr's own AST-walking switch calls -- so neither
// path can drift from the other; see this project's CLAUDE.md for the
// "grep for ponytail:" convention this deliberately does NOT need, since
// there is no divergence here to flag.
enum class Op {
    PushConst,      // a = constant pool index
    PushBool,       // a = 0/1
    LoadLocal,      // a = slot index
    StoreLocal,     // a = slot index (pops)
    LoadDyn,        // a = name pool index ($-prefixed)
    StoreDyn,       // a = name pool index (pops; also marks dynExplicit)
    LoadFree,       // a = name pool index -- Evaluator::evalIdentifier fallback (warnIfUndef=true)
    // Same as LoadFree, but Evaluator::evalIdentifier's warnIfUndef=false --
    // only ever emitted for a PrimaryCall's own callee probe (bytecode_
    // compiler.cpp's PrimaryCall case), mirroring evalFunctionCall's own
    // `evalIdentifier(leftId->name, ..., /*warnIfUndef=*/false)` fallback
    // exactly: probing whether a bare identifier resolves to a callable
    // Closure value shouldn't itself warn "unknown variable" when it
    // doesn't -- Op::CallDynamic's own runtime handler already warns
    // "unknown function" once the probe comes back non-Closure, so using
    // plain LoadFree here doubled that up into two warnings for one
    // genuinely-unknown callee.
    LoadFreeNoWarn, // a = name pool index
    // a = slot index, b = name pool index (pops). Only emitted for a
    // compiled ASSIGNMENT BLOCK (Evaluator::tryCompileAssignmentBlock,
    // user_calls.cpp -- a run of sibling `name = expr;` statements sharing
    // one scope, exactly what Evaluator::evalChildren already isolates as
    // its own `assignments` sub-list before any non-assignment statement
    // runs). Stores into BOTH the slot (so a LATER sibling assignment in
    // the SAME batch can read this one back via a fast LoadLocal, mirroring
    // how LetOp's own bindings already work) AND ctx.let_ (so code OUTSIDE
    // this compiled batch -- a later non-assignment statement, a nested
    // block, a module call -- still sees it the ordinary way, since this
    // batch's own slots are invisible once it returns). Plain StoreLocal
    // alone would lose that outer visibility entirely; plain "store to
    // let_ only" would lose the fast intra-batch slot reads a batch of
    // assignments exists to buy back in the first place.
    StoreLocalAndLet,
    Range,          // pops step, pops end, pops start (pushed in that order); pushes applyRange(start, end, step)
    Index,          // pops idx, pops obj, pushes applyIndexAccess(obj, idx)
    Member,         // a = name pool index (member name); pops obj, pushes applyMemberAccess(obj, name)
    UnaryOp,        // a = (int)oscad::NodeKind; pops 1, pushes applyUnaryOp
    BinaryOp,       // a = (int)oscad::NodeKind; pops 2, pushes applyBinaryOp
    Jump,           // a = target pc
    JumpIfFalse,    // pops; if !truthy(v), pc = a
    JumpIfTrue,     // pops; if truthy(v), pc = a
    OpenLocalScope, // a = slotStart, b = slotCount -- resets slots[a, a+b) to undef
    BuildList,      // a = element count; pops that many values (in reverse push order), pushes a fresh ValueList
    CallFn,         // a = index into CompiledChunk::callSites, b = argument count (pops that many, in push order)

    // -- List-comprehension clause support (Phase 3) ---------------------
    // A single accumulator (a native vector<Value>, not a stack Value) is
    // open for the whole duration of one `[...]` literal that contains any
    // real comprehension clause -- every clause's own contribution (however
    // deeply chained/nested) targets whichever accumulator is topmost,
    // mirroring the interpreter's own single `out` reference threaded
    // through evalListElement/evalListCompBody's recursion. See
    // bytecode_compiler.cpp's compileListElement for the exact mirroring.
    AccumOpen,       // pushes a new empty accumulator
    AccumAppendOne,  // pops one Value (from the value stack), push_back onto the topmost accumulator
    AccumAppendEach, // pops one Value (from the value stack), "each"-flattens it into the topmost accumulator
                     // (see appendEachInto, value.hpp)
    AccumClose,      // pops the topmost accumulator, pushes it as a fresh ValueList (onto the value stack)
    // `each` wrapping a real clause (as opposed to a plain expression, see
    // AccumAppendEach): pops the topmost accumulator's own RAW items
    // (without ever wrapping them in a ValueList) and each-flattens EVERY
    // one of them, individually, into the accumulator that's now on top --
    // mirrors `for (const Value& item : evalListCompBody(inner, ctx))
    // appendEachInto(out, item)` exactly (a per-item flatten, not "flatten
    // the whole inner result as one unit", which AccumClose+AccumAppendEach
    // would do instead and get wrong for a clause whose own items are
    // themselves lists).
    AccumMergeEach,

    // ListCompFor's per-assignment iteration -- materialize once (a
    // dimension's own RHS expression is evaluated exactly once, matching
    // the interpreter's own upfront `pairs.push_back(...)` loop), then
    // reset+iterate however many times that dimension is (re-)entered
    // (once per outer-dimension iteration for a nested `for`). `a` = an
    // iterList id (see CompiledChunk::numIterLists), unique per assignment
    // across the whole chunk.
    IterMaterialize, // pops one Value (the assignment's RHS), expandIterable()s it into iterLists[a], resets its index
    IterReset,       // resets iterLists[a]'s index to 0 WITHOUT re-materializing (a re-entry, not the first entry)
    IterNext,        // a = loop-variable slot, b = iterList id, c = jump target for exhaustion: if iterLists[b] has a
                     // next value (at its current index), write it to slots[a], advance the index, fall through;
                     // else jump to c (index is left as-is; the next IterReset for this id will restart it)

    // ListCompCFor's runaway-loop guard, mirroring evalListElement's own
    // 1,000,000-iteration safety limit exactly (see its own doc comment
    // there for why the limit exists at all). a = a slot reserved purely
    // as this loop's own iteration counter (never a real local, never
    // resolvable by name), b = the limit. `node` (not `pos`) is used here
    // since Evaluator::error() needs a real ASTNode for its TRACE walk,
    // not just a Position.
    CheckIterLimit,

    // -- Closures/upvalues (Phase 2) --------------------------------------
    // a = index into CompiledChunk::upvalues. At runtime, searches the live
    // call stack (Evaluator::findUpvalue) for a still-active call of the
    // upvalue's own target declaration and reads that call's own slot --
    // undef if that call already returned. Never needs a STORE counterpart:
    // a captured variable is always a parameter/let-binding of some
    // ENCLOSING, already-executing call, and nothing this phase compiles
    // ever writes back into an enclosing call's own slot (let/ListCompLet
    // assignments always target a freshly-opened scope of their OWN, never
    // a parent's).
    //
    // NOTE: as of Op::MakeClosure (below), no CompiledChunk that's ever
    // actually registered (chunkCache_/literalChunkCache_) has a non-empty
    // `upvalues` list any more -- a FunctionLiteral with any real capture
    // need (direct or transitively bubbled up from a nested literal, see
    // bytecode_compiler.cpp's FunctionLiteral case) is now handled via
    // Op::MakeClosure instead, and always runs interpreted when invoked
    // (Evaluator::lookupCompiledLiteralChunk finds no entry for it). This
    // opcode and Evaluator::findUpvalue/CallStackFrame::upvalueParent are
    // therefore unreachable dead code today, left in place rather than
    // removed in the same change that made them unreachable -- deleting
    // them is a distinct, lower-risk cleanup, not bundled in here.
    LoadUpvalue,

    // Escaping-closure support. a = index into CompiledChunk::closureSites.
    // Builds a FRESH captured environment every time this instruction
    // actually runs (never a compile-time constant -- a loop creating one
    // closure per iteration must capture THAT iteration's own values, not
    // share one snapshot) by reading each of the site's own captures
    // straight out of the CURRENTLY EXECUTING frame's own `slots` (see
    // ClosureSite's own doc comment for why every capture, however deeply
    // the literal was originally nested, always resolves against the one
    // chunk actually running Op::MakeClosure), and constructs
    // Closure{node, capturedTrail} -- reusing Closure::capturedLet (value.hpp)
    // and callCtxFor's existing capturedLet-rooting (user_calls.cpp)
    // UNCHANGED: invoking this closure later works through the exact same
    // machinery any interpreter-created escaping closure always has, no new
    // invocation-side code needed at all (its own body may itself run
    // compiled too now, see Evaluator::lookupCompiledLiteralChunk). Replaces
    // Op::PushConst's frozen `Closure{&n, nullptr}` specifically for a
    // literal whose (transitively merged) capture set is non-empty; a
    // literal that closes over nothing at all -- even transitively -- still
    // takes the cheaper PushConst path, unchanged.
    MakeClosure,

    // Letrec support for TWO OR MORE sibling `let`-bound closures that
    // reference each other by name (fnliterals.scad-style mutual
    // recursion, e.g. isEven/isOdd each calling the other) -- see the
    // LetOp compile case's own doc comment (bytecode_compiler.cpp) for why
    // a forward reference (an EARLIER closure referencing a LATER sibling
    // that doesn't exist yet at the point the earlier one is constructed)
    // can't be resolved by Op::MakeClosure itself the way a self-reference
    // is: there's nothing to read OR self-patch into, since the later
    // sibling hasn't even been created yet. a = consumer's own local slot
    // (already holds a MakeClosure-created closure, via a preceding
    // Op::StoreLocal), b = name pool index (the captured name, inside the
    // consumer's own capturedTrail, that's finally ready), c = the local
    // slot the just-constructed sibling was JUST stored into (also via a
    // preceding Op::StoreLocal) -- patches slots[a]'s own capturedTrail
    // with slots[c], keyed by name b.
    PatchClosureCapture,

    // A call whose callee isn't statically resolvable to a builtin or a
    // named FunctionDeclaration (see CallFn) -- the callee expression is
    // compiled to push its own Value first, THEN each argument (so at
    // runtime: pop args, pop callee, in that order). a = index into
    // CompiledChunk::callSites (its `calleeName` is only ever used for the
    // "unknown function" warning here, empty unless the callee was a
    // plain identifier -- matches evalFunctionCall's own behavior), b =
    // argument count. If the popped callee doesn't hold a FunctionLiteral
    // pointer, warns (if calleeName is non-empty) and pushes undef,
    // exactly like the interpreter's own fallback.
    CallDynamic,

    // -- Tail-call optimization (Phase B) ---------------------------------
    // Same operands/semantics as CallFn/CallDynamic, emitted instead of
    // them ONLY for a PrimaryCall the compiler determined is in tail
    // position (bytecode_compiler.cpp's `tail` param threaded through
    // compileExpr) AND resolves to a non-builtin callee. Whether a given
    // hop is actually eligible to trampoline (isolated, i.e. not
    // closure-nested inside the currently-executing call -- see
    // Evaluator::isolatedCallCtxFor's own doc comment) is a RUNTIME fact
    // the compiler can't know, so runChunk's handler for these two
    // opcodes checks it dynamically: eligible -> fills the caller-supplied
    // TailCallRequest and returns immediately (short-circuiting runChunk's
    // own pc loop); not eligible -> falls back to doing exactly what
    // CallFn/CallDynamic already do (a real recursive call), no different
    // from today.
    CallFnTail,
    CallDynamicTail,

    // -- echo()/assert() expression forms, import()/object() calls -------
    // a = index into CompiledChunk::echoSites, b = argument count (pops
    // that many, in push order, same convention as CallFn). Formats+emits
    // via Evaluator::emitEcho, then falls straight through to whatever
    // follows in the instruction stream (the body -- no jump needed,
    // exactly like LetOp's own straight-line "assignments then body"
    // shape; echo() always evaluates every argument unconditionally, so
    // there's no branch to emit here).
    Echo,

    // Only ever emitted for a non-empty-argument AssertOp (see
    // bytecode_compiler.cpp's own AssertOp case -- a zero-argument
    // assert() is unconditionally true and compiles straight through with
    // no check at all). Reached only on the condition-false path (guarded
    // by a JumpIfTrue the compiler emits around it, mirroring LogicalOrOp's
    // own jump-then-patch shape) -- always throws via Evaluator::error(),
    // i.e. [[noreturn]] at runtime. a = 1 if a message Value was compiled
    // and pushed (pop it), else 0. b = constant-pool index of condText
    // (the condition expression's own source text, interned ONCE at
    // compile time via its toString() -- cheaper than the interpreter,
    // which recomputes this on every failing call). pos/node: same
    // Evaluator::error() TRACE-walk need CheckIterLimit already has.
    AssertFail,

    // -- Module-body compilation (Stage 2) --------------------------------
    // A call to a user module resolved to a specific ModuleDeclaration AT
    // COMPILE TIME (see CompiledChunk::ModuleCallSite) -- the module-side
    // analog of Op::CallFn. a = index into CompiledChunk::moduleCallSites.
    // Arguments are NOT pre-pushed onto the operand stack the way CallFn's
    // are (bindArgs runs directly against site.callNode->arguments at
    // runtime instead) -- module-call argument evaluation was never the
    // recursion-safety target here, so there's no need to teach the
    // compiler a separate "push each arg" step just for this. Produces no
    // stack Value at all: its effect is entirely the side effect of
    // splicing (or wrapping into a synthetic "union") whatever CSGNodes
    // the callee's body produces into the CURRENT treeStack_ frame -- see
    // Evaluator::spliceModuleChildren's own doc comment (bytecode_vm.cpp)
    // for why this is always the splice branch, never the "wrap as a
    // builtin-tagged CSGNode" one evalModularCall's own general form also
    // has (Op::CallModule only ever targets a resolved USER module, for
    // which splice is unconditional).
    CallModule,

    // -- Builtin-wrap compilation (closes the "NativeStatement gap" for ----
    // -- translate/rotate/scale/mirror/multmatrix/resize/color/#/%/!) ------
    // A builtin-with-children statement that isn't a "call" the way
    // Op::CallModule's target is (no callStack_/profiling participation,
    // no named scope with upvalue semantics -- see CompiledChunk::
    // BuiltinWrapSite's own doc comment for the full "why this doesn't
    // need vmCallStack_/pushBracketedModuleFrame-style bracketing"
    // reasoning) -- so unlike Op::CallModule, its children compile INLINE
    // into the SAME instruction stream (via compileStatementList, exactly
    // like Op::OpenLetScope's own body does for ModularLet) rather than
    // becoming a separate chunk. Exists because falling through to
    // Op::NativeStatement here was a REAL, proven Windows/MSVC crash risk
    // for a recursive translate()/multmatrix()-wrapped module chain (see
    // this project's own session notes: BOSL2's attachable() machinery
    // needs native reentry depth ~55, past the empirically-proven-unsafe
    // Windows ceiling for that reentry mechanism -- no depth-guard tuning
    // could satisfy both "the real script works" and "Windows doesn't
    // crash", only eliminating the native reentry itself could).
    //
    // a = index into CompiledChunk::builtinWrapSites. Runtime handler:
    // captures ev.randsCallCount() BEFORE any argument resolution
    // (mirrors Evaluator::buildTreeNode's own ordering exactly -- doing
    // this AFTER would silently drop uncacheable/ManifoldCache taint
    // tracking for a rands() call embedded in the wrapper's own
    // arguments), computes this site's own params (and, for Transform/
    // Color kinds, a possibly-$-scoped child EvalContext, pushed onto
    // f.ctxChain unconditionally -- Modifier kind needs neither params
    // nor a ctx push), pushes a fresh ev.treeStack_ frame, and stashes
    // {params, randsBefore, siteIdx} onto VmFrame::builtinWrapStack (a
    // real per-frame LIFO stack, not a single slot: Push/PopBuiltinWrap
    // pairs can nest or sequence within one frame's own instruction
    // stream, e.g. `translate(a) translate(b) recur();`). The compiler
    // always emits a plain Op::CheckDebugStatement immediately before
    // this (see emitBuiltinWrap, bytecode_compiler.cpp), mirroring
    // ModularEcho/ModularAssert's own pattern -- NOT skipped the way
    // Op::CallModule's own call site skips one: this is a genuine
    // statement doing real work here, unlike a module call (which
    // transfers control to a declaration whose OWN body statements each
    // get their own check instead).
    PushBuiltinWrap,

    // Closes the matching Op::PushBuiltinWrap bracket: pops
    // VmFrame::builtinWrapStack's own top entry FIRST (before anything
    // that can itself throw, e.g. Evaluator::setTreeDepthOrThrow below --
    // so the exception-teardown path's own pending count is already
    // correct if THIS throws), pops the ctx push Push made (Transform/
    // Color kinds only), pops ev.treeStack_ to retrieve the children this
    // bracket's own body produced, and builds the tagged CSGNode exactly
    // like Evaluator::buildTreeNode's own post-resolveBody() half does
    // (kind/node/isBuiltin=true/children/params/uncacheable/
    // setTreeDepthOrThrow), pushing it onto the new top of treeStack_. a
    // = index into CompiledChunk::builtinWrapSites (same site Push used).
    //
    // teardownVmCallStackDownTo's own exception path (bytecode_vm.cpp)
    // pops frame->builtinWrapStack.size() additional treeStack_ entries
    // per torn-down frame, on top of its existing ownsModuleSplice-gated
    // pop -- required because an exception can leave N of these brackets
    // open in one frame, unlike the "at most one Op::CallModule splice
    // per frame" shape that gated pop was originally sized for. See
    // releaseVmFrame's own doc comment (user_calls.cpp) for a real,
    // previously-ASan-caught bug from exactly this class of mismatched-
    // bookkeeping-vs-treeStack_-depth mistake (a 1-bit version of the
    // same seam, now a counter).
    PopBuiltinWrap,

    // -- CSG-wrap compilation (closes the LAST native-reentry source in ----
    // -- the original NativeStatement gap: union()/difference()/           -
    // -- intersection()/intersection_for) -----------------------------------
    // union()/difference()/intersection() weren't covered by the original
    // Op::PushBuiltinWrap (see that op's own doc comment) because they need
    // bespoke bookkeeping PushBuiltinWrap's single all-children bracket
    // doesn't do: Evaluator's own resolveCsg (booleans.cpp) evaluates each
    // TOP-LEVEL child statement of the block SEPARATELY and records how
    // many CSGNodes it individually contributed ("group_sizes") -- e.g.
    // `difference(){ A; B; C; }` = A - (B u C), preserving A's own group
    // even when A itself expands to more than one body (an attachable()-
    // style call returning parent+children as one operand). A flat
    // all-children bracket the way Transform/Color/Modifier use would lose
    // that grouping entirely. This op pair (PushCsgWrap/PopCsgWrap) plus
    // Op::CsgGroupStart/CsgGroupEnd (below) replicate resolveCsg's exact
    // two-pass shape (all assignments first, THEN one evalChildren-per-
    // geometry-statement) as inline compiled bytecode instead, exactly as
    // PushBuiltinWrap already did for translate/rotate/scale/mirror/
    // multmatrix/resize/color/#/%/! -- same rationale (a recursive
    // union()/difference()/intersection()-wrapped module chain was still a
    // real Windows native-reentry depth risk, just never independently
    // fixed when PushBuiltinWrap's own set was).
    //
    // a = index into CompiledChunk::csgWrapSites. Runtime handler: captures
    // ev.randsCallCount() BEFORE argument resolution (same rands-in-args
    // taint reasoning as PushBuiltinWrap); for a `hasArgs` site (union/
    // difference/intersection) resolves the (rare, $-only) arguments via
    // resolveCallArgs exactly like resolveCsg itself does (discarding the
    // positional/named result -- these take no real parameters -- keeping
    // only the possibly-$-scoped child ctx) and pushes that ctx onto
    // f.ctxChain unconditionally (mirrors Transform/Color's own
    // unconditional push -- a bare `union() {...}` with no `$fn=...`
    // override still pushes a ctx that's merely a copy, cheap and uniform
    // rather than a special-cased branch); intersection_for (`hasArgs`
    // false -- it isn't a call, has no `.arguments`) skips both entirely,
    // matching resolveIntersectionFor's own use of its caller's ctx
    // unchanged (its per-ITERATION child ctxs are a completely separate
    // concern, handled by the compiled cartesian loop's own Op::
    // ForIterNext, not by this bracket). Either way: pushes a fresh
    // ev.treeStack_ frame (every child statement's/iteration's own
    // CSGNode(s) land flat in this ONE frame -- mirrors Evaluator::
    // buildTreeNode/evalModularCall's own single treeStack_.emplace_back()
    // around the whole call, not one per group), and stashes {op,
    // randsBefore, siteIdx, empty groupSizes} onto VmFrame::csgWrapStack (a
    // real per-frame LIFO, same reasoning as builtinWrapStack: nested/
    // sequenced CSG wraps within one frame's own instruction stream, e.g.
    // `union() { difference() {...} }`). The compiler always emits a plain
    // Op::CheckDebugStatement immediately before this (see emitCsgWrap/
    // compileIntersectionForLoop, bytecode_compiler.cpp), mirroring
    // emitBuiltinWrap's own pattern -- this is a genuine statement doing
    // real work here, not a call transferring control to a declaration.
    PushCsgWrap,

    // Opens one "group" within an already-open Op::PushCsgWrap bracket --
    // emitted immediately before each top-level GEOMETRY child statement's
    // own inline-compiled bytecode (compileStatementList of exactly that
    // one statement). Records ev.treeStack_.back().size() into
    // csgWrapStack.back().groupStartSize -- always operates on the
    // TOPMOST (innermost still-open) csgWrapStack entry, matching
    // PopBuiltinWrap's own back()-is-always-mine LIFO discipline, so no
    // operand is needed. Assignments among the block's children are
    // compiled separately, BEFORE any CsgGroupStart/End pair at all (see
    // emitCsgWrap) -- mirrors resolveCsg's own two-pass split exactly
    // (Evaluator::evalChildren(assignNodes, effCtx) always runs to
    // completion before the per-geoNode loop starts).
    CsgGroupStart,

    // Closes the matching Op::CsgGroupStart: computes
    // ev.treeStack_.back().size() - csgWrapStack.back().groupStartSize
    // (how many CSGNodes THIS one top-level statement just contributed,
    // however many that turned out to be -- 0 for a statement whose own
    // evaluation spliced nothing, e.g. a no-op unknown-module warning; >1
    // for an attachable()-style multi-body operand) and appends it, as a
    // Value, onto csgWrapStack.back().groupSizes -- exactly one entry per
    // top-level geometry statement, in source order, mirroring resolveCsg's
    // own `groupSizes.push_back(Value{...})` loop.
    CsgGroupEnd,

    // Closes the matching Op::PushCsgWrap bracket: pops
    // VmFrame::csgWrapStack's own top entry FIRST (before anything that can
    // itself throw, e.g. setTreeDepthOrThrow below -- same "exception-
    // teardown's own pending count must already be right" reasoning as
    // PopBuiltinWrap), pops the ctx Push pushed (only when `hasArgs`),
    // pops ev.treeStack_ to retrieve every group's own CSGNode(s) (flat,
    // exactly like resolveCsg's own `children` result -- group boundaries
    // live only in group_sizes, never in the CSGNode list's own shape), and
    // builds the tagged CSGNode exactly like Evaluator::buildTreeNode's own
    // post-resolveBody() half does, with params = {"group_sizes":
    // ValueList(pending.groupSizes)} plus, only when `includeOpParam`,
    // "op": site.op -- byte-for-byte what native resolveCsg returns for
    // union/difference/intersection (generateCsg needs "op" to
    // disambiguate the ONE function it shares across all 3) and what
    // native resolveIntersectionFor returns (no "op" key at all --
    // generateIntersectionFor is its own dedicated, separately-registered
    // function, never needs one) -- pushing the result onto the new top of
    // treeStack_. a = index into CompiledChunk::csgWrapSites (same site
    // Push used).
    //
    // teardownVmCallStackDownTo's own exception path (bytecode_vm.cpp) pops
    // frame->csgWrapStack.size() additional treeStack_ entries per
    // torn-down frame, alongside its existing builtinWrapStack/
    // ownsModuleSplice accounting -- same reasoning as PopBuiltinWrap's own
    // doc comment: an exception can leave N of these brackets open in one
    // frame.
    PopCsgWrap,

    // A `children()` / `children(N)` statement -- the runtime-varying
    // sibling of Op::CallModule/Op::PushBuiltinWrap, closing the LAST
    // dominant native-reentry source (BOSL2's attachable() calls
    // children() at nearly every wrapper level; measured 85 of 93 native
    // reentries in a real script). Unlike PushBuiltinWrap's constructs,
    // the forwarded children aren't known at compile time (they're the
    // CALLER's own call-site statements, carried on ctx.childrenNodes/
    // childrenCallerCtx -- see Evaluator::buildModuleChildCtx), so they
    // can't compile inline; instead the handler resolves the list at
    // RUNTIME, looks up/compiles its chunk (the same
    // childrenListChunkCache_ tryRunCompiledChildren uses, via
    // lookupOrCompileChildrenListChunk -- gated on useBytecodeVm() &&
    // inResolvePass_, the cache is pass-scoped), and pushes it onto
    // vmCallStack_ directly, mirroring Op::CallModule's own zero-native-
    // call push -- but with a THIRD frame shape: splice-owning like a
    // module frame (ownsModuleSplice=true, mirroring evalModularCall's
    // own unconditional splice branch for "children"), yet bracketless
    // like a bare frame (children() never gets a callStack_/profiling
    // entry natively either -- only enterUserCall pushes those, and
    // resolveChildren/builtinChildren never call it). driveVm's existing
    // completion branch and teardownVmCallStackDownTo both already
    // handle that combination (their bracket and splice concerns are
    // independent).
    //
    // Handler ordering is load-bearing: checkDebug fires in-handler
    // against the scope-wrapped ctx (byte-for-byte what Op::
    // NativeStatement does -- NOT via an emitted Op::CheckDebugStatement,
    // whose handler passes the un-wrapped ctx); randsBefore is captured
    // BEFORE argument resolution (rands-in-args taint, same lesson
    // PushBuiltinWrap already encodes); and treeStack_ is pushed LAST,
    // immediately before the frame push / native fallback (an early-out
    // or a throw during arg resolution then has nothing to clean up --
    // the native path's own catch{pop;throw} has no equivalent here, so
    // the reorder IS the exception-safety mechanism). The not-eligible
    // fallback reuses the ALREADY-resolved args (never evalStatement,
    // which would re-resolve them: double rands(), double side effects)
    // by inlining evalModularCall's own children branch around a native
    // evalChildren call. a = index into CompiledChunk::nativeStatements
    // (the ModularCall node -- arguments, error position, splice node;
    // no separate site table needed).
    CallChildren,

    // A single "native passthrough" statement -- every builtin module call
    // not covered by Op::PushBuiltinWrap/Op::PushCsgWrap (see those ops'
    // own doc comments for exactly which builtins ARE covered -- cube/
    // sphere/hull/linear_extrude/etc., the ones that never wrap a
    // recursive call in idiomatic OpenSCAD), the `*` modifier's own no-op
    // case, or a user-module call that didn't resolve at compile time
    // (shadowed, forward-declared, or otherwise not statically known) --
    // anything compileStatementList doesn't give its own real bytecode.
    // (Assignment/ModularEcho/ModularAssert/ModularLet used to fall here
    // too; they now have their own real bytecode -- Op::StoreModuleVar/
    // Op::Echo/Op::AssertStatement/Op::OpenLetScope+StoreLetVar -- purely
    // a throughput change, since none of these were ever the recursion-
    // depth risk this compiler targets. `#`/`%`/`!` modifiers and
    // translate/rotate/scale/mirror/multmatrix/resize/color used to fall
    // here too, for the SAME throughput reasoning -- WRONG in that one
    // specific case: a recursive module call wrapped in one of these is
    // exactly the pattern that made this op's own native reentry a
    // genuine Windows crash risk in practice, not just a missed
    // optimization. See Op::PushBuiltinWrap's own doc comment for the
    // real story and the fix. children() fell here too, and was the LAST
    // and largest such reentry source once PushBuiltinWrap's own set was
    // covered -- it now has Op::CallChildren, above. union()/difference()/
    // intersection() fell here too, and were the last remaining REAL
    // native-reentry risk among ModularCall-shaped statements (bespoke
    // group_sizes bookkeeping meant they couldn't just reuse
    // PushBuiltinWrap's own bracket) -- they now have Op::PushCsgWrap,
    // above. intersection_for -- NOT a ModularCall at all, its own
    // NodeKind -- fell here too and was the true LAST native-reentry gap;
    // it now shares Op::PushCsgWrap's own bracket (CsgWrapSite::hasArgs/
    // includeOpParam both false for it) wrapped around a compiled
    // cartesian-product loop reusing Op::ForIterNext/ForIterEnd/
    // NativeIterMaterialize verbatim (see compileIntersectionForLoop,
    // bytecode_compiler.cpp) -- one Op::CsgGroupStart/CsgGroupEnd pair per
    // full iteration instead of per source statement.)
    // a = index into CompiledChunk::nativeStatements. Runtime just does
    // what Evaluator::evalChildren's own per-statement loop already does
    // for one node: derive childCtx via ctx.withScope(...), checkDebug,
    // evalStatement. These are still "leaf-shaped" for what's left here
    // now that Op::PushBuiltinWrap/Op::PushCsgWrap have peeled off every
    // proven-risky construct: real recursion safety for a recursive module
    // chain is covered by CallModule/PushBuiltinWrap/PushCsgWrap/
    // ForIterNext/the Jump-based if/for control flow, not by how many of
    // THESE sit alongside them in the same body.
    NativeStatement,

    // If/if-else's own condition, evaluated NATIVELY (Evaluator::
    // evalExprMaybeCompiled -- gets the SAME expression-level compilation
    // a bare statement-context expression already would, just not folded
    // into THIS chunk's own instruction stream) rather than compiled
    // inline -- module-body compilation's job is the STATEMENT-level
    // control flow (so a recursive module call nested inside doesn't hide
    // behind a native evalStatement call), not re-deriving expression
    // compilation tryCompileStatementExpr already does. a = index into
    // CompiledChunk::nativeExprs, b = jump target on false (mirrors
    // JumpIfFalse's own semantics, just with a native condition source
    // instead of a popped stack Value).
    NativeCondJumpIfFalse,

    // The "entering this branch/iteration" expr-level checkDebug marker
    // ModularIf/ModularIfElse/ModularFor's own interpreted forms each fire
    // once (see evalStatement's ModularIf/ModularIfElse cases and evalFor,
    // stmt_eval.cpp) -- a = index into CompiledChunk::nativeStatements
    // (reusing that same table; the entry here is whichever ASTNode the
    // interpreter would have passed to checkDebug -- the branch/body's
    // own first statement, or the ModularIf/For node itself when that
    // branch/body is empty).
    NativeCheckDebugExprLevel,

    // One ModularFor assignment's own RHS range/list expression, evaluated
    // NATIVELY (same reasoning as NativeCondJumpIfFalse) and materialized
    // into an IterList exactly like the existing (list-comprehension)
    // Op::IterMaterialize -- a = index into CompiledChunk::nativeExprs, b
    // = iterList id.
    NativeIterMaterialize,

    // Statement-for's own per-iteration advance -- the module-body analog
    // of Op::IterNext, but binding into ctx.let_ (a real, dynamically-
    // discoverable statement-scope variable, exactly like the
    // interpreter's own per-iteration childCtx.let_->set()) instead of a
    // slot, and -- critically -- pushing a FRESH child EvalContext onto
    // f.ctxChain for the upcoming iteration rather than mutating the
    // current one in place: evalFor derives a brand-new childCtx per
    // iteration (see stmt_eval.cpp), so an ordinary local assignment made
    // inside the loop body doesn't trip evalAssignment's own "was
    // assigned on line N but overwritten" warning on iteration 2 onward,
    // and so a value bound in one iteration never leaks into the next.
    // a = name-pool index (the loop variable's own name), b = iterList id,
    // c = jump target on exhaustion (taken WITHOUT pushing a new ctx --
    // mirrors the cartesian loop simply not entering the body once that
    // dimension is exhausted). `node` = the Assignment AST node, for the
    // matching per-iteration checkDebug call (mirrors evalFor's own
    // `checkDebug(*node.assignments[depth], childCtx)`). Always paired
    // with a matching Op::ForIterEnd at the bottom of the SAME dimension's
    // loop body.
    ForIterNext,

    // Pops the ctx Op::ForIterNext just pushed for the iteration that's
    // ending (f.ctxChain.pop_back()) and jumps back to that same
    // ForIterNext instruction to attempt the next one. a = jump target
    // (the matching ForIterNext's own pc).
    ForIterEnd,

    // -- Module-body leaf-statement compilation (throughput, not a --------
    // -- recursion-safety concern -- see NativeStatement's own doc comment
    // for why THAT stayed native for these same node kinds in Stage 2;
    // this closes the "every sub-expression is a separate nested driveVm
    // call via evalExprMaybeCompiled" gap instead by inlining them into
    // the enclosing chunk's own instruction stream) --------------------
    //
    // A module-body statement's own top-level checkDebug -- the compiled
    // analog of what evalChildren's native per-statement loop / Op::
    // NativeStatement's own handler already does for the SAME node
    // (`ev.checkDebug(*stmt, ctx)`, exprLevel=false, forced=false) -- kept
    // as a real op (not simply omitted) so a compiled statement's debug
    // behavior is identical to its native form, not just "usually doesn't
    // matter because fast-continue only compiles breakpoint-free chunks."
    // a = index into CompiledChunk::nativeStatements (reusing that table).
    CheckDebugStatement,

    // Assignment's own compiled form -- pops one Value (already evaluated
    // via an ordinary inline compileExpr, NOT native evalExprMaybeCompiled)
    // and replicates Evaluator::evalAssignment exactly: a `$`-prefixed name
    // goes to ctx.dyn (+ dynExplicit), everything else checks
    // ctx.dynPositions for the "assigned but overwritten" warning, then
    // writes ctx.let_ and records this position. Never slot-addressed --
    // module-level/top-level assignments are always ctx-visible, dynamic-
    // scope writes, exactly like today. a = name pool index, pos = the
    // Assignment node's own position (for both the overwritten warning and
    // the position recorded into dynPositions).
    StoreModuleVar,

    // ModularEcho/ModularAssert's own statement form share Op::Echo/a
    // dedicated AssertStatement op (below) for the "pop N already-compiled
    // argument values" mechanics -- see those ops' own doc comments
    // (Op::Echo, above; Op::AssertStatement, below) for why the
    // EXPRESSION-form opcodes are directly reusable (statement-shaped
    // already: no value pushed, straight fall-through) while assert's
    // needs its own op (named-argument support + eager-not-lazy message
    // evaluation, both genuinely different from AssertOp's own contract).

    // Isolates ONE inline-compiled sub-expression's own `$`-writes from the
    // rest of this chunk's shared, long-lived ctx -- pushes ctx.
    // letChildCtx() (reads still see through to the parent level; only
    // WRITES are isolated) onto f.ctxChain, mirroring exactly what
    // runCompiledExprChunk's own wrapper already does for a native call
    // into a compiled EXPRESSION chunk (evalExprMaybeCompiled). Needed
    // because a module-body statement's own inline-compiled sub-
    // expression (Assignment's RHS, one echo()/assert() argument) has NO
    // such wrapper of its own the way a genuinely separate chunk call
    // does -- without this, `v1 = let($fn=55) f(); v2 = f();` leaks $fn=55
    // into v2 too (caught for real by UserFunction.
    // DollarVarLetAsAssignmentRhsDoesNotLeak once Assignment got its own
    // inline compiled form). Always emitted, unconditionally, around
    // EVERY such sub-expression -- not just ones statically known to
    // contain a `let($...)` -- for the same reason runCompiledExprChunk's
    // own wrapper is unconditional: a nested call/closure could still
    // reach one indirectly, and the cost when nothing $-scoped is present
    // is just one cheap trail-level open, not a real expression re-walk.
    OpenExprScope,
    // Pops the ctx OpenExprScope just pushed (f.ctxChain.pop_back()) --
    // always paired, immediately after the sub-expression's own compiled
    // code finishes (its RESULT, if any, is already on f.stack by then;
    // this only tears down the ctx, never touches the value stack). Also
    // reused to close Op::OpenLetScope's own child ctx (below) -- both are
    // just "pop the current ctx level", identical either way.
    CloseExprScope,

    // ModularLet's own (statement-form) child scope -- mirrors
    // Evaluator::evalLetBlock's own `ctx.childCtx(nullptr, std::nullopt,
    // ctx.childrenNodes, ctx.childrenCallerCtx)` call exactly, pushed onto
    // f.ctxChain (Op::CloseExprScope, above, pops it back off once the
    // let-block's own body finishes compiling/running). Every assignment's
    // own RHS is compiled+evaluated BEFORE this runs (while the PARENT ctx
    // is still current -- see ModularLet's own compileOneStatement case,
    // bytecode_compiler.cpp, for why: the statement form's RHS must never
    // see an EARLIER sibling's own write in the SAME let-block, unlike the
    // let-EXPRESSION form's sequential visibility), each left on f.stack;
    // only the WRITES (Op::StoreLetVar, below) happen after this runs.
    OpenLetScope,

    // Op::StoreLetVar -- pops one value and writes it into the CURRENT ctx
    // (by then, the child Op::OpenLetScope just pushed) via the same
    // `$`-prefix branch Op::StoreModuleVar uses, but WITHOUT its
    // dynPositions "assigned but overwritten" bookkeeping -- evalLetBlock's
    // own writes never do that check either (a let-block's own child scope
    // is always fresh, so "reassigning the same name in the same scope"
    // can never happen the way a module-body Assignment's own can). a =
    // name pool index, pos = the Assignment node's own position (unused by
    // the handler itself, kept for consistency/future debugging only).
    StoreLetVar,

    // a = index into CompiledChunk::assertSites. Pops site.argCount values
    // (all of this statement's own arguments, already compiled+pushed in
    // source order via compileExpr -- EVERY one, not just condition/
    // message, mirrors evalAssertStatement's own eager resolveArgs() over
    // AssertOp's lazy-message contract) and replicates
    // Evaluator::evalAssertStatement exactly: named "condition"/"message"
    // win over positional 0/1 (resolved at COMPILE time into
    // AssertSite::conditionArgIndex/messageArgIndex -- the argument SHAPE
    // is static, only values vary per call), throws via Evaluator::error()
    // on failure, otherwise runs any chained node.children natively (rare;
    // not worth its own compiled path -- see AssertSite's own doc comment).
    AssertStatement,
};

struct Instruction {
    Op op;
    int a = 0;
    int b = 0;
    // Only meaningful for UnaryOp/BinaryOp (the cases that can warn()).
    const oscad::Position* pos = nullptr;
    // Only meaningful for CheckIterLimit (needs a real ASTNode, not just a
    // Position, for Evaluator::error()'s TRACE walk).
    const oscad::ASTNode* node = nullptr;
    // Only meaningful for IterNext (needs a 3rd int alongside a/b) --
    // declared last so every existing `{op, a, b, pos}` 4-positional
    // aggregate-init call site stays valid unchanged (c/node/pos all
    // default when trailing members are omitted).
    int c = 0;
};

// A compiled FunctionDeclaration. `params` are always plain-named (a
// $-prefixed parameter bails compilation entirely -- see
// bytecode_compiler.cpp) and each owns exactly one local slot;
// `defaultCode[i]` (parallel to `params`) is that parameter's own default-
// value expression, compiled with an EMPTY local-name scope (see
// bytecode_compiler.cpp's compileDefault) so any reference to ANY parameter
// name inside it -- including its own -- falls through to LOAD_FREE and
// resolves to undef via the ParameterDeclaration fallback, exactly mirroring
// applyDefaults()'s existing sibling-isolated defaultCtx; empty if that
// parameter has no default. `bodyCode` is the function's own expression body.
struct CompiledChunk {
    struct Param {
        std::string name;
        int slot = 0; // unused when isDyn is true -- $-params are never slot-addressed
        // $-prefixed parameter: binds through ctx.dyn (dynamically scoped)
        // instead of a slot -- see compileFunctionLike's own doc comment,
        // bytecode_compiler.cpp, and bindCompiledArgs', bytecode_vm.cpp.
        bool isDyn = false;
    };

    // A call site statically resolved at compile time (see
    // bytecode_compiler.cpp's PrimaryCall case): either a builtin function
    // name (`decl == nullptr`, dispatched through evalBuiltinFunction --
    // `object()` gets its own special dispatch to mergeObjectArgs instead,
    // see bytecode_vm.cpp's CallFn handler, despite still setting
    // isBuiltin=true, since it's already in isBuiltinFunctionName's table),
    // `import` (`isImport=true`, dispatched to importAsValue instead), or a
    // user FunctionDeclaration found via the enclosing declaration's own
    // static scope -- a callee that doesn't resolve to any of these (a
    // function-literal *value* call, an unknown name) bails compilation of
    // the whole containing function instead of appearing here. `argNames[i]`
    // is nullopt for a positional argument, else the named argument's name
    // -- mirrors bindArgs'/resolveArgs' own positional-index-among-
    // positional-args-only rule, replayed at runtime (see bytecode_vm.cpp's
    // CallFn handler).
    struct CallSite {
        bool isBuiltin = false;
        bool isImport = false;
        std::string calleeName;
        const oscad::FunctionDeclaration* decl = nullptr;
        std::vector<std::optional<std::string>> argNames;
        const oscad::ASTNode* callNode = nullptr;
    };

    // An upvalue read (see Op::LoadUpvalue): `targetDecl` is the enclosing
    // FunctionDeclaration/FunctionLiteral this chunk was lexically nested
    // inside AT COMPILE TIME (matched at runtime via Evaluator::
    // findUpvalue's exact-identity search over the live call stack, the
    // same identity CallStackFrame::declNode is stamped with), `slot` is
    // that declaration's own slot number for the captured variable. `name`
    // is that same variable's source name -- unused by Op::LoadUpvalue
    // itself, needed by Op::MakeClosure's ClosureSite (below), which builds
    // a NAME-keyed captured environment (Closure::capturedLet is a
    // TrailView<Value>, looked up by name at invocation time, exactly like
    // every other escaping closure) rather than a slot-indexed one.
    struct UpvalueRef {
        const oscad::ASTNode* targetDecl = nullptr;
        int slot = 0;
        std::string name;
        // Set only when this entry is a `let(name = function(...) ...)`
        // closure's own reference to ITSELF (bytecode_compiler.cpp's LetOp
        // case, the letrec-style pre-declared-slot path) -- Op::MakeClosure
        // can't read this one out of `slots` the way it does every other
        // capture: at the moment it runs, the closure it's building hasn't
        // been constructed yet (StoreLocal for `name` hasn't executed), so
        // `slots[slot]` would still hold whatever was there before this
        // call. See Op::MakeClosure's own runtime doc comment
        // (bytecode_vm.cpp) for how this is actually resolved (deferred,
        // then patched in after construction).
        bool isSelfReference = false;
    };

    // One FunctionLiteral's own capture list for Op::MakeClosure, computed
    // at compile time (bytecode_compiler.cpp's FunctionLiteral case) as the
    // TRANSITIVE union of: every free variable `node` itself references
    // from outside its own body, PLUS -- for each FunctionLiteral nested
    // anywhere within `node` (however deep) -- whatever THAT nested
    // literal's own ClosureSite still needed from beyond `node`'s own scope
    // (i.e. any capture whose resolution target isn't `node` itself). This
    // "bubbling" is what makes a closure nested inside another compiled-
    // creating closure resolve correctly: at the OUTERMOST level a given
    // capture is actually snapshotted (see Op::MakeClosure's own runtime
    // doc comment, bytecode_vm.cpp), `captures`'s entries resolve against
    // that running chunk's own `slots` array whenever `targetDecl` matches
    // its own declaration -- and, when it doesn't (a capture bubbled up
    // from an even-more-deeply-nested literal), against the enclosing
    // frame via Evaluator::findUpvalue or, failing that, `ctx.let_`
    // (rooted at that OUTER closure's own capturedLet whenever it's itself
    // an escaped invocation) -- regardless of how many literal-within-
    // literal levels `node` was originally nested through in the source.
    //
    // `node`'s own body IS also registered (Evaluator::
    // lookupCompiledLiteralChunk finds a real cache entry for it, same as
    // a zero-capture literal), so a closure invocation can run compiled
    // too, not just its creation: Op::LoadUpvalue/Op::MakeClosure inside
    // that body fall back to `ctx.let_` exactly as described above
    // whenever the live-call-stack walk misses, which is precisely what
    // happens once this closure has actually escaped its creator.
    struct ClosureSite {
        const oscad::FunctionLiteral* node = nullptr;
        std::vector<UpvalueRef> captures;
    };

    // One echo() expression form's own argument names, in source order --
    // see Op::Echo. Not folded into CallSite: an echo() has no callee/decl/
    // isBuiltin identity at all, so those fields would sit permanently
    // unused and misleadingly defaulted.
    struct EchoSite {
        std::vector<std::optional<std::string>> argNames;
    };

    // One Op::CallModule site -- a ModularCall statically resolved (at
    // compile time, via the enclosing ModuleDeclaration's own static
    // scope) to a specific user ModuleDeclaration. Unlike CallSite,
    // there's no isBuiltin/isImport branch to represent: a callee that
    // ISN'T a resolved user ModuleDeclaration (a builtin, children(), an
    // unresolvable name) never gets one of these -- it's compiled as a
    // NativeStatement instead (see Op::CallModule's own doc comment for
    // why that's exactly right, not a missed optimization).
    struct ModuleCallSite {
        const oscad::ModuleDeclaration* decl = nullptr;
        const oscad::ModularCall* callNode = nullptr;
        std::string calleeName;
    };

    // One Op::PushBuiltinWrap/PopBuiltinWrap site pair -- a builtin-with-
    // children statement recognized at COMPILE time as one of the small,
    // closed set this covers (see Op::PushBuiltinWrap's own doc comment
    // for the full list and the Windows-crash rationale). Unlike
    // ModuleCallSite, this is NOT "a call" -- see this struct's own `kind`
    // discriminator and Evaluator::evalModularCall/buildTreeNode/
    // evalModifier (csg_resolve.cpp): builtins never touch callStack_ or
    // fire profileEnter/profileExit, native or compiled, so there's no
    // enterUserCall-style bracketing to represent here, just enough to
    // replay the native param-resolution step and rebuild the tagged
    // CSGNode at Pop time.
    //
    // `node`'s concrete type depends on `kind`: Transform/Color sites are
    // genuine `ModularCall`s (down-cast to read `.name`/`.arguments`/
    // `.children`); Modifier sites are the wrapper node itself
    // (`ModularModifierHighlight`/`Background`/`ShowOnly` -- `!`/`#`/`%`;
    // `*`/Disable never reaches here, its child never evaluates at all,
    // so it stays Op::NativeStatement's no-op-shaped native path), which
    // does NOT derive from ModularCall (it derives from
    // ModuleInstantiation directly and carries a single `child`) -- read
    // via its own concrete type when kind == Modifier, never as a
    // ModularCall. Stored as the generic `ASTNode*` here (not a tagged
    // union of concrete pointers) because that's all Op::PopBuiltinWrap
    // itself ever needs `node` for: the resulting CSGNode's own
    // `node`/error-position field.
    struct BuiltinWrapSite {
        enum class Kind { Transform, Color, Modifier };
        Kind kind;
        std::string tagName; // "translate"/"rotate"/.../"color"/"highlight"/"background"/"show_only"
        const oscad::ASTNode* node = nullptr;
    };

    // One Op::PushCsgWrap/PopCsgWrap site pair -- see those ops' own doc
    // comments for the full "why union/difference/intersection need
    // bespoke group_sizes bookkeeping instead of just reusing
    // BuiltinWrapSite" rationale. `op` is "union"/"difference"/
    // "intersection" (the 3 names resolveDispatch() maps to resolveCsg,
    // always a genuine ModularCall) OR "intersection_for" (NOT a
    // ModularCall -- a distinct NodeKind/ModuleInstantiation subtype with
    // its own `.assignments`/`.body`, see compileIntersectionForLoop,
    // bytecode_compiler.cpp) -- `node` is therefore the generic ASTNode*
    // BuiltinWrapSite's own Modifier kind already established this pattern
    // for, down-cast to ModularCall only when `hasArgs` is true.
    //
    // `hasArgs`/`includeOpParam` are both true for union/difference/
    // intersection, both false for intersection_for: it isn't a call at
    // all (no `.arguments`, so Op::PushCsgWrap's handler must skip
    // resolveCallArgs/the ctx push entirely -- matches native
    // resolveIntersectionFor, which uses the SAME ctx its caller passed in
    // unchanged, never a $-scoped child of its own), and its own
    // CSGParams has no "op" key at all (generateIntersectionFor is a
    // dedicated, separately-registered generate function, unlike
    // generateCsg which needs "op" in params to disambiguate union/
    // difference/intersection from ONE shared function).
    struct CsgWrapSite {
        std::string op;
        bool hasArgs = true;
        bool includeOpParam = true;
        const oscad::ASTNode* node = nullptr;
    };

    // One Op::AssertStatement site -- see that op's own doc comment for
    // the full contract. `conditionArgIndex`/`messageArgIndex` are indices
    // into the site's own argCount-sized popped-argument array (source
    // order), resolved at COMPILE time via the same "named wins over
    // positional 0/1" priority Evaluator::getArg applies at runtime --
    // nullopt means "no argument supplies this logical parameter" (mirrors
    // getArg's own defaultValue fallback: absent condition reads as
    // `Value{true}`, absent message means no ": ..." suffix).
    // `condTextConstIdx`: constants[] index of the condition argument's own
    // source text (or "false" when no condition argument exists, matching
    // evalAssertStatement's `node.arguments.empty() ? "false" : ...`),
    // interned once at compile time. `node`: for both Evaluator::error()'s
    // TRACE walk and the (rare) chained-children fallback below.
    //
    // ponytail: a script with TWO named args of the SAME name (e.g.
    // `assert(condition=true, condition=false)`) would resolve to the
    // LAST one here, not getArg's own first-match; not worth the extra
    // bookkeeping to fix for input that's already nonsensical.
    struct AssertSite {
        int argCount = 0;
        std::optional<int> conditionArgIndex;
        std::optional<int> messageArgIndex;
        int condTextConstIdx = -1;
        const oscad::ModularAssert* node = nullptr;
    };

    // Every distinct chunk this SPECIFIC CompiledChunk owns is either a
    // function body (bodyCode is one expression's own instruction stream)
    // or a module body (bodyCode is a compiled STATEMENT list -- see
    // tryCompileModuleBody, bytecode_compiler.cpp) -- never both. `params`/
    // `defaultCode` mean the same thing either way (a ModuleDeclaration's
    // parameters are shaped identically to a FunctionDeclaration's).
    // Gates driveVm's own frame-completion branch (bytecode_vm.cpp):a
    // function frame's completion produces a Value (popped off f.stack);
    // a module frame's produces nothing on the stack at all -- its whole
    // effect already landed in treeStack_ as a side effect of running its
    // own body.
    bool isModule = false;

    // The FunctionDeclaration/FunctionLiteral this chunk's own body was
    // compiled from (bytecode_compiler.cpp's compileFunctionLike sets this
    // to its own `selfDecl` parameter) -- Op::MakeClosure's runtime handler
    // (bytecode_vm.cpp) compares a ClosureSite capture's own `targetDecl`
    // against this to tell "genuinely local to the running chunk" (read
    // `slots` directly) from "bubbled up from a nested literal" (needs the
    // enclosing-frame/capturedLet fallback) apart.
    const oscad::ASTNode* selfDecl = nullptr;

    std::vector<Param> params;
    std::vector<std::vector<Instruction>> defaultCode;
    std::vector<Instruction> bodyCode;
    std::vector<Value> constants;
    std::vector<std::string> names;
    std::vector<CallSite> callSites;
    std::vector<UpvalueRef> upvalues;
    std::vector<ClosureSite> closureSites;
    std::vector<EchoSite> echoSites;
    // Module-body-only tables (see Op::CallModule/NativeStatement/
    // NativeCondJumpIfFalse/NativeCheckDebugExprLevel/NativeIterMaterialize's
    // own doc comments, above) -- always empty for a function chunk.
    std::vector<ModuleCallSite> moduleCallSites;
    std::vector<BuiltinWrapSite> builtinWrapSites;
    std::vector<CsgWrapSite> csgWrapSites;
    std::vector<AssertSite> assertSites;
    std::vector<const oscad::Expression*> nativeExprs;
    std::vector<const oscad::ASTNode*> nativeStatements;
    int numSlots = 0;
    // Count of distinct ListCompFor-assignment iterLists allocated across
    // this whole chunk (see IterMaterialize/IterReset/IterNext) -- runChunk
    // sizes its own native iterLists/iterIndex scratch vectors to this.
    int numIterLists = 0;

    // Source-file/line span this chunk's own body+defaults were compiled
    // from -- the min/max of every AST node's own position().line actually
    // visited during compilation (see Compiler::trackSpan, bytecode_compiler.cpp),
    // not just the declaration's own header line. Used by Evaluator::
    // chunkEligibleNow (debug "fast continue" mode) to decide, per
    // function, whether ANY currently-set breakpoint line could fall
    // inside this chunk's own body -- a plain integer-range check, chosen
    // specifically so that decision never needs the original source text
    // or an offset->line conversion at eligibility-check time (called on
    // every single compiled call, not just once). origin empty / minLine >
    // maxLine (never updated) should not happen for any chunk that
    // actually compiled a real body -- every function has at least one
    // expression to visit.
    std::string origin;
    int minLine = std::numeric_limits<int>::max();
    int maxLine = 0;

    // FunctionLiteral chunks discovered while compiling THIS chunk (a
    // FunctionLiteral expression appearing anywhere in its body/defaults),
    // recursively containing their own nested literals in turn. The
    // top-level caller (Evaluator::lookupOrCompileChunk) flattens this
    // whole tree into Evaluator::literalChunkCache_ after a successful
    // compile, so Evaluator::evalFunctionLiteral can find a given
    // FunctionLiteral node's own precompiled chunk directly by pointer,
    // without needing to search through whatever chunk originally
    // discovered it.
    std::vector<std::pair<const oscad::FunctionLiteral*, CompiledChunk>> nestedLiterals;
};

} // namespace oscadeval
