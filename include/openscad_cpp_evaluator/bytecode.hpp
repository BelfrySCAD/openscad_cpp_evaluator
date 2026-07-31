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
    LoadFree,       // a = name pool index -- Evaluator::evalIdentifier fallback
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
