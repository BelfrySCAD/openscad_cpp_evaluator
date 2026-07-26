#pragma once

#include "openscad_cpp_evaluator/bound_args.hpp"
#include "openscad_cpp_evaluator/bytecode.hpp"
#include "openscad_cpp_evaluator/bytecode_vm.hpp"
#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/csg_node.hpp"
#include "openscad_cpp_evaluator/debug_hooks.hpp"
#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/font_provider.hpp"
#include "openscad_cpp_evaluator/manifold_cache.hpp"
#include "openscad_cpp_evaluator/profile.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace oscadeval {

// Walks a parsed OpenSCAD AST, producing Value results for expressions and
// ColoredBody geometry (via the resolve/generate CSG-tree split) for
// statements.
//
// Ownership contract: the AST (and the Scope tree from buildScopes()) this
// evaluator walks must outlive the Evaluator instance and every Value/
// EvalContext/CSGNode/idToNode entry it produces -- every AST/Scope pointer
// this class touches is non-owning, mirroring openscad_cpp_parser's own
// unique_ptr-owned AST. A typical caller (the CLI, a test) keeps the parsed
// `vector<unique_ptr<ASTNode>>` and `unique_ptr<Scope>` alive in locals that
// outlive both the Evaluator and any use of its results.
//
// Phase 1: expression evaluation (literals, identifiers, arithmetic,
// comparison, logical ops, ternary, ranges, plain vector literals) and
// Assignment-statement handling.
//
// Phase 2/3: the CSG-tree two-pass pipeline (resolveTree()/generateTree()/
// evaluate()); full primitive/transform/boolean coverage plus the 4
// modifier wrappers and color().
//
// Phase 4 (new): control flow (if/if-else/for/let/echo/assert/
// intersection_for), list comprehension clauses, function calls (user
// functions, function-literal values), user module calls + children()
// (with closure-detection scoping via callCtxFor -- see its own doc
// comment), render()/breakpoint() passthrough. Math/string/type-check
// builtins, hull/minkowski/projection, extrusion/roof, surface/import,
// text, and the debugger/profiler land in later phases.
class Evaluator {
public:
    // `fontProvider`: injection seam for text()/textmetrics()/fontmetrics()
    // (see font_provider.hpp). Left null (the common case for this repo's
    // own tests/CLI, and for any host that hasn't wired in its own) lazily
    // constructs the built-in StbFontProvider default on first use, not in
    // the constructor -- avoids paying for font-file parsing on every
    // Evaluator even when a script never calls text()/textmetrics().
    // `manifoldCache`: opt-in content-hash cache (see manifold_cache.hpp)
    // shared across evaluate() calls on possibly-different Evaluator/AST
    // instances -- left null (the default) for a bare Evaluator(), which
    // is every existing single-shot caller (the CLI, this project's own
    // tests): no cache means generateTree() always does real Manifold
    // work, unaffected in behavior or performance by this parameter's
    // mere existence.
    // `debugHooks`: opt-in debugger injection (see debug_hooks.hpp) --
    // default-constructed (every callback null) means debugging is
    // entirely off, the same zero-overhead-when-unused shape as every
    // other optional constructor parameter here.
    // `profiling`: opt-in per-call-site timing (see profile.hpp) --
    // populates the public `profileResult` member after evaluate()
    // returns; false (the default) means evaluate() does no extra timing
    // work at all.
    explicit Evaluator(EchoFn echoFn = {}, std::shared_ptr<FontProvider> fontProvider = nullptr,
                        std::shared_ptr<ManifoldCache> manifoldCache = nullptr, DebugHooks debugHooks = {},
                        bool profiling = false);

    // Text/font builtins' shared access to the resolved FontProvider
    // (lazily constructing the built-in default on first call if none was
    // injected via the constructor). Public: builtins/text.cpp's
    // resolve/generate functions are free functions, same reasoning as
    // tagGenerated()/builtinChildren().
    FontProvider& fontProvider();

    // Called by rands() (function_builtins.cpp's evalBuiltinFunction) each
    // time it actually runs, so buildTreeNode()/evalModularCall() can
    // detect "did rands() fire anywhere while resolving this node" via a
    // before/after comparison and mark the node uncacheable -- see
    // CSGNode::uncacheable's doc comment. Public for the same reason
    // tagGenerated()/builtinChildren() are: the call site is a free
    // function, not a method.
    void noteRandsCall() { ++randsCallCount_; }

    // Dispatches on node.kind() (switch, not a visitor -- matches
    // openscad_cpp_parser's own dispatch convention). Throws
    // std::logic_error for a NodeKind not yet implemented by this phase
    // (a builtin/unknown function call -- math/string/type-check builtins
    // land in Phase 5) rather than silently returning undef -- "not yet
    // supported" should fail loudly, not produce a plausible-looking wrong
    // answer.
    Value evalExpr(const oscad::Expression& node, EvalContext& ctx);

    // Evaluates a block's statements in OpenSCAD's assignment-before-
    // geometry order (all Assignment nodes first, then everything else,
    // each group preserving source order), each against its own lexical
    // scope but sharing dyn/let state with `ctx` (see
    // EvalContext::withScope) so an earlier sibling's assignment is visible
    // to a later one.
    void evalChildren(const std::vector<std::unique_ptr<oscad::ASTNode>>& children, EvalContext& ctx);
    // Same, over a non-owning pointer list -- lets a resolve function (e.g.
    // _resolve_csg's per-top-level-statement grouping) build ad-hoc
    // sub-groups of an existing children list without needing ownership.
    void evalChildren(const std::vector<const oscad::ASTNode*>& children, EvalContext& ctx);

    // Top-level entry point: resolveTree() + generateTree(), then the
    // top-level `!` (show_only) filter -- if any returned body has
    // BodyRole::ShowOnly, the result is filtered down to just
    // ShowOnly+Highlight bodies. Mirrors the reference's evaluate().
    //
    // `viewportParams`: seeds arbitrary `$`-prefixed entries into
    // `ctx.dyn` (e.g. `$vpt`/`$vpr`/`$vpd`/`$vpf` from the current camera,
    // or `$t` for animation time) before evaluation starts -- applied via
    // a direct `ctx.dyn` write, deliberately bypassing whatever marks a
    // name in `ctx.dynExplicit`, so a caller can tell "the script itself
    // assigned this $-var" (dynExplicit) apart from "this $-var is merely
    // present because the caller seeded it" (present in `dyn`, absent
    // from `dynExplicit`) after evaluate() returns -- inspect `ctx.dyn`/
    // `ctx.dynExplicit` directly on the same `ctx` passed in, since unlike
    // the reference (whose `ctx` is constructed inside evaluate() itself,
    // forcing it to expose a separate `_root_ctx` escape hatch for this)
    // this port's `ctx` is always caller-owned already. Mirrors the
    // reference's `evaluate(nodes, root_scope, viewport_params=None)`.
    std::vector<ColoredBody> evaluate(const std::vector<std::unique_ptr<oscad::ASTNode>>& nodes, EvalContext& ctx,
                                       const std::unordered_map<std::string, Value>& viewportParams = {});
    // Same, but over a non-owning pointer list -- needed by `use <file>`
    // resolution (eval_use.hpp/.cpp), whose "processed nodes" list combines
    // declarations owned by more than one independently-owned AST vector,
    // so it can't be represented as a single vector<unique_ptr<ASTNode>>.
    // Mirrors evalChildren's existing dual-overload shape.
    std::vector<ColoredBody> evaluate(const std::vector<const oscad::ASTNode*>& nodes, EvalContext& ctx,
                                       const std::unordered_map<std::string, Value>& viewportParams = {});

    // Resolve pass: resets idToNode/idToColor and the tree-build stack,
    // then walks `nodes` via evalChildren(), returning the completed (but
    // not yet generated) CSG tree. Exposed separately from evaluate() for
    // callers/tests that want to inspect tree shape before generation.
    std::vector<std::unique_ptr<CSGNode>> resolveTree(const std::vector<std::unique_ptr<oscad::ASTNode>>& nodes,
                                                        EvalContext& ctx);
    // Same, but over a non-owning pointer list -- see evaluate()'s own
    // raw-pointer overload above.
    std::vector<std::unique_ptr<CSGNode>> resolveTree(const std::vector<const oscad::ASTNode*>& nodes, EvalContext& ctx);

    // Size of the tree-build frame currently on top of the stack -- lets a
    // resolve function (union/difference/intersection/intersection_for's
    // per-top-level-statement or per-iteration grouping, "group_sizes" in
    // the reference) measure how many CSGNodes a single child statement
    // contributed by comparing this before/after evaluating just that one
    // statement.
    size_t currentTreeFrameSize() const { return treeStack_.back().size(); }

    // Generate pass: walks `tree` bottom-up (children before their own
    // node), calling each node's registered GenerateFn (falling back to
    // concatenating children's bodies for a kind with none registered).
    // Mirrors the reference's generate_tree(). Does NOT apply the
    // top-level show_only filter -- see evaluate().
    std::vector<ColoredBody> generateTree(const std::vector<std::unique_ptr<CSGNode>>& tree);

    // Provenance tables populated by tagGenerated() during generate --
    // originalID -> the AST node that produced it / that node's own
    // resolved color. Public, read by a caller (CLI, WYSIWYG picking) after
    // evaluate()/resolveTree()+generateTree() complete. Mirrors the
    // reference's Evaluator.id_to_node/id_to_color (also plain public
    // attributes there).
    std::unordered_map<uint32_t, const oscad::ASTNode*> idToNode;
    std::unordered_map<uint32_t, std::optional<std::array<float, 4>>> idToColor;

    // Called by primitive-construction generate functions (cube, sphere,
    // cylinder, polyhedron) right after building a brand-new Manifold:
    // reads back its mesh's runOriginalID run(s) and records each against
    // `node`/`color` in idToNode/idToColor, then wraps `body` as a
    // ColoredBody. Mirrors the reference's _tag_generated(). Public because
    // GenerateFn is a free function, not a method -- it needs to call this
    // on the Evaluator& it's handed (see dispatch.hpp's GenerateFn doc
    // comment for why that reference exists at all).
    ColoredBody tagGenerated(manifold::Manifold body, const oscad::ASTNode& node, const Value& colorValue);

    // Raises an OpenSCAD-level runtime error: throws EvalError with the
    // formatted "ERROR: {msg}{TRACE lines}" message (see eval_error.hpp),
    // walking the real user function/module call stack for the TRACE lines
    // -- now that Phase 4 has one. `innermostFrame` names the failing
    // construct for its own leading TRACE line (e.g. "assert"), matching
    // the reference's error(..., innermost_frame=...); omit for a plain
    // validation error with no such frame (polyhedron/polygon argument
    // checks). Mirrors the reference's Evaluator.error(), minus the
    // error_break_fn/error-log bookkeeping (no debugger until Phase 9).
    [[noreturn]] void error(const std::string& msg, const oscad::ASTNode& node, const std::string& innermostFrame = "");

    // Evaluates a call's arguments against `params`' declared names:
    // NamedArgument binds directly by name; PositionalArgument binds to
    // `params[i]`'s name in declaration order (silently dropped past
    // `params.size()`, matching the reference -- OpenSCAD doesn't error on
    // extra positional arguments, it just ignores them). Missing
    // parameters are NOT filled in here (see applyDefaults). Shared by
    // user module calls, user function calls, and function-literal calls.
    // Public: builtins/control.cpp's children()/render() plumbing and a
    // future phase's own call sites may need it directly.
    BoundArgs bindArgs(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                       const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx);

    // children()/children(N) -- deferred evaluation of the calling module's
    // own children, re-injecting the current $-variables. `args` is the
    // resolved argument bag from children()'s own call (an "index"
    // argument selects a single child *statement*, not output body -- see
    // the reference's own doc comment on this distinction). Called from
    // builtins/control.cpp's resolveChildren for the side effect of
    // building CSGNodes into whatever tree-build frame is currently on
    // top. Public for the same reason tagGenerated() is (a free function
    // needs to call it on the Evaluator& it's handed). Mirrors
    // _builtin_children/_eval_children_lazy.
    void builtinChildren(const CallArgs& args, EvalContext& ctx);

    // "WARNING: {message}{locSuffix(position)}" via echoFn_, no-op if unset.
    // Public: builtins/import.cpp's not-manifold warning is emitted from a
    // free function, same reasoning as tagGenerated()/builtinChildren().
    void warn(const std::string& message, const oscad::Position* position);

    // parent_module(idx) support: walks the live module-call stack
    // (innermost last) and returns the name `idx` levels up from the
    // current module, or undef if out of range. Public because
    // evalBuiltinFunction (function_builtins.cpp) is a free function, same
    // reasoning as tagGenerated()/builtinChildren(). Mirrors
    // _builtin_parent_module.
    Value parentModuleName(int idx) const;

    // Identifier resolution: let_ -> ($-prefixed only) dyn -> PI -> static
    // Scope::lookupVariable fallback (re-evaluates that declaration's own
    // expr against `ctx`). Public so the bytecode VM's LOAD_FREE opcode
    // (bytecode_vm.cpp) can call it directly for any identifier that isn't
    // a compile-time-resolved local slot -- this already gives the correct
    // answer for a compiled function's own parameter names too (their slot
    // never gets written into ctx.let_, so this just falls through to the
    // scope-fallback's own ParameterDeclaration -> undef rule, exactly
    // matching applyDefaults' existing sibling-isolation behavior; see
    // bytecode_compiler.cpp's own header comment on default-value
    // compilation for why that's relied on, not incidental).
    Value evalIdentifier(const std::string& name, const oscad::Position* position, EvalContext& ctx, bool warnIfUndef);

    // Pure Value x Value -> Value implementations shared by evalExpr's own
    // switch cases and the bytecode VM's generic UNARY_OP/BINARY_OP/INDEX/
    // MEMBER opcodes (bytecode_vm.cpp) -- moved out of evalExpr verbatim
    // (no behavior change) so neither copy risks drifting from the other.
    // `pos` is only consulted by the handful of cases that can warn()
    // (comparisons, bitwise ops).
    Value applyBinaryOp(oscad::NodeKind kind, const Value& a, const Value& b, const oscad::Position& pos);
    Value applyUnaryOp(oscad::NodeKind kind, const Value& v, const oscad::Position& pos);
    Value applyIndexAccess(const Value& obj, const Value& idx);
    Value applyMemberAccess(const Value& obj, const std::string& member);
    Value applyRange(const Value& startV, const Value& endV, const Value& stepV);

    // Entry point for a call site resolved INSIDE compiled bytecode (the
    // CALL_FN opcode, bytecode_vm.cpp): `bound` is already fully evaluated
    // (each argument expression was compiled and run against the CALLING
    // chunk's own slots, unlike bindArgs, which evaluates raw AST argument
    // nodes against a live ctx) -- otherwise mirrors evalUserFunction
    // exactly (callCtxFor, compiled-vs-interpreted dispatch, the callstack/
    // profile/debug bracket via the same evalUserFunctionCore both share).
    // Public: bytecode_vm.cpp is a separate translation unit.
    Value evalUserFunctionFromBound(const std::string& name, const oscad::FunctionDeclaration& decl, BoundArgs bound,
                                     EvalContext& ctx, const oscad::Position* callPos);

    // Same, for a call whose callee was only resolvable at runtime (the
    // CALL_DYNAMIC opcode, once it's popped a Value and confirmed it holds
    // a FunctionLiteral pointer -- the closure-calling case Phase 2 exists
    // for) -- `bound` is matched against `funcNode`'s OWN parameters
    // (discovered now, not at compile time, since the callee wasn't
    // statically known). Mirrors evalFunctionLiteral exactly otherwise.
    Value evalFunctionLiteralFromBound(const oscad::FunctionLiteral& funcNode, BoundArgs bound, EvalContext& ctx,
                                        const oscad::Position* callPos);

    // Testing-only override, checked before the (once-cached) env var --
    // lets test_bytecode_compiler.cpp force VM-on for specific tests
    // without depending on process-startup environment (the env-var read
    // in bytecodeVmEnabled() is cached forever after its first call, so a
    // plain setenv() from inside a test body would have no effect on later
    // calls within the same test binary). Pass std::nullopt to go back to
    // the env var.
    static void setBytecodeVmEnabledForTesting(std::optional<bool> enabled);

    // Checks whether a debug pause should happen at `node` (via the
    // injected DebugHooks::debugHook, if any -- a no-op otherwise),
    // applying any `mods` the hook returns to `ctx.let_` and throwing
    // EvalError if it returns stop=true. `forced=true` bypasses nothing on
    // this port's side (unlike the reference's own step/breakpoint-line
    // filtering, which lives entirely in the injected hook, not here) --
    // it's simply passed through so the hook itself can distinguish an
    // unconditional breakpoint()/function-entry pause from a normal
    // statement-boundary check. Public: called from evalChildren() (every
    // ordinary statement), evalUserFunction()/evalFunctionLiteral() (function
    // body entry), and builtins/control.cpp's resolveBreakpoint (a free
    // function). See debug_hooks.hpp's DebugHookFn doc comment for the
    // exact statement-level-only scope decision. Mirrors
    // Evaluator._check_debug.
    void checkDebug(const oscad::ASTNode& node, EvalContext& ctx, bool forced = false);

    // Whole-evaluate() profiling summary (see profile.hpp), populated by
    // evaluate() when constructed with profiling=true; nullopt otherwise
    // (including for direct resolveTree()/generateTree() callers that
    // bypass evaluate() -- profiling only ever wraps the exact
    // resolve+generate bracket evaluate() itself runs). Mirrors the
    // reference's Evaluator.profile_result.
    std::optional<ProfileResult> profileResult;

private:
    // Shared body for resolveTree()'s/evaluate()'s two overloads (owning vs.
    // raw-pointer node list) -- a template rather than duplicating each
    // ~15-40 line body, instantiated only for the two container types their
    // public overloads call it with (defined in csg_resolve.cpp, used only
    // there, so no explicit-instantiation declaration is needed).
    template <typename NodeList>
    std::vector<std::unique_ptr<CSGNode>> resolveTreeImpl(const NodeList& nodes, EvalContext& ctx);
    template <typename NodeList>
    std::vector<ColoredBody> evaluateImpl(const NodeList& nodes, EvalContext& ctx,
                                           const std::unordered_map<std::string, Value>& viewportParams);

    // Evaluates one top-level element of a `[...]` list literal (a plain
    // expression, or one of the 6 list-comprehension clause kinds --
    // for/c-for/let/if/if-else/each), appending whatever it contributes to
    // `out`. Mirrors the reference's _eval_list_comp's per-element
    // dispatch. `evalListLiteral` calls this once per top-level element;
    // `evalListCompBody` (a clause's own *body*, which can itself be
    // another clause -- e.g. `for(i=...) if(cond) x` -- or a nested `[...]`
    // literal) is the recursive counterpart, mirroring the reference's
    // separate _eval_list_comp/_eval_list_comp_body pair exactly (their
    // wrapping rules genuinely differ -- see each one's own body).
    void evalListElement(const oscad::ASTNode& elem, EvalContext& ctx, std::vector<Value>& out);
    std::vector<Value> evalListCompBody(const oscad::ASTNode& body, EvalContext& ctx);
    Value evalListLiteral(const oscad::ListComprehension& node, EvalContext& ctx);
    Value evalRangeLiteral(const oscad::RangeLiteral& node, EvalContext& ctx);
    Value evalFunctionCall(const oscad::PrimaryCall& node, EvalContext& ctx);
    Value evalLetExpr(const oscad::LetOp& node, EvalContext& ctx);
    Value evalEchoExpr(const oscad::EchoOp& node, EvalContext& ctx);
    Value evalAssertExpr(const oscad::AssertOp& node, EvalContext& ctx);
    void bindLetName(EvalContext& ctx, const std::string& name, const Value& v);

    void evalStatement(const oscad::ASTNode& node, EvalContext& ctx);
    void evalAssignment(const oscad::Assignment& node, EvalContext& ctx);
    void evalModularCall(const oscad::ModularCall& node, EvalContext& ctx);
    void evalFor(const oscad::ModularFor& node, EvalContext& ctx);
    void evalLetBlock(const oscad::ModularLet& node, EvalContext& ctx);
    void evalAssertStatement(const oscad::ModularAssert& node, EvalContext& ctx);
    void evalIntersectionForNode(const oscad::ModularIntersectionFor& node, EvalContext& ctx);
    void doEcho(const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx);

    // Shared by the 3 tag-modifier wrappers (#/%/!) -- ModularModifierDisable
    // (*) never calls this, it's a pure no-op (its child is never
    // evaluated at all, matching the reference exactly). `kind` is
    // "highlight"/"background"/"show_only", looked up in generateDispatch()
    // the same way a ModularCall's own name is.
    void evalModifier(const oscad::ModuleInstantiation& child, const oscad::ASTNode& wrapperNode,
                       const std::string& kind, EvalContext& ctx);

    // Shared push/resolve/pop/build-node sequence behind evalModularCall,
    // evalModifier, and evalIntersectionForNode: pushes a fresh treeStack_
    // frame, runs `resolveBody` (whose own recursive evalChildren/
    // evalStatement calls populate that frame as a side effect), pops it
    // as this node's `children`, builds the CSGNode, and appends it to
    // whatever frame is now on top (the parent's). Exception-safe: a frame
    // pushed here is always popped, even if `resolveBody` throws, so a
    // raised EvalError cleanly discards the in-progress node rather than
    // corrupting a parent's accumulator.
    void buildTreeNode(const std::string& kind, const oscad::ASTNode& node,
                        const std::function<CSGParams()>& resolveBody);

    // -- User function/module calls (Phase 4) --------------------------

    // Picks between an isolated call scope (callCtx()) and a closure-
    // capturing one (childCtx()) for entering `decl`'s body: walks the
    // live call stack, and if `decl`'s own declaration span is *strictly*
    // contained within an already-active frame's declaration span (i.e.
    // `decl` is a module/function declared lexically inside the body of a
    // module/function currently being evaluated -- a closure over that
    // call's locals), uses childCtx() to inherit them; otherwise uses the
    // isolated callCtx(). Direct recursion (a declaration's span
    // containing itself) is excluded from "nested" so a recursive call
    // doesn't inherit its own in-progress locals as if they were its
    // caller's. Mirrors the reference's _call_ctx_for exactly.
    //
    // childCtx()/callCtx() themselves are now O(1) "open a new trail
    // level" operations (see scope_trail.hpp) regardless of whether the
    // callee declares a $-parameter -- this used to matter a great deal
    // (a `shareDyn` fast path skipped an O(map-size) copy when provably
    // safe, worth ~25% of total evaluate() wall time on a real BOSL2-heavy
    // script) back when every derivation copied the whole map; deleted
    // once the trail redesign made that copy cost disappear entirely, see
    // git history for the removed shareDyn/hasDollarParam machinery.
    // `usedChildCtx`, if non-null, is set to true/false with which branch
    // this call took -- needed by evalUserFunctionCore's callers (Phase 2)
    // to compute the new frame's own upvalue-ancestry parent correctly:
    // childCtx() alone does NOT mean a closure over `decl`'s enclosing
    // frame is actually reachable -- it only means "continue whatever
    // ancestry `ctx` itself already has," which can already be severed
    // (see CallStackFrame::upvalueParent's own doc comment for the real
    // scenario this was caught on -- a closure passed through an
    // unrelated intermediary function).
    EvalContext callCtxFor(const oscad::ASTNode& decl, EvalContext& ctx, const oscad::Scope* scope,
                            std::shared_ptr<const ChildrenNodeList> childrenNodes = nullptr,
                            const EvalContext* childrenCallerCtx = nullptr, bool* usedChildCtx = nullptr);

    // Fills in any parameter not already present in `bound` (bindArgs'
    // own return value -- the authoritative "did the caller actually
    // supply this name" record) from its own default-value expression,
    // evaluated purely lexically against the declaration's own scope
    // (childCtx.scope) with an empty `let_` and `dyn` shared with childCtx
    // -- a default sees neither the caller's locals nor this call's other
    // (sibling) parameters, though $-vars remain dynamically scoped. A
    // parameter with no default expression at all binds to undef. Writes
    // a $-prefixed parameter's result into childCtx.dyn, everything else
    // into childCtx.let_ -- matching the bound-argument loop just above
    // each call site exactly (bindArgs/callers split $ vs. non-$ names the
    // same way). `bound` (not childCtx.let_/dyn) must be the source of
    // truth for "already bound": childCtx.dyn is always pre-seeded with
    // $fn/$fa/$fs/$t/$parent_modules regardless of what the caller passed
    // (see eval_context.cpp), so checking dyn's mere presence can never
    // distinguish "the caller explicitly passed $fn" from "$fn merely
    // exists because of that ambient seed" -- a real bug this signature
    // fixes (previously checked only childCtx.let_, which missed a
    // $-prefixed name bound into dyn just as badly, in the other
    // direction: it always looked unbound, clobbering the real value with
    // undef).
    void applyDefaults(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params, const BoundArgs& bound,
                        EvalContext& childCtx);

    // Shared tail of evalUserFunction/evalUserFunctionFromBound/
    // evalFunctionLiteral/evalFunctionLiteralFromBound's interpreter
    // branch: splits `bound` into childCtx.dyn ($-prefixed) vs.
    // childCtx.let_ (everything else), then calls applyDefaults for
    // whatever `bound` didn't cover. Factored out (identical four-way
    // duplication before this) specifically so the tail-call trampoline's
    // own FunctionCall case (simplifyTailStep, added alongside this) can
    // reuse it instead of a fifth copy.
    void bindCallArgsInto(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params, BoundArgs bound,
                           EvalContext& childCtx);

    // Shared by checkDebug() and error()'s own errorBreak hook call: `let`
    // + `$`-prefixed `dyn` entries from `ctx`, plus (if there's an active
    // user call) any top-level script variable not shadowed locally.
    // `ctx` may be null (error()'s lastCtx_ fallback can itself be null
    // before any statement has ever run) -- returns an empty frame then.
    DebugFrame buildDebugFrame(const EvalContext* ctx) const;

    // Shared by evalUserFunction/evalUserFunctionFromBound/
    // evalFunctionLiteral: the callstack/profile/debug bracket, factored
    // out so these three entry points (raw-AST-argument vs. already-bound-
    // value vs. function-literal call sites) can't drift on it.
    // `computeResult` is called with the call already on callStack_/
    // profiling active -- it's just "evaluate the body," binding (which
    // differs by entry point and compiled-vs-interpreted) already done by
    // the caller before this runs. `declNode`/`bodyExpr` take a plain
    // ASTNode/Expression rather than FunctionDeclaration specifically so
    // FunctionLiteral (`.body`/`.position()`, the same shape under
    // different member names) can share this too -- also where
    // CallStackFrame::declNode gets stamped, the identity
    // Evaluator::findUpvalue's runtime search matches against.
    // `upvalueParent`: the callStack_ index the new frame's own
    // closure-visibility chain continues from, or -1 -- see
    // CallStackFrame::upvalueParent's own doc comment; the caller computes
    // this from callCtxFor's `usedChildCtx` out-param, since the decision
    // needs to be known before the new frame is pushed here.
    Value evalUserFunctionCore(const std::string& name, const oscad::ASTNode& declNode, const oscad::Expression& bodyExpr,
                                EvalContext& childCtx, const oscad::Position* callPos, int upvalueParent,
                                const std::function<Value()>& computeResult);

    // -- Tail-call optimization (interpreter path) -----------------------
    //
    // Mirrors upstream real OpenSCAD's own trampoline (Expression.cc's
    // FunctionCall::evaluate()/simplify_function_body): a chain of
    // TernaryOp/LetOp/EchoOp/AssertOp/FunctionCall nodes in tail position
    // is walked with a loop (evalFunctionBodyTrampoline) instead of genuine
    // C++ recursion, so a deep tail-recursive OpenSCAD function (e.g.
    // `function sum(n,acc=0) = n==0 ? acc : sum(n-1,acc+n);`) runs in O(1)
    // native stack space instead of overflowing. evalExpr's own switch is
    // completely untouched -- simplifyTailStep is a separate, narrower
    // dispatch over the same 5 node kinds, used only from
    // evalFunctionBodyTrampoline; anything it doesn't specially handle (or
    // a FunctionCall that isn't eligible -- see tryTailStepFor) is
    // evaluated for real via the ordinary evalExpr/evalUserFunction path,
    // exactly as it always has been.

    // One trampoline step: either a final Value, or "continue with
    // `nextExpr`/`ctx`" (isNewLogicalCall marks a genuine call boundary --
    // the trampoline mutates callStack_'s top frame and records a
    // profiling hop only for these, not for Ternary/Let/Echo/Assert
    // unwrapping, which stay logically part of the *same* call).
    struct TailStep {
        const oscad::Expression* nextExpr = nullptr;
        EvalContext ctx;
        bool isNewLogicalCall = false;
        std::string calleeName;
        const oscad::ASTNode* calleeDecl = nullptr;
        const oscad::Position* callPos = nullptr;
    };

    // Returns a TailStep for an isolated (non-closure-nested, see below),
    // uncompiled call to a declaration sharing FunctionDeclaration/
    // FunctionLiteral's common shape (name/params/body known by the
    // caller) -- shared by simplifyTailStep's two callee-kind branches.
    // Returns nullopt when this hop isn't eligible for trampolining:
    //  - `hasCompiledChunk`: the callee has its own compiled bytecode
    //    chunk -- its own tail calls are Phase B's (bytecode VM) job; this
    //    port's interpreter-path trampoline doesn't try to run compiled
    //    code, so crossing into a compiled callee pays one real C++ frame
    //    here, by design (see this plan's own "mixed compiled/interpreted
    //    tail chain" note).
    //  - closure-nested (callCtxFor's usedChildCtx==true): this port's
    //    closure lookup (findUpvalue) walks CallStackFrame::upvalueParent
    //    as an index chain across DISTINCT call-stack slots (unlike
    //    upstream OpenSCAD, which captures its closure context directly).
    //    Collapsing a closure-nested hop into the trampoline's single
    //    mutated frame would make that frame's own upvalueParent point at
    //    itself -- findUpvalue would then loop forever the first time such
    //    a closure reads an ancestor two or more levels up. Isolated hops
    //    (self-recursion, mutual recursion, accumulator-style helpers --
    //    the overwhelming common case) are unaffected and get full
    //    O(1)-stack treatment; a closure-nested tail call just pays one
    //    real C++ stack frame, same as today.
    std::optional<TailStep> tryTailStepFor(const std::string& calleeName, const oscad::ASTNode& declNode,
                                            const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                                            const oscad::Expression& body, bool hasCompiledChunk,
                                            const std::vector<std::unique_ptr<oscad::Argument>>& arguments,
                                            EvalContext& ctx, const oscad::Position& callPos);

    // Marker for "not one of the 5 trampolinable node kinds" -- the
    // trampoline falls back to a genuine evalExpr(node, ctx) call for
    // these (unchanged, real recursion), exactly as it always has.
    struct NotTailStep {};

    std::variant<Value, TailStep, NotTailStep> simplifyTailStep(const oscad::Expression& node, EvalContext& ctx);

    // The trampoline itself -- replaces a single `evalExpr(*decl.expr,
    // childCtx)` call as the body of evalUserFunctionCore's `computeResult`
    // lambda, for the interpreter (non-compiled) branch only. The FIRST
    // logical call's CallStackFrame/profiling are already set up by
    // evalUserFunctionCore/profileEnter before this runs -- this function
    // only needs to mutate them on a *subsequent* tail hop, via
    // recordTailCallHop (also shared with the bytecode-VM trampoline). `ctx`
    // is taken by reference to the SAME storage evalUserFunctionCore already
    // pointed lastCtx_ at before computeResult() runs -- each iteration
    // move-assigns into it (never declares a fresh local), so lastCtx_
    // never dangles and needs no extra bookkeeping.
    Value evalFunctionBodyTrampoline(const oscad::Expression& bodyExpr, EvalContext& ctx);

    void evalUserModule(const oscad::ModuleDeclaration& decl, const oscad::ModularCall& call, EvalContext& ctx);
    Value evalUserFunction(const std::string& name, const oscad::FunctionDeclaration& decl,
                            const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                            const oscad::ASTNode* callNode);
    Value evalFunctionLiteral(const oscad::FunctionLiteral& funcNode,
                               const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                               const oscad::ASTNode* callNode);

    // -- Bytecode VM (Phase 1) -------------------------------------------

    // Compile-attempt cache keyed by declaration pointer identity, populated
    // lazily on first call -- safe under `use <file>`'s AST-node-sharing
    // (see eval_use.hpp's own doc comment): a compiled chunk's slot layout
    // depends only on the declaration's own parameters/body, never on which
    // combining scope it's evaluated under. nullopt means "tried, doesn't
    // compile" (see tryCompileFunction) -- cached too, so a function that
    // uses e.g. echo() isn't re-attempted on every single call.
    std::unordered_map<const oscad::FunctionDeclaration*, std::optional<CompiledChunk>> chunkCache_;

    // FunctionLiteral chunks (Phase 2/closures): populated only as a side
    // effect of successfully compiling some OTHER declaration that
    // lexically contains the literal (see CompiledChunk::nestedLiterals'
    // own doc comment for why eager discovery, not lazy-on-first-call, is
    // required) -- flattenNestedLiterals() walks a freshly compiled
    // chunk's own nestedLiterals tree (recursively, for a literal
    // containing another literal) and merges every entry in here.
    // Presence = compiled; a literal that failed to compile, or was never
    // reached by any compiled container, simply has no entry -- no
    // optional wrapper needed (unlike chunkCache_, which also has to
    // remember "tried once, don't retry").
    std::unordered_map<const oscad::FunctionLiteral*, CompiledChunk> literalChunkCache_;
    void flattenNestedLiterals(CompiledChunk& chunk);

public:
    // lookupOrCompileChunk/lookupCompiledLiteralChunk: public (not just
    // private helpers of evalUserFunction*/evalFunctionLiteral* anymore)
    // since bytecode_vm.cpp's Phase B trampoline (runCompiledFunctionTrampoline/
    // runCompiledFunctionFromBoundTrampoline, a separate translation unit)
    // needs to resolve a tail-called declaration's own chunk directly, the
    // same way runChunk's CallFnTail/CallDynamicTail handlers need it to
    // decide whether a hop even HAS a compiled chunk to jump to (a callee
    // that doesn't compile falls back to a real call instead -- see
    // bytecode.hpp's own CallFnTail/CallDynamicTail doc comment).
    const CompiledChunk* lookupOrCompileChunk(const oscad::FunctionDeclaration& decl);
    const CompiledChunk* lookupCompiledLiteralChunk(const oscad::FunctionLiteral& node) const;

    // Pooled VmFrame scratch buffers (see VmFrame's own doc comment,
    // bytecode_vm.hpp, for why pooling exists at all -- a measured, not
    // assumed, fix for 3-fresh-vector-allocations-per-call erasing the
    // whole point of the VM). Public: bytecode_vm.cpp is a separate
    // translation unit. `acquireVmFrame()` reuses an already-returned
    // frame's allocated capacity when one is available (a still-active
    // recursive/nested call's own frame is never in the pool, so this
    // never aliases two concurrent calls onto the same buffers).
    std::unique_ptr<VmFrame> acquireVmFrame();
    void releaseVmFrame(std::unique_ptr<VmFrame> frame);

    // Closures/upvalues (Phase 2). `setCurrentCallVmFrame` stamps the
    // just-acquired VmFrame onto the CURRENT (topmost) call-stack entry --
    // called from runCompiledFunction/runCompiledFunctionFromBound right
    // after acquiring, before running any bytecode, so a nested compiled
    // call's own LOAD_UPVALUE can find it immediately.
    // `findUpvalue` walks the live call stack innermost-first for a frame
    // whose own declaration identity matches `targetDecl` -- returns
    // nullptr if that call already returned (this codebase has no
    // escaping closures) or was never compiled (vmFrame still null, e.g.
    // an interpreted enclosing call -- see this project's own scope-trail
    // notes on why that combination can't arise for code this phase
    // actually compiles: a container is only ever compiled if every
    // FunctionLiteral nested inside it also compiled, so an upvalue's own
    // target is always itself compiled whenever it's actually active).
    void setCurrentCallVmFrame(void* frame) {
        if (!callStack_.empty()) callStack_.back().vmFrame = frame;
    }
    const Value* findUpvalue(const oscad::ASTNode* targetDecl, int slot) const;

    // -- Tail-call optimization support shared by both execution paths ---
    //
    // Bytecode-VM analog of tryTailStepFor's own isolation check (see that
    // function's doc comment for the exact closure-nesting hazard this
    // avoids -- identical reasoning applies here). Public: runChunk's
    // Op::CallFnTail/CallDynamicTail handlers (bytecode_vm.cpp, a separate
    // translation unit) call this to decide whether a hop is eligible to
    // trampoline. Returns the already-derived isolated-call EvalContext
    // (callCtxFor's own callCtx() branch) on success -- the caller uses it
    // directly rather than re-deriving, exactly one callCtxFor call either
    // way (same as the interpreter path's own tryTailStepFor) -- or
    // nullopt if `declNode` is closure-nested inside the currently-active
    // call, in which case the caller must fall back to a real recursive
    // call (evalUserFunctionFromBound/evalFunctionLiteralFromBound).
    std::optional<EvalContext> isolatedCallCtxFor(const oscad::ASTNode& declNode, EvalContext& ctx);

    // Shared per-hop bookkeeping for BOTH trampolines (evalFunctionBodyTrampoline,
    // user_calls.cpp, AND runCompiledFunctionTrampoline/
    // runCompiledFunctionFromBoundTrampoline, bytecode_vm.cpp): mutates
    // callStack_'s current (already-pushed, by the outer evalUserFunctionCore)
    // frame in place to reflect the new callee's identity (name/declNode/
    // declPosition/callPosition, upvalueParent reset to -1 -- always
    // isolated, or this hop wouldn't be here), checks/bumps `recursionGuard`
    // against the same 1,000,000-iteration cap real upstream OpenSCAD uses
    // (throws via error() past it, message text mirroring upstream's own
    // RecursionException), and records a lightweight profiling hop
    // (profileRecordTailHop). Public: bytecode_vm.cpp needs it too.
    void recordTailCallHop(const std::string& calleeName, const oscad::ASTNode& calleeDecl,
                            const oscad::Position* callPos, unsigned& recursionGuard);

    // "ECHO: ..." formatting + emission given ALREADY-EVALUATED
    // (name-or-nullopt, Value) pairs in call-site order -- the shared core
    // doEcho (below) wraps (evaluate each raw argument, then call this).
    // Public: the bytecode VM's compiled echo() expression form (Op::Echo,
    // bytecode_vm.cpp, a separate translation unit) pops its own
    // already-evaluated arguments off the compiled stack and calls this
    // directly, so `ModularEcho` (statement form), evalEchoExpr
    // (interpreted expression form), and the compiled form all share
    // exactly one formatting rule -- can't drift, same reasoning as
    // applyBinaryOp/applyUnaryOp being shared between the interpreter and
    // VM already.
    void emitEcho(const std::vector<std::pair<std::optional<std::string>, Value>>& pairs);

private:
    std::vector<std::unique_ptr<VmFrame>> vmFramePool_;

    // Reads the OSCAD_BYTECODE_VM env var exactly once (function-local
    // static) -- default-ON as of Phase 5's rollout decision: unset, or
    // any value other than "0", leaves it on; OSCAD_BYTECODE_VM=0 is the
    // opt-out escape hatch. Flipped from the earlier default-off after
    // three rounds of real-world validation found zero regressions and a
    // consistent, measured (not assumed) ~20-30% wall-time win on a real
    // BOSL2-heavy stress script (Anklet.scad) -- see this phase's own
    // Instruments profile: total sampled CPU-busy time dropped from
    // 1.904s to 1.497s (VM off vs on, same optimized native-arm64 build),
    // composition unchanged (malloc/string-hash still dominate what's
    // left -- the VM's own dispatch overhead is a mere ~1.6% of runtime;
    // the win is from avoiding work, not from being fast itself), with no
    // new hotspot introduced. Deliberately process-wide via getenv, not a
    // constructor parameter, since threading a flag through every
    // existing test's/caller's Evaluator construction isn't practical
    // (see this phase's own plan notes on the differential-run validation
    // strategy this enables).
    static bool bytecodeVmEnabled();

    // -- Profiling (Phase 9) --------------------------------------------

    // (kind, name, callOrigin, callLine) -- the exact call-site identity a
    // CallSiteProfile aggregates by. std::map (not unordered_map): a plain
    // tuple of comparable fields already gets a free operator< from the
    // standard library, so this needs no custom hash function for a table
    // that's never more than a few thousand entries even on a large script.
    using ProfileSiteKey = std::tuple<std::string, std::string, std::string, int>;

    // Pushes profiling state for a user module/function call about to
    // start -- shared by evalUserModule/evalUserFunction/
    // evalFunctionLiteral so the timing/aggregation logic lives in one
    // place. Returns nullopt if profiling is off (the common case),
    // otherwise the key/recursion-flag/start-time an RAII-style caller
    // hands back to profileExit() on the matching pop. Mirrors
    // Evaluator._profile_enter.
    struct ProfileHandle {
        ProfileSiteKey key;
        bool recursiveReentry;
        std::chrono::steady_clock::time_point start;
    };
    std::optional<ProfileHandle> profileEnter(const std::string& kind, const std::string& name,
                                               const oscad::Position* callPos, const oscad::Position* declPos);
    void profileExit(const ProfileHandle& handle);

    // Lightweight sibling of profileEnter/profileExit for a single
    // trampolined tail-call hop (evalFunctionBodyTrampoline/
    // runCompiledFunctionTrampoline): only does the find-or-create-site +
    // callCount+=1 half, no timing and no profileActive_/profileChildTime_
    // touch. A tail chain only ever gets ONE real profileEnter/profileExit
    // bracket (the outer call evalUserFunctionCore already wraps, unchanged
    // by TCO) -- without this, every intermediate site visited only via a
    // tail-hop would show zero recorded calls at all, not just imprecise
    // timing, since it would never reach profileSites_. Wall time is
    // deliberately NOT sliced per hop -- it lumps onto the outermost call
    // as one simplification (every hop's own selfTime/cumulativeTime reads
    // 0.0 except the outermost); every hop's own callCount is accurate.
    void profileRecordTailHop(const std::string& kind, const std::string& name, const oscad::Position* callPos,
                               const oscad::Position* declPos);

    // The tree-build stack (mirrors the reference's self._tree_stack): each
    // frame is the children accumulator for whichever CSGNode is currently
    // being resolved (or, at the bottom, the top-level tree itself). A
    // resolve function that recurses into its own node's children (via
    // evalChildren -> evalModularCall/evalModifier/evalIntersectionForNode)
    // causes those children's completed CSGNodes to land in whatever frame
    // is on top *at that moment* -- the frame this node's own build call
    // pushed just before resolving. See csg_resolve.cpp for the full
    // push/call/pop sequence.
    std::vector<std::vector<std::unique_ptr<CSGNode>>> treeStack_;

    // User function/module call stack: (kind, name, call position, decl
    // position) per active call, innermost last. Drives callCtxFor's
    // closure detection and error()'s TRACE lines. Mirrors the reference's
    // self._call_stack (there, 4-tuples; here, CallStackFrame -- see
    // eval_error.hpp).
    std::vector<CallStackFrame> callStack_;

    EchoFn echoFn_;
    std::shared_ptr<FontProvider> fontProvider_; // null until first fontProvider() call if not injected
    std::shared_ptr<ManifoldCache> manifoldCache_; // opt-in, see the constructor's own doc comment
    std::uint64_t randsCallCount_ = 0; // see noteRandsCall()

    // -- Debugging (Phase 9) --------------------------------------------

    DebugHooks debugHooks_;
    // The root EvalContext resolveTree() was called with -- lets
    // checkDebug()'s locals snapshot fall back to top-level script
    // variables when paused inside a nested user call, the same way the
    // reference's own outer_scope merge does. Non-owning; valid only for
    // the duration of the resolveTree()/evaluate() call that set it (same
    // lifetime contract as every other raw AST/EvalContext pointer this
    // class holds).
    const EvalContext* rootCtx_ = nullptr;
    // The most recent ctx seen by evalStatement() -- error()'s only way to
    // reach a locals snapshot for its own errorBreak hook call, since
    // error() itself (called from dozens of builtin resolve functions)
    // doesn't take a ctx parameter. Mirrors the reference's `_last_ctx`
    // fallback exactly, including the same "only as fresh as the last
    // *statement* boundary" caveat (an error thrown from partway through a
    // function body's own expression evaluation sees the calling
    // statement's ctx, not the function body's).
    const EvalContext* lastCtx_ = nullptr;

    // -- Profiling (Phase 9) ----------------------------------------------

    bool profiling_ = false;
    std::map<ProfileSiteKey, CallSiteProfile> profileSites_;
    std::set<ProfileSiteKey> profileActive_; // site keys with a call currently on callStack_
    std::vector<double> profileChildTime_;   // parallel aux stack to callStack_
};

} // namespace oscadeval
