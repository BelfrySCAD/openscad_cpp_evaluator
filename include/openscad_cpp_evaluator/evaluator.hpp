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

#include <atomic>
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
#include <utility>
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
    // Read-only accessor for the same reason: Op::CallModule's own runtime
    // handler (bytecode_vm.cpp) needs its own "randsBefore" snapshot,
    // mirroring evalModularCall's own local of the same name.
    std::uint64_t randsCallCount() const { return randsCallCount_; }

    // Dispatches on node.kind() (switch, not a visitor -- matches
    // openscad_cpp_parser's own dispatch convention). Throws
    // std::logic_error for a NodeKind not yet implemented by this phase
    // (a builtin/unknown function call -- math/string/type-check builtins
    // land in Phase 5) rather than silently returning undef -- "not yet
    // supported" should fail loudly, not produce a plausible-looking wrong
    // answer.
    Value evalExpr(const oscad::Expression& node, EvalContext& ctx);

    // Statement-context sibling of evalExpr -- same result for every input,
    // just a possibly-compiled path for a STATEMENT-level expression
    // (assignment RHS, if/for condition, module-call or echo()/assert()
    // argument) instead of module/top-level code's ordinary AST walk. Not a
    // new evaluation semantics: looks up (compiling and caching on first
    // use, see stmtExprChunkCache_/tryCompileStatementExpr) a bytecode
    // chunk for `node`, runs it via runCompiledExprChunk when the VM is on,
    // this specific chunk is debugger-eligible right now (chunkEligibleNow,
    // the identical per-chunk gate every compiled function call already
    // goes through), AND a resolveTree()/evaluate() pass is actually
    // active (inResolvePass_ -- see stmtExprChunkCache_'s own doc comment
    // for why caching outside one is unsafe) -- falling straight back to
    // evalExpr(node, ctx) otherwise. Compilation failing, the VM being off,
    // a debugger needing this exact span, or no active pass are all
    // silently equivalent to "just interpret it," same as functions' own
    // fallback.
    Value evalExprMaybeCompiled(const oscad::Expression& node, EvalContext& ctx);

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

    // Best-effort generate over whatever's been resolved SO FAR, for a live
    // partial render while genuinely paused inside a debug hook mid-
    // evaluation (resolveTree()/evaluate() haven't returned yet) -- flattens
    // treeStack_ across every nesting level and runs the same bottom-up
    // generate walk generateTree() uses, via generateTreeImpl(), over
    // non-owning pointers into nodes treeStack_ (not this call) still owns.
    // Safe to call mid-resolve: the resolve pass never reads CSGNode::bodies,
    // only ever writes kind/params/children, so populating .bodies early
    // here cannot corrupt or race the resolve that's paused around this
    // call -- and if a ManifoldCache is set, the real generateTree() call at
    // the end of evaluate() gets a cache hit for any subtree this already
    // generated, so a debug session pays no repeated Manifold work session-
    // wide (see manifold_cache.hpp's own doc comment on this exact use
    // case). Only meaningful to call synchronously from within a
    // DebugHookFn/ErrorBreakFn callback; the caller must not retain a
    // reference into anything treeStack_ owns past that callback's return.
    // Mirrors the reference's `ev.generate_tree(partial_nodes)` called from
    // `_generate_partial_render`.
    std::vector<ColoredBody> generatePartialTree();

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
    // The call site that entered the current call chain from the top
    // level (callStack_ is outermost-first), or nullptr at top level.
    // Captured into each CSGNode at resolve time so a warning raised
    // during generate -- after the stack has unwound -- can still name the
    // user's own line. See CSGNode::warnEntry.
    const oscad::Position* currentWarnEntry() const {
        return callStack_.empty() ? nullptr : callStack_.front().callPosition;
    }

    // Set by generateTreeImpl() to the CSGNode currently being generated,
    // so warn() can name the user's own call site during a phase where
    // callStack_ is necessarily empty. Public for the same reason
    // tagGenerated() is: a GenerateFn is a free function holding only an
    // Evaluator&. Read only by warn(); a plain scoped save/restore rather
    // than anything reentrancy-aware, since generateTreeImpl recurses
    // depth-first on one thread.
    const oscad::Position* generateWarnEntry = nullptr;

    ColoredBody tagGenerated(manifold::Manifold body, const oscad::ASTNode& node, const Value& colorValue);

    // tagGenerated()'s counterpart for a mesh Manifold refused to build --
    // an open polyhedron(). Returns a display-only ColoredBody (see
    // ColoredBody::rawMesh) carrying the triangle soup as-is, so it can
    // still be drawn even though no CSG operation can ever touch it.
    //
    // A failed Manifold has no run IDs of its own, so one is reserved here
    // and written onto the raw mesh -- otherwise the body would be
    // invisible to selection and to the originalID -> AST node mapping
    // that drives click-to-source, which is exactly when a user most wants
    // to click the broken thing and be shown the line that made it.
    ColoredBody tagDisplayOnly(manifold::MeshGL mesh, const oscad::ASTNode& node, const Value& colorValue);

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

    // The "which nodes, evaluated against what context" half of
    // builtinChildren, shared with Op::CallChildren's runtime handler
    // (bytecode_vm.cpp) so the native and compiled paths cannot drift on
    // any of the subtle parts: the caller-ctx re-derivation, the
    // $-forwarding loop (which must read the post-resolveCallArgs effCtx
    // -- a children($fn=12) named-$ override lives only there), and the
    // children(N) statement-index filtering/bounds-check. nullopt means
    // "nothing to evaluate" (no forwarded children in scope, empty list,
    // or index out of range) -- a silent no-op in every existing caller,
    // exactly matching builtinChildren's own early returns. `ctx` must be
    // the effCtx resolveCallArgs returned, same as builtinChildren's own
    // parameter today. Public for the same free-function reasoning as
    // builtinChildren itself.
    struct ChildrenForward {
        EvalContext evalCtx;
        std::vector<const oscad::ASTNode*> nodes;
    };
    std::optional<ChildrenForward> prepareChildrenForward(const CallArgs& args, EvalContext& ctx);

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
    Value evalFunctionLiteralFromBound(const Closure& closure, BoundArgs bound, EvalContext& ctx,
                                        const oscad::Position* callPos);

    // Testing-only override, checked before the (once-cached) env var --
    // lets test_bytecode_compiler.cpp force VM-on for specific tests
    // without depending on process-startup environment (the env-var read
    // in bytecodeVmEnabled() is cached forever after its first call, so a
    // plain setenv() from inside a test body would have no effect on later
    // calls within the same test binary). Pass std::nullopt to go back to
    // the env var.
    static void setBytecodeVmEnabledForTesting(std::optional<bool> enabled);

    // Tells this Evaluator it's safe to speed up function calls with the
    // bytecode VM even though a debugger is attached, PROVIDED the caller
    // has already verified none of the reasons that's normally unsafe
    // apply right now: no step command (`into`/`over`/`out`/`to_child`)
    // pending, no explicit pause requested, and the session's own initial
    // break-on-first stop already consumed. `breakpoints` is exactly the
    // caller's own current breakpoint set (origin file -> line numbers) --
    // see useBytecodeVm()/chunkEligibleNow's own doc comments for how it's
    // used (a per-function span check, not a blanket switch: a function
    // whose own body could contain one of these lines still always runs
    // interpreted). Call with std::nullopt to go back to "no debugger
    // exception at all" (today's original, always-safe behavior) --
    // e.g. the instant a step command starts or a pause is requested.
    // Cheap to call as often as the debugger's own state changes: this
    // just updates two members, no cache invalidation needed (chunkEligibleNow/
    // checkDebug are re-checked on every call, not cached themselves -- only
    // the compiled bytecode ITSELF, which never depends on debugger state,
    // is cached forever).
    //
    // `hookSkippable`: a STRICTLY narrower claim than "breakpoints is
    // accurate" above -- true only when NOTHING needs to inspect a
    // statement-level checkpoint's own line/depth at all, not just "nothing
    // needs to inspect it unless it's a compiled function's entry." This is
    // false for step_over/step_out even though THEY also pass a real
    // breakpoints set (chunkEligibleNow/useBytecodeVm still apply for them):
    // both need checkDebug() to keep calling into the debug hook on every
    // statement so the caller's own step_hit logic (line/depth comparison
    // against the step's own starting point) can run -- there is no way to
    // decide that in advance the way a breakpoint LOCATION can be. Only a
    // plain "Continue" with no step pending can safely skip the call
    // entirely for a line with no breakpoint -- see checkDebug's own doc
    // comment (debug_profile.cpp) for where this is actually consulted.
    void setFastContinueBreakpoints(std::optional<std::unordered_map<std::string, std::set<int>>> breakpoints,
                                     bool hookSkippable = false) {
        fastContinueBreakpoints_ = std::move(breakpoints);
        fastContinueHookSkippable_ = hookSkippable;
    }

    // The other half of hook-skippable mode's safety net. checkDebug()'s
    // whole premise (see its own doc comment, debug_profile.cpp) is that it
    // can skip calling into Python for a line with no breakpoint -- but the
    // caller that decided that (DebugSession, on the MAIN/GUI thread) needs
    // a way to say "actually, don't skip the very next one" from OUTSIDE any
    // hook call, since debug_evaluate() runs as one single blocking call
    // with the GIL released for its whole duration: there is no live,
    // Python-callable Evaluator handle to invoke setFastContinueBreakpoints
    // on directly the way a synchronous API would allow. A user clicking
    // Pause, or toggling a breakpoint in an editor tab, while a hook-
    // skippable render is mid-flight needs to take effect on the very next
    // checkpoint, not "whenever a breakpoint happens to be hit next" (which,
    // in hook-skippable mode, could be never, for a script with none set).
    //
    // `flag` is a plain shared_ptr<atomic<bool>> -- lock-free and GIL-free
    // by construction, so the caller (bindings/module.cpp wraps it in a
    // small Python-visible class) can set it from the main thread at any
    // moment with no synchronization needed beyond the atomic itself.
    // checkDebug() atomically test-and-clears it (exchange) on every call
    // that would otherwise skip: if it was set, this call falls through and
    // actually invokes the Python hook instead, which re-derives and pushes
    // fresh breakpoints/hookSkippable state via its own existing logic --
    // clearing it here (not from Python) means there's no separate
    // "acknowledge" round-trip needed. Never itself compared against
    // anything else; nullptr (the default, and what a plain debugger
    // attach that never wires this up leaves it at) simply means hook-
    // skippable mode -- if ever engaged at all -- can't be interrupted this
    // way, which is only actually reachable if a caller opts into
    // hookSkippable=true above without also providing this.
    void setFastContinueInterruptFlag(std::shared_ptr<std::atomic<bool>> flag) {
        fastContinueInterrupt_ = std::move(flag);
    }

    // Checks whether a debug pause should happen at `node` (via the
    // injected DebugHooks::debugHook, if any -- a no-op otherwise),
    // applying any `mods` the hook returns to `ctx.let_` and throwing
    // EvalError if it returns stop=true. `forced=true` bypasses nothing on
    // this port's side (unlike the reference's own step/breakpoint-line
    // filtering, which lives entirely in the injected hook, not here) --
    // it's simply passed through so the hook itself can distinguish an
    // unconditional breakpoint()/function-entry pause from a normal
    // statement-boundary check. `exprLevel=true` marks a sub-statement
    // (expression-granularity) checkpoint that a stepping debugger should
    // not treat as a statement boundary -- see debug_hooks.hpp's
    // DebugHookFn doc comment for the full list of call sites and the
    // exprLevel contract. Public: called from evalChildren() (every
    // ordinary statement), evalUserFunction()/evalFunctionLiteral()
    // (function body entry), and builtins/control.cpp's
    // resolveBreakpoint/resolveIntersectionFor (free functions). Mirrors
    // Evaluator._check_debug, including its parameter defaults.
    void checkDebug(const oscad::ASTNode& node, EvalContext& ctx, bool forced = false, bool exprLevel = false);

    // (origin, line) for each top-level, non-declaration child of the
    // node checkDebug() was just called with -- i.e., if that node is a
    // ModularCall with its own `{ ... }` block, the source positions
    // children()/children(N) might forward control to. Recomputed
    // unconditionally on every checkDebug() call (nullopt if the checked
    // node isn't a ModularCall, or has no non-declaration children) and
    // stashed here rather than threaded through DebugHookFn's own
    // signature, so adding it didn't change that callback contract.
    // Public so debug_repl.cpp's "child" (step-to-child) command can read
    // it via the Evaluator reference DebugRepl::attachEvaluator() wires in
    // -- mirrors the reference's Evaluator._last_children_positions
    // exactly, including the "read via a stashed field, not a parameter"
    // rationale (see that field's own doc comment).
    const std::optional<std::vector<std::pair<std::string, int>>>& lastChildrenPositions() const {
        return lastChildrenPositions_;
    }

    // Whole-evaluate() profiling summary (see profile.hpp), populated by
    // evaluate() when constructed with profiling=true; nullopt otherwise
    // (including for direct resolveTree()/generateTree() callers that
    // bypass evaluate() -- profiling only ever wraps the exact
    // resolve+generate bracket evaluate() itself runs). Mirrors the
    // reference's Evaluator.profile_result.
    std::optional<ProfileResult> profileResult;

    // The resolved (and, after evaluate() runs, generated) CSG tree from the
    // most recent evaluate() call -- moved here from evaluateImpl's local
    // `tree` after generateTree() populates each node's own `bodies`, so a
    // caller can inspect tree shape/params after evaluate() returns (CSG-tree
    // dump, live partial-render). Empty for direct resolveTree()/
    // generateTree() callers that bypass evaluate() -- same scoping as
    // profileResult above. Mirrors the reference's Evaluator.csg_tree.
    std::vector<std::unique_ptr<CSGNode>> csgTree;

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

    // Shared bottom-up generate walk behind generateTree()/generatePartialTree()
    // -- see generatePartialTree()'s own doc comment and this function's
    // definition (csg_generate.cpp) for why non-owning pointers, not
    // unique_ptr, let both callers share one implementation.
    std::vector<ColoredBody> generateTreeImpl(const std::vector<CSGNode*>& tree);

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

    // Bracketed public in place: Stage 2's Op::NativeStatement
    // (bytecode_vm.cpp, a separate translation unit) needs it directly to
    // run one native-passthrough statement from inside an otherwise-
    // compiled module body -- exactly the same dispatch evalChildren's own
    // per-statement loop already does.
public:
    void evalStatement(const oscad::ASTNode& node, EvalContext& ctx);
private:
    void evalAssignment(const oscad::Assignment& node, EvalContext& ctx);
    // Tries to compile+run `assignments` (evalChildren's own leading
    // assignment-run, stmt_eval.cpp) as ONE chunk via
    // tryCompileAssignmentBlock/runCompiledAssignmentBlock -- see their own
    // doc comments (bytecode_compiler.hpp/bytecode_vm.hpp) and
    // assignBlockChunkCache_'s, above. Returns false (nothing run, `ctx`
    // untouched) whenever compiling/running the WHOLE batch this way isn't
    // possible or eligible right now, so the caller can fall back to its
    // existing per-statement loop -- exactly evalExprMaybeCompiled's own
    // "silently equivalent to interpreting it" contract, just for a whole
    // block instead of one leaf expression.
    bool tryRunCompiledAssignmentBlock(const std::vector<const oscad::ASTNode*>& assignments, EvalContext& ctx);
    // Same contract as tryRunCompiledAssignmentBlock, above, but for
    // evalChildren's own FULL `children` list (not just its leading
    // assignment run) -- see tryCompileChildrenList's own doc comment
    // (bytecode_compiler.hpp) for why every evalChildren() call site
    // tries this FIRST now, including a builtin module's own children
    // (translate()/union()/etc.) and a for-loop's/let-block's body, not
    // just module/top-level bodies. `ctx` is used directly (not a scoped
    // child), matching runCompiledModuleBody's own contract -- a
    // children-list chunk's own writes (assignments, $-vars) must persist
    // into the caller's scope exactly the way the native per-statement
    // loop's own writes already do.
    bool tryRunCompiledChildren(const std::vector<const oscad::ASTNode*>& children, EvalContext& ctx);
public:
    // The cache-lookup half of tryRunCompiledChildren, shared with
    // Op::CallChildren's runtime handler (bytecode_vm.cpp, a free
    // function -- public for the same no-friend-declaration reasoning as
    // vmCallStack_/treeStack_). Returns the eligible compiled chunk for
    // `children` or nullptr. Caller owns the useBytecodeVm()/
    // inResolvePass_ gate -- the pass gate is load-bearing
    // (childrenListChunkCache_ is pass-scoped, see its own doc comment),
    // not defensive.
    const CompiledChunk* lookupOrCompileChildrenListChunk(const std::vector<const oscad::ASTNode*>& children);
private:
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

public:
    // Sets node.treeDepth from its own (already-finalized) `children`,
    // throwing (kMaxCsgTreeDepth's own doc comment, above, for why) if
    // it's now too deep. Called by every csg_resolve.cpp site that
    // finalizes a CSGNode's own `children` -- buildTreeNode,
    // evalModularCall's non-splice tail, spliceModuleChildren's
    // union-wrap branch -- right after assigning `children`, before the
    // node is ever linked into anything a caller could see. Public (not
    // alongside buildTreeNode/evalModularCall, just above, which stay
    // private member functions): Op::PopBuiltinWrap's own runtime handler
    // (bytecode_vm.cpp, a free function, same "needs direct access, no
    // friend declaration in play" reasoning as treeStack_/randsCallCount()
    // above) calls this directly, replicating buildTreeNode's own
    // post-resolveBody() half rather than going through it (the "body"
    // there already ran as bytecode, not a callback buildTreeNode could
    // invoke).
    void setTreeDepthOrThrow(CSGNode& node, const oscad::ASTNode& errNode);
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
    // `capturedLet`: non-null only for a FunctionLiteral *value* callee
    // (see Closure's doc comment, value.hpp) -- when the closure-nesting
    // check below comes back false (the defining call has already
    // returned -- an ESCAPED closure), the isolated call context is rooted
    // at `capturedLet` instead of `ctx`'s own trail, so the closure's
    // captured variables stay resolvable. Ignored (as if null) when the
    // check comes back true: an actually-still-live enclosing call is
    // already reachable via `ctx`'s own ancestry, the ordinary path.
    EvalContext callCtxFor(const oscad::ASTNode& decl, EvalContext& ctx, const oscad::Scope* scope,
                            std::shared_ptr<const ChildrenNodeList> childrenNodes = nullptr,
                            const EvalContext* childrenCallerCtx = nullptr, bool* usedChildCtx = nullptr,
                            const std::shared_ptr<TrailView<Value>>& capturedLet = nullptr);

private:
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

    // One frame's snapshot: `let` + `$`-prefixed `dyn` entries from `ctx`
    // (localScope, with dynNames = the let-bound subset), plus (when
    // `includeOuter` and inside a call) unshadowed top-level script vars
    // (outerScope); `locals` is the merged view. `ctx` may be null (returns
    // an empty frame then).
    DebugFrame buildDebugFrame(const EvalContext* ctx, bool includeOuter) const;

    // The whole call stack's per-frame snapshots, innermost first: frame 0
    // from `ctx` (the paused statement's scope, with outerScope), then each
    // enclosing call's own bodyCtx, then a top-level frame when inside a
    // call. Mirrors the reference's _build_frame_locals' all_frame_locals.
    std::vector<DebugFrame> buildDebugFrames(const EvalContext* ctx) const;

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
    //
    // Templated on the callable rather than taking `const
    // std::function<Value()>&`: every call site passes a `[&]() -> Value
    // {...}` lambda capturing several variables by reference, which
    // std::function's small-buffer optimization can't always absorb --
    // erasing its type would then cost a real heap allocation on every
    // single user function call (hundreds of thousands in a BOSL2-heavy
    // script), just to invoke a callback exactly once, synchronously, in
    // the same stack frame. A template parameter keeps the concrete
    // closure type all the way through, so the compiler can call (and
    // often inline) `computeResult()` directly -- no erasure, no
    // allocation, same one-call-site-per-caller shape as before.
    // A synchronous call-and-block wrapper around enterUserCall/
    // exitUserCallSuccess/exitUserCallException (public, below) -- kept as
    // the ordinary entry point for the tree-walking interpreter's own
    // recursive call sites (user_calls.cpp), which genuinely do want "call
    // and block for the result" semantics. The explicit-frame-stack VM
    // driver (bytecode_vm.cpp) calls the two halves directly instead,
    // since ITS whole point is a compiled-to-compiled call must NOT block
    // a native frame waiting for the callee -- see enterUserCall's own doc
    // comment for the split rationale. This wrapper's own behavior is
    // byte-for-byte identical to the pre-split version; the split changed
    // nothing observable here.
    template <typename F>
    Value evalUserFunctionCore(const std::string& name, const oscad::ASTNode& declNode, const oscad::Expression& bodyExpr,
                                EvalContext& childCtx, const oscad::Position* callPos, int upvalueParent,
                                F&& computeResult) {
        UserCallHandle h = enterUserCall(name, declNode, &bodyExpr, childCtx, callPos, upvalueParent);
        Value result;
        try {
            result = computeResult();
        } catch (...) {
            exitUserCallException(h);
            throw;
        }
        exitUserCallSuccess(name, h, result);
        return result;
    }

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
                                            EvalContext& ctx, const oscad::Position& callPos,
                                            const std::shared_ptr<TrailView<Value>>& capturedLet = nullptr);

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
    // Bracketed public in place: Op::CallModule's own runtime handler and
    // driveVm's module-frame completion branch (bytecode_vm.cpp, a
    // separate translation unit) need buildModuleChildCtx/
    // runModuleBodyNative/lookupOrCompileModuleChunk/spliceModuleChildren
    // directly -- the same reason callCtxFor/enterUserCall/useBytecodeVm
    // were bracketed public for Stage 1's own driver.
public:
    // The shared setup evalUserModule's own compiled-or-native dispatch AND
    // Op::CallModule's runtime handler (bytecode_vm.cpp) both need,
    // factored out so neither duplicates it: resolves the module's own
    // child scope, builds childrenNodes, derives childCtx via callCtxFor,
    // sets $children/$parent_modules, binds `bound`'s entries and applies
    // defaults. Exactly what evalUserModule used to do inline before Stage
    // 2 split it out -- see this file's own git history for the pre-split
    // version if a byte-for-byte diff is ever needed.
    EvalContext buildModuleChildCtx(const oscad::ModuleDeclaration& decl, const oscad::ModularCall& call,
                                     EvalContext& ctx, BoundArgs bound);
    // Runs `decl`'s own body natively against the already-prepared
    // `childCtx` -- depth guard, callStack_/profiling bracket (Kind::
    // Module), evalChildren, teardown. This is exactly what evalUserModule
    // used to do unconditionally before Stage 2 taught it to try a
    // compiled chunk first (see lookupOrCompileModuleChunk) -- still the
    // fallback whenever `decl` doesn't compile, called both from
    // evalUserModule itself and from Op::CallModule's own "callee doesn't
    // compile" branch.
    void runModuleBodyNative(const oscad::ModuleDeclaration& decl, EvalContext& childCtx, const oscad::Position* callPos);
    // Compile-attempt cache for module bodies, mirroring chunkCache_
    // exactly (nullopt = tried, doesn't compile). See tryCompileModuleBody
    // (bytecode_compiler.hpp).
    const CompiledChunk* lookupOrCompileModuleChunk(const oscad::ModuleDeclaration& decl);
    // Taints every one of `children` if `randsBefore` differs from the
    // current randsCallCount_, then either wraps >1 sibling into a
    // synthetic "union" CSGNode (stamped with `callNode`) or splices each
    // child directly into treeStack_.back() -- the CALLER's own frame,
    // already exposed at the top of treeStack_ by the time this runs
    // (`children` itself came from popping the CALLEE's own frame, already
    // done by the caller before this runs -- see each call site). Factored
    // out of evalModularCall's own post-processing (csg_resolve.cpp) so
    // Op::CallModule's own frame-completion (bytecode_vm.cpp's driveVm)
    // can reuse it exactly -- always the splice branch in practice for a
    // caller that reached here via a resolved user-module call
    // (evalModularCall's OWN "is this splice or wrap-as-a-tagged-node"
    // branch already decided `splice` before ever reaching a user module,
    // so this helper never needs that decision itself).
    void spliceModuleChildren(std::vector<std::unique_ptr<CSGNode>> children, std::uint64_t randsBefore,
                               const oscad::ASTNode& callNode);
private:
    Value evalUserFunction(const std::string& name, const oscad::FunctionDeclaration& decl,
                            const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                            const oscad::ASTNode* callNode);
    Value evalFunctionLiteral(const Closure& closure,
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

    // Same, for module bodies (Stage 2) -- see tryCompileModuleBody
    // (bytecode_compiler.hpp) and lookupOrCompileModuleChunk's own doc
    // comment, above.
    std::unordered_map<const oscad::ModuleDeclaration*, std::optional<CompiledChunk>> moduleChunkCache_;

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

    // Statement-context expression chunks (Phase 1, module/top-level
    // compilation) -- see tryCompileStatementExpr's own doc comment
    // (bytecode_compiler.hpp) and evalExprMaybeCompiled's, below. Keyed by
    // expression node identity, same nullopt-means-tried-and-failed
    // convention as chunkCache_ above (an expression using a construct this
    // wrapper doesn't support isn't re-attempted on every statement it's
    // part of).
    //
    // UNLIKE chunkCache_/literalChunkCache_ (keyed by a FunctionDeclaration/
    // FunctionLiteral -- a NAMED, script-lifetime declaration), a bare
    // statement-context Expression node has no such guarantee: it's only
    // alive as long as the AST it's part of. A caller that re-parses and
    // re-evaluates DIFFERENT, independently-lived expressions against one
    // reused Evaluator (several existing tests do exactly this, and it's
    // not implausible for real usage -- e.g. a debugger REPL evaluating an
    // ad-hoc watch expression) can free one Expression node and have a
    // LATER parse's allocator hand back the exact same address for an
    // unrelated node -- stmtExprChunkCache_.find() would then silently
    // return a chunk compiled for the WRONG expression. Caught for real:
    // MathBuiltins.AbsSignCeilFloorRound-style tests returning values from
    // an entirely different, already-freed call's own constant.
    //
    // Kept safe by TWO measures, both required: resolveTreeImpl (csg_
    // resolve.cpp) clears this at the start of every top-level resolveTree()/
    // evaluate() call (bounds staleness to "within one render pass" even
    // when a host reuses one Evaluator across separate, re-parsed renders),
    // and evalExprMaybeCompiled only reads/writes it at all while
    // inResolvePass_ is true (bounds it away entirely for any direct
    // evalExpr()-family call made outside a resolveTree()/evaluate() pass --
    // exactly the ad-hoc/test-helper shape above, which has no pass
    // boundary to tie cache validity to in the first place).
    std::unordered_map<const oscad::Expression*, std::optional<CompiledChunk>> stmtExprChunkCache_;

    // Assignment-BLOCK chunks (see tryCompileAssignmentBlock's own doc
    // comment, bytecode_compiler.hpp, and tryRunCompiledAssignmentBlock's,
    // below) -- a whole run of sibling assignments compiled as ONE chunk,
    // not stmtExprChunkCache_'s one-chunk-per-leaf-expression (whose own
    // per-call overhead turned out to dominate for a typical block of
    // several small assignments -- see this feature's own commit message
    // for the measured regression that motivated batching them instead).
    // Keyed by the run's own FIRST assignment -- stable and unique per
    // block: evalChildren's own assignments/others split is deterministic
    // over a fixed AST, so a given block's leading-assignment-run always
    // has the same first member across repeated evaluations. Same nullopt-
    // means-tried-and-failed convention, same dangling-pointer hazard and
    // same two-part fix (cleared alongside stmtExprChunkCache_ at the top
    // of every resolveTreeImpl call, only read/written while
    // inResolvePass_) as stmtExprChunkCache_ itself -- see its own doc
    // comment above for the full reasoning.
    std::unordered_map<const oscad::Assignment*, std::optional<CompiledChunk>> assignBlockChunkCache_;

    // Whole-CHILDREN-LIST chunks (see tryCompileChildrenList's own doc
    // comment, bytecode_compiler.hpp, and tryRunCompiledChildren's,
    // below) -- every evalChildren() call now tries the FULL list it was
    // given as one chunk first (not just its own assignments sub-list),
    // including a builtin module's own children (translate()/union()/
    // etc.'s internal evalChildren call) and a for-loop's/let-block's
    // body -- so a resolvable module call anywhere in that list gets
    // Op::CallModule bytecode, and any if/for control flow around it gets
    // real Jump-based bytecode too, instead of falling through to the
    // native per-statement loop.
    //
    // Keyed by (FIRST element, list SIZE) -- NOT the first element alone,
    // the original convention borrowed from assignBlockChunkCache_
    // (where it IS sufficient: a leading-assignment run's first member
    // uniquely determines the run). Here it wasn't: children()'s
    // forwarding produces two DIFFERENT lists sharing a first element --
    // bare `children()` forwards the caller's whole list, `children(0)`
    // a single-element slice of it -- and keying by front alone made
    // whichever form ran first poison the cache for the other (caught
    // for real: `module m() { children(); children(0); }\n
    // m() { cube(); sphere(); }` made children(0) emit BOTH shapes,
    // or bare children() emit only one, depending on statement order).
    // (front, size) fully disambiguates every real producer: fixed AST
    // lists have unique fronts per call site, and the only same-front
    // pair (bare vs indexed forwarding) always differs in size except
    // when the lists are literally identical anyway. Same nullopt-means-
    // tried-and-failed convention, same dangling-pointer hazard, same
    // two-part fix (cleared alongside stmtExprChunkCache_ at the top of
    // every resolveTreeImpl call, only read/written while inResolvePass_)
    // as stmtExprChunkCache_ itself.
    struct ChildrenListKeyHash {
        size_t operator()(const std::pair<const oscad::ASTNode*, size_t>& k) const {
            // Standard hash-combine (boost-style golden-ratio mix) --
            // either half alone collides by construction here.
            const size_t h1 = std::hash<const oscad::ASTNode*>{}(k.first);
            const size_t h2 = std::hash<size_t>{}(k.second);
            return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
        }
    };
    std::unordered_map<std::pair<const oscad::ASTNode*, size_t>, std::optional<CompiledChunk>, ChildrenListKeyHash>
        childrenListChunkCache_;

    // See stmtExprChunkCache_'s own doc comment, immediately above, for why
    // this exists. Not a reentrancy guard (resolveTreeImpl is never called
    // while another one is already active), just an on/off switch for
    // whether evalExprMaybeCompiled's cache is safe to touch right now.
    bool inResolvePass_ = false;

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
    // `capturedLet`: see callCtxFor's own doc comment -- pass the callee
    // closure's captured trail for a FunctionLiteral-value callee (nullptr
    // for a named FunctionDeclaration callee, which has none).
    std::optional<EvalContext> isolatedCallCtxFor(const oscad::ASTNode& declNode, EvalContext& ctx,
                                                   const std::shared_ptr<TrailView<Value>>& capturedLet = nullptr);

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

    // Per-evaluator gate actually consulted before using a compiled chunk.
    // Compiled bytecode has no per-AST-node debug checkpoints (its whole
    // point is to flatten those nodes away), so a function body run on the
    // VM would silently skip every checkDebug() call site the interpreter
    // path makes -- ternary branches, let()/echo()/assert() expression
    // forms, list-comprehension clauses, nested call sites. Debugging
    // therefore always takes the interpreter path, where the port matches
    // the reference's _check_debug placement exactly. Costs nothing when
    // no debugger is attached, which is every non-debug render.
    //
    // EXCEPT: a caller that knows none of that applies right now -- no
    // step command pending, no explicit pause requested, the session's
    // own initial break-on-first stop already consumed -- can call
    // setFastContinueBreakpoints() with the CURRENT breakpoint set instead
    // of leaving it unset. That alone doesn't turn the VM on for
    // everything; see chunkEligibleNow, which still forces the
    // interpreter for any specific function whose own compiled span could
    // contain one of those lines. This only ever helps a plain "Continue,
    // pause only at a known breakpoint" -- the far more common case in a
    // long debug session than active single-stepping, and exactly the
    // case where "every function pays the interpreter tax because a
    // debugger happens to be attached somewhere" wastes the most time
    // (see docs/debugger.md and this method's own callers for the full
    // story).
public:
    // Public (not just a private helper of evalUserFunction*/
    // evalFunctionLiteral* anymore): the explicit-frame-stack driver
    // (bytecode_vm.cpp, a separate translation unit) needs this same gate
    // for a NESTED call made from within already-compiled code, exactly
    // the same decision evalUserFunction*/evalFunctionLiteral* already
    // make before calling lookupOrCompileChunk/lookupCompiledLiteralChunk
    // themselves.
    bool useBytecodeVm() const {
        return bytecodeVmEnabled() && (!debugHooks_.debugHook || fastContinueBreakpoints_.has_value());
    }

    // Read accessor for inResolvePass_ (private, below) -- Op::CallChildren's
    // runtime handler (bytecode_vm.cpp, a free function) must gate its
    // childrenListChunkCache_ access on it, exactly like
    // tryRunCompiledChildren's own self-gate: the cache is pass-scoped
    // (cleared per resolveTreeImpl, AST-address-reuse hazard -- see
    // stmtExprChunkCache_'s own doc comment), and driveVm CAN run outside
    // the resolve pass (a host calling evalChildren directly reaches
    // module opcodes via lookupOrCompileModuleChunk, which is NOT
    // pass-scoped).
    bool inResolvePass() const { return inResolvePass_; }

private:
    // The fine-grained half of the check above: even when useBytecodeVm()
    // says compiling/using bytecode is on the table at all, a SPECIFIC
    // chunk is only actually safe to run compiled if nothing about it
    // could matter to the debugger right now. No debugger attached at
    // all -- always fine, unaffected by any of this. Debugger attached: only
    // safe in fast-continue mode (useBytecodeVm() already checked that),
    // and only if the chunk's own [minLine, maxLine] span (see
    // CompiledChunk's own doc comment, bytecode.hpp) contains none of the
    // breakpoint lines currently set for its origin file -- a checkpoint
    // for one of those lines could fire mid-body, and compiled code has no
    // way to stop there.
    bool chunkEligibleNow(const CompiledChunk& chunk) const {
        if (!debugHooks_.debugHook) return true;
        if (!fastContinueBreakpoints_) return false;
        auto it = fastContinueBreakpoints_->find(chunk.origin);
        if (it == fastContinueBreakpoints_->end()) return true;
        for (int line : it->second) {
            if (line >= chunk.minLine && line <= chunk.maxLine) return false;
        }
        return true;
    }

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

public:
    // -- User-call bracket, split into enter/exit halves ------------------
    // (explicit-frame-stack VM groundwork)
    //
    // evalUserFunctionCore (above) used to inline this whole bracket
    // (depth-guard check, profileEnter, callStack_ push, checkDebug,
    // computeResult() called SYNCHRONOUSLY in the same native frame,
    // returnHook, callStack_ pop, profileExit) as one block wrapping a
    // caller-supplied lambda. That shape is fine for the tree-walking
    // interpreter, which always wants genuine call-and-block semantics --
    // but it can't be reused by an explicit-frame-stack bytecode VM driver
    // (bytecode_vm.cpp), whose entire point is that a compiled-to-compiled
    // call must NOT block a native C++ frame waiting for the callee's
    // result; the driver instead needs to PUSH a new frame and return
    // control to its own outer loop, applying "what happens on return"
    // later, when that pushed frame's own execution reaches its end.
    //
    // Split into two explicit halves so both callers -- evalUserFunctionCore
    // itself (a thin synchronous wrapper around them, see above, kept for
    // the interpreter's own recursive call sites) and the VM driver's own
    // push/pop points -- can each drive the SAME bracket logic without
    // duplicating it. `enterUserCall` does everything up to and including
    // the body-entry checkDebug() stop; `exitUserCallSuccess`/
    // `exitUserCallException` do the matching teardown, mirroring
    // evalUserFunctionCore's own try/catch split exactly (returnHook fires
    // ONLY on the success path, matching today's behavior precisely).
    struct UserCallHandle {
        std::optional<ProfileHandle> prof;
        // Mirrors the pushed CallStackFrame's own kind -- exitUserCall*
        // needs it to decrement moduleCallDepth_ correctly without
        // re-reading callStack_ (already popped by the time exitUserCall*
        // runs). See moduleCallDepth_'s own doc comment for why this
        // exists at all.
        CallStackFrame::Kind kind = CallStackFrame::Kind::Function;
        // The declaration this call pushed, for the SAME reason as
        // `kind` above: exitUserCall* needs it to decrement
        // activeDeclRefcount_'s own count for exactly this declaration,
        // and callStack_.back() is already gone by the time it runs. See
        // activeDeclRefcount_'s own doc comment.
        const oscad::ASTNode* declNode = nullptr;
        // True only when THIS call incremented nativeUserCallDepth_ (i.e.
        // skipDepthGuard was false) -- exitUserCall* needs it to decrement
        // symmetrically, for the same "callStack_.back() is already gone"
        // reason as `kind`/`declNode` above. See nativeUserCallDepth_'s
        // own doc comment for why this can't just be "always decrement".
        bool countedTowardNativeDepth = false;
    };
    // `skipDepthGuard`: kMaxUserCallDepth=50 exists purely to keep the
    // NATIVE C++ stack from overflowing (see its own doc comment) -- true
    // only for pushBracketedCallFrame's/pushBracketedModuleFrame's own
    // calls (bytecode_vm.cpp), a compiled-to-compiled push that never
    // grows the native stack at all (it's serviced by driveVm's own loop,
    // bounded instead by kMaxVmCallStackDepth, checked separately by the
    // caller before this runs). evalUserFunctionCore's own call (the
    // interpreter-fallback boundary, still a genuine native recursive
    // call either way) always passes false, unchanged. `kind`: Function
    // for every existing call site; Module for pushBracketedModuleFrame's
    // own (Stage 2) -- selects both the CallStackFrame::Kind pushed (so
    // $parent_modules/backtraces count a compiled module call exactly
    // like a native evalUserModule one) and the profiling site's own
    // "function"/"module" label. `bodyExpr`: nullptr for a module call --
    // unlike a function, the native evalUserModule path fires no
    // "body entry" checkDebug of its own at all (each of the module's own
    // STATEMENT children gets one instead, via Op::NativeStatement's own
    // runtime handler / evalChildren's per-statement loop) -- passing a
    // real bodyExpr here for a module call would add a stop that never
    // existed before.
    UserCallHandle enterUserCall(const std::string& name, const oscad::ASTNode& declNode,
                                  const oscad::Expression* bodyExpr, EvalContext& childCtx,
                                  const oscad::Position* callPos, int upvalueParent, bool skipDepthGuard = false,
                                  CallStackFrame::Kind kind = CallStackFrame::Kind::Function);
    // `fireReturnHook`: false only for a module frame's own completion --
    // the native evalUserModule path never calls debugHooks_.returnHook
    // either (a module call has no "return value" concept the debugger
    // reports), so a compiled module call must not start doing so just
    // because it happens to reuse this same exit path.
    void exitUserCallSuccess(const std::string& name, const UserCallHandle& handle, const Value& result,
                              bool fireReturnHook = true);
    void exitUserCallException(const UserCallHandle& handle);

    // Decrements activeDeclRefcount_[declNode], erasing the entry once it
    // reaches 0 (keeps the map's own size bounded by the number of
    // CURRENTLY active distinct declarations, not the number ever seen) --
    // shared by exitUserCallSuccess/exitUserCallException/
    // recordTailCallHop's own "retire the old declaration" half.
    void noteActiveDeclExit(const oscad::ASTNode* declNode);

    // The CALLER's own callStack_ index right before a new frame is pushed
    // for it -- exactly the `callerFrameIdx` every evalUserFunction*
    // variant already computes inline (user_calls.cpp) before deciding
    // upvalueParent, needed here too by the explicit-frame-stack driver's
    // own push helper (a free function in a separate translation unit,
    // bytecode_vm.cpp, so it can't read callStack_ directly). Must be
    // read BEFORE enterUserCall's own callStack_.push_back(), same
    // ordering constraint the inline computation already has.
    int callStackTopIndex() const { return callStack_.empty() ? -1 : static_cast<int>(callStack_.size()) - 1; }

    // The innermost active call's own declaration node, for error()'s TRACE
    // walk -- needed by driveVm's own driveVmNativeDepth_ guard (see that
    // field's doc comment), which has no single reliable ASTNode of its own
    // to report against (the top VmFrame's chunk may be a children-list
    // chunk with a null selfDecl -- see tryCompileChildrenList's own doc
    // comment, bytecode_compiler.cpp). callStack_ is guaranteed non-empty
    // by the time this guard can fire (it only trips past
    // kMaxDriveVmNativeDepth levels of nesting, and every level pushes a
    // CallStackFrame first via enterUserCall). Same "free function, no
    // friend declaration" reasoning as callStackTopIndex, just above.
    const oscad::ASTNode* currentCallDeclNode() const {
        return callStack_.empty() ? nullptr : callStack_.back().declNode;
    }

    // The explicit, heap-allocated VM call stack that replaced native C++
    // recursion for a call between two compiled chunks -- see this
    // project's own session notes ("iterate over the code, don't use the
    // C++ call stack for recursion") and VmFrame's own doc comment
    // (bytecode_vm.hpp) for the full rationale. Public, and manipulated
    // directly (push_back/pop_back/back()) by bytecode_vm.cpp's driver
    // loop rather than through narrow wrapper methods -- that driver IS
    // this stack's only real owner/user, in a hot loop, the same pragmatic
    // choice already made for treeStack_ (csg_resolve.cpp). A
    // unique_ptr<VmFrame> has an address independent of the surrounding
    // vector -- reallocating `vmCallStack_` itself never moves a VmFrame
    // in memory, so CallStackFrame::vmFrame (a raw pointer into whichever
    // element is current) stays valid exactly as it always has.
    std::vector<std::unique_ptr<VmFrame>> vmCallStack_;

    // Parallel to vmCallStack_ (same size, same push/pop points, always) --
    // holds this frame's own callStack_/profiling bracket handle when it
    // has one (nullopt for a "bare" frame: a top-level compiled-call entry
    // point's own first frame, a bare statement expression, an assignment
    // block, a parameter default) -- see VmFrame's own doc comment for why
    // this can't just be a VmFrame member (evaluator.hpp can't be
    // forward-referenced from bytecode_vm.hpp, which evaluator.hpp itself
    // includes).
    std::vector<std::optional<UserCallHandle>> vmCallBrackets_;

    // Separate from kMaxUserCallDepth (which stays exactly as-is, still
    // guarding the interpreter-fallback/native-recursion boundary -- native
    // stack margin is still the real concern there). This one gates a
    // PUSH onto vmCallStack_ itself: unbounded growth from a genuinely
    // runaway non-tail-recursive compiled function is now a heap/memory
    // concern, not a native-stack-overflow one, so it needs a much higher
    // ceiling -- reusing kTcoIterationCap's own figure (the tail-loop
    // runaway guard, user_calls.cpp) for consistency rather than inventing
    // an unrelated third number. Revisit with the same CI-diagnostic-loop
    // method already used for kMaxUserCallDepth if this ever needs
    // recalibrating for real.
    static constexpr size_t kMaxVmCallStackDepth = 1'000'000;

    // Counts native C++ nesting of driveVm itself -- NOT logical call
    // depth (that's vmCallStack_.size()/callStack_.size(), both of which
    // grow just as much for a pure VM-internal Op::CallModule hop, which
    // costs zero native stack). A hop entirely serviced by driveVm's own
    // while loop (Op::CallModule/CallFn/CallFnTail/CallDynamic/
    // CallDynamicTail) never increments this. It DOES increment when a
    // compiled body statement isn't one of the specially-compiled forms
    // (e.g. `translate(v) recur(n-1);` -- a builtin-with-children falls to
    // Op::NativeStatement) and that native evalStatement call itself
    // re-enters the VM (evalChildren -> tryRunCompiledChildren/
    // tryRunCompiledAssignmentBlock/evalExprMaybeCompiled -> a fresh
    // driveVm call, nested on the native stack, unlike Op::CallModule's own
    // push). This is a REAL native recursive call each time, same hazard
    // kMaxUserCallDepth guards against -- caught for real: `module
    // recur(n) { translate([0,0,n]) recur2(n); } module recur2(n) { if
    // (n>0) recur(n-1); else cube(1); }` segfaulted (exit 139) around
    // n=3000 with no guard at all, since kMaxCsgTreeDepth only catches this
    // AFTER the (already-crashed) native descent would have unwound. Public
    // for the same reason vmCallStack_ is: driveVm (bytecode_vm.cpp) is a
    // free function, not a member, so it needs direct access with no
    // friend declaration in play.
    size_t driveVmNativeDepth_ = 0;
    // Originally set to kMaxUserCallDepth's own figure (30) on the
    // assumption the two native-reentry chains cost a comparable handful
    // of frames each -- WRONG in practice: a real differential sweep
    // against BOSL2's own test corpus found 3 genuine scripts
    // (test_ball_bearing, test_teardrop_corner_mask, test_rounding_hole_
    // mask -- all built on deeply layered _translate()/_show_highlight()/
    // attachment-wrapper composition, not runaway recursion) that used to
    // render successfully but started failing with "Recursion too deep
    // (native call stack)" once this guard shipped at 30.
    //
    // Locally binary-searched (macOS) to a first candidate of 50, the
    // smallest value tried that let all 3 succeed again, then FIRST tried
    // doubled to 100 for headroom -- that immediately segfaulted on
    // Windows CI (ModuleBodyCompiles.
    // NativeReentrantRecursionHitsAControlledErrorInsteadOfCrashing, its
    // own guard-safety regression test at the time -- since renamed/
    // repurposed, see Op::PushBuiltinWrap below). Backed off to 50 --
    // STILL segfaulted there. Confirms this chain's real per-level
    // native-frame cost is heavier than kMaxUserCallDepth's own (which
    // needed the SAME kind of comedown, an initial 50 down to 30, for a
    // shallower chain), and that Windows' real safe ceiling for THIS
    // chain sits somewhere below 50, above the already-Windows-verified-
    // safe 30 (confirmed safe by PR #60's own successful merge, before
    // this value ever changed). Landed at 40.
    //
    // A real stack-margin check (nativeStackMarginLow(), since removed --
    // see git history for `native_stack.hpp`/`.cpp` if the exact mechanism
    // is ever needed again) was tried as a REPLACEMENT for this fixed
    // count, specifically because BOSL2's attachable() machinery
    // (Anklet.scad) needed native reentry depth 55, past this ceiling --
    // and confirmed working for that: Anklet.scad rendered successfully
    // with an unmodified build. But the margin-based check was THEN
    // confirmed, via two separate real Windows CI runs, to segfault for a
    // DIFFERENT deep native-reentry chain (union()-wrapped recursion)
    // regardless of total thread stack size (an 8 MiB Windows stack,
    // matching macOS's own default byte-for-byte, still crashed at the
    // same depth) -- i.e. this fixed count, not the margin check, is the
    // mechanism actually proven safe on Windows for genuine deep native
    // reentry; the margin check is not a safe general replacement for it.
    //
    // The REAL fix for Anklet.scad turned out to be architectural, not a
    // better safety-check mechanism: Op::PushBuiltinWrap/PopBuiltinWrap
    // (bytecode.hpp/bytecode_compiler.cpp/bytecode_vm.cpp) eliminates
    // native reentry ENTIRELY for the specific pattern that needed depth
    // 55 (translate/rotate/scale/mirror/multmatrix/resize/color/#/%/!-
    // wrapped recursion) by compiling it to run on the heap-based
    // vmCallStack_ instead of falling to Op::NativeStatement -- so this
    // guard no longer needs to accommodate that case at all. What's left
    // uncovered (union/difference/intersection-wrapped recursion, other
    // builtins, fully interpreted-mode scripts) is genuinely "leaf-shaped"
    // again in the sense Stage 2's original design assumed, comfortably
    // within this fixed, Windows-proven-safe ceiling for any realistic
    // script. Do not raise this value again based on local (macOS/Linux)
    // testing alone -- only a real Windows CI pass is evidence of safety
    // here. Deliberately its own named constant (not a reuse of
    // kMaxUserCallDepth) so the two can be recalibrated independently.
    static constexpr size_t kMaxDriveVmNativeDepth = 40;

    // Bracketed public in place: Op::CallModule's own runtime handler and
    // driveVm's module-frame completion branch (bytecode_vm.cpp, a
    // separate translation unit) push/pop this directly -- exactly the
    // same "that driver IS this stack's own only other real owner/user"
    // reasoning vmCallStack_ was already made public for (Stage 1).
public:
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

private:

    // User function/module call stack: (kind, name, call position, decl
    // position) per active call, innermost last. Drives callCtxFor's
    // closure detection and error()'s TRACE lines. Mirrors the reference's
    // self._call_stack (there, 4-tuples; here, CallStackFrame -- see
    // eval_error.hpp).
    std::vector<CallStackFrame> callStack_;

    // Count of callStack_ entries that ACTUALLY cost native C++ stack --
    // i.e. pushed with skipDepthGuard=false (enterUserCall's own
    // interpreted-call sites: evalUserFunctionCore, evalUserModule).
    // Deliberately NOT the same number as callStack_.size(): a compiled-
    // to-compiled push (pushBracketedCallFrame/pushBracketedModuleFrame,
    // bytecode_vm.cpp, skipDepthGuard=true) still grows callStack_ (for
    // TRACE/closure-detection/$parent_modules bookkeeping) but costs ZERO
    // native stack, serviced by driveVm's own heap-based loop instead.
    //
    // Checking callStack_.size() itself against kMaxUserCallDepth (the
    // ORIGINAL design) conflates these two: a real BOSL2 script's own
    // AMBIENT callStack_ depth, inflated by many cheap skip=true compiled
    // module-call pushes sitting on callStack_ already, could push a
    // LATER genuinely-native-recursive call (e.g. an interpreted-path
    // function call like BOSL2's own `ident()`) over the threshold even
    // though the REAL native C++ nesting at that point was nowhere near
    // it -- caught for real: Anklet.scad's own ambient callStack_ depth
    // (mostly cheap Op::CallModule pushes) reached the mid-30s, tripping
    // kMaxUserCallDepth=30 for an `ident()` call that itself was nested
    // only a couple of GENUINE native frames deep. Exactly the same class
    // of "fixed-count guard conflates logical depth with native-stack
    // cost" bug driveVmNativeDepth_ already exists to avoid for the
    // OTHER guard site (bytecode_vm.cpp) -- this is that same fix,
    // applied here. Incremented/decremented symmetrically by
    // enterUserCall/exitUserCallSuccess/exitUserCallException, gated on
    // UserCallHandle::countedTowardNativeDepth (skipDepthGuard is only
    // available at enter time; the handle carries the decision forward
    // to exit time, since callStack_.back() -- and thus skipDepthGuard's
    // own original context -- is already gone by then).
    size_t nativeUserCallDepth_ = 0;

    // Running count of Module-kind frames currently on callStack_ --
    // maintained incrementally by enterUserCall/exitUserCall* rather than
    // rescanned from callStack_ on every module call (buildModuleChildCtx's
    // own $parent_modules) the way it used to be. That rescan is O(depth)
    // per call, so a script recursing deep enough to actually need Stage
    // 2's own new depth ceiling turned it into real O(depth^2) wall time --
    // measured directly: a script recursing through recur() alone (no
    // geometry cost at all) went from 0.49s at depth 16,000 to 7.19s at
    // depth 64,000 (quadratic, not the expected ~4x-for-4x-the-work
    // linear scaling) before this fix. Harmless at the OLD native-
    // recursion depths (capped at kMaxUserCallDepth=50, this scan was
    // never more than 50 entries) -- only became a real problem once
    // Stage 2 made much deeper module recursion possible at all.
    int moduleCallDepth_ = 0;

    // Refcounted set of DISTINCT declarations currently active anywhere on
    // callStack_ -- keyed by declaration ASTNode identity (the same
    // pointer CallStackFrame::declNode/enterUserCall's own `declNode` use
    // throughout), maintained incrementally by enterUserCall/
    // exitUserCall*/recordTailCallHop (a tail hop swaps the ACTIVE
    // declaration for the SAME stack slot without a matching enter/exit
    // pair of its own, so it has to update this too, symmetrically:
    // decrement the frame's old declNode, increment the new one).
    //
    // callCtxFor's own closure/lexical-nesting detection used to walk the
    // FULL callStack_ testing every single frame's declPosition for span
    // containment -- O(depth) per call, invisible at the old native-
    // recursion-bound depths (~50) but genuinely quadratic once compiled
    // calls (Stage 1/2, see this project's own session notes) made much
    // deeper recursion possible: a plain non-tail-recursive `f(n) = n<=0
    // ? 0 : 1+f(n-1)` went 10k->0.15s, 20k->0.54s, 40k->2.1s (measured --
    // textbook quadratic, not the expected ~2x-per-2x-depth linear
    // scaling). The fix exploits a structural fact about REAL recursion:
    // a deep call chain is overwhelmingly the SAME declaration repeated
    // many times, not many DISTINCT declarations -- so the number of
    // DISTINCT active declarations stays small and roughly constant
    // regardless of recursion depth, even though callStack_ itself grows
    // unboundedly. Since containment for a given declaration X depends
    // only on X's own (compile-time-fixed) position, never on WHICH of
    // its (possibly many) active occurrences is checked, checking each
    // DISTINCT X once is exactly equivalent to checking every occurrence
    // -- not an approximation.
    std::unordered_map<const oscad::ASTNode*, int> activeDeclRefcount_;

    // Shared cap on callStack_'s own size, checked by both
    // evalUserFunctionCore (below) and evalUserModule (user_calls.cpp) --
    // see evalUserFunctionCore's own doc comment for the full calibration
    // story (two rounds of CI stack-depth diagnostics on the worst-case
    // platform -- Windows, whose default thread stack is much smaller than
    // macOS/Linux's). One constant because both push onto the SAME
    // callStack_ -- a function recursing into a module recursing into a
    // function is the same native-stack-growth hazard regardless of which
    // kind of frame is on top at any given depth.
    //
    // Re-lowered from 50 (Stage 2, module-body compilation): factoring
    // evalUserModule's own native-fallback body into a separate
    // runModuleBodyNative function (shared with Op::CallModule's own
    // native-fallback branch, bytecode_vm.cpp -- needed so neither
    // duplicates the enterUserCall/evalChildren/exitUserCall bracket)
    // added ONE sustained extra native stack frame to every level of
    // INTERPRETED module recursion (the one recursive shape this cap still
    // protects -- compiled module recursion runs through the explicit
    // vmCallStack_ instead, see kMaxVmCallStackDepth, and never touches
    // this at all). 50 was tight enough on Windows CI that this alone
    // segfaulted UserModule.DeepNonTailRecursionHitsAControlledError
    // Interpreted (PR #59) instead of throwing cleanly at the guard.
    static constexpr size_t kMaxUserCallDepth = 30;

    // Cap on CSGNode::treeDepth (csg_node.hpp), checked wherever a CSGNode
    // is finalized with its own `children` already known (buildTreeNode,
    // evalModularCall's own non-splice tail, spliceModuleChildren's
    // union-wrap branch -- csg_resolve.cpp) -- i.e. enforced DURING
    // resolve, at tree-CONSTRUCTION time, not by walking the tree
    // afterward. This is deliberately NOT a generateTreeImpl-side guard
    // (an earlier version of this fix was exactly that, and it didn't
    // work): the RESOLVE pass's own depth guards (kMaxUserCallDepth,
    // kMaxVmCallStackDepth for compiled module recursion) only bound how
    // many CALLS happen, not how deep the CSGNode TREE those calls build
    // ends up being -- confirmed directly that resolveTree() itself
    // returns successfully for a 100,000-deep incrementally-nested tree
    // (`module recur(n) { if (n>0) { cube(0.1); recur(n-1); } else {
    // cube(1); } }` -- an ordinary "spiral/chain of shapes" pattern, not
    // exotic: each level's own module-call splice sees >1 child, so
    // evalModularCall wraps them in a synthetic "union" CSGNode instead of
    // collapsing away, and the tree genuinely grows one level per call).
    // The crash isn't even IN generateTreeImpl's own walk -- it's in
    // std::unique_ptr<CSGNode>'s default RECURSIVE destructor, freeing
    // that same 100,000-deep tree the moment it goes out of scope (a
    // standalone repro confirmed this directly: resolveTree() returns
    // fine and prints its own result; the crash happens purely from
    // letting that return value be destroyed, generateTree() never even
    // called). A destructor can't throw a catchable error and a real
    // stack overflow isn't a C++ exception at all -- there is no way to
    // "guard" the walk or the destructor after the fact, only to stop the
    // tree from ever getting that deep in the first place. Picked
    // conservatively (not from a precise Windows measurement, no access
    // to one locally) -- a chain 10x this depth already showed a real
    // performance cliff in Manifold's own deeply-nested boolean unions
    // (unrelated to this guard, but confirms nothing legitimate needs to
    // go anywhere near this many levels); revisit with the same
    // CI-diagnostic-loop method kMaxUserCallDepth's own history used if
    // it ever needs recalibrating.
    static constexpr int kMaxCsgTreeDepth = 2000;

    // See lastChildrenPositions()'s own doc comment.
    std::optional<std::vector<std::pair<std::string, int>>> lastChildrenPositions_;

    EchoFn echoFn_;
    std::shared_ptr<FontProvider> fontProvider_; // null until first fontProvider() call if not injected
    std::shared_ptr<ManifoldCache> manifoldCache_; // opt-in, see the constructor's own doc comment
    std::uint64_t randsCallCount_ = 0; // see noteRandsCall()

    // -- Debugging (Phase 9) --------------------------------------------

    DebugHooks debugHooks_;
    // See setFastContinueBreakpoints's own doc comment. nullopt (the
    // default, and what a plain debugger attach without ever calling that
    // setter leaves it at) means "no fast-continue exception" -- exactly
    // today's original, always-safe behavior.
    std::optional<std::unordered_map<std::string, std::set<int>>> fastContinueBreakpoints_;
    // See setFastContinueBreakpoints's own doc comment for why this is a
    // separate, narrower flag from fastContinueBreakpoints_ itself -- false
    // (the default) whenever a debugger is attached without explicitly
    // opting in, matching today's always-safe "checkDebug always calls the
    // hook" behavior.
    bool fastContinueHookSkippable_ = false;
    // See setFastContinueInterruptFlag's own doc comment. nullptr (the
    // default) means hook-skippable mode, if ever engaged, can't be
    // interrupted from outside a hook call.
    std::shared_ptr<std::atomic<bool>> fastContinueInterrupt_;
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
