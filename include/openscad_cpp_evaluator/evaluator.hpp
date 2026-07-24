#pragma once

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
    std::unordered_map<std::string, Value> bindArgs(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                                                      const std::vector<std::unique_ptr<oscad::Argument>>& arguments,
                                                      EvalContext& ctx);

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

    Value evalIdentifier(const std::string& name, const oscad::Position* position, EvalContext& ctx, bool warnIfUndef);
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
    // ponytail: the reference also computes a `share_dyn` optimization here
    // (skip copying ctx.dyn when the callee has no $-prefixed parameter,
    // aliasing it instead) -- measured at ~9.6% of total evaluate() time on
    // a BOSL2-heavy script in the *Python* reference, where dict-copy cost
    // is dominated by interpreter overhead. This port always copies
    // (matching childCtx()/callCtx()'s existing "always copy" simplicity);
    // revisit only if C++ profiling on a real script shows dyn-copying as
    // a hot path -- the copy itself is far cheaper here than in Python.
    EvalContext callCtxFor(const oscad::ASTNode& decl, EvalContext& ctx, const oscad::Scope* scope,
                            std::shared_ptr<const ChildrenNodeList> childrenNodes = nullptr,
                            const EvalContext* childrenCallerCtx = nullptr);

    // Fills in any parameter not already bound in childCtx.let_ from its
    // own default-value expression, evaluated purely lexically against the
    // declaration's own scope (childCtx.scope) with an empty `let_` and
    // `dyn` shared with childCtx -- a default sees neither the caller's
    // locals nor this call's other (sibling) parameters, though $-vars
    // remain dynamically scoped. A parameter with no default expression at
    // all binds to undef. Mirrors the reference's _apply_defaults exactly
    // (verified there against real OpenSCAD -- see its own doc comment).
    void applyDefaults(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params, EvalContext& childCtx);

    // Shared by checkDebug() and error()'s own errorBreak hook call: `let`
    // + `$`-prefixed `dyn` entries from `ctx`, plus (if there's an active
    // user call) any top-level script variable not shadowed locally.
    // `ctx` may be null (error()'s lastCtx_ fallback can itself be null
    // before any statement has ever run) -- returns an empty frame then.
    DebugFrame buildDebugFrame(const EvalContext* ctx) const;

    void evalUserModule(const oscad::ModuleDeclaration& decl, const oscad::ModularCall& call, EvalContext& ctx);
    Value evalUserFunction(const std::string& name, const oscad::FunctionDeclaration& decl,
                            const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                            const oscad::ASTNode* callNode);
    Value evalFunctionLiteral(const oscad::FunctionLiteral& funcNode,
                               const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                               const oscad::ASTNode* callNode);

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
