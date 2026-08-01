#include "openscad_cpp_evaluator/bytecode_compiler.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/dispatch.hpp"
#include "openscad_cpp_evaluator/function_builtins.hpp"

#include "builtins/builtins.hpp"

#include "openscad_cpp_parser/ast/declarations.hpp"
#include "openscad_cpp_parser/ast/expression.hpp"
#include "openscad_cpp_parser/ast/module_instantiation.hpp"
#include "openscad_cpp_parser/ast/vector_element.hpp"
#include "openscad_cpp_parser/scope.hpp"

#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace oscadeval {

namespace {

// Thrown by compileExpr wherever it hits a construct this phase doesn't
// compile yet -- tryCompileFunction catches it and returns nullopt (falls
// back to the ordinary AST interpreter for that whole function). Doubling
// compilation itself as the compilability check (rather than a separate
// pre-scan switch) means there's only one NodeKind dispatch to keep in sync
// with evalExpr's own, not two that could silently drift apart.
struct NotCompilable {};

// Mirrors expr_eval.cpp's own (anonymous-namespace, not shared) helper of
// the same name -- small enough not to be worth plumbing through a header.
bool isListCompClauseKind(oscad::NodeKind kind) {
    using oscad::NodeKind;
    switch (kind) {
        case NodeKind::ListCompIf:
        case NodeKind::ListCompIfElse:
        case NodeKind::ListCompFor:
        case NodeKind::ListCompCFor:
        case NodeKind::ListCompLet:
        case NodeKind::ListCompEach:
            return true;
        default:
            return false;
    }
}

// Merges `site`'s own captures into `out` for every entry that escapes
// `owner` itself (targetDecl != owner) -- see ClosureSite's own doc comment
// (bytecode.hpp) for why this "bubbling" is necessary: a literal nested
// inside `owner` may need something from further out than `owner`'s own
// scope, so `owner`'s OWN Op::MakeClosure must snapshot that name too --
// otherwise the inner literal's own capturedLet, rooted at whatever `owner`
// itself captured, would simply never contain it.
// Dedupes by (targetDecl, slot): the same enclosing binding reached through
// two different nested literals (or the same literal referencing it twice)
// is still one capture to make.
void bubbleEscapingCaptures(const CompiledChunk::ClosureSite& site, const oscad::ASTNode* owner,
                             std::vector<CompiledChunk::UpvalueRef>& out) {
    for (const auto& cap : site.captures) {
        if (cap.targetDecl == owner) continue;
        bool already = false;
        for (const auto& existing : out) {
            if (existing.targetDecl == cap.targetDecl && existing.slot == cap.slot) {
                already = true;
                break;
            }
        }
        if (!already) out.push_back(cap);
    }
}

// True if any instruction in `chunk`'s own body/defaults is Op::LoadFree --
// i.e. an identifier that resolved to neither a local, a statically-known
// enclosing upvalue, nor a dyn ($-prefixed) name at compile time (see
// compileExpr's own Identifier case). A `let(a = function(...) ...
// a(...) ...)` self-reference, mutual recursion between two-or-more
// sibling `let`-bound closures (fnliterals.scad-style isEven/isOdd, each
// calling the other), and either of those wrapped in a ternary (`let(a =
// cond ? function(...) ...a(...)... : function(...) ...)`, every reachable
// branch unambiguously a closure) no longer hit this -- see the LetOp
// case's own letrec pre-declare, below, and collectLetrecCandidateLiterals'
// own doc comment for exactly which RHS shapes qualify. Anything the
// pre-declare doesn't cover still does: a reference buried in something
// other than a direct-or-ternary closure RHS (`let(a = some_call() ?
// function(...) ...a(...)... : function(...) ...)`, a non-ternary
// conditional shape, etc.), or any other free variable this phase simply
// can't resolve statically. For any of those, Evaluator::evalIdentifier's
// own ctx.scope->lookupVariable() fallback re-evaluates the let-binding's
// RHS FRESH on every single call, producing a new Closure whose
// capturedLet is THIS invocation's own ctx.let_ (see expr_eval.cpp's
// FunctionLiteral case) rather than a stable snapshot -- registering such
// a chunk for compiled invocation would still be functionally correct,
// but each recursive call's own capturedLet->openChild() then nests ONE
// level deeper than the last (confirmed empirically: an O(n) list
// reduce() degrading to O(n^2) once compiled, before the letrec pre-
// declare existed), since nothing about that repeated fresh-derivation
// ever re-roots the trail. Excluding a LoadFree-containing chunk from
// registration leaves it running interpreted exactly as before --
// correct, if not optimized.
bool containsLoadFree(const std::vector<Instruction>& code) {
    for (const Instruction& ins : code) {
        if (ins.op == Op::LoadFree) return true;
    }
    return false;
}

bool containsLoadFree(const CompiledChunk& chunk) {
    if (containsLoadFree(chunk.bodyCode)) return true;
    for (const auto& defaultCode : chunk.defaultCode) {
        if (containsLoadFree(defaultCode)) return true;
    }
    return false;
}

// Recursively collects every FunctionLiteral `expr` could actually
// evaluate to at runtime -- a bare literal, or a ternary (or nested
// ternaries) whose branches ALL recursively qualify, e.g. `cond ?
// function(...) ...a(...)... : function(...) ...` (the ternary-wrapped
// self/mutual-reference case the direct-RHS-only letrec pre-declare,
// below, doesn't reach on its own). Returns std::nullopt -- not just an
// empty vector -- the moment ANY reachable branch ISN'T reliably a
// closure (a plain value, a call, anything else): letrec pre-declaration
// only makes sense when EVERY runtime path through `expr` really does
// produce a fresh closure that might reference the let-binding's own
// name, never a mix. A CommentedExpr wrapper is transparent, matching
// compileExpr's own handling of it everywhere else.
std::optional<std::vector<const oscad::FunctionLiteral*>> collectLetrecCandidateLiterals(const oscad::ASTNode& expr) {
    using oscad::NodeKind;
    if (expr.kind() == NodeKind::CommentedExpr) {
        return collectLetrecCandidateLiterals(*static_cast<const oscad::CommentedExpr&>(expr).expr);
    }
    if (expr.kind() == NodeKind::FunctionLiteral) {
        return std::vector<const oscad::FunctionLiteral*>{static_cast<const oscad::FunctionLiteral*>(&expr)};
    }
    if (expr.kind() == NodeKind::TernaryOp) {
        auto& t = static_cast<const oscad::TernaryOp&>(expr);
        auto trueLits = collectLetrecCandidateLiterals(*t.trueExpr);
        if (!trueLits) return std::nullopt;
        auto falseLits = collectLetrecCandidateLiterals(*t.falseExpr);
        if (!falseLits) return std::nullopt;
        trueLits->insert(trueLits->end(), falseLits->begin(), falseLits->end());
        return trueLits;
    }
    return std::nullopt;
}

// Compile-time name -> slot resolution, mirroring the LetOp/nested-scope
// shadowing rules callCtx()/letChildCtx() apply at runtime: one frame per
// LetOp (pushed/popped around its own assignments+body), innermost frame
// wins. A parameter-default expression is compiled against a scope with
// ZERO initial frames (see tryCompileFunction) -- deliberately unable to see
// ANY parameter, including its own -- so any parameter-name reference inside
// a default falls through to LOAD_FREE, which resolves via
// Evaluator::evalIdentifier's own ParameterDeclaration -> undef rule. This
// reproduces applyDefaults()'s existing `let_->openChild(isolate=true)`
// sibling-isolation (and the self-referential-default-doesn't-recurse case)
// by construction, with no special-casing needed here.
class CompileScope {
public:
    void push() { frames_.emplace_back(); }
    void pop() { frames_.pop_back(); }
    void declare(const std::string& name, int slot) { frames_.back()[name] = slot; }
    std::optional<int> resolve(const std::string& name) const {
        for (auto it = frames_.rbegin(); it != frames_.rend(); ++it) {
            auto found = it->find(name);
            if (found != it->end()) return found->second;
        }
        return std::nullopt;
    }

private:
    std::vector<std::unordered_map<std::string, int>> frames_;
};

// One level of compile-time lexical nesting a FunctionLiteral being
// compiled can capture a free variable from -- `decl` is that level's own
// declaration identity (matches what CallStackFrame::declNode is stamped
// with at runtime, for Evaluator::findUpvalue's exact-match search),
// `scope` is that level's own CompileScope AS OF THE POINT the nested
// literal appears (a pointer into a still-live stack frame -- the whole
// enclosing chain is only ever used synchronously, during the single
// recursive-descent compile of the outermost declaration, so this never
// dangles). Ordered innermost-last (matching `enclosing_`'s own push/pop
// discipline below).
struct EnclosingLevel {
    const oscad::ASTNode* decl;
    const CompileScope* scope;
};

class Compiler;

// Compiles one FunctionDeclaration/FunctionLiteral-shaped body (same
// parameters+single-expression-body shape both node kinds share) into
// `chunk`. Shared by tryCompileFunction (the top-level entry point, empty
// `enclosing`) and the FunctionLiteral case in Compiler::compileExpr below
// (a non-empty `enclosing` chain, letting the literal's own free-variable
// references resolve as upvalues into whatever lexically contains it).
// Returns false (chunk left partially populated, discarded by the caller)
// for any NotCompilable thrown while compiling.
bool compileFunctionLike(CompiledChunk& chunk, const oscad::Scope* staticScope, const oscad::ASTNode* selfDecl,
                          std::vector<EnclosingLevel> enclosing,
                          const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                          const oscad::Expression& bodyExpr);

class Compiler {
public:
    // `scope`: the declaration currently being compiled's own static scope
    // (buildScopes() output, context-free -- safe to use at compile time
    // with no live EvalContext) -- used by the PrimaryCall case below to
    // resolve a callee name to a FunctionDeclaration. Direct (or mutual)
    // recursion through a resolved call site is deliberately NOT excluded
    // here: each call allocates its own fresh slot array (a local
    // std::vector, ordinary C++ call-stack semantics -- no pooled/shared
    // slot storage a re-entrant call could alias), so recursion is already
    // safe under this design; see tests/test_bytecode_compiler.cpp's own
    // recursive-function case for the empirical proof, not just this
    // reasoning. `selfDecl`/`enclosing`: this chunk's own identity and the
    // chain of lexically-enclosing declarations available for upvalue
    // resolution (empty for a top-level FunctionDeclaration; non-empty
    // when compiling a FunctionLiteral nested inside another compile --
    // see the FunctionLiteral case in compileExpr).
    Compiler(CompiledChunk& chunk, const oscad::Scope* scope, const oscad::ASTNode* selfDecl,
              std::vector<EnclosingLevel> enclosing)
        : chunk_(chunk), scope_(scope), selfDecl_(selfDecl), enclosing_(std::move(enclosing)) {}

    int nextSlot() const { return nextSlot_; }
    int nextIterList() const { return nextIterList_; }

    int declareLocal(CompileScope& scope, const std::string& name) {
        int slot = nextSlot_++;
        scope.declare(name, slot);
        return slot;
    }

    // Shared by the Identifier case (below) and PrimaryCall's own callee
    // probe (a bare-identifier callee that didn't resolve to a builtin/user
    // function statically -- "maybe it's a closure value") -- same name
    // resolution either way, only the terminal free-variable fallback
    // differs: an ordinary read always warns on a genuinely unknown name
    // (LoadFree), but the callee probe must not (LoadFreeNoWarn) --
    // mirrors evalFunctionCall's own `evalIdentifier(..., warnIfUndef)`
    // split exactly (user_calls.cpp). See LoadFreeNoWarn's own doc comment,
    // bytecode.hpp, for the double-warning bug this fixes.
    void compileIdentifierLoad(const std::string& name, const oscad::Position& pos, std::vector<Instruction>& out,
                                CompileScope& scope, bool warnIfUndef) {
        if (!name.empty() && name[0] == '$') {
            out.push_back({Op::LoadDyn, internName(name), 0, &pos});
        } else if (auto slot = scope.resolve(name)) {
            out.push_back({Op::LoadLocal, *slot, 0, &pos});
        } else if (auto upvalSlot = resolveEnclosing(name)) {
            int idx = static_cast<int>(chunk_.upvalues.size());
            chunk_.upvalues.push_back(*upvalSlot);
            out.push_back({Op::LoadUpvalue, idx, 0, &pos});
        } else {
            out.push_back({warnIfUndef ? Op::LoadFree : Op::LoadFreeNoWarn, internName(name), 0, &pos});
        }
    }

    // Widens chunk_'s own [minLine, maxLine] span to include `node` --
    // called at the top of every AST-node-visiting entry point
    // (compileExpr/compileListElement/compileListCompBody) so the chunk's
    // final span covers every line actually compiled into it, not just the
    // declaration's own header line. See CompiledChunk::origin/minLine/
    // maxLine's own doc comment (bytecode.hpp) for why this matters --
    // Evaluator::chunkEligibleNow's debug "fast continue" check.
    void trackSpan(const oscad::ASTNode& node) {
        const oscad::Position& pos = node.position();
        chunk_.origin = pos.origin;
        if (pos.line < chunk_.minLine) chunk_.minLine = pos.line;
        if (pos.line > chunk_.maxLine) chunk_.maxLine = pos.line;
    }

    // Walks `enclosing_` from innermost (back()) to outermost, looking for
    // a level whose own CompileScope resolves `name` -- the first (most
    // deeply nested, i.e. most recently active at runtime) match wins,
    // mirroring how the interpreter's own trail-ancestry lookup would
    // naturally resolve to the innermost active binding too.
    std::optional<CompiledChunk::UpvalueRef> resolveEnclosing(const std::string& name) const {
        for (auto it = enclosing_.rbegin(); it != enclosing_.rend(); ++it) {
            if (auto slot = it->scope->resolve(name)) return CompiledChunk::UpvalueRef{it->decl, *slot, name};
        }
        return std::nullopt;
    }

    int internConst(Value v) {
        chunk_.constants.push_back(std::move(v));
        return static_cast<int>(chunk_.constants.size()) - 1;
    }

    // No dedup -- ponytail: a function's own name pool is tiny (its own
    // identifiers/members/dyn-vars), a linear scan-free append keeps this
    // simple; revisit only if a real chunk's pool is shown to matter.
    int internName(const std::string& name) {
        chunk_.names.push_back(name);
        return static_cast<int>(chunk_.names.size()) - 1;
    }

    // `tail`: true iff `node`'s own value, once produced, becomes the
    // enclosing function/literal body's result with nothing further done
    // to it -- i.e. a genuine tail position (see bytecode.hpp's own
    // CallFnTail/CallDynamicTail doc comment). Defaults false: only
    // compileFunctionLike's own top-level body call starts true;
    // TernaryOp/LetOp forward their own `tail` to whichever sub-expression
    // is itself in tail position (both branches for Ternary, just the
    // body for Let -- an assignment's own RHS is never tail); every other
    // node kind's sub-expressions are unconditionally non-tail (an
    // operand, an argument, a condition -- something is always done with
    // their result afterward). Only affects the PrimaryCall case, which
    // emits CallFnTail/CallDynamicTail instead of CallFn/CallDynamic when
    // `tail` is true and the callee is a non-builtin (builtins are never
    // trampolined, matching upstream's own BuiltinFunction early-return
    // via a genuine call).
    void compileExpr(const oscad::Expression& node, std::vector<Instruction>& out, CompileScope& scope,
                      bool tail = false) {
        using oscad::NodeKind;
        trackSpan(node);
        switch (node.kind()) {
            case NodeKind::NumberLiteral:
                out.push_back({Op::PushConst, internConst(Value{static_cast<const oscad::NumberLiteral&>(node).val}), 0,
                                nullptr});
                return;
            case NodeKind::BooleanLiteral:
                out.push_back({Op::PushBool, static_cast<const oscad::BooleanLiteral&>(node).val ? 1 : 0, 0, nullptr});
                return;
            case NodeKind::StringLiteral:
                out.push_back({Op::PushConst, internConst(Value{static_cast<const oscad::StringLiteral&>(node).val}), 0,
                                nullptr});
                return;
            case NodeKind::UndefinedLiteral:
                out.push_back({Op::PushConst, internConst(Value{}), 0, nullptr});
                return;
            case NodeKind::CommentedExpr:
                // Transparent wrapper -- forwards `tail` unchanged.
                compileExpr(*static_cast<const oscad::CommentedExpr&>(node).expr, out, scope, tail);
                return;
            case NodeKind::Identifier: {
                auto& n = static_cast<const oscad::Identifier&>(node);
                compileIdentifierLoad(n.name, n.position(), out, scope, /*warnIfUndef=*/true);
                return;
            }
            case NodeKind::ListComprehension: {
                // Also how plain vector literals (`[1, 2, 3]`) are
                // represented -- see ListComprehension's own header comment
                // in vector_element.hpp. A literal with no real
                // comprehension clause at all takes the cheaper fixed-count
                // BuildList path (Phase 1.5a); one containing any of the 6
                // clause kinds uses the general accumulator-based path
                // (Phase 3, see compileListElement).
                auto& n = static_cast<const oscad::ListComprehension&>(node);
                bool anyClause = false;
                for (const auto& elemPtr : n.elements) {
                    if (isListCompClauseKind(elemPtr->kind())) {
                        anyClause = true;
                        break;
                    }
                }
                if (!anyClause) {
                    for (const auto& elemPtr : n.elements) {
                        compileExpr(static_cast<const oscad::Expression&>(*elemPtr), out, scope);
                    }
                    out.push_back({Op::BuildList, static_cast<int>(n.elements.size()), 0, &n.position()});
                    return;
                }
                out.push_back({Op::AccumOpen, 0, 0, nullptr});
                for (const auto& elemPtr : n.elements) compileListElement(*elemPtr, out, scope);
                out.push_back({Op::AccumClose, 0, 0, nullptr});
                return;
            }
            // A function-literal *value* is compiled EAGERLY here, as a
            // side effect of compiling whatever contains it -- there is no
            // other point where its own free-variable references could be
            // resolved against this enclosing chain (AST nodes have no
            // parent pointers to discover it from later, see this file's
            // own module notes). If the literal itself fails to compile as
            // bytecode at all (contains an echo/assert/unsupported
            // construct), the WHOLE containing declaration must also bail
            // -- propagated by simply rethrowing -- since there's no lighter
            // way left to find out what it captures (see
            // bubbleEscapingCaptures' own doc comment for why a literal
            // that DOES compile as bytecode no longer needs this bail just
            // for referencing enclosing state).
            case NodeKind::FunctionLiteral: {
                auto& n = static_cast<const oscad::FunctionLiteral&>(node);
                std::vector<EnclosingLevel> childEnclosing = enclosing_;
                childEnclosing.push_back({selfDecl_, &scope});
                CompiledChunk literalChunk;
                if (!compileFunctionLike(literalChunk, n.scope(), &n, std::move(childEnclosing), n.parameters,
                                          *n.body)) {
                    throw NotCompilable{};
                }
                // Effective capture set: `n`'s own direct free-variable
                // references, plus -- transitively -- whatever any literal
                // nested inside `n` still needs from beyond `n`'s own scope
                // (see bubbleEscapingCaptures' own doc comment, above, and
                // ClosureSite's, bytecode.hpp).
                std::vector<CompiledChunk::UpvalueRef> effectiveCaptures = literalChunk.upvalues;
                for (const auto& nestedSite : literalChunk.closureSites) {
                    bubbleEscapingCaptures(nestedSite, &n, effectiveCaptures);
                }
                // Registered (Evaluator::lookupCompiledLiteralChunk finds
                // this by node pointer whenever the closure is later
                // invoked, whether that's a plain PushConst-created value
                // below or a MakeClosure-snapshotted one) whenever it's
                // actually safe to run compiled: always for a zero-capture
                // literal (a single frozen constant, unaffected by anything
                // below), and for a captures-having one only when its own
                // body contains no Op::LoadFree -- see containsLoadFree's
                // own doc comment, above, for why a LoadFree-containing
                // captures-having closure must still be left interpreted:
                // its own capturedLet-chaining, once registered, degrades
                // an O(n) reduction into O(n^2). (The reduce()/
                // accumulate()/while() idiom itself -- a closure directly
                // assigned via `let` that calls itself by name -- no longer
                // hits this: see the LetOp case's own letrec pre-declare,
                // below, which lets that specific self-reference resolve as
                // a real upvalue instead of falling through to LoadFree.)
                // Everything else (Op::LoadUpvalue/Op::MakeClosure inside a
                // REGISTERED captures-having chunk) is no longer assumed to
                // resolve only against a still-live creator frame (see
                // those opcodes' own runtime fallback, bytecode_vm.cpp):
                // when the live-call-stack walk misses (the creator's frame
                // is gone -- a genuinely escaped closure), they fall back to
                // `ctx.let_`, which callCtxFor roots at this exact closure's
                // own capturedLet snapshot whenever it's invoked -- the same
                // snapshot `effectiveCaptures` describes below.
                if (effectiveCaptures.empty() || !containsLoadFree(literalChunk)) {
                    chunk_.nestedLiterals.emplace_back(&n, std::move(literalChunk));
                }
                if (effectiveCaptures.empty()) {
                    // Closes over nothing at all, even transitively -- a
                    // single frozen constant works for every invocation, no
                    // per-call snapshot needed.
                    out.push_back(
                        {Op::PushConst, internConst(Value{std::make_shared<const Closure>(Closure{&n, nullptr})}), 0,
                         nullptr});
                    return;
                }
                // Escaping-closure support (Op::MakeClosure, bytecode.hpp).
                int siteIdx = static_cast<int>(chunk_.closureSites.size());
                chunk_.closureSites.push_back(CompiledChunk::ClosureSite{&n, std::move(effectiveCaptures)});
                out.push_back({Op::MakeClosure, siteIdx, 0, nullptr});
                return;
            }
            case NodeKind::RangeLiteral: {
                auto& n = static_cast<const oscad::RangeLiteral&>(node);
                compileExpr(*n.start, out, scope);
                compileExpr(*n.end, out, scope);
                compileExpr(*n.step, out, scope);
                out.push_back({Op::Range, 0, 0, &n.position()});
                return;
            }
            case NodeKind::PrimaryIndex: {
                auto& n = static_cast<const oscad::PrimaryIndex&>(node);
                compileExpr(*n.left, out, scope);
                compileExpr(*n.index, out, scope);
                out.push_back({Op::Index, 0, 0, &n.position()});
                return;
            }
            case NodeKind::PrimaryMember: {
                auto& n = static_cast<const oscad::PrimaryMember&>(node);
                compileExpr(*n.left, out, scope);
                out.push_back({Op::Member, internName(n.member->name), 0, &n.position()});
                return;
            }
            case NodeKind::TernaryOp: {
                auto& n = static_cast<const oscad::TernaryOp&>(node);
                compileExpr(*n.condition, out, scope); // condition is never tail
                size_t jumpToFalse = out.size();
                out.push_back({Op::JumpIfFalse, 0, 0, nullptr});
                compileExpr(*n.trueExpr, out, scope, tail); // both branches inherit
                size_t jumpToEnd = out.size();
                out.push_back({Op::Jump, 0, 0, nullptr});
                out[jumpToFalse].a = static_cast<int>(out.size());
                compileExpr(*n.falseExpr, out, scope, tail);
                out[jumpToEnd].a = static_cast<int>(out.size());
                return;
            }
            case NodeKind::LogicalAndOp: {
                auto& n = static_cast<const oscad::LogicalAndOp&>(node);
                compileExpr(*n.left, out, scope);
                size_t j1 = out.size();
                out.push_back({Op::JumpIfFalse, 0, 0, nullptr});
                compileExpr(*n.right, out, scope);
                size_t j2 = out.size();
                out.push_back({Op::JumpIfFalse, 0, 0, nullptr});
                out.push_back({Op::PushBool, 1, 0, nullptr});
                size_t jend = out.size();
                out.push_back({Op::Jump, 0, 0, nullptr});
                int falseTarget = static_cast<int>(out.size());
                out[j1].a = falseTarget;
                out[j2].a = falseTarget;
                out.push_back({Op::PushBool, 0, 0, nullptr});
                out[jend].a = static_cast<int>(out.size());
                return;
            }
            case NodeKind::LogicalOrOp: {
                auto& n = static_cast<const oscad::LogicalOrOp&>(node);
                compileExpr(*n.left, out, scope);
                size_t j1 = out.size();
                out.push_back({Op::JumpIfTrue, 0, 0, nullptr});
                compileExpr(*n.right, out, scope);
                size_t j2 = out.size();
                out.push_back({Op::JumpIfTrue, 0, 0, nullptr});
                out.push_back({Op::PushBool, 0, 0, nullptr});
                size_t jend = out.size();
                out.push_back({Op::Jump, 0, 0, nullptr});
                int trueTarget = static_cast<int>(out.size());
                out[j1].a = trueTarget;
                out[j2].a = trueTarget;
                out.push_back({Op::PushBool, 1, 0, nullptr});
                out[jend].a = static_cast<int>(out.size());
                return;
            }
            // A call whose callee statically resolves (by name, at compile
            // time) to either a builtin function or a user
            // FunctionDeclaration found via this chunk's own enclosing
            // scope -- everything else (a dynamic function-literal-value
            // call, `import`/`object()`'s own special raw-argument-order
            // handling, an unresolvable name) bails compilation of the
            // WHOLE containing function, matching evalFunctionCall's own
            // precedence order exactly (checked in the same sequence here
            // as there) rather than silently falling back per-call-site,
            // which would require a runtime "was this actually resolvable"
            // branch this phase doesn't build.
            case NodeKind::PrimaryCall: {
                auto& n = static_cast<const oscad::PrimaryCall&>(node);
                const auto* leftId = (n.left->kind() == NodeKind::Identifier)
                                          ? static_cast<const oscad::Identifier*>(n.left.get())
                                          : nullptr;

                CompiledChunk::CallSite site;
                site.callNode = &n;
                bool isStatic = false;
                if (leftId) {
                    const std::string& calleeName = leftId->name;
                    site.calleeName = calleeName; // also Op::CallDynamic's own "unknown function" warning text
                    if (calleeName == "import") {
                        // Not in isBuiltinFunctionName's table (special-
                        // cased earlier in evalFunctionCall, bypassing the
                        // builtin dispatch table entirely) -- needs its
                        // own flag. Runtime dispatch: bytecode_vm.cpp's
                        // CallFn handler calls importAsValue instead of
                        // evalBuiltinFunction when isImport is set.
                        site.isImport = true;
                        isStatic = true;
                    } else if (isBuiltinFunctionName(calleeName)) {
                        // object() is already in this table -- it gets
                        // isBuiltin=true here like any other builtin, and
                        // is special-cased at RUNTIME instead (bytecode_vm.cpp's
                        // CallFn handler dispatches calleeName=="object" to
                        // mergeObjectArgs instead of evalBuiltinFunction,
                        // since it needs its arguments merged in exact
                        // call-site interleaved order, not resolveArgs'
                        // split positional/named CallArgs shape).
                        site.isBuiltin = true;
                        isStatic = true;
                    } else {
                        const oscad::ASTNode* found = scope_ ? scope_->lookupFunction(calleeName) : nullptr;
                        if (found && found->kind() == NodeKind::FunctionDeclaration) {
                            site.decl = static_cast<const oscad::FunctionDeclaration*>(found);
                            isStatic = true;
                        }
                    }
                }

                if (!isStatic) {
                    // Dynamic dispatch: the callee resolves to a runtime
                    // Value -- a captured closure held in a local/upvalue
                    // (e.g. `let(g = function(y) y + x) g(5)`, the whole
                    // reason closures need this to be reachable at all --
                    // see this file's own module notes) or any other
                    // computed callee expression. Mirrors
                    // evalFunctionCall's own function-literal-value-probe
                    // fallback exactly, including that its "unknown
                    // function" warning only ever names the callee when it
                    // WAS a plain identifier (site.calleeName stays empty
                    // otherwise) -- see Op::CallDynamic's own runtime
                    // handler. Never itself in tail position (it's the
                    // CALLEE being resolved, not this call's own result).
                    //
                    // A bare-identifier callee (leftId) goes through
                    // compileIdentifierLoad directly with warnIfUndef=false
                    // (LoadFreeNoWarn for the free-variable fallback,
                    // matching evalFunctionCall's own
                    // evalIdentifier(..., /*warnIfUndef=*/false) probe) --
                    // NOT the generic compileExpr(*n.left, ...), which
                    // would use plain Identifier compilation (LoadFree,
                    // warnIfUndef=true) and double up "unknown variable"
                    // on top of CallDynamic's own "unknown function"
                    // warning for a genuinely unresolvable name. Anything
                    // else (index/member access, another call's result)
                    // has no such special-cased probe in the interpreter
                    // either -- ordinary compileExpr, whatever warnings
                    // THAT naturally produces.
                    if (leftId) {
                        compileIdentifierLoad(leftId->name, leftId->position(), out, scope, /*warnIfUndef=*/false);
                    } else {
                        compileExpr(*n.left, out, scope);
                    }
                }

                for (const auto& argPtr : n.arguments) {
                    if (argPtr->kind() == NodeKind::NamedArgument) {
                        auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
                        compileExpr(*a.expr, out, scope); // an argument is never tail
                        site.argNames.push_back(a.name->name);
                    } else {
                        auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
                        compileExpr(*a.expr, out, scope);
                        site.argNames.push_back(std::nullopt);
                    }
                }
                int siteIdx = static_cast<int>(chunk_.callSites.size());
                chunk_.callSites.push_back(std::move(site));
                // Builtins and import() never trampoline (matching
                // upstream's own BuiltinFunction early-return via a
                // genuine call) -- CallFnTail is only emitted for `tail`
                // AND a plain user-function callee. Required, not
                // optional: CallFnTail's own runtime handler unconditionally
                // dereferences site.decl->parameters.size() -- site.decl is
                // null for an import site, so omitting !site.isImport here
                // would crash on a tail-position import() call. Eligibility
                // beyond that (isolated vs. closure-nested) is a runtime
                // fact the compiler can't know -- see bytecode.hpp's own
                // doc comment on these two opcodes.
                Op op;
                if (isStatic) {
                    op = (tail && !site.isBuiltin && !site.isImport) ? Op::CallFnTail : Op::CallFn;
                } else {
                    op = tail ? Op::CallDynamicTail : Op::CallDynamic;
                }
                out.push_back({op, siteIdx, static_cast<int>(n.arguments.size()), &n.position()});
                return;
            }

            case NodeKind::LetOp: {
                auto& n = static_cast<const oscad::LetOp&>(node);
                scope.push();
                size_t placeholderIdx = out.size();
                out.push_back({Op::OpenLocalScope, 0, 0, nullptr});
                int slotStart = nextSlot_;
                // Letrec pre-pass: every assignment whose RHS is GUARANTEED
                // to evaluate to a closure -- a direct `name =
                // function(...) ...`, or `name = cond ? function(...) ... :
                // function(...) ...` (or nested ternaries of the same
                // shape) -- gets its own slot declared up front, before ANY
                // of them are compiled -- not just so a closure can
                // reference itself (see the self-reference handling
                // below), but so an EARLIER one can also reference a LATER
                // sibling (mutual recursion, e.g. fnliterals.scad-style
                // isEven/isOdd calling each other). `letrecSlot[i]` stays
                // -1 for anything else, which keeps the existing after-the-
                // fact declareLocal path below completely unchanged
                // (`let(x = x + 1)` must still see the OUTER x, not a
                // fresh, not-yet-assigned local shadowing it -- only an
                // RHS that's UNAMBIGUOUSLY always a closure has "see
                // myself/a sibling" as a sensible reading at all; see
                // collectLetrecCandidateLiterals' own doc comment, above,
                // for why a mixed/uncertain RHS shape doesn't qualify).
                // `letrecCandidates[i]` is every FunctionLiteral node `i`'s
                // own RHS could actually construct at runtime -- more than
                // one for a ternary -- each independently checked for a
                // self/sibling reference below, since either one might be
                // the value `name` ends up holding.
                std::vector<int> letrecSlot(n.assignments.size(), -1);
                std::vector<std::vector<const oscad::FunctionLiteral*>> letrecCandidates(n.assignments.size());
                std::unordered_set<int> pendingLetrecSlots;
                for (size_t i = 0; i < n.assignments.size(); ++i) {
                    const auto& assign = n.assignments[i];
                    const std::string& name = assign->name->name;
                    if (name.empty() || name[0] == '$') continue;
                    if (auto candidates = collectLetrecCandidateLiterals(*assign->expr)) {
                        letrecSlot[i] = declareLocal(scope, name);
                        letrecCandidates[i] = std::move(*candidates);
                        pendingLetrecSlots.insert(letrecSlot[i]);
                    }
                }
                // consumerSlot/name pairs an EARLIER closure's own
                // Op::MakeClosure left unresolved because they targeted a
                // sibling that didn't exist yet at that point -- resolved
                // (an Op::PatchClosureCapture emitted) the moment that
                // sibling's own StoreLocal runs, below.
                std::unordered_map<int, std::vector<std::pair<int, std::string>>> pendingWaiters;
                for (size_t i = 0; i < n.assignments.size(); ++i) {
                    const auto& assign = n.assignments[i];
                    const std::string& name = assign->name->name;
                    const int preDeclaredSlot = letrecSlot[i];
                    const bool selfBinding = preDeclaredSlot >= 0;
                    compileExpr(*assign->expr, out, scope); // RHS is never tail
                    for (const oscad::FunctionLiteral* candidate : letrecCandidates[i]) {
                        CompiledChunk::ClosureSite* site = nullptr;
                        for (auto& s : chunk_.closureSites) {
                            if (s.node == candidate) {
                                site = &s;
                                break;
                            }
                        }
                        // No entry at all means this particular candidate
                        // (e.g. one branch of a ternary) never referenced
                        // itself/a sibling/anything else -- zero captures,
                        // still the cheaper Op::PushConst path, nothing
                        // here to patch.
                        if (!site) continue;
                        auto& captures = site->captures;
                        for (auto it = captures.begin(); it != captures.end();) {
                            if (it->targetDecl != selfDecl_ || !pendingLetrecSlots.count(it->slot)) {
                                ++it;
                                continue;
                            }
                            if (it->slot == preDeclaredSlot) {
                                // See Op::MakeClosure's own runtime handler
                                // (bytecode_vm.cpp): the closure being
                                // created doesn't exist yet at the moment
                                // its own captures are normally snapshotted,
                                // so this one is deferred and patched in
                                // right after construction instead of read
                                // eagerly like every other capture.
                                it->isSelfReference = true;
                                ++it;
                            } else {
                                // Forward reference to a sibling that hasn't
                                // been constructed AT ALL yet -- unlike the
                                // self-reference case above, there's nothing
                                // to read OR self-patch into here. Left out
                                // of `captures` entirely (nothing for
                                // Op::MakeClosure to even attempt) and
                                // deferred to a real Op::PatchClosureCapture
                                // once that sibling's own StoreLocal runs.
                                pendingWaiters[it->slot].emplace_back(preDeclaredSlot, it->name);
                                it = captures.erase(it);
                            }
                        }
                    }
                    if (!name.empty() && name[0] == '$') {
                        out.push_back({Op::StoreDyn, internName(name), 0, &assign->position()});
                    } else {
                        int slot = selfBinding ? preDeclaredSlot : declareLocal(scope, name);
                        out.push_back({Op::StoreLocal, slot, 0, &assign->position()});
                        if (selfBinding) {
                            pendingLetrecSlots.erase(slot);
                            auto waitersIt = pendingWaiters.find(slot);
                            if (waitersIt != pendingWaiters.end()) {
                                for (const auto& [consumerSlot, capName] : waitersIt->second) {
                                    out.push_back({Op::PatchClosureCapture, consumerSlot, internName(capName),
                                                    &assign->position(), nullptr, slot});
                                }
                                pendingWaiters.erase(waitersIt);
                            }
                        }
                    }
                }
                out[placeholderIdx].a = slotStart;
                out[placeholderIdx].b = nextSlot_ - slotStart;
                compileExpr(*n.body, out, scope, tail); // body inherits
                scope.pop();
                return;
            }

            case NodeKind::EchoOp: {
                // Always evaluates every argument unconditionally (unlike
                // AssertOp's message, see below) -- same shape as an
                // ordinary call's own argument compilation.
                auto& n = static_cast<const oscad::EchoOp&>(node);
                CompiledChunk::EchoSite site;
                for (const auto& argPtr : n.arguments) {
                    if (argPtr->kind() == NodeKind::NamedArgument) {
                        auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
                        compileExpr(*a.expr, out, scope); // an argument is never tail
                        site.argNames.push_back(a.name->name);
                    } else {
                        auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
                        compileExpr(*a.expr, out, scope);
                        site.argNames.push_back(std::nullopt);
                    }
                }
                int siteIdx = static_cast<int>(chunk_.echoSites.size());
                chunk_.echoSites.push_back(std::move(site));
                out.push_back({Op::Echo, siteIdx, static_cast<int>(n.arguments.size()), &n.position()});
                compileExpr(*n.body, out, scope, tail); // body inherits, no jump needed (Echo always falls through)
                return;
            }

            case NodeKind::AssertOp: {
                auto& n = static_cast<const oscad::AssertOp&>(node);
                if (n.arguments.empty()) {
                    // Mirrors evalAssertExpr's own `raw.empty() || ...`
                    // short-circuit exactly: a zero-argument assert() is
                    // unconditionally true, so the check can never fail --
                    // skip it entirely rather than compiling a check that
                    // can never trigger.
                    compileExpr(*n.body, out, scope, tail);
                    return;
                }
                compileExpr(*argExpr(*n.arguments[0]), out, scope); // condition, never tail
                size_t jumpPassed = out.size();
                out.push_back({Op::JumpIfTrue, 0, 0, nullptr});
                const bool hasMessage = n.arguments.size() > 1;
                // Lazily compiled: only reachable on the condition-false
                // path (guarded by the JumpIfTrue above) -- mirrors
                // evalAssertExpr's own lazy evaluation of the message
                // argument exactly (only evaluated when the assertion
                // actually fails). Slot resolution is unaffected by which
                // runtime path reaches this code -- CompileScope is purely
                // a compile-time name/index table, same as every other
                // conditionally-executed branch this compiler already
                // handles (Ternary, LogicalAnd/Or, ListCompIf).
                if (hasMessage) compileExpr(*argExpr(*n.arguments[1]), out, scope); // message, never tail
                // Precomputed ONCE at compile time (cheaper than the
                // interpreter, which recomputes this on every failing
                // call) -- toString() is a pure, ctx-free AST-to-text
                // operation.
                int condTextIdx = internConst(Value{argExpr(*n.arguments[0])->toString()});
                out.push_back({Op::AssertFail, hasMessage ? 1 : 0, condTextIdx, &n.position(), &n});
                out[jumpPassed].a = static_cast<int>(out.size());
                compileExpr(*n.body, out, scope, tail);
                return;
            }

#define OSCAD_COMPILE_UNARY(Kind)                                                                                    \
    case NodeKind::Kind: {                                                                                            \
        auto& n = static_cast<const oscad::Kind&>(node);                                                              \
        compileExpr(*n.expr, out, scope);                                                                             \
        out.push_back({Op::UnaryOp, static_cast<int>(NodeKind::Kind), 0, &n.position()});                             \
        return;                                                                                                       \
    }
                OSCAD_COMPILE_UNARY(UnaryMinusOp)
                OSCAD_COMPILE_UNARY(LogicalNotOp)
                OSCAD_COMPILE_UNARY(BitwiseNotOp)
#undef OSCAD_COMPILE_UNARY

#define OSCAD_COMPILE_BINARY(Kind)                                                                                    \
    case NodeKind::Kind: {                                                                                            \
        auto& n = static_cast<const oscad::Kind&>(node);                                                              \
        compileExpr(*n.left, out, scope);                                                                             \
        compileExpr(*n.right, out, scope);                                                                            \
        out.push_back({Op::BinaryOp, static_cast<int>(NodeKind::Kind), 0, &n.position()});                            \
        return;                                                                                                       \
    }
                OSCAD_COMPILE_BINARY(AdditionOp)
                OSCAD_COMPILE_BINARY(SubtractionOp)
                OSCAD_COMPILE_BINARY(MultiplicationOp)
                OSCAD_COMPILE_BINARY(DivisionOp)
                OSCAD_COMPILE_BINARY(ModuloOp)
                OSCAD_COMPILE_BINARY(ExponentOp)
                OSCAD_COMPILE_BINARY(EqualityOp)
                OSCAD_COMPILE_BINARY(InequalityOp)
                OSCAD_COMPILE_BINARY(GreaterThanOp)
                OSCAD_COMPILE_BINARY(GreaterThanOrEqualOp)
                OSCAD_COMPILE_BINARY(LessThanOp)
                OSCAD_COMPILE_BINARY(LessThanOrEqualOp)
                OSCAD_COMPILE_BINARY(BitwiseOrOp)
                OSCAD_COMPILE_BINARY(BitwiseAndOp)
                OSCAD_COMPILE_BINARY(BitwiseShiftLeftOp)
                OSCAD_COMPILE_BINARY(BitwiseShiftRightOp)
#undef OSCAD_COMPILE_BINARY

            default:
                // Safety net for any Expression NodeKind without its own
                // case above -- falls back to the interpreter for the
                // whole containing function, same as always. Not
                // maintained as an exhaustive list of exclusions in this
                // comment (that list has gone stale before -- see git
                // history); every construct known to matter as of this
                // writing (including echo()/assert()/import()/object(),
                // previously excluded here) now has its own case.
                throw NotCompilable{};
        }
    }

    // Mirrors Evaluator::evalListElement's own per-clause dispatch exactly
    // (same 6 NodeKind cases, same recursive structure) -- but rather than
    // returning/appending into a caller-supplied `vector<Value>&`, every
    // case here emits bytecode that appends into whichever ACCUM_OPEN
    // accumulator is currently topmost at runtime (see bytecode.hpp's own
    // Accum*/Iter* opcode comments). Called once per top-level element of
    // a ListComprehension that contains at least one real clause (see
    // compileExpr's own ListComprehension case); the default (plain-
    // expression) case is what handles every element in a "no real clause"
    // list too, when reached via a clause's own nested body.
    void compileListElement(const oscad::ASTNode& elem, std::vector<Instruction>& out, CompileScope& scope) {
        using oscad::NodeKind;
        trackSpan(elem);
        switch (elem.kind()) {
            case NodeKind::ListCompFor: {
                auto& n = static_cast<const oscad::ListCompFor&>(elem);
                // Each assignment's RHS is evaluated exactly once, against
                // the OUTER scope (never seeing an earlier sibling
                // assignment's own loop variable) -- mirrors the
                // interpreter's own upfront `pairs.push_back(...)` loop,
                // and vector_element.hpp's own doc comment on this
                // asymmetry with ListCompCFor.
                std::vector<int> iterIds;
                iterIds.reserve(n.assignments.size());
                for (const auto& assign : n.assignments) {
                    compileExpr(*assign->expr, out, scope);
                    int id = nextIterList_++;
                    iterIds.push_back(id);
                    out.push_back({Op::IterMaterialize, id, 0, &assign->position()});
                }
                scope.push();
                const bool isNestedLc = (n.body->kind() == NodeKind::ListComprehension);
                std::function<void(size_t)> emitDim = [&](size_t depth) {
                    if (depth == n.assignments.size()) {
                        if (isNestedLc) {
                            compileExpr(static_cast<const oscad::Expression&>(*n.body), out, scope);
                            out.push_back({Op::AccumAppendOne, 0, 0, nullptr});
                        } else {
                            compileListCompBody(*n.body, out, scope);
                        }
                        return;
                    }
                    // Every re-entry (every outer-dimension iteration)
                    // restarts this dimension from its already-materialized
                    // values -- IterReset, not another IterMaterialize (the
                    // RHS expression itself is never re-evaluated).
                    if (depth > 0) out.push_back({Op::IterReset, iterIds[depth], 0, nullptr});
                    int loopVarSlot = declareLocal(scope, n.assignments[depth]->name->name);
                    size_t loopStart = out.size();
                    Instruction iterNext;
                    iterNext.op = Op::IterNext;
                    iterNext.a = loopVarSlot;
                    iterNext.b = iterIds[depth];
                    size_t iterNextIdx = out.size();
                    out.push_back(iterNext);
                    emitDim(depth + 1);
                    out.push_back({Op::Jump, static_cast<int>(loopStart), 0, nullptr});
                    out[iterNextIdx].c = static_cast<int>(out.size());
                };
                emitDim(0);
                scope.pop();
                return;
            }
            case NodeKind::ListCompCFor: {
                auto& n = static_cast<const oscad::ListCompCFor&>(elem);
                scope.push();
                // One shared scope for the WHOLE loop (not per-iteration) --
                // mirrors the interpreter's single `loopCtx` exactly, so a
                // c-style for's own variables persist/mutate across
                // iterations instead of getting a fresh binding each time.
                // Inits/incrs write via plain STORE_LOCAL unconditionally
                // (never StoreDyn even for a $-prefixed name) -- matches
                // the interpreter's own `loopCtx.let_->set(...)` call,
                // which has no $-branch either (an asymmetry with LetOp
                // this compiler reproduces rather than "fixes").
                for (const auto& assign : n.inits) {
                    compileExpr(*assign->expr, out, scope);
                    int slot = declareLocal(scope, assign->name->name);
                    out.push_back({Op::StoreLocal, slot, 0, &assign->position()});
                }
                // Pre-declare every incr-list name NOW, before compiling the
                // condition/body/incr expressions below -- an incr name
                // introduced fresh (not in the init list, e.g. BOSL2's
                // skin.scad: `best_i = result[0]<bestcost ? i : best_i,`)
                // can be READ, including by its OWN incr assignment's RHS
                // (a self-reference to its prior iteration's value) or by
                // the loop body, before its OWN StoreLocal below would
                // otherwise have declared it. Without this, that read
                // compiles to Op::LoadFree (the "not a known local/upvalue/
                // dyn-var, might be dynamic, warn if truly missing"
                // fallback) -- permanently, since compilation happens once
                // but the resulting instruction runs every iteration: every
                // read silently misses the real value stored into the
                // local slot moments later at runtime, not just an extra
                // warning but a genuinely wrong result (confirmed: BOSL2's
                // own best_i accumulator came back undef on every read).
                // Declaring the slot here first means the SAME reads below
                // resolve as ordinary Op::LoadLocal instead.
                for (const auto& assign : n.incrs) {
                    if (!scope.resolve(assign->name->name)) declareLocal(scope, assign->name->name);
                }
                int counterSlot = nextSlot_++;
                out.push_back({Op::PushConst, internConst(Value{0.0}), 0, nullptr});
                out.push_back({Op::StoreLocal, counterSlot, 0, nullptr});

                size_t loopStart = out.size();
                compileExpr(*n.condition, out, scope);
                size_t jend = out.size();
                out.push_back({Op::JumpIfFalse, 0, 0, nullptr});
                Instruction checkLimit;
                checkLimit.op = Op::CheckIterLimit;
                checkLimit.a = counterSlot;
                checkLimit.b = 1'000'000;
                checkLimit.node = &n;
                out.push_back(checkLimit);
                const bool isNestedLc = (n.body->kind() == NodeKind::ListComprehension);
                if (isNestedLc) {
                    compileExpr(static_cast<const oscad::Expression&>(*n.body), out, scope);
                    out.push_back({Op::AccumAppendOne, 0, 0, nullptr});
                } else {
                    compileListCompBody(*n.body, out, scope);
                }
                for (const auto& assign : n.incrs) {
                    compileExpr(*assign->expr, out, scope);
                    auto slot = scope.resolve(assign->name->name);
                    // An incr assignment can introduce a brand-new name
                    // never present in the init list, then have a LATER
                    // incr expression in the same list read it -- e.g.
                    // BOSL2's nurbs.scad: `inc_k = ...; kind = inc_k ? ...
                    // : kind;` (verified against real OpenSCAD and this
                    // port's own AST interpreter, evalListElement's
                    // ListCompCFor case in expr_eval.cpp -- both handle it
                    // correctly). The old assumption here ("incrs always
                    // reference an already-declared init name") was wrong
                    // and silently dereferenced a disengaged optional (UB)
                    // whenever it didn't hold. Declare a fresh slot on
                    // first appearance, exactly like an init assignment.
                    int resolvedSlot = slot ? *slot : declareLocal(scope, assign->name->name);
                    out.push_back({Op::StoreLocal, resolvedSlot, 0, &assign->position()});
                }
                out.push_back({Op::Jump, static_cast<int>(loopStart), 0, nullptr});
                out[jend].a = static_cast<int>(out.size());
                scope.pop();
                return;
            }
            case NodeKind::ListCompIf: {
                auto& n = static_cast<const oscad::ListCompIf&>(elem);
                compileExpr(*n.condition, out, scope);
                size_t j = out.size();
                out.push_back({Op::JumpIfFalse, 0, 0, nullptr});
                compileListCompBody(*n.trueExpr, out, scope);
                out[j].a = static_cast<int>(out.size());
                return;
            }
            case NodeKind::ListCompIfElse: {
                auto& n = static_cast<const oscad::ListCompIfElse&>(elem);
                compileExpr(*n.condition, out, scope);
                size_t j = out.size();
                out.push_back({Op::JumpIfFalse, 0, 0, nullptr});
                compileListCompBody(*n.trueExpr, out, scope);
                size_t jend = out.size();
                out.push_back({Op::Jump, 0, 0, nullptr});
                out[j].a = static_cast<int>(out.size());
                compileListCompBody(*n.falseExpr, out, scope);
                out[jend].a = static_cast<int>(out.size());
                return;
            }
            case NodeKind::ListCompLet: {
                auto& n = static_cast<const oscad::ListCompLet&>(elem);
                scope.push();
                size_t placeholderIdx = out.size();
                out.push_back({Op::OpenLocalScope, 0, 0, nullptr});
                int slotStart = nextSlot_;
                for (const auto& assign : n.assignments) {
                    compileExpr(*assign->expr, out, scope);
                    const std::string& name = assign->name->name;
                    if (!name.empty() && name[0] == '$') {
                        out.push_back({Op::StoreDyn, internName(name), 0, &assign->position()});
                    } else {
                        int slot = declareLocal(scope, name);
                        out.push_back({Op::StoreLocal, slot, 0, &assign->position()});
                    }
                }
                out[placeholderIdx].a = slotStart;
                out[placeholderIdx].b = nextSlot_ - slotStart;
                compileListCompBody(*n.body, out, scope);
                scope.pop();
                return;
            }
            case NodeKind::ListCompEach: {
                auto& n = static_cast<const oscad::ListCompEach&>(elem);
                const oscad::ASTNode& inner = *n.body;
                if (isListCompClauseKind(inner.kind())) {
                    // Flatten EVERY item the inner clause itself
                    // contributes, individually -- mirrors `for (item :
                    // evalListCompBody(inner, ctx)) appendEachInto(out,
                    // item)` exactly; see AccumMergeEach's own doc comment
                    // (bytecode.hpp) for why this can't be "close the inner
                    // accumulator as one ValueList then AccumAppendEach
                    // once" instead (that would flatten only one level,
                    // not per-item).
                    out.push_back({Op::AccumOpen, 0, 0, nullptr});
                    compileListElement(inner, out, scope);
                    out.push_back({Op::AccumMergeEach, 0, 0, nullptr});
                } else {
                    compileExpr(static_cast<const oscad::Expression&>(inner), out, scope);
                    out.push_back({Op::AccumAppendEach, 0, 0, nullptr});
                }
                return;
            }
            default:
                compileExpr(static_cast<const oscad::Expression&>(elem), out, scope);
                out.push_back({Op::AccumAppendOne, 0, 0, nullptr});
                return;
        }
    }

    // Mirrors Evaluator::evalListCompBody exactly: a body that is itself a
    // plain vector literal is compiled as ONE self-contained value (its own
    // nested accumulator, via compileExpr's ListComprehension case) and
    // appended as a single item; anything else recurses into
    // compileListElement (which may itself be another chained clause).
    void compileListCompBody(const oscad::ASTNode& body, std::vector<Instruction>& out, CompileScope& scope) {
        if (body.kind() == oscad::NodeKind::ListComprehension) {
            compileExpr(static_cast<const oscad::Expression&>(body), out, scope);
            out.push_back({Op::AccumAppendOne, 0, 0, nullptr});
            return;
        }
        compileListElement(body, out, scope);
    }

    // -- Module-body statement-list compilation (Stage 2) -----------------
    // See tryCompileModuleBody's own doc comment (bytecode_compiler.hpp)
    // for the overall shape: assignment/if/for get real bytecode (Jump-
    // based control flow, so a recursive module call nested inside one
    // doesn't hide behind a native call boundary); a resolved user-module
    // call gets Op::CallModule; everything else -- echo/assert/let-blocks/
    // modifiers/intersection_for, a builtin or unresolved module call, a
    // plain assignment's own STORE (its RHS value, unlike a function's,
    // is never slot-addressed here -- see this file's own module-chunk
    // doc comment, bytecode.hpp, for why module bodies never allocate
    // slots at all) -- is a native passthrough (Op::NativeStatement),
    // exactly the same dispatch evalChildren's own per-statement loop
    // already does for that one node.

    int internNativeExpr(const oscad::Expression* expr) {
        chunk_.nativeExprs.push_back(expr);
        return static_cast<int>(chunk_.nativeExprs.size()) - 1;
    }
    int internNativeStatement(const oscad::ASTNode* node) {
        chunk_.nativeStatements.push_back(node);
        return static_cast<int>(chunk_.nativeStatements.size()) - 1;
    }

    // Mirrors evalChildren's own assignments-then-others partition
    // (stmt_eval.cpp) exactly -- OpenSCAD runs every assignment in a scope
    // before anything else, each group preserving its own source order.
    // Reordering at COMPILE time (rather than delegating each statement
    // in SOURCE order and relying on some runtime reordering) is what lets
    // every statement -- assignment or not -- collapse to the same simple
    // "compile once, in the right position" treatment.
    void compileStatementList(const std::vector<std::unique_ptr<oscad::ASTNode>>& children, std::vector<Instruction>& out) {
        std::vector<const oscad::ASTNode*> raw;
        raw.reserve(children.size());
        for (const auto& c : children) raw.push_back(c.get());
        compileStatementList(raw, out);
    }

    // Same, for a list of raw (non-owning) pointers -- Evaluator::
    // evalChildren's own primary overload already receives its `children`
    // this shape (a caller-owned list, e.g. a builtin module's own
    // node.children, or a for-loop's freshly-built bodyNodes vector, not
    // necessarily one this Compiler's own declaration owns) -- see
    // tryCompileChildrenList's own doc comment, below, for the entry point
    // that needs this shape directly.
    void compileStatementList(const std::vector<const oscad::ASTNode*>& children, std::vector<Instruction>& out) {
        std::vector<const oscad::ASTNode*> assignments;
        std::vector<const oscad::ASTNode*> others;
        for (const oscad::ASTNode* c : children) {
            (c->kind() == oscad::NodeKind::Assignment ? assignments : others).push_back(c);
        }
        for (const oscad::ASTNode* stmt : assignments) compileOneStatement(*stmt, out);
        for (const oscad::ASTNode* stmt : others) compileOneStatement(*stmt, out);
    }

    // Compiles one inline sub-expression wrapped in Op::OpenExprScope/
    // CloseExprScope -- see those ops' own doc comment (bytecode.hpp) for
    // why every module-body statement's own inline-compiled RHS/argument
    // needs this (a `$`-write leak this session's own regression test,
    // UserFunction.DollarVarLetAsAssignmentRhsDoesNotLeak, caught for
    // real). Shared by Assignment/ModularEcho/ModularAssert's own
    // compileOneStatement cases, below.
    void compileIsolatedExpr(const oscad::Expression& expr, std::vector<Instruction>& out, CompileScope& scope) {
        out.push_back({Op::OpenExprScope, 0, 0, nullptr});
        compileExpr(expr, out, scope);
        out.push_back({Op::CloseExprScope, 0, 0, nullptr});
    }

    // Shared by ModularCall's Transform/Color-kind branch and the 3 real
    // modifier cases (ModularModifierHighlight/Background/ShowOnly),
    // below -- emits one Op::PushBuiltinWrap/PopBuiltinWrap bracket around
    // `children` compiled inline (exactly like ModularLet's own body
    // does, via this same compileStatementList recursion). See
    // Op::PushBuiltinWrap's own doc comment (bytecode.hpp) for the full
    // design and why this specific set doesn't fall to Op::NativeStatement
    // like other builtins-with-children still do.
    //
    // Op::CheckDebugStatement emitted first, mirroring ModularEcho/
    // ModularAssert's own pattern (both above) -- NOT Op::CallModule's own
    // lack of one: that omission is specific to a resolved USER MODULE
    // call, whose own body statements each get their own check instead
    // (mirrors evalUserModule's "no body-entry checkDebug of its own",
    // evaluator.hpp). A builtin wrap statement isn't "control transferring
    // to a declaration" the way a module call is -- it's a plain statement
    // representing real work happening HERE, exactly like echo/assert/
    // assignment, which is why those get their own checkpoint. Confirmed
    // for real: DebugHooks.FastContinueNotHookSkippableStillFiresEvery-
    // Checkpoint (a translate()-wrapped script) caught the miscount when
    // this was first omitted by analogy to Op::CallModule.
    void emitBuiltinWrap(CompiledChunk::BuiltinWrapSite::Kind kind, const std::string& tagName,
                          const oscad::ASTNode& wrapperNode, const std::vector<const oscad::ASTNode*>& children,
                          std::vector<Instruction>& out) {
        out.push_back({Op::CheckDebugStatement, internNativeStatement(&wrapperNode), 0, nullptr});
        CompiledChunk::BuiltinWrapSite site;
        site.kind = kind;
        site.tagName = tagName;
        site.node = &wrapperNode;
        chunk_.builtinWrapSites.push_back(std::move(site));
        const int idx = static_cast<int>(chunk_.builtinWrapSites.size()) - 1;
        out.push_back({Op::PushBuiltinWrap, idx, 0, &wrapperNode.position()});
        compileStatementList(children, out);
        out.push_back({Op::PopBuiltinWrap, idx, 0, &wrapperNode.position()});
    }

    void compileOneStatement(const oscad::ASTNode& stmt, std::vector<Instruction>& out) {
        using oscad::NodeKind;
        trackSpan(stmt);
        // A statement's own .scope() can differ from this whole chunk's
        // fixed scope_ member (e.g. a `use` statement earlier in the SAME
        // script/module body changes what later statements can see) --
        // compileExpr's own PrimaryCall case resolves a callee via scope_
        // DIRECTLY, not per-node, unlike the ModularCall case just below
        // (which already does its own stmt.scope()-first lookup). Any
        // inline sub-expression compiled from Assignment/ModularEcho/
        // ModularAssert, below, needs scope_ swapped to match THIS
        // statement for its own duration, or a nested function call inside
        // it resolves against the wrong (chunk-wide) scope instead of its
        // own. Caught for real: UseStatement.NestedUseNotReExported
        // started spuriously resolving a NOT-re-exported nested `use`'s
        // function once echo() got its own inline-compiled argument.
        // RAII (not "restore before every return") since NotCompilable can
        // unwind through here -- harmless either way (the whole Compiler
        // is discarded on that path), but cheap to get right regardless.
        struct ScopeGuard {
            const oscad::Scope*& scope;
            const oscad::Scope* saved;
            ScopeGuard(const oscad::Scope*& s, const oscad::ASTNode& node) : scope(s), saved(s) {
                if (const oscad::Scope* stmtScope = node.scope()) scope = stmtScope;
            }
            ~ScopeGuard() { scope = saved; }
        } scopeGuard(scope_, stmt);
        switch (stmt.kind()) {
            // Pure declarations, no-ops at statement-eval time (already
            // hoisted into scope by buildScopes()) -- matches evalStatement's
            // own default case; not even worth a NativeStatement entry.
            case NodeKind::ModuleDeclaration:
            case NodeKind::FunctionDeclaration:
                return;
            // Assignment/ModularEcho/ModularAssert: real bytecode instead
            // of Op::NativeStatement -- a throughput improvement (see
            // CheckDebugStatement/StoreModuleVar/AssertStatement's own doc
            // comments, bytecode.hpp), NOT a recursion-safety one (these
            // were never the risk Stage 2 targeted -- see NativeStatement's
            // own doc comment). Assignment specifically closes a real
            // regression: it used to get a genuinely optimized path
            // (Evaluator::tryRunCompiledAssignmentBlock) via evalChildren's
            // own native fallback loop, but tryRunCompiledChildren's
            // broader whole-list compile (the NativeStatement-gap fix)
            // ALWAYS succeeds first now, silently shadowing it -- every
            // assignment fell back to generic Op::NativeStatement, one
            // nested driveVm call per sub-expression via
            // evalExprMaybeCompiled, instead of this inline form.
            case NodeKind::Assignment: {
                auto& n = static_cast<const oscad::Assignment&>(stmt);
                out.push_back({Op::CheckDebugStatement, internNativeStatement(&stmt), 0, nullptr});
                CompileScope exprScope; // module-level vars are never slot-addressed --
                                         // see StoreModuleVar's own doc comment.
                compileIsolatedExpr(*n.expr, out, exprScope);
                out.push_back({Op::StoreModuleVar, internName(n.name->name), 0, &n.position()});
                return;
            }
            case NodeKind::ModularEcho: {
                // Op::Echo itself is reused verbatim from the EXPRESSION
                // form's own compile case (AssertOp's sibling, above in
                // compileExpr) -- already statement-shaped (no value
                // pushed, straight fall-through), so nothing new is needed
                // there. `.children` is deliberately never read here,
                // matching evalStatement's own ModularEcho case (doEcho
                // only ever takes `.arguments` -- an echo statement's
                // trailing children, if the grammar even produces any, are
                // not evaluated today; this mirrors that exactly rather
                // than silently changing behavior).
                auto& n = static_cast<const oscad::ModularEcho&>(stmt);
                out.push_back({Op::CheckDebugStatement, internNativeStatement(&stmt), 0, nullptr});
                CompiledChunk::EchoSite site;
                CompileScope exprScope;
                for (const auto& argPtr : n.arguments) {
                    if (argPtr->kind() == NodeKind::NamedArgument) {
                        auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
                        compileIsolatedExpr(*a.expr, out, exprScope);
                        site.argNames.push_back(a.name->name);
                    } else {
                        auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
                        compileIsolatedExpr(*a.expr, out, exprScope);
                        site.argNames.push_back(std::nullopt);
                    }
                }
                int siteIdx = static_cast<int>(chunk_.echoSites.size());
                chunk_.echoSites.push_back(std::move(site));
                out.push_back({Op::Echo, siteIdx, static_cast<int>(n.arguments.size()), &n.position()});
                return;
            }
            case NodeKind::ModularAssert: {
                // Genuinely different contract from AssertOp's own compiled
                // form (Op::AssertFail): the statement form supports named
                // arguments AND evaluates every argument EAGERLY (mirrors
                // Evaluator::evalAssertStatement's own resolveArgs() call
                // exactly -- unlike AssertOp's lazily-compiled message,
                // guarded by a runtime JumpIfTrue), so every argument is
                // compiled+pushed here unconditionally, in source order.
                auto& n = static_cast<const oscad::ModularAssert&>(stmt);
                out.push_back({Op::CheckDebugStatement, internNativeStatement(&stmt), 0, nullptr});
                CompiledChunk::AssertSite site;
                site.node = &n;
                site.argCount = static_cast<int>(n.arguments.size());
                CompileScope exprScope;
                for (size_t i = 0; i < n.arguments.size(); ++i) {
                    compileIsolatedExpr(*argExpr(*n.arguments[i]), out, exprScope);
                    if (n.arguments[i]->kind() == NodeKind::NamedArgument) {
                        auto& na = static_cast<const oscad::NamedArgument&>(*n.arguments[i]);
                        if (na.name->name == "condition") site.conditionArgIndex = static_cast<int>(i);
                        else if (na.name->name == "message") site.messageArgIndex = static_cast<int>(i);
                    }
                }
                // Second pass: positional 0/1 fill whichever logical
                // parameter a named arg didn't already claim -- run AFTER
                // the named pass above (not interleaved) so a named arg
                // wins regardless of its position relative to its
                // positional counterpart in source, matching
                // Evaluator::getArg's own "named first" priority exactly.
                int posCounter = 0;
                for (size_t i = 0; i < n.arguments.size(); ++i) {
                    if (n.arguments[i]->kind() == NodeKind::NamedArgument) continue;
                    if (posCounter == 0 && !site.conditionArgIndex) site.conditionArgIndex = static_cast<int>(i);
                    else if (posCounter == 1 && !site.messageArgIndex) site.messageArgIndex = static_cast<int>(i);
                    ++posCounter;
                }
                // Precomputed once, like AssertFail's own condText --
                // mirrors evalAssertStatement's `node.arguments.empty() ?
                // "false" : argExpr(*node.arguments[0])->toString()`
                // exactly (a MISSING condition arg -- e.g. a bare
                // `assert(message="x");` -- reads the same as an EMPTY
                // arg list here: no arg supplies "condition" either way).
                site.condTextConstIdx = internConst(Value{
                    site.conditionArgIndex
                        ? argExpr(*n.arguments[static_cast<size_t>(*site.conditionArgIndex)])->toString()
                        : std::string("false")});
                int siteIdx = static_cast<int>(chunk_.assertSites.size());
                chunk_.assertSites.push_back(std::move(site));
                out.push_back({Op::AssertStatement, siteIdx, 0, &n.position()});
                return;
            }
            case NodeKind::ModularLet: {
                // Mirrors Evaluator::evalLetBlock exactly (stmt_eval.cpp):
                // every assignment's RHS is evaluated against the
                // ORIGINAL (parent) ctx, never an earlier sibling's own
                // write in THIS SAME let-block (a documented, deliberate
                // divergence from the let-EXPRESSION form's sequential
                // visibility) -- so every RHS is compiled+evaluated FIRST,
                // all left on f.stack, and only written into the freshly-
                // opened child scope (Op::OpenLetScope) AFTER every one of
                // them has already run against the still-current PARENT
                // ctx. No outer statement-level check for the let-block
                // node itself (matches evalChildren's own ModularLet
                // exclusion) -- each assignment gets its own instead,
                // exactly like evalLetBlock's own per-assignment
                // checkDebug. The body (n.children) compiles inline,
                // recursively, against the now-current child ctx -- same
                // Compiler instance, so nested statements get every bit of
                // this same real-bytecode treatment too.
                auto& n = static_cast<const oscad::ModularLet&>(stmt);
                CompileScope exprScope;
                for (const auto& assign : n.assignments) {
                    out.push_back({Op::CheckDebugStatement, internNativeStatement(assign.get()), 0, nullptr});
                    compileIsolatedExpr(*assign->expr, out, exprScope);
                }
                out.push_back({Op::OpenLetScope, 0, 0, nullptr});
                // Values pop in REVERSE push order (LIFO) -- iterate the
                // assignment list backwards so each Op::StoreLetVar is
                // paired with the value that was actually pushed for IT.
                for (auto it = n.assignments.rbegin(); it != n.assignments.rend(); ++it) {
                    out.push_back({Op::StoreLetVar, internName((*it)->name->name), 0, &(*it)->position()});
                }
                compileStatementList(n.children, out);
                out.push_back({Op::CloseExprScope, 0, 0, nullptr});
                return;
            }
            case NodeKind::ModularCall: {
                auto& call = static_cast<const oscad::ModularCall&>(stmt);
                const oscad::Scope* lookupScope = stmt.scope() ? stmt.scope() : scope_;
                const oscad::ASTNode* resolved = lookupScope ? lookupScope->lookupModule(call.name->name) : nullptr;
                if (resolved && resolved->kind() == NodeKind::ModuleDeclaration) {
                    CompiledChunk::ModuleCallSite site;
                    site.decl = static_cast<const oscad::ModuleDeclaration*>(resolved);
                    site.callNode = &call;
                    site.calleeName = call.name->name;
                    chunk_.moduleCallSites.push_back(std::move(site));
                    out.push_back(
                        {Op::CallModule, static_cast<int>(chunk_.moduleCallSites.size()) - 1, 0, &call.position()});
                    return;
                }
                // A user module never shadows a builtin transform/color
                // name in a way that reaches here -- the `resolved` check
                // above already ruled that out (isBuiltin per
                // evalModularCall's own comment). So a name found in
                // resolveDispatch() here is unambiguously the real C++
                // builtin. Detected by FUNCTION-POINTER identity against
                // &resolveTransform/&resolveColor, not a second,
                // independently-maintained name list -- stays in sync
                // with registry.cpp automatically if a 7th transform name
                // is ever added there. See Op::PushBuiltinWrap's own doc
                // comment (bytecode.hpp) for why this specific subset
                // gets real bytecode instead of falling to
                // Op::NativeStatement below like every other builtin
                // still does.
                const auto& dispatch = resolveDispatch();
                auto dispatchIt = dispatch.find(call.name->name);
                std::optional<CompiledChunk::BuiltinWrapSite::Kind> wrapKind;
                if (dispatchIt != dispatch.end()) {
                    if (dispatchIt->second == &resolveTransform) {
                        wrapKind = CompiledChunk::BuiltinWrapSite::Kind::Transform;
                    } else if (dispatchIt->second == &resolveColor) {
                        wrapKind = CompiledChunk::BuiltinWrapSite::Kind::Color;
                    }
                }
                if (wrapKind) {
                    std::vector<const oscad::ASTNode*> children;
                    children.reserve(call.children.size());
                    for (const auto& c : call.children) children.push_back(c.get());
                    emitBuiltinWrap(*wrapKind, call.name->name, call, children, out);
                    return;
                }
                // children() -- the runtime-varying forwarding builtin,
                // detected by the same function-pointer-identity pattern
                // as Transform/Color above. Deliberately NO preceding
                // Op::CheckDebugStatement (unlike emitBuiltinWrap's own
                // emission): CallChildren's handler fires checkDebug
                // itself against the SCOPE-WRAPPED ctx, byte-for-byte
                // matching Op::NativeStatement's own handler -- the
                // CheckDebugStatement handler passes the un-wrapped ctx,
                // which would be a subtle behavior change for this node.
                // See Op::CallChildren's own doc comment (bytecode.hpp).
                if (dispatchIt != dispatch.end() && dispatchIt->second == &resolveChildren) {
                    out.push_back({Op::CallChildren, internNativeStatement(&stmt), 0, &stmt.position()});
                    return;
                }
                // Every other builtin, or a name that didn't resolve to a
                // user module statically -- native passthrough, exactly
                // like echo/assert/etc. below. Never the recursion-depth
                // risk this compiler targets for THESE (see
                // NativeStatement's own doc comment, bytecode.hpp).
                out.push_back({Op::NativeStatement, internNativeStatement(&stmt), 0, &stmt.position()});
                return;
            }
            case NodeKind::ModularModifierHighlight: {
                auto& n = static_cast<const oscad::ModularModifierHighlight&>(stmt);
                emitBuiltinWrap(CompiledChunk::BuiltinWrapSite::Kind::Modifier, "highlight", n, {n.child.get()}, out);
                return;
            }
            case NodeKind::ModularModifierBackground: {
                auto& n = static_cast<const oscad::ModularModifierBackground&>(stmt);
                emitBuiltinWrap(CompiledChunk::BuiltinWrapSite::Kind::Modifier, "background", n, {n.child.get()}, out);
                return;
            }
            case NodeKind::ModularModifierShowOnly: {
                auto& n = static_cast<const oscad::ModularModifierShowOnly&>(stmt);
                emitBuiltinWrap(CompiledChunk::BuiltinWrapSite::Kind::Modifier, "show_only", n, {n.child.get()}, out);
                return;
            }
            case NodeKind::ModularIf: {
                auto& n = static_cast<const oscad::ModularIf&>(stmt);
                const int jumpFalseIdx = static_cast<int>(out.size());
                out.push_back({Op::NativeCondJumpIfFalse, internNativeExpr(n.condition.get()), 0, &n.condition->position()});
                const oscad::ASTNode* marker = n.trueBranch.empty() ? &stmt : n.trueBranch.front().get();
                out.push_back({Op::NativeCheckDebugExprLevel, internNativeStatement(marker), 0, nullptr});
                compileStatementList(n.trueBranch, out);
                out[static_cast<size_t>(jumpFalseIdx)].b = static_cast<int>(out.size());
                return;
            }
            case NodeKind::ModularIfElse: {
                auto& n = static_cast<const oscad::ModularIfElse&>(stmt);
                const int jumpFalseIdx = static_cast<int>(out.size());
                out.push_back({Op::NativeCondJumpIfFalse, internNativeExpr(n.condition.get()), 0, &n.condition->position()});
                const oscad::ASTNode* trueMarker = n.trueBranch.empty() ? &stmt : n.trueBranch.front().get();
                out.push_back({Op::NativeCheckDebugExprLevel, internNativeStatement(trueMarker), 0, nullptr});
                compileStatementList(n.trueBranch, out);
                const int jumpEndIdx = static_cast<int>(out.size());
                out.push_back({Op::Jump, 0, 0, nullptr});
                out[static_cast<size_t>(jumpFalseIdx)].b = static_cast<int>(out.size());
                const oscad::ASTNode* falseMarker = n.falseBranch.empty() ? &stmt : n.falseBranch.front().get();
                out.push_back({Op::NativeCheckDebugExprLevel, internNativeStatement(falseMarker), 0, nullptr});
                compileStatementList(n.falseBranch, out);
                out[static_cast<size_t>(jumpEndIdx)].a = static_cast<int>(out.size());
                return;
            }
            case NodeKind::ModularFor: {
                compileForLoop(static_cast<const oscad::ModularFor&>(stmt), out);
                return;
            }
            default:
                // ModularModifierDisable (`*`) -- deliberately native: its
                // child never evaluates at all (see evalStatement's own
                // ModularModifierDisable case, stmt_eval.cpp), so there's
                // no recursion to eliminate here in the first place.
                // intersection_for -- deliberately native (bespoke
                // per-statement grouping, same reasoning as union/
                // difference/intersection; not covered by
                // Op::PushBuiltinWrap, see that op's own doc comment).
                // `#`/`%`/`!` and translate/rotate/scale/mirror/multmatrix/
                // resize/color have their own real bytecode cases now,
                // above; echo/assert/assignment/let-block already did.
                out.push_back({Op::NativeStatement, internNativeStatement(&stmt), 0, &stmt.position()});
                return;
        }
    }

    // Cartesian nested loop over N assignments, unrolled into flat Jump-
    // based code at COMPILE time (bounded by the source's own for-clause
    // count, always small -- never runtime-driven) rather than the
    // interpreter's own runtime recursion (evalFor's `recurse(depth+1,
    // ...)`, stmt_eval.cpp). Each dimension: materialize once (native RHS
    // eval + expandIterable), then a ForIterNext/ForIterEnd pair wrapping
    // everything inner -- see Op::ForIterNext/ForIterEnd's own doc
    // comments (bytecode.hpp) for exactly why a fresh per-iteration ctx
    // (not an in-place mutation) is required for correctness, not just
    // parity.
    void compileForLoop(const oscad::ModularFor& n, std::vector<Instruction>& out) {
        const size_t numDims = n.assignments.size();
        std::vector<int> iterListIds(numDims);
        for (size_t d = 0; d < numDims; ++d) {
            iterListIds[d] = nextIterList_++;
            out.push_back({Op::NativeIterMaterialize, internNativeExpr(n.assignments[d]->expr.get()),
                            iterListIds[d], &n.assignments[d]->position()});
        }
        std::vector<size_t> topIdx(numDims);
        std::vector<size_t> forIterNextIdx(numDims);
        for (size_t d = 0; d < numDims; ++d) {
            // Every dimension but the outermost needs its own IterList
            // index reset to 0 each time it's (re-)entered from the
            // ENCLOSING dimension's own successful bind -- NativeIterMaterialize
            // only resets it once, up front, before ANY iteration runs; by
            // the second (and every later) outer-dimension value, this
            // dimension's own index is still sitting at "exhausted" from
            // the PREVIOUS pass, and would immediately look exhausted
            // again without this. Placed strictly BEFORE this dimension's
            // own ForIterNext/topIdx (not at it) so this dimension's OWN
            // "try the next value" jump (its own ForIterEnd, below) lands
            // AT ForIterNext directly and skips the reset -- only the
            // fall-through from the enclosing dimension's bind passes
            // through it. Harmless (a no-op) the very first time, since
            // the index is already 0 from NativeIterMaterialize then.
            if (d > 0) out.push_back({Op::IterReset, iterListIds[d], 0, nullptr});
            topIdx[d] = out.size();
            forIterNextIdx[d] = out.size();
            Instruction ins;
            ins.op = Op::ForIterNext;
            ins.a = internName(n.assignments[d]->name->name);
            ins.b = iterListIds[d];
            ins.node = n.assignments[d].get();
            out.push_back(ins); // ins.c (exhaustion target) patched below
        }
        const oscad::ASTNode* marker = n.body.empty() ? static_cast<const oscad::ASTNode*>(&n) : n.body.front().get();
        out.push_back({Op::NativeCheckDebugExprLevel, internNativeStatement(marker), 0, nullptr});
        compileStatementList(n.body, out);
        for (size_t i = numDims; i-- > 0;) {
            out.push_back({Op::ForIterEnd, static_cast<int>(topIdx[i]), 0, nullptr});
            out[forIterNextIdx[i]].c = static_cast<int>(out.size());
        }
    }

private:
    CompiledChunk& chunk_;
    const oscad::Scope* scope_;
    const oscad::ASTNode* selfDecl_;
    std::vector<EnclosingLevel> enclosing_;
    int nextSlot_ = 0;
    int nextIterList_ = 0;
};

bool compileFunctionLike(CompiledChunk& chunk, const oscad::Scope* staticScope, const oscad::ASTNode* selfDecl,
                          std::vector<EnclosingLevel> enclosing,
                          const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& params,
                          const oscad::Expression& bodyExpr) {
    chunk.selfDecl = selfDecl;
    Compiler compiler(chunk, staticScope, selfDecl, std::move(enclosing));
    CompileScope bodyScope;
    bodyScope.push();
    for (const auto& p : params) {
        // $-prefixed parameters bind through ctx.dyn (dynamically scoped)
        // rather than a slot -- never declared as a local, so a reference
        // to one inside the body already compiles to Op::LoadDyn via the
        // Identifier case's own $-check regardless of scope visibility.
        // Still occupies its declared POSITION among all parameters (see
        // bindCompiledArgs, bytecode_vm.cpp) for positional-argument
        // matching, exactly like the interpreter's own bindArgs.
        const bool isDyn = !p->name->name.empty() && p->name->name[0] == '$';
        const int slot = isDyn ? 0 : compiler.declareLocal(bodyScope, p->name->name);
        chunk.params.push_back(CompiledChunk::Param{p->name->name, slot, isDyn});
    }
    chunk.defaultCode.resize(params.size());

    try {
        for (size_t i = 0; i < params.size(); ++i) {
            if (params[i]->defaultValue) {
                CompileScope defaultScope; // zero frames: isolated from every parameter, see class comment
                compiler.compileExpr(*params[i]->defaultValue, chunk.defaultCode[i], defaultScope);
            }
        }
        compiler.compileExpr(bodyExpr, chunk.bodyCode, bodyScope, /*tail=*/true);
    } catch (const NotCompilable&) {
        return false;
    }

    chunk.numSlots = compiler.nextSlot();
    chunk.numIterLists = compiler.nextIterList();
    return true;
}

} // namespace

std::optional<CompiledChunk> tryCompileFunction(const oscad::FunctionDeclaration& decl) {
    CompiledChunk chunk;
    if (!compileFunctionLike(chunk, decl.scope(), &decl, {}, decl.parameters, *decl.expr)) return std::nullopt;
    return chunk;
}

std::optional<CompiledChunk> tryCompileStatementExpr(const oscad::Expression& expr, const oscad::Scope* scope) {
    static const std::vector<std::unique_ptr<oscad::ParameterDeclaration>> kNoParams;
    CompiledChunk chunk;
    if (!compileFunctionLike(chunk, scope, nullptr, {}, kNoParams, expr)) return std::nullopt;
    // See this function's own doc comment (bytecode_compiler.hpp) for why a
    // captures-having nested closure can't be supported by this bare
    // wrapper -- selfDecl is nullptr and enclosing is empty above, so its
    // own upvalue(s) would target a CallStackFrame this path never pushes.
    // A zero-capture closure never reaches closureSites at all (see the
    // FunctionLiteral case's own PushConst early-return, above) so it's
    // unaffected by this check.
    if (!chunk.closureSites.empty()) return std::nullopt;
    return chunk;
}

std::optional<CompiledChunk> tryCompileAssignmentBlock(const std::vector<const oscad::Assignment*>& assigns,
                                                         const oscad::Scope* scope) {
    // Reassignment-warning fidelity (see this function's own doc comment,
    // bytecode_compiler.hpp) -- cheap, one-time scan before touching the
    // compiler at all.
    std::unordered_set<std::string> seenNames;
    int topLevelDollarAssignments = 0;
    for (const oscad::Assignment* a : assigns) {
        const std::string& name = a->name->name;
        if (!name.empty() && name[0] == '$') {
            ++topLevelDollarAssignments; // evalAssignment never warns for a $-name either -- not tracked in seenNames
        } else if (!seenNames.insert(name).second) {
            return std::nullopt;
        }
    }

    CompiledChunk chunk;
    Compiler compiler(chunk, scope, nullptr, {});
    CompileScope compileScope;
    compileScope.push();
    try {
        for (const oscad::Assignment* a : assigns) {
            compiler.compileExpr(*a->expr, chunk.bodyCode, compileScope); // RHS is never tail
            const std::string& name = a->name->name;
            if (!name.empty() && name[0] == '$') {
                chunk.bodyCode.push_back({Op::StoreDyn, compiler.internName(name), 0, &a->position()});
            } else {
                int slot = compiler.declareLocal(compileScope, name);
                chunk.bodyCode.push_back({Op::StoreLocalAndLet, slot, compiler.internName(name), &a->position()});
            }
        }
    } catch (const NotCompilable&) {
        return std::nullopt;
    }
    chunk.numSlots = compiler.nextSlot();
    chunk.numIterLists = compiler.nextIterList();

    // See this function's own doc comment for why: runCompiledAssignmentBlock
    // (bytecode_vm.cpp) runs directly against the caller's own ctx, with no
    // scope of its own for a nested let()'s dyn write to be contained by.
    if (!chunk.closureSites.empty()) return std::nullopt;
    // Every StoreDyn this loop itself emitted above is accounted for by
    // topLevelDollarAssignments; any MORE than that in the compiled body
    // can only have come from a nested let()/list-comprehension-let clause
    // (StoreDyn's only other two emission sites) inside one of these
    // assignments' own RHS -- refuse the whole block rather than risk that
    // write leaking into the caller's ctx.dyn permanently.
    int storeDynCount = 0;
    for (const Instruction& ins : chunk.bodyCode) {
        if (ins.op == Op::StoreDyn) ++storeDynCount;
    }
    if (storeDynCount != topLevelDollarAssignments) return std::nullopt;
    return chunk;
}

std::optional<CompiledChunk> tryCompileModuleBody(const oscad::ModuleDeclaration& decl) {
    CompiledChunk chunk;
    chunk.isModule = true;
    chunk.selfDecl = &decl;
    // No params/defaultCode here -- a module's own parameters are bound
    // natively (Evaluator::buildModuleChildCtx, called by the CALLER
    // before this chunk's own body ever runs), never slot-addressed the
    // way a function's are. numSlots CAN still be nonzero though: a
    // nested let-EXPRESSION inside a compiled echo/assert/assignment
    // argument (Assignment/ModularEcho/ModularAssert/ModularLet's own
    // compileOneStatement cases) reuses LetOp's existing slot-allocating
    // compile path, just scoped to that one statement's own sub-
    // expression -- see this file's own module-chunk doc comment
    // (bytecode.hpp) for the full reasoning.
    Compiler compiler(chunk, decl.scope(), nullptr, {});
    try {
        compiler.compileStatementList(decl.children, chunk.bodyCode);
    } catch (const NotCompilable&) {
        return std::nullopt;
    }
    chunk.numIterLists = compiler.nextIterList();
    chunk.numSlots = compiler.nextSlot();
    return chunk;
}

std::optional<CompiledChunk> tryCompileChildrenList(const std::vector<const oscad::ASTNode*>& children,
                                                     const oscad::Scope* scope) {
    CompiledChunk chunk;
    // Same completion semantics as a module chunk (no return value, its
    // whole effect is the side effect of what lands in treeStack_) --
    // reuses runCompiledModuleBody's own bare-frame entry point as-is
    // (bytecode_vm.cpp) to run it, no new runtime machinery needed.
    // selfDecl stays null -- this list has no single declaration identity
    // of its own the way a ModuleDeclaration's body does (no upvalues are
    // ever resolved against it either way; module bodies don't create
    // escaping closures).
    chunk.isModule = true;
    Compiler compiler(chunk, scope, nullptr, {});
    try {
        compiler.compileStatementList(children, chunk.bodyCode);
    } catch (const NotCompilable&) {
        return std::nullopt;
    }
    chunk.numIterLists = compiler.nextIterList();
    chunk.numSlots = compiler.nextSlot();
    return chunk;
}

} // namespace oscadeval
