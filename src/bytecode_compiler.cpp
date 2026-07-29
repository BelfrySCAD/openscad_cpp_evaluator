#include "openscad_cpp_evaluator/bytecode_compiler.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/function_builtins.hpp"

#include "openscad_cpp_parser/ast/declarations.hpp"
#include "openscad_cpp_parser/ast/expression.hpp"
#include "openscad_cpp_parser/ast/vector_element.hpp"
#include "openscad_cpp_parser/scope.hpp"

#include <functional>
#include <unordered_map>

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
// for a $-prefixed parameter or any NotCompilable thrown while compiling.
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
            if (auto slot = it->scope->resolve(name)) return CompiledChunk::UpvalueRef{it->decl, *slot};
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
                if (!n.name.empty() && n.name[0] == '$') {
                    out.push_back({Op::LoadDyn, internName(n.name), 0, &n.position()});
                } else if (auto slot = scope.resolve(n.name)) {
                    out.push_back({Op::LoadLocal, *slot, 0, &n.position()});
                } else if (auto upvalSlot = resolveEnclosing(n.name)) {
                    int idx = static_cast<int>(chunk_.upvalues.size());
                    chunk_.upvalues.push_back(*upvalSlot);
                    out.push_back({Op::LoadUpvalue, idx, 0, &n.position()});
                } else {
                    out.push_back({Op::LoadFree, internName(n.name), 0, &n.position()});
                }
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
            // own module notes). If the literal itself fails to compile
            // (contains a call/echo/assert/unsupported construct, or its
            // own free-variable resolution needs an upvalue chain going
            // even deeper than what's captured here), the WHOLE containing
            // declaration must also bail -- propagated by simply
            // rethrowing, since an interpreter-executed literal nested
            // inside a compiled container would silently read stale/undef
            // values for the container's own (slot-only, never written to
            // ctx.let_) locals otherwise. On success, the literal's own
            // chunk is stashed in chunk_.nestedLiterals for
            // Evaluator::lookupOrCompileChunk to flatten into
            // literalChunkCache_; the emitted instruction just pushes the
            // AST pointer as a Value, exactly like the interpreter's own
            // `Value{&node}` (evalExpr's own FunctionLiteral case).
            case NodeKind::FunctionLiteral: {
                auto& n = static_cast<const oscad::FunctionLiteral&>(node);
                std::vector<EnclosingLevel> childEnclosing = enclosing_;
                childEnclosing.push_back({selfDecl_, &scope});
                CompiledChunk literalChunk;
                if (!compileFunctionLike(literalChunk, n.scope(), &n, std::move(childEnclosing), n.parameters,
                                          *n.body)) {
                    throw NotCompilable{};
                }
                // A nested literal that reads ANY enclosing variable
                // (literalChunk.upvalues non-empty) is compiled via
                // Op::LoadUpvalue -- a live-call-stack walk (findUpvalue)
                // that only ever resolves a still-active enclosing call,
                // never a captured environment (see LoadUpvalue/
                // findUpvalue's own doc comments: "this codebase has no
                // escaping closures"). A closure that escapes its creating
                // call (returned, stored, passed on -- see Closure's own
                // doc comment, value.hpp, for the motivating BOSL2
                // example) would silently read undef for every captured
                // variable through this path. Bailing the WHOLE containing
                // compilation here forces such a function to run through
                // the interpreter instead, which DOES support escaping
                // closures correctly (evalExpr's FunctionLiteral case
                // captures ctx.let_ itself). A literal with no upvalues at
                // all (doesn't reference anything from an enclosing scope)
                // has nothing that needs escaping-capture support, so it's
                // unaffected and keeps compiling normally.
                if (!literalChunk.upvalues.empty()) throw NotCompilable{};
                chunk_.nestedLiterals.emplace_back(&n, std::move(literalChunk));
                // No captured `let_` trail here (nullptr) -- a compile-time
                // constant can't carry per-invocation runtime state, but
                // (per the upvalues check just above) this literal doesn't
                // reference anything outside itself, so there is nothing
                // capture. Not a regression; just not (yet) extended here.
                out.push_back({Op::PushConst, internConst(Value{std::make_shared<const Closure>(Closure{&n, nullptr})}),
                               0, nullptr});
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
                    compileExpr(*n.left, out, scope);
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
                for (const auto& assign : n.assignments) {
                    compileExpr(*assign->expr, out, scope); // RHS is never tail
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
    for (const auto& p : params) {
        // $-prefixed parameters bind through ctx.dyn (dynamically scoped,
        // never slot-addressed) via the existing applyDefaults/bindArgs
        // machinery -- out of scope for this design, see this file's
        // header comment.
        if (!p->name->name.empty() && p->name->name[0] == '$') return false;
    }

    Compiler compiler(chunk, staticScope, selfDecl, std::move(enclosing));
    CompileScope bodyScope;
    bodyScope.push();
    for (const auto& p : params) {
        int slot = compiler.declareLocal(bodyScope, p->name->name);
        chunk.params.push_back(CompiledChunk::Param{p->name->name, slot});
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

} // namespace oscadeval
