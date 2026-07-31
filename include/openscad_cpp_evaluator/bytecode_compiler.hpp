#pragma once

#include "openscad_cpp_evaluator/bytecode.hpp"

#include <optional>
#include <vector>

namespace oscad {
class Assignment;
class FunctionDeclaration;
class ModuleDeclaration;
class Expression;
class Scope;
} // namespace oscad

namespace oscadeval {

// Attempts to compile `decl`'s parameter defaults + body to bytecode.
// Returns nullopt if `decl` uses a construct this compiler doesn't handle
// yet -- the caller falls back to the ordinary AST interpreter for that
// whole function, unconditionally correct since nothing about this
// function's behavior changes based on whether it happened to compile.
std::optional<CompiledChunk> tryCompileFunction(const oscad::FunctionDeclaration& decl);

// Compiles a bare STATEMENT-context expression -- an assignment's RHS, an
// if/for condition, a module-call or echo()/assert() argument -- rather
// than a whole function body. No parameters, no enclosing/upvalue chain, no
// self declaration: every name reference resolves through Op::LoadFree/
// Op::LoadDyn, the identical ctx.let_/ctx.dyn lookup evalExpr's own
// Identifier case already does for a statement-level local (a for-loop
// variable, an assignment-declared name) -- this buys back the AST
// dispatch overhead compileExpr already saves for functions, not faster
// variable access (statement-level locals were never slot-addressable to
// begin with; they're declared by scattered Assignment statements, not a
// fixed parameter list known in advance). A nested let() inside the
// expression still gets a real compile-time slot, exactly as it would
// inside a function body.
//
// Refuses to compile (returns nullopt) if the expression creates any
// closure that captures something (chunk.closureSites non-empty after
// compileFunctionLike returns) -- with no enclosing chain and no self
// declaration for this bare wrapper, such a closure's own upvalue
// resolution would target a CallStackFrame that will never exist (this
// path pushes none), which is silently WRONG (stale/undef) rather than a
// safe fallback. Not a hypothetical carve-out: it's the one shape this
// wrapper cannot support correctly, so it's excluded rather than risked.
// `scope` is the expression's own lexical scope (Expression::scope(),
// always set by buildScope() regardless of node kind) -- needed to resolve
// any callee inside it statically, the same way a function body's call
// sites are.
std::optional<CompiledChunk> tryCompileStatementExpr(const oscad::Expression& expr, const oscad::Scope* scope);

// Compiles a run of SIBLING assignment statements sharing one scope --
// exactly Evaluator::evalChildren's own `assignments` sub-list (stmt_eval.
// cpp), which OpenSCAD already runs as a self-contained group before any
// non-assignment statement in the same block. Unlike tryCompileStatementExpr
// (one independent chunk per leaf expression -- fine for a single
// assignment, but its own per-call overhead dominated a WHOLE block of
// them, see this feature's own commit message for the measured regression),
// this compiles the ENTIRE run as one chunk with one shared CompileScope:
// each assignment's own name gets a real slot (declared in source order,
// mirroring LetOp's own sequential-visibility rule -- `y = x + 1;` after
// `x = 1;` reads `x` via a fast LoadLocal, not a name lookup), and each
// value is written out via Op::StoreLocalAndLet so code OUTSIDE this batch
// still sees it the ordinary way (see runCompiledAssignmentBlock's own doc
// comment, bytecode_vm.hpp, for why `ctx` is used directly, not a scoped
// child).
//
// Refuses to compile (returns nullopt) for any of:
// - the same plain (non-$) name assigned more than once in this run --
//   evalAssignment's own "was assigned on line N but overwritten" warning
//   would otherwise silently stop firing for a reassignment inside a
//   compiled batch; safer to fall back than to lose it silently.
// - any closure that captures something (same hazard, same reasoning, as
//   tryCompileStatementExpr's own doc comment above).
// - a $-prefixed assignment nested inside a let()/list-comprehension-let
//   clause ANYWHERE in this run (post-compile: any Op::StoreDyn in the
//   resulting bodyCode) -- runCompiledAssignmentBlock runs directly against
//   the caller's own `ctx`, not a scoped child the way a bare expression
//   chunk gets (letChildCtx(), see runCompiledExprChunk's own doc comment)
//   -- there is no scope here for that nested let()'s own dyn write to be
//   contained by, so it would leak into the caller's ctx.dyn permanently
//   instead of just for that one assignment's own RHS.
std::optional<CompiledChunk> tryCompileAssignmentBlock(const std::vector<const oscad::Assignment*>& assigns,
                                                        const oscad::Scope* scope);

// Attempts to compile `decl`'s parameter defaults + STATEMENT-list body
// (decl.children) to bytecode (Stage 2) -- the module-side analog of
// tryCompileFunction. Returns nullopt (falls back to the ordinary
// interpreter for this whole module) for a construct compileStatementList
// doesn't handle -- see its own doc comment (bytecode_compiler.cpp) for
// exactly which statement kinds get real bytecode (assignment/if/for/a
// resolved user-module call) versus a native passthrough (everything
// else: echo/assert/let-blocks/modifiers/intersection_for/builtin or
// unresolved module calls) -- the native passthrough shapes never bail
// compilation of the WHOLE body the way an unsupported EXPRESSION
// construct does for tryCompileFunction; only a genuinely unsupported
// EXPRESSION (inside an if-condition/for-range/argument) can still bail
// (via tryCompileStatementExpr's own NotCompilable, propagated up).
std::optional<CompiledChunk> tryCompileModuleBody(const oscad::ModuleDeclaration& decl);

// Same compilation (assignment/if/for/resolved-module-call get real
// bytecode, everything else a native passthrough), but for an ARBITRARY
// statement list, not just a ModuleDeclaration's own `children` -- the
// general form behind Evaluator::tryRunCompiledChildren, which every
// evalChildren() call now tries first (stmt_eval.cpp), including a
// builtin module's own children (translate()/union()/etc.'s internal
// evalChildren call, src/builtins/*.cpp) and a for-loop's/let-block's
// body. This is what actually reaches the "native passthrough wraps a
// recursive module call" case Op::NativeStatement alone can't -- see this
// project's own session notes on the Stage 2 "NativeStatement gap" for
// why a builtin call itself (translate, not just what's declared inside
// this project's own module bodies) still can't become a SINGLE flat
// instruction stream with its caller this way (its own resolve function,
// e.g. resolveTransform, computes params AND evaluates children as one
// opaque native call -- decoupling that is a separate, larger project),
// but this DOES mean the children of any wrapping builtin get their own,
// independent chance to compile, cutting the number of native stack
// frames needed per recursive "wrap-then-recurse" level rather than
// eliminating them entirely. `scope`: the list's own lexical scope for
// resolving a call site's callee -- callers pass `children.front()->
// scope()`, mirroring tryRunCompiledAssignmentBlock's own convention.
std::optional<CompiledChunk> tryCompileChildrenList(const std::vector<const oscad::ASTNode*>& children,
                                                     const oscad::Scope* scope);

} // namespace oscadeval
