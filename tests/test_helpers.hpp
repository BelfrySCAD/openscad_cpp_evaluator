#pragma once

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_parser/api.hpp"

// Shared helpers for the evaluator test suite, mirroring
// openscad_cpp_parser/tests/test_helpers.hpp's own parseSrc/exprSrc pattern.
namespace oscadeval::test {

inline std::vector<std::unique_ptr<oscad::ASTNode>> parseSrc(const std::string& code) {
    return oscad::getASTFromString(code);
}

// Parses `code` as a single top-level assignment's RHS: `x = {code};`.
// Returned pointer is only valid as long as the caller keeps `ast` alive.
inline const oscad::Expression* exprSrc(const std::string& code, std::vector<std::unique_ptr<oscad::ASTNode>>& ast) {
    ast = oscad::getASTFromString("x = " + code + ";");
    auto* a = dynamic_cast<oscad::Assignment*>(ast[0].get());
    return a ? a->expr.get() : nullptr;
}

// Parses + resolves + generates `code` in one shot, keeping the AST/scope/
// tree alive alongside the Evaluator and result bodies (all of which
// borrow into it) for as long as the caller holds this struct -- the
// standard shape every CSG-producing test needs.
struct Evaluated {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    std::unique_ptr<oscad::Scope> scope;
    Evaluator ev;
    std::vector<std::unique_ptr<CSGNode>> tree;
    std::vector<ColoredBody> bodies;
};

inline Evaluated evalSrc(const std::string& code, EchoFn echoFn = {}) {
    Evaluated e{parseSrc(code), nullptr, Evaluator(std::move(echoFn)), {}, {}};
    e.scope = oscad::buildScopes(e.ast);
    EvalContext ctx = EvalContext::makeRoot(e.scope.get());
    e.tree = e.ev.resolveTree(e.ast, ctx);
    e.bodies = e.ev.generateTree(e.tree);
    return e;
}

// Same, but via Evaluator::evaluate() (applies the top-level show_only
// filter) rather than calling resolveTree()+generateTree() directly.
inline Evaluated evaluateSrc(const std::string& code, EchoFn echoFn = {}) {
    Evaluated e{parseSrc(code), nullptr, Evaluator(std::move(echoFn)), {}, {}};
    e.scope = oscad::buildScopes(e.ast);
    EvalContext ctx = EvalContext::makeRoot(e.scope.get());
    e.bodies = e.ev.evaluate(e.ast, ctx);
    return e;
}

// Same as evalSrc(), but with an externally-owned ManifoldCache attached --
// lets a test share one cache across several independent Evaluated
// instances (each with its own fresh AST/Evaluator, matching how a real
// long-lived host reuses a ManifoldCache across separate evaluate() calls).
inline Evaluated evalSrcWithCache(const std::string& code, std::shared_ptr<ManifoldCache> cache, EchoFn echoFn = {}) {
    Evaluated e{parseSrc(code), nullptr, Evaluator(std::move(echoFn), nullptr, std::move(cache)), {}, {}};
    e.scope = oscad::buildScopes(e.ast);
    EvalContext ctx = EvalContext::makeRoot(e.scope.get());
    e.tree = e.ev.resolveTree(e.ast, ctx);
    e.bodies = e.ev.generateTree(e.tree);
    return e;
}

} // namespace oscadeval::test
