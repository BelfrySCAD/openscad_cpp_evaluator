#include "openscad_cpp_evaluator/eval_context.hpp"

namespace oscadeval {

EvalContext EvalContext::makeRoot(const oscad::Scope* rootScope) {
    EvalContext ctx;
    ctx.scope = rootScope;
    auto dynIntern = std::make_shared<DynNameIntern>();
    ctx.dyn = IndexedTrailView<Value>::makeRoot(dynIntern);
    ctx.dyn->set("$fn", Value{0.0});
    ctx.dyn->set("$fa", Value{12.0});
    ctx.dyn->set("$fs", Value{2.0});
    ctx.dyn->set("$t", Value{0.0});
    ctx.dyn->set("$parent_modules", Value{0.0});
    ctx.let_ = TrailView<Value>::makeRoot();
    ctx.dynPositions = TrailView<const oscad::Position*>::makeRoot();
    ctx.dynExplicit = DynExplicitTrail::makeRoot(dynIntern);
    ctx.childrenNodes = std::make_shared<const ChildrenNodeList>();
    ctx.childrenCallerCtx = nullptr;
    return ctx;
}

EvalContext EvalContext::withScope(const oscad::Scope* newScope) const {
    EvalContext copy = *this;
    copy.scope = newScope;
    return copy;
}

EvalContext EvalContext::childCtx(const oscad::Scope* newScope, std::optional<std::array<double, 4>> newColor,
                                   std::shared_ptr<const ChildrenNodeList> newChildrenNodes,
                                   const EvalContext* newChildrenCallerCtx) const {
    EvalContext result;
    result.scope = newScope ? newScope : scope;
    result.dyn = dyn->openChild(/*isolate=*/false);
    result.dynExplicit = dynExplicit->openChild(false);
    result.let_ = let_->openChild(false);
    result.dynPositions = dynPositions->openChild(/*isolate=*/true); // fresh -- the reference's asymmetric rule
    result.color = newColor.has_value() ? newColor : color;
    result.childrenNodes = newChildrenNodes ? newChildrenNodes : childrenNodes;
    result.childrenCallerCtx = newChildrenCallerCtx ? newChildrenCallerCtx : childrenCallerCtx;
    return result;
}

EvalContext EvalContext::callCtx(const oscad::Scope* newScope, std::optional<std::array<double, 4>> newColor,
                                  std::shared_ptr<const ChildrenNodeList> newChildrenNodes,
                                  const EvalContext* newChildrenCallerCtx) const {
    EvalContext result;
    result.scope = newScope ? newScope : scope;
    result.dyn = dyn->openChild(/*isolate=*/false); // stays dynamically scoped through
    result.dynExplicit = dynExplicit->openChild(false);
    result.let_ = let_->openChild(/*isolate=*/true); // isolated call scope
    result.dynPositions = dynPositions->openChild(true);
    result.color = newColor.has_value() ? newColor : color;
    result.childrenNodes = newChildrenNodes ? newChildrenNodes : std::make_shared<const ChildrenNodeList>();
    result.childrenCallerCtx = newChildrenCallerCtx; // not inherited, see header comment
    return result;
}

EvalContext EvalContext::letChildCtx() const {
    EvalContext result = *this;
    result.dyn = dyn->openChild(/*isolate=*/false);
    result.let_ = let_->openChild(false);
    result.dynPositions = dynPositions->openChild(false);
    result.dynExplicit = dynExplicit->openChild(false);
    return result;
}

} // namespace oscadeval
