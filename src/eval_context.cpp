#include "openscad_cpp_evaluator/eval_context.hpp"

namespace oscadeval {

EvalContext EvalContext::makeRoot(const oscad::Scope* rootScope) {
    EvalContext ctx;
    ctx.scope = rootScope;
    ctx.dyn = std::make_shared<DynMap>(DynMap{
        {"$fn", Value{0.0}},
        {"$fa", Value{12.0}},
        {"$fs", Value{2.0}},
        {"$t", Value{0.0}},
        {"$parent_modules", Value{0.0}},
    });
    ctx.let_ = std::make_shared<LetMap>();
    ctx.dynPositions = std::make_shared<DynPositionMap>();
    ctx.dynExplicit = std::make_shared<NameSet>();
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
                                   const EvalContext* newChildrenCallerCtx, std::shared_ptr<DynMap> newDyn) const {
    EvalContext result;
    result.scope = newScope ? newScope : scope;
    if (newDyn) {
        result.dyn = std::move(newDyn);
        result.dynPositions = dynPositions; // inherited, not reset -- see header comment
    } else {
        result.dyn = std::make_shared<DynMap>(*dyn);
        result.dynPositions = std::make_shared<DynPositionMap>(); // fresh
    }
    result.let_ = std::make_shared<LetMap>(*let_);
    result.dynExplicit = std::make_shared<NameSet>(*dynExplicit);
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
    result.dyn = std::make_shared<DynMap>(*dyn);
    result.let_ = std::make_shared<LetMap>();
    result.dynPositions = std::make_shared<DynPositionMap>();
    result.dynExplicit = std::make_shared<NameSet>(*dynExplicit);
    result.color = newColor.has_value() ? newColor : color;
    result.childrenNodes = newChildrenNodes ? newChildrenNodes : std::make_shared<const ChildrenNodeList>();
    result.childrenCallerCtx = newChildrenCallerCtx; // not inherited, see header comment
    return result;
}

EvalContext EvalContext::letChildCtx() const {
    EvalContext result = *this;
    result.dyn = std::make_shared<DynMap>(*dyn);
    result.let_ = std::make_shared<LetMap>(*let_);
    result.dynPositions = std::make_shared<DynPositionMap>(*dynPositions);
    result.dynExplicit = std::make_shared<NameSet>(*dynExplicit);
    return result;
}

} // namespace oscadeval
