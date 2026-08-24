#include "openscad_cpp_evaluator/eval_context.hpp"

namespace oscadeval {

EvalContext EvalContext::makeRoot(const oscad::Scope* rootScope) {
    EvalContext ctx;
    ctx.scope = rootScope;
    // dyn/dynExplicit share one underlying trail -- see scope_trail.hpp's
    // own doc comment on DynValueView/DynExplicitView.
    auto dynTrail = IndexedTrailView<DynEntry>::makeRoot(std::make_shared<DynNameIntern>());
    ctx.dyn = DynValueView(dynTrail);
    ctx.dynExplicit = DynExplicitView(dynTrail);
    ctx.dyn->set("$fn", Value{0.0});
    ctx.dyn->set("$fa", Value{12.0});
    ctx.dyn->set("$fs", Value{2.0});
    ctx.dyn->set("$t", Value{0.0});
    ctx.dyn->set("$parent_modules", Value{0.0});
    // Always false: there is no preview mode here, every render is a full
    // CSG render. The variable still has to EXIST, though -- a script that
    // branches on it (the `$preview ? cheap : real` idiom) would otherwise
    // see undef, take the falsy branch by accident rather than by rule, and
    // warn about an unknown variable while doing it.
    ctx.dyn->set("$preview", Value{false});
    // $_BELFRYSCAD -- present only here, so a script can tell which
    // evaluator it is running under. OpenSCAD leaves it undef, which is the
    // whole point: `if ($_BELFRYSCAD == undef)` is the portable test, and it
    // works because an unknown $-variable is undef rather than an error.
    //
    // Shaped like OpenSCAD's own version() -- a [major, minor, patch] list
    // rather than a string -- so comparisons need no string splitting.
    // Which FEATURES exist is a separate question with a better answer:
    // supported_feature() (function_builtins.cpp).
    // $_SUPPORTED_FEATURE -- "supported_feature() is callable here". Set by
    // any implementation that provides the function, not just this one, so a
    // script can check for the FUNCTION rather than for a vendor:
    //
    //     if ($_SUPPORTED_FEATURE && supported_feature("separate-children"))
    //
    // Solves the bootstrapping problem -- you cannot safely call
    // supported_feature() without first knowing it exists -- and, because
    // && short-circuits, an evaluator lacking it warns once about this
    // variable instead of once about the variable and again about the call.
    // If OpenSCAD ever adds supported_feature(), it sets this too and the
    // same script starts working there with no edit.
    ctx.dyn->set("$_SUPPORTED_FEATURE", Value{true});
    ctx.dyn->set("$_BELFRYSCAD",
                 Value{std::make_shared<const ValueList>(ValueList{{Value{double{OSCAD_EVAL_VERSION_MAJOR}},
                                                                    Value{double{OSCAD_EVAL_VERSION_MINOR}},
                                                                    Value{double{OSCAD_EVAL_VERSION_PATCH}}}})});
    ctx.let_ = TrailView<Value>::makeRoot();
    ctx.dynPositions = TrailView<const oscad::Position*>::makeRoot();
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
    auto newDynTrail = dyn.trail()->openChild(/*isolate=*/false);
    result.dyn = DynValueView(newDynTrail);
    result.dynExplicit = DynExplicitView(newDynTrail);
    result.let_ = let_->openChild(false);
    result.dynPositions = dynPositions->openChild(/*isolate=*/true); // fresh -- the reference's asymmetric rule
    result.color = newColor.has_value() ? newColor : color;
    result.childrenNodes = newChildrenNodes ? newChildrenNodes : childrenNodes;
    result.childrenCallerCtx = newChildrenCallerCtx ? newChildrenCallerCtx : childrenCallerCtx;
    result.viaChildren = viaChildren;
    return result;
}

EvalContext EvalContext::callCtx(const oscad::Scope* newScope, std::optional<std::array<double, 4>> newColor,
                                  std::shared_ptr<const ChildrenNodeList> newChildrenNodes,
                                  const EvalContext* newChildrenCallerCtx) const {
    EvalContext result;
    result.scope = newScope ? newScope : scope;
    auto newDynTrail = dyn.trail()->openChild(/*isolate=*/false); // stays dynamically scoped through
    result.dyn = DynValueView(newDynTrail);
    result.dynExplicit = DynExplicitView(newDynTrail);
    result.let_ = let_->openChild(/*isolate=*/true); // isolated call scope
    result.dynPositions = dynPositions->openChild(true);
    result.color = newColor.has_value() ? newColor : color;
    result.childrenNodes = newChildrenNodes ? newChildrenNodes : std::make_shared<const ChildrenNodeList>();
    result.childrenCallerCtx = newChildrenCallerCtx; // not inherited, see header comment
    return result;
}

EvalContext EvalContext::callCtxFromCapturedLet(const std::shared_ptr<TrailView<Value>>& capturedLet,
                                                 const oscad::Scope* newScope,
                                                 std::optional<std::array<double, 4>> newColor,
                                                 std::shared_ptr<const ChildrenNodeList> newChildrenNodes,
                                                 const EvalContext* newChildrenCallerCtx) const {
    EvalContext result;
    result.scope = newScope ? newScope : scope;
    auto newDynTrail = dyn.trail()->openChild(/*isolate=*/false); // stays dynamically scoped through the CALL SITE
    result.dyn = DynValueView(newDynTrail);
    result.dynExplicit = DynExplicitView(newDynTrail);
    // isolate=false: continue the ancestry chain THROUGH capturedLet, not
    // terminate at it -- the entire point is inheriting the closure's own
    // captured bindings (isolate=true would sever that immediately,
    // making the capture pointless; the isolation this needs -- NOT
    // seeing the call SITE's own locals -- already comes for free from
    // rooting at capturedLet instead of at `this->let_` in the first
    // place).
    result.let_ = capturedLet->openChild(/*isolate=*/false);
    result.dynPositions = dynPositions->openChild(true);
    result.color = newColor.has_value() ? newColor : color;
    result.childrenNodes = newChildrenNodes ? newChildrenNodes : std::make_shared<const ChildrenNodeList>();
    result.childrenCallerCtx = newChildrenCallerCtx;
    return result;
}

EvalContext EvalContext::letChildCtx() const {
    EvalContext result = *this;
    auto newDynTrail = dyn.trail()->openChild(/*isolate=*/false);
    result.dyn = DynValueView(newDynTrail);
    result.dynExplicit = DynExplicitView(newDynTrail);
    result.let_ = let_->openChild(false);
    result.dynPositions = dynPositions->openChild(false);
    return result;
}

} // namespace oscadeval
