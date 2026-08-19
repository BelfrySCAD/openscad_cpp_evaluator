#pragma once

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/dispatch.hpp"

#include <optional>

// Internal (not part of the public include/ API): prototypes for every
// builtin's resolve/generate function pair, collected here so
// registry.cpp doesn't need one #include per builtin file.
namespace oscadeval {

// Resolves a file argument (a string, or coerced via fmtValue) to an
// absolute path, relative to the *source .scad file's own directory* (not
// the process CWD) when given as a relative path. Shared by every
// file-reading builtin: import()/surface()/DXF/SVG. Defined in import.cpp.
std::string resolveFilePath(const Value& fileArg, const oscad::ASTNode& node);

CSGParams resolveCube(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateCube(Evaluator& ev, const CSGParams& params,
                                       const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

CSGParams resolveSphere(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateSphere(Evaluator& ev, const CSGParams& params,
                                         const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

CSGParams resolveCylinder(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateCylinder(Evaluator& ev, const CSGParams& params,
                                           const std::vector<std::unique_ptr<CSGNode>>& children,
                                           const oscad::ASTNode& node);

CSGParams resolvePolyhedron(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generatePolyhedron(Evaluator& ev, const CSGParams& params,
                                             const std::vector<std::unique_ptr<CSGNode>>& children,
                                             const oscad::ASTNode& node);

// circle/square/polygon share one resolve/generate pair.
CSGParams resolve2d(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generate2d(Evaluator& ev, const CSGParams& params,
                                     const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

// Shared return shape for computeTransformParams/computeColorParams,
// below -- the "resolve args, build params, compute a possibly-$-scoped
// child ctx" half of resolveTransform/resolveColor, split out so
// Op::PushBuiltinWrap's own runtime handler (bytecode_vm.cpp) can call it
// directly instead of going through the full resolve function (whose
// OTHER half, `ev.evalChildren(...)`, is exactly the native reentry this
// split exists to avoid). `ctx` is the input ctx unchanged unless this
// builtin's own resolution opened a new one (mirrors ResolvedCallArgs's
// own `ctx` field, call_args.hpp).
struct BuiltinWrapParams {
    CSGParams params;
    EvalContext ctx;
};

// translate/rotate/scale/mirror/multmatrix/resize share one resolve/
// generate pair (2D/3D dispatch happens per-body at generate time).
BuiltinWrapParams computeTransformParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
CSGParams resolveTransform(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateTransform(Evaluator& ev, const CSGParams& params,
                                            const std::vector<std::unique_ptr<CSGNode>>& children,
                                            const oscad::ASTNode& node);

BuiltinWrapParams computeColorParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
CSGParams resolveColor(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateColor(Evaluator& ev, const CSGParams& params,
                                        const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

// union/difference/intersection share one resolve/generate pair.
CSGParams resolveCsg(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateCsg(Evaluator& ev, const CSGParams& params,
                                      const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

// Shared by hull/minkowski/projection/linear_extrude/rotate_extrude/roof
// (topology.cpp, extrude.cpp, roof.cpp) -- defined in booleans.cpp since
// they're the same shape as union/difference/intersection's own role
// splitting/2D-merge logic.
struct RoleSplit {
    std::vector<ColoredBody> background;
    std::vector<ColoredBody> foreground; // role != Background && role != ShowOnly, and not display-only
    std::vector<ColoredBody> highlight;  // the subset of `foreground` that's role == Highlight
    std::vector<ColoredBody> showOnly;
    // Bodies with no Manifold to operate on (an open polyhedron -- see
    // ColoredBody::rawMesh). Split out for the same reason showOnly is:
    // they must survive a CSG/hull/minkowski step untouched rather than be
    // merged into it, since there is nothing to merge. Kept separate from
    // `background` because they render as ordinary geometry, not as a
    // translucent ghost.
    std::vector<ColoredBody> displayOnly;
};
RoleSplit splitByRole(const std::vector<ColoredBody>& bodies);

// manifold::ToString(Manifold::Error) only exists under MANIFOLD_DEBUG --
// not enabled in this build -- so this mirrors it. Shared by import.cpp and
// primitives_3d.cpp, both of which report why a mesh wouldn't build.
std::string manifoldErrorName(manifold::Manifold::Error e);
std::optional<manifold::CrossSection> toCrossSection(const std::vector<ColoredBody>& bodies);

// Merges a statement's bodies into one (3D union, else 2D union, else an
// empty Manifold) -- defined in control.cpp (intersection_for's own
// per-iteration combine step), reused by projection.cpp. Mirrors _combine.
ColoredBody combineBodies(const std::vector<ColoredBody>& bodies);

// The 3 tag modifiers -- registered in generateDispatch() only (evalModifier
// doesn't consult resolveDispatch, see csg_resolve.cpp).
std::vector<ColoredBody> generateHighlight(Evaluator& ev, const CSGParams& params,
                                            const std::vector<std::unique_ptr<CSGNode>>& children,
                                            const oscad::ASTNode& node);
std::vector<ColoredBody> generateBackground(Evaluator& ev, const CSGParams& params,
                                             const std::vector<std::unique_ptr<CSGNode>>& children,
                                             const oscad::ASTNode& node);
std::vector<ColoredBody> generateShowOnly(Evaluator& ev, const CSGParams& params,
                                           const std::vector<std::unique_ptr<CSGNode>>& children,
                                           const oscad::ASTNode& node);

// children() -- registered in resolveDispatch()/generateDispatch() under
// "children" like any other builtin, but evalModularCall specially splices
// its resolved subtree (see csg_resolve.cpp) rather than wrapping it in its
// own CSGNode, so generateChildren is never actually reached -- kept
// registered anyway so a lookup for "children" is never a dispatch miss.
CSGParams resolveChildren(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateChildren(Evaluator& ev, const CSGParams& params,
                                           const std::vector<std::unique_ptr<CSGNode>>& children,
                                           const oscad::ASTNode& node);

// render() -- a display hint; resolve just evaluates children for the tree-
// building side effect. No generate function is registered (falls to
// generateTree()'s default child-concatenation, exactly reproducing
// passthrough).
CSGParams resolveRender(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);

// breakpoint() -- no-op as of Phase 4 (no debugger until Phase 9); still
// evaluates/validates its optional `condition` argument for parity.
CSGParams resolveBreakpoint(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);

// intersection_for -- NOT a ModularCall (a distinct NodeKind), so it isn't
// looked up via resolveDispatch()/name string; Evaluator::
// evalIntersectionForNode calls resolveIntersectionFor directly. Still
// registered under "intersection_for" in generateDispatch() since
// generateTree() dispatches by CSGNode::kind uniformly regardless of the
// AST node type that produced it.
CSGParams resolveIntersectionFor(Evaluator& ev, const oscad::ModularIntersectionFor& node, EvalContext& ctx);
std::vector<ColoredBody> generateIntersectionFor(Evaluator& ev, const CSGParams& params,
                                                  const std::vector<std::unique_ptr<CSGNode>>& children,
                                                  const oscad::ASTNode& node);

// import() as a geometry statement (module context) -- STL/OBJ/OFF/3MF only
// as of Phase 5; DXF/SVG land alongside their expression-context (Region)
// counterparts in a later phase. See import_builtin.hpp for import() used
// as an expression instead.
CSGParams resolveImport(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateImport(Evaluator& ev, const CSGParams& params,
                                         const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

// hull()/minkowski() -- topology.cpp. Both splice their children
// transparently like union/difference/intersection (resolve just evaluates
// them for the tree-building side effect); the real work happens in
// generate against the flattened+role-split child bodies.
CSGParams resolveHull(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateHull(Evaluator& ev, const CSGParams& params,
                                       const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

// dxf_dim()/dxf_cross() -- read a dimension or a cross's centre out of a
// DXF file. Functions, not modules; dispatched from evalBuiltinFunction.
Value builtinDxfDim(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node);
Value builtinDxfCross(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node);

CSGParams resolveFill(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateFill(Evaluator& ev, const CSGParams& params,
                                       const std::vector<std::unique_ptr<CSGNode>>& children,
                                       const oscad::ASTNode& node);
CSGParams resolveMinkowski(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateMinkowski(Evaluator& ev, const CSGParams& params,
                                            const std::vector<std::unique_ptr<CSGNode>>& children,
                                            const oscad::ASTNode& node);

// linear_extrude()/rotate_extrude()/projection() -- extrude.cpp.
// computeXParams: the pure "resolve args, build params" half, split out
// like computeTransformParams/computeColorParams so Op::PushBuiltinWrap's
// own runtime handler can call it directly (bytecode_vm.cpp) instead of
// through the whole resolve function, whose OTHER half (evalChildren) is
// exactly the native reentry that split exists to avoid.
BuiltinWrapParams computeLinearExtrudeParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
CSGParams resolveLinearExtrude(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateLinearExtrude(Evaluator& ev, const CSGParams& params,
                                                const std::vector<std::unique_ptr<CSGNode>>& children,
                                                const oscad::ASTNode& node);

BuiltinWrapParams computeRotateExtrudeParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
CSGParams resolveRotateExtrude(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateRotateExtrude(Evaluator& ev, const CSGParams& params,
                                                const std::vector<std::unique_ptr<CSGNode>>& children,
                                                const oscad::ASTNode& node);

BuiltinWrapParams computeProjectionParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
CSGParams resolveProjection(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateProjection(Evaluator& ev, const CSGParams& params,
                                             const std::vector<std::unique_ptr<CSGNode>>& children,
                                             const oscad::ASTNode& node);

BuiltinWrapParams computeOffsetParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
CSGParams resolveOffset(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateOffset(Evaluator& ev, const CSGParams& params,
                                         const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

// roof() -- roof.cpp. Tier 1 (exact straight skeleton, stable single-
// contour polygons) + Tier 3 (SDF/level-set fallback) only, per the plan --
// Tier 2 (general multi-contour/hole straight skeleton) is a named,
// documented follow-up, not silently dropped (see CLAUDE.md).
// computeRoofParams: UNLIKE computeLinearExtrudeParams and friends above,
// this is the params-computation half taken from AFTER evalChildren (it
// calls ev.warn(), an observable side effect that must stay ordered after
// any child's own echo/warn output) -- takes already-resolved CallArgs,
// not a raw node+ctx, since Op::PushBuiltinWrap's own runtime handler
// calls this at POP time. See its own doc comment (roof.cpp).
CSGParams computeRoofParams(Evaluator& ev, const CallArgs& args, EvalContext& effCtx);
CSGParams resolveRoof(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateRoof(Evaluator& ev, const CSGParams& params,
                                       const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

// surface() -- surface.cpp.
CSGParams resolveSurface(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateSurface(Evaluator& ev, const CSGParams& params,
                                          const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

// text() -- text.cpp.
CSGParams resolveText(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx);
std::vector<ColoredBody> generateText(Evaluator& ev, const CSGParams& params,
                                       const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode& node);

} // namespace oscadeval
