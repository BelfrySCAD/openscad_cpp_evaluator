// nanobind Python extension for openscad_cpp_evaluator.
//
// Runs the canonical parse -> resolveUseScopes -> Evaluator::evaluate ->
// toRenderableBodies pipeline (mirroring tools/cli/cli_lib.cpp) and returns
// each body's mesh as raw numpy arrays plus batched echo/warning output.
// The mesh boundary is arrays-only by design: the app's pip `manifold3d`
// module and this binding's own statically-linked Manifold are distinct C++
// types that must never exchange objects.
//
// GIL: released around the pure-C++ parse+evaluate; the echo/log callbacks
// invoked during that window only push_back into a std::vector (no Python),
// so they need no reacquire. numpy/dict building happens after, GIL held.

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "openscad_cpp_evaluator/colored_body.hpp"
#include "openscad_cpp_evaluator/csg_node.hpp"
#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/eval_use.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/manifold_cache.hpp"
#include "openscad_cpp_evaluator/profile.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <cstdint>
#include <exception>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace nb = nanobind;

namespace {

// Moves a std::vector's storage into a NumPy array that owns it via a
// capsule, so the data survives after this function returns without a copy.
template <typename T>
nb::object vecToNumpy(std::vector<T>&& src, std::vector<size_t> shape) {
    auto* heap = new std::vector<T>(std::move(src));
    nb::capsule owner(heap, [](void* p) noexcept { delete static_cast<std::vector<T>*>(p); });
    return nb::cast(nb::ndarray<nb::numpy, T>(heap->data(), shape.size(), shape.data(), owner));
}

// Python scalar/sequence -> oscadeval::Value, for viewport_params seeding
// ($t/$fn/$vpr/... -- numbers, bools, and numeric vectors). bool must be
// checked before int (Python bool is an int subclass). Anything else -> undef.
oscadeval::Value pyToValue(nb::handle v) {
    if (nb::isinstance<nb::bool_>(v)) return oscadeval::Value{nb::cast<bool>(v)};
    if (nb::isinstance<nb::int_>(v) || nb::isinstance<nb::float_>(v)) return oscadeval::Value{nb::cast<double>(v)};
    if (nb::isinstance<nb::str>(v)) return oscadeval::Value{nb::cast<std::string>(v)};
    if (nb::isinstance<nb::list>(v) || nb::isinstance<nb::tuple>(v)) {
        oscadeval::ValueList vl;
        for (nb::handle item : nb::borrow<nb::sequence>(v)) vl.items.push_back(pyToValue(item));
        return oscadeval::Value{oscadeval::ListPtr(std::make_shared<const oscadeval::ValueList>(std::move(vl)))};
    }
    return oscadeval::Value{}; // monostate = undef
}

std::unordered_map<std::string, oscadeval::Value> toViewportParams(nb::dict d) {
    std::unordered_map<std::string, oscadeval::Value> out;
    for (auto [k, v] : d) out.emplace(nb::cast<std::string>(k), pyToValue(v));
    return out;
}

// A facade class handle (openscad_cpp_evaluator.<name>), imported lazily at
// first use and leaked deliberately -- never destroyed, so no nb::object
// destructor runs after Python finalization (the app exits via os._exit,
// which skips finalizers; a normal-shutdown consumer would otherwise crash).
nb::object facadeAttr(const char* name) {
    static nb::object* mod = new nb::object(nb::module_::import_("openscad_cpp_evaluator"));
    return mod->attr(name);
}

nb::object valueToPy(const oscadeval::Value& v); // recursive

nb::object valueToPy(const oscadeval::Value& v) {
    if (std::holds_alternative<std::monostate>(v)) return nb::none();
    if (const bool* b = std::get_if<bool>(&v)) return nb::cast(*b);
    if (const double* d = std::get_if<double>(&v)) return nb::cast(*d);
    if (const std::string* s = std::get_if<std::string>(&v)) return nb::cast(*s);
    if (const oscadeval::ListPtr* lp = std::get_if<oscadeval::ListPtr>(&v)) {
        nb::list out;
        if (*lp)
            for (const oscadeval::Value& item : (*lp)->items) out.append(valueToPy(item));
        return out;
    }
    if (const oscadeval::ObjectPtr* op = std::get_if<oscadeval::ObjectPtr>(&v)) {
        nb::dict data;
        if (*op)
            for (const auto& [k, val] : (*op)->items) data[nb::str(k.c_str())] = valueToPy(val);
        return facadeAttr("OscObject")(data);
    }
    // OscRange / FunctionLiteral: no direct Python analog -- wrap the
    // OpenSCAD text so the debugger displays it cleanly (unquoted).
    return facadeAttr("_ScadValue")(oscadeval::fmtValue(v));
}

nb::object posToPy(const oscad::Position* p) {
    if (!p) return nb::none();
    return facadeAttr("_Position")(p->line, p->column, p->origin, p->start_offset, p->end_offset);
}

// Mirrors the reference's ColoredBody.role string values exactly (minus
// "highlight_ghost" -- see BodyRole's own ponytail comment in
// colored_body.hpp for why that 5th, renderer-overlay-only state isn't
// modeled here yet).
const char* roleToString(oscadeval::BodyRole role) {
    switch (role) {
        case oscadeval::BodyRole::Highlight:  return "highlight";
        case oscadeval::BodyRole::Background: return "background";
        case oscadeval::BodyRole::ShowOnly:   return "show_only";
        case oscadeval::BodyRole::Normal:
        default:                              return "normal";
    }
}

// One evaluated body's mesh + attributes, shaped to match what the renderer
// already reads off a manifold3d mesh (vert_properties (nVert, numProp),
// tri_verts (nTri, 3), run_original_id (nRun,), run_index (nRun+1,) -- the
// same flat 3*triangle units manifold3d's own run_index uses).
nb::dict bodyToDict(oscadeval::ColoredBody& cb) {
    manifold::MeshGL mesh = cb.body->GetMeshGL();
    const size_t numProp = mesh.numProp;
    const size_t numVert = numProp ? mesh.vertProperties.size() / numProp : 0;
    const size_t numTri = mesh.triVerts.size() / 3;

    nb::dict d;
    d["vert_properties"] = vecToNumpy(std::move(mesh.vertProperties), {numVert, numProp});
    d["tri_verts"] = vecToNumpy(std::move(mesh.triVerts), {numTri, 3});
    d["run_original_id"] = vecToNumpy(std::move(mesh.runOriginalID), {mesh.runOriginalID.size()});
    d["run_index"] = vecToNumpy(std::move(mesh.runIndex), {mesh.runIndex.size()});
    d["num_prop"] = static_cast<int>(numProp);
    d["flat_preview"] = cb.flatPreview;
    d["role"] = roleToString(cb.role);
    if (cb.color)
        d["color"] = nb::make_tuple((*cb.color)[0], (*cb.color)[1], (*cb.color)[2], (*cb.color)[3]);
    else
        d["color"] = nb::none();
    // Per-triangle RGBA, set only for a real multi-color CSG merge (see
    // Evaluator::attachTriColors) -- None for the common single-material
    // case, matching the reference's ColoredBody.tri_colors exactly (an
    // (numTri, 4) float32 array or None). The renderer reads this to split
    // one merged body's triangles across separate opaque/translucent draw
    // buffers.
    if (cb.triColors) {
        const std::vector<std::array<float, 4>>& src = *cb.triColors;
        std::vector<float> flat;
        flat.reserve(src.size() * 4);
        for (const auto& rgba : src) flat.insert(flat.end(), rgba.begin(), rgba.end());
        d["tri_colors"] = vecToNumpy(std::move(flat), {src.size(), 4});
    } else {
        d["tri_colors"] = nb::none();
    }
    return d;
}

// originalID -> source span of the AST node that produced it, for WYSIWYG
// picking. Read out of ev.idToNode before the AST it points into is destroyed
// (the outlive contract) -- only plain ints/strings escape, never an ASTNode*.
struct IdSpan {
    uint32_t id;
    int start, end, line, column;
    std::string origin;
};

void collectIdSpans(const oscadeval::Evaluator& ev, std::vector<IdSpan>& out) {
    out.reserve(ev.idToNode.size());
    for (const auto& [id, node] : ev.idToNode) {
        const oscad::Position& p = node->position();
        out.push_back({id, p.start_offset, p.end_offset, p.line, p.column, p.origin});
    }
}

nb::list bodiesToList(std::vector<oscadeval::ColoredBody>& bodies) {
    nb::list out;
    for (oscadeval::ColoredBody& cb : bodies) {
        if (!cb.body || cb.body->IsEmpty()) continue;
        out.append(bodyToDict(cb));
    }
    return out;
}

nb::dict idSpansToDict(const std::vector<IdSpan>& idSpans) {
    nb::dict d;
    for (const IdSpan& s : idSpans)
        d[nb::cast(s.id)] = nb::make_tuple(s.start, s.end, s.line, s.column, s.origin);
    return d;
}

// CSGNode -> a facade `_CSGNode` (kind/params/bodies/is_builtin/children),
// recursively, eagerly converting every field to plain Python data. No
// lifetime extension needed -- CSGParams (Value-only, see csg_node.hpp) and
// ColoredBody never hold AST pointers, so the result is fully self-contained
// once built, unlike CSGNode::node itself (deliberately not exposed: the AST
// it points into does not outlive this binding call, and the CSG-tree dump
// consumer only ever needs kind/params/children/bodies -- see
// format_csg_tree's own doc comment).
nb::object csgNodeToPy(const oscadeval::CSGNode& node) {
    nb::dict params;
    for (const auto& [k, v] : node.params) params[nb::str(k.c_str())] = valueToPy(v);
    nb::list bodies;
    for (const oscadeval::ColoredBody& cb : node.bodies) {
        if (!cb.body || cb.body->IsEmpty()) continue;
        // ColoredBody here is const in this walk (tree ownership stays with
        // the caller) -- GetMeshGL() itself is logically read-only, so a
        // const_cast here is safe/local, mirroring bodyToDict's mutating
        // signature only because manifold3d's own API isn't const-qualified.
        bodies.append(bodyToDict(const_cast<oscadeval::ColoredBody&>(cb)));
    }
    nb::list children;
    for (const std::unique_ptr<oscadeval::CSGNode>& child : node.children) children.append(csgNodeToPy(*child));
    return facadeAttr("_CSGNode")(node.kind, params, bodies, node.isBuiltin, children);
}

nb::list csgTreeToPy(const std::vector<std::unique_ptr<oscadeval::CSGNode>>& tree) {
    nb::list out;
    for (const std::unique_ptr<oscadeval::CSGNode>& node : tree) out.append(csgNodeToPy(*node));
    return out;
}

nb::object profileResultToPy(const std::optional<oscadeval::ProfileResult>& pr) {
    if (!pr) return nb::none();
    nb::list sites;
    for (const oscadeval::CallSiteProfile& s : pr->callSites) {
        sites.append(facadeAttr("CallSiteProfile")(s.kind, s.name, s.callerName, s.callOrigin, s.callLine, s.declOrigin,
                                                     s.declLine, s.callCount, s.selfTime, s.cumulativeTime));
    }
    return facadeAttr("ProfileResult")(sites, pr->resolveTime, pr->generateTime, pr->totalTime, pr->unattributedTime);
}

// ctx.dyn / ctx.dynExplicit -> (dict[str, Any], set[str]) -- every currently-
// visible $-prefixed dynamic variable, and the subset the script itself
// assigned (vs. merely seeded by viewportParams). ctx is caller-owned in this
// port (see evaluate()'s own doc comment in evaluator.hpp), so this is a
// direct readback, not a separate tracking mechanism.
std::pair<nb::dict, nb::object> dynStateToPy(const oscadeval::EvalContext& ctx) {
    nb::dict dyn;
    for (const auto& [name, v] : ctx.dyn.items()) dyn[nb::str(name.c_str())] = valueToPy(v);
    nb::set explicitNames;
    for (const auto& [name, isExplicit] : ctx.dynExplicit.items())
        if (isExplicit) explicitNames.add(nb::str(name.c_str()));
    return {dyn, explicitNames};
}

// (list[body-dict], list[echo-str], dict[int, span], list[_CSGNode],
// ProfileResult|None, dict[str, Any], set[str]). Raises the C++ error
// message as a Python exception on ParseError/EvalError (nanobind maps
// std::exception -> RuntimeError, whose str() is the already-formatted
// "ERROR:..."/caret diagnostic; the facade re-raises as EvalError).
nb::object evaluate(const std::string& path, nb::dict viewportParams,
                     std::shared_ptr<oscadeval::ManifoldCache> manifoldCache, bool profile) {
    std::unordered_map<std::string, oscadeval::Value> vp = toViewportParams(viewportParams);

    std::vector<oscadeval::ColoredBody> bodies;
    std::vector<std::string> echoes;
    std::vector<IdSpan> idSpans;
    std::vector<std::unique_ptr<oscadeval::CSGNode>> csgTree;
    std::optional<oscadeval::ProfileResult> profileResult;
    nb::dict dyn;
    nb::object dynExplicit;
    {
        nb::gil_scoped_release rel;
        auto logFn = [&echoes](const std::string& m) { echoes.push_back(m); };
        std::vector<std::unique_ptr<oscad::ASTNode>> ast = oscad::getASTFromFile(path);
        oscadeval::ResolvedUseScopes used = oscadeval::resolveUseScopes(ast, path, logFn);
        oscadeval::Evaluator ev(logFn, nullptr, manifoldCache, oscadeval::DebugHooks{}, profile);
        oscadeval::EvalContext ctx = oscadeval::EvalContext::makeRoot(used.rootScope.get());
        bodies = oscadeval::toRenderableBodies(ev.evaluate(used.processedNodes, ctx, vp));
        collectIdSpans(ev, idSpans);
        csgTree = std::move(ev.csgTree);
        profileResult = std::move(ev.profileResult);
        {
            nb::gil_scoped_acquire g; // building Python objects needs the GIL back
            std::tie(dyn, dynExplicit) = dynStateToPy(ctx);
        }
    }

    nb::list echoList;
    for (const std::string& s : echoes) echoList.append(s);
    return nb::make_tuple(bodiesToList(bodies), echoList, idSpansToDict(idSpans), csgTreeToPy(csgTree),
                           profileResultToPy(profileResult), dyn, dynExplicit);
}

// ------------------------------------------------------------------------
// Debugger: DebugHooks trampolines calling back into Python, plus Value ->
// Python conversion for locals inspection.
// ------------------------------------------------------------------------

// One debug frame -> {"local_scope", "outer_scope", "dyn_names"}, the shape
// the debugger pane's _filtered_vars consumes.
nb::dict frameToDict(const oscadeval::DebugFrame& f) {
    nb::dict local, outer;
    for (const auto& [k, v] : f.localScope) local[nb::str(k.c_str())] = valueToPy(v);
    for (const auto& [k, v] : f.outerScope) outer[nb::str(k.c_str())] = valueToPy(v);
    nb::set dyn;
    for (const std::string& n : f.dynNames) dyn.add(nb::str(n.c_str()));
    nb::dict d;
    d["local_scope"] = local;
    d["outer_scope"] = outer;
    d["dyn_names"] = dyn;
    return d;
}

// call_stack entries: (kind, name, call_pos, decl_pos) -- the tuple shape the
// debugger pane's _populate_stack reads (entry[0] kind, [1] name, [2] call
// pos, [3] decl pos, each pos exposing .origin/.line).
nb::list callStackToPy(const std::vector<oscadeval::CallStackFrame>& cs) {
    nb::list out;
    for (const oscadeval::CallStackFrame& f : cs) {
        const char* kind = (f.kind == oscadeval::CallStackFrame::Kind::Module) ? "module" : "function";
        out.append(nb::make_tuple(kind, f.name, posToPy(f.callPosition), posToPy(f.declPosition)));
    }
    return out;
}

// Called from within a debug_hook/error_break callback (never after it
// returns) to best-effort generate a live render of whatever's been
// resolved so far -- see Evaluator::generatePartialTree()'s own doc
// comment for why this is safe to call mid-resolve.
using GeneratePartialFn = std::function<std::vector<oscadeval::ColoredBody>()>;

// Wraps a GeneratePartialFn as a Python callable returning a plain list of
// body-dicts (bodiesToList's own shape) -- the facade's _generate_partial_render
// converts these to ColoredBody and catches whatever exception a GenerateFn
// raises, the same try/except shape it already had calling generate_tree()
// directly. No error handling needed here: a C++ exception crossing into
// Python via this cpp_function is nanobind's normal std::exception ->
// RuntimeError mapping, which that Python try/except already catches.
nb::object generatePartialTrampoline(const GeneratePartialFn& generatePartial) {
    return nb::cpp_function([&generatePartial]() -> nb::list {
        std::vector<oscadeval::ColoredBody> bodies = generatePartial();
        return bodiesToList(bodies);
    });
}

// Trampoline: C++ debug hook -> the Python DebugSession hook. Runs with the
// (origin, line) targets children()/children(N) might forward control to
// from the just-checked debug-hook node, for the "step to child" command.
// See Evaluator::lastChildrenPositions()'s own doc comment: valid only
// synchronously within the current hook call, same as get_frames.
using GetChildrenPositionsFn = std::function<const std::optional<std::vector<std::pair<std::string, int>>>&()>;

// Trampoline: lets the Python hook tell the Evaluator it's safe to speed
// up function calls with the bytecode VM right now -- see
// Evaluator::setFastContinueBreakpoints's own doc comment (evaluator.hpp)
// for the exact contract (only when no step is pending/no pause was
// requested/the initial break-on-first stop is already consumed) and
// Evaluator::chunkEligibleNow for how the breakpoint set is used (a
// per-function span check, not a blanket switch). `breakpoints=None`
// means "no exception right now" -- i.e. every function call runs
// interpreted, same as if this were never called at all. A callable
// (not a return-tuple field) so DebugSession's own hook() can call it
// exactly where it already computes should_pause, without needing to
// widen hook()'s existing (cmd, mods) return-tuple contract -- mirrors
// generate_partial/get_children_positions's own "extra kwarg the Python
// side may or may not call" shape.
using SetFastContinueFn = std::function<void(std::optional<std::unordered_map<std::string, std::set<int>>>)>;

// Wraps a SetFastContinueFn as a Python callable taking either a
// dict[str, set[int]]/dict[str, list[int]] (origin -> breakpoint lines) or
// None. Mirrors generatePartialTrampoline's own shape.
nb::object setFastContinueTrampoline(const SetFastContinueFn& setFastContinue) {
    return nb::cpp_function(
        [&setFastContinue](nb::object breakpoints) {
            if (breakpoints.is_none()) {
                setFastContinue(std::nullopt);
                return;
            }
            std::unordered_map<std::string, std::set<int>> bp;
            for (auto [k, v] : nb::cast<nb::dict>(breakpoints)) {
                std::set<int> lines;
                for (nb::handle line : v) lines.insert(nb::cast<int>(line));
                bp.emplace(nb::cast<std::string>(k), std::move(lines));
            }
            setFastContinue(std::move(bp));
        },
        // Without .none(), nanobind rejects a Python `None` argument for an
        // `nb::object`-typed parameter registered this ad-hoc way (unlike a
        // `.def()`-declared function parameter, which accepts None by
        // default) -- confirmed via a real call: "incompatible function
        // arguments ... Invoked with types: NoneType" the first time this
        // was called with breakpoints=None (DebugSession's own
        // _apply_fast_continue calls it with None whenever fast-continue
        // mode isn't safe right now -- see debugger.py).
        nb::arg("breakpoints").none());
}

// GIL held (reacquired by the caller). `getFrame`/`callStack`/`generatePartial`/
// `getChildrenPositions` are valid only for this synchronous call, so their
// closures are only ever invoked from within the Python hook, before it returns.
oscadeval::DebugAction callPyDebugHook(nb::handle hook, int line, int depth, bool forced, bool exprLevel,
                                        const std::string& origin,
                                        const std::vector<oscadeval::CallStackFrame>& callStack,
                                        const oscadeval::DebugFramesFn& getFrame,
                                        const GeneratePartialFn& generatePartial,
                                        const GetChildrenPositionsFn& getChildrenPositions,
                                        const SetFastContinueFn& setFastContinue) {
    auto getFramesPy = nb::cpp_function([&callStack, &getFrame]() -> nb::object {
        std::vector<oscadeval::DebugFrame> frames = getFrame();
        nb::list allFrameLocals;
        for (const oscadeval::DebugFrame& f : frames) allFrameLocals.append(frameToDict(f));
        // last_locals: the innermost frame's editable (let-bound) names.
        nb::dict lastLocals;
        if (!frames.empty()) {
            const oscadeval::DebugFrame& f0 = frames.front();
            std::set<std::string> editable(f0.dynNames.begin(), f0.dynNames.end());
            for (const auto& [k, v] : f0.localScope)
                if (editable.count(k)) lastLocals[nb::str(k.c_str())] = valueToPy(v);
        }
        return nb::make_tuple(nb::make_tuple(lastLocals, allFrameLocals), callStackToPy(callStack));
    });
    nb::object generatePartialPy = generatePartialTrampoline(generatePartial);
    auto getChildrenPositionsPy = nb::cpp_function([&getChildrenPositions]() -> nb::object {
        const std::optional<std::vector<std::pair<std::string, int>>>& positions = getChildrenPositions();
        if (!positions) return nb::none();
        nb::list out;
        for (const auto& [origin, line] : *positions) out.append(nb::make_tuple(origin, line));
        return out;
    });
    nb::object setFastContinuePy = setFastContinueTrampoline(setFastContinue);

    // expr_depth stays hardcoded at 0 -- this port deliberately doesn't
    // track the reference's _expr_depth counter (no consumer reads it);
    // expr_level, which consumers do read, is the real threaded value.
    nb::object ret = hook(line, depth, nb::arg("forced") = forced, nb::arg("expr_level") = exprLevel,
                          nb::arg("expr_depth") = 0, nb::arg("origin") = origin, nb::arg("get_frames") = getFramesPy,
                          nb::arg("generate_partial") = generatePartialPy,
                          nb::arg("get_children_positions") = getChildrenPositionsPy,
                          nb::arg("set_fast_continue") = setFastContinuePy);
    nb::tuple t = nb::cast<nb::tuple>(ret);
    oscadeval::DebugAction action;
    action.stop = (nb::cast<std::string>(t[0]) == "stop");
    nb::dict mods = nb::cast<nb::dict>(t[1]);
    for (auto [k, v] : mods) action.mods[nb::cast<std::string>(k)] = pyToValue(v);
    return action;
}

// Trampoline: C++ errorBreak -> the Python DebugSession error hook.
void callPyErrorBreak(nb::handle errorBreak, int line, const std::string& header, const std::string& origin,
                       const std::vector<oscadeval::CallStackFrame>& callStack,
                       const oscadeval::DebugFramesFn& getFrame, const GeneratePartialFn& generatePartial) {
    std::vector<oscadeval::DebugFrame> frames = getFrame();
    nb::list allFrameLocals;
    for (const oscadeval::DebugFrame& f : frames) allFrameLocals.append(frameToDict(f));
    nb::object generatePartialPy = generatePartialTrampoline(generatePartial);
    errorBreak(line, header, allFrameLocals, callStackToPy(callStack), nb::arg("origin") = origin,
               nb::arg("generate_partial") = generatePartialPy);
}

// Trampoline: C++ returnHook -> the Python DebugSession return hook. Only
// fires for user function/function-literal calls (never modules, never
// builtins -- see DebugHooks::returnHook's own doc comment).
void callPyReturnHook(nb::handle returnHook, const std::string& name, const oscadeval::Value& result, int depth) {
    returnHook(name, valueToPy(result), depth);
}

// evaluate() with the debugger wired in. echo is delivered live via echoFn
// (not batched), so a paused session can still print. Returns (bodies, [],
// id_to_node, dyn, dyn_explicit). The GIL is released around the C++
// evaluate and reacquired inside every callback trampoline; the actual pause
// blocks on the Python side (threading.Event) so the GUI thread keeps running.
nb::object debugEvaluate(const std::string& path, nb::dict viewportParams, nb::callable debugHook,
                          nb::callable errorBreak, nb::callable echoFn,
                          std::shared_ptr<oscadeval::ManifoldCache> manifoldCache, nb::object returnHook) {
    std::unordered_map<std::string, oscadeval::Value> vp = toViewportParams(viewportParams);

    std::vector<oscadeval::ColoredBody> bodies;
    std::vector<IdSpan> idSpans;
    std::exception_ptr err;
    nb::dict dyn;
    nb::object dynExplicit;
    {
        nb::gil_scoped_release rel;
        auto echoCpp = [&echoFn](const std::string& m) {
            nb::gil_scoped_acquire g;
            echoFn(m);
        };
        // Set right after `ev` is constructed below, before evaluate() runs
        // -- every hook fires only from within that call, by which point
        // this is always valid. Needed because `hooks` (captured by the
        // lambdas below) must be fully built and passed into Evaluator's
        // constructor before `ev` exists, so the lambdas can't capture `ev`
        // itself; they capture this pointer's address instead, which DOES
        // already exist (see generatePartial's own doc comment on why
        // GeneratePartialFn needs a live Evaluator& at all).
        oscadeval::Evaluator* evPtr = nullptr;
        GeneratePartialFn generatePartial = [&evPtr]() -> std::vector<oscadeval::ColoredBody> {
            return evPtr->generatePartialTree();
        };
        GetChildrenPositionsFn getChildrenPositions =
            [&evPtr]() -> const std::optional<std::vector<std::pair<std::string, int>>>& {
            return evPtr->lastChildrenPositions();
        };
        SetFastContinueFn setFastContinue =
            [&evPtr](std::optional<std::unordered_map<std::string, std::set<int>>> breakpoints) {
            evPtr->setFastContinueBreakpoints(std::move(breakpoints));
        };
        oscadeval::DebugHooks hooks;
        hooks.debugHook = [&](int line, int depth, bool forced, bool exprLevel, const std::string& origin,
                              const std::vector<oscadeval::CallStackFrame>& cs,
                              const oscadeval::DebugFramesFn& gf) -> oscadeval::DebugAction {
            nb::gil_scoped_acquire g;
            return callPyDebugHook(debugHook, line, depth, forced, exprLevel, origin, cs, gf, generatePartial,
                                    getChildrenPositions, setFastContinue);
        };
        hooks.errorBreak = [&](int line, const std::string& header, const std::string& origin,
                               const std::vector<oscadeval::CallStackFrame>& cs, const oscadeval::DebugFramesFn& gf) {
            nb::gil_scoped_acquire g;
            callPyErrorBreak(errorBreak, line, header, origin, cs, gf, generatePartial);
        };
        if (!returnHook.is_none()) {
            hooks.returnHook = [&](const std::string& name, const oscadeval::Value& result, int depth) {
                nb::gil_scoped_acquire g;
                callPyReturnHook(returnHook, name, result, depth);
            };
        }
        try {
            std::vector<std::unique_ptr<oscad::ASTNode>> ast = oscad::getASTFromFile(path);
            oscadeval::ResolvedUseScopes used = oscadeval::resolveUseScopes(ast, path, echoCpp);
            oscadeval::Evaluator ev(echoCpp, nullptr, manifoldCache, hooks, false);
            evPtr = &ev;
            oscadeval::EvalContext ctx = oscadeval::EvalContext::makeRoot(used.rootScope.get());
            bodies = oscadeval::toRenderableBodies(ev.evaluate(used.processedNodes, ctx, vp));
            collectIdSpans(ev, idSpans);
            {
                nb::gil_scoped_acquire g;
                std::tie(dyn, dynExplicit) = dynStateToPy(ctx);
            }
        } catch (...) {
            err = std::current_exception();
        }
    }
    if (err) std::rethrow_exception(err); // re-raised with the GIL held -> Python exception

    return nb::make_tuple(bodiesToList(bodies), nb::list(), idSpansToDict(idSpans), dyn, dynExplicit);
}

// Top-level declaration names + source spans, for the editor's completion and
// go-to-definition. Walks the use-resolved node list (so `use <file>`-injected
// declarations show up too, matching build_scopes' root-scope contents), one
// tuple per Assignment/FunctionDeclaration/ModuleDeclaration:
// (namespace, name, start_offset, end_offset, line, column, origin).
nb::list parseDecls(const std::string& path) {
    struct Decl {
        const char* ns;
        std::string name;
        int start, end, line, column;
        std::string origin;
    };
    std::vector<Decl> decls;
    {
        nb::gil_scoped_release rel;
        std::vector<std::unique_ptr<oscad::ASTNode>> ast = oscad::getASTFromFile(path);
        oscadeval::ResolvedUseScopes used = oscadeval::resolveUseScopes(ast, path, [](const std::string&) {});
        for (const oscad::ASTNode* n : used.processedNodes) {
            const oscad::Position& p = n->position();
            const char* ns = nullptr;
            const std::string* name = nullptr;
            switch (n->kind()) {
            case oscad::NodeKind::Assignment:
                ns = "variable";
                name = &static_cast<const oscad::Assignment&>(*n).name->name;
                break;
            case oscad::NodeKind::FunctionDeclaration:
                ns = "function";
                name = &static_cast<const oscad::FunctionDeclaration&>(*n).name->name;
                break;
            case oscad::NodeKind::ModuleDeclaration:
                ns = "module";
                name = &static_cast<const oscad::ModuleDeclaration&>(*n).name->name;
                break;
            default:
                continue;
            }
            decls.push_back({ns, *name, p.start_offset, p.end_offset, p.line, p.column, p.origin});
        }
    }
    nb::list out;
    for (const Decl& d : decls)
        out.append(nb::make_tuple(d.ns, d.name, d.start, d.end, d.line, d.column, d.origin));
    return out;
}

} // namespace

NB_MODULE(_openscad_cpp_evaluator, m) {
    m.doc() = "C++ OpenSCAD evaluator (nanobind binding)";

    // Opt-in, shared across evaluate() calls -- see manifold_cache.hpp.
    // Only a constructor and clear() are ever called from Python (get/put
    // are internal to generateTree()'s own C++ implementation).
    nb::class_<oscadeval::ManifoldCache>(m, "ManifoldCache")
        .def(nb::init<>())
        .def("clear", &oscadeval::ManifoldCache::clear);

    m.def("evaluate", &evaluate, nb::arg("path"), nb::arg("viewport_params"), nb::arg("manifold_cache") = nullptr,
          nb::arg("profile") = false,
          "Evaluate a .scad file; return (bodies, echoes, id_to_node, csg_tree, profile_result, dyn, dyn_explicit).");
    m.def("parse_decls", &parseDecls, nb::arg("path"),
          "Parse a .scad file; return top-level declaration (namespace, name, start, end, line, column, origin) tuples.");
    m.def("debug_evaluate", &debugEvaluate, nb::arg("path"), nb::arg("viewport_params"), nb::arg("debug_hook"),
          nb::arg("error_break"), nb::arg("echo_fn"), nb::arg("manifold_cache") = nullptr,
          nb::arg("return_hook") = nb::none(),
          "Evaluate with the debugger wired in; returns (bodies, [], id_to_node, dyn, dyn_explicit). Callbacks fire under the GIL.");
}
