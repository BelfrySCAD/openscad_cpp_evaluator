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
#include <nanobind/stl/string.h>

#include "openscad_cpp_evaluator/colored_body.hpp"
#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/eval_use.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
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
    if (cb.color)
        d["color"] = nb::make_tuple((*cb.color)[0], (*cb.color)[1], (*cb.color)[2], (*cb.color)[3]);
    else
        d["color"] = nb::none();
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

// (list[body-dict], list[echo-str], dict[int, span]). Raises the C++ error
// message as a Python exception on ParseError/EvalError (nanobind maps
// std::exception -> RuntimeError, whose str() is the already-formatted
// "ERROR:..."/caret diagnostic; the facade re-raises as EvalError).
nb::object evaluate(const std::string& path, nb::dict viewportParams) {
    std::unordered_map<std::string, oscadeval::Value> vp = toViewportParams(viewportParams);

    std::vector<oscadeval::ColoredBody> bodies;
    std::vector<std::string> echoes;
    std::vector<IdSpan> idSpans;
    {
        nb::gil_scoped_release rel;
        auto logFn = [&echoes](const std::string& m) { echoes.push_back(m); };
        std::vector<std::unique_ptr<oscad::ASTNode>> ast = oscad::getASTFromFile(path);
        oscadeval::ResolvedUseScopes used = oscadeval::resolveUseScopes(ast, path, logFn);
        oscadeval::Evaluator ev(logFn);
        oscadeval::EvalContext ctx = oscadeval::EvalContext::makeRoot(used.rootScope.get());
        bodies = oscadeval::toRenderableBodies(ev.evaluate(used.processedNodes, ctx, vp));
        collectIdSpans(ev, idSpans);
    }

    nb::list echoList;
    for (const std::string& s : echoes) echoList.append(s);
    return nb::make_tuple(bodiesToList(bodies), echoList, idSpansToDict(idSpans));
}

// ------------------------------------------------------------------------
// Debugger: DebugHooks trampolines calling back into Python, plus Value ->
// Python conversion for locals inspection.
// ------------------------------------------------------------------------

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

// Trampoline: C++ debug hook -> the Python DebugSession hook. Runs with the
// GIL held (reacquired by the caller). `getFrame`/`callStack` are valid only
// for this synchronous call, so the get_frames closure is only ever invoked
// from within the Python hook, before it returns.
oscadeval::DebugAction callPyDebugHook(nb::handle hook, int line, int depth, bool forced, const std::string& origin,
                                        const std::vector<oscadeval::CallStackFrame>& callStack,
                                        const oscadeval::DebugFramesFn& getFrame) {
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

    nb::object ret = hook(line, depth, nb::arg("forced") = forced, nb::arg("expr_level") = false,
                          nb::arg("expr_depth") = 0, nb::arg("origin") = origin, nb::arg("get_frames") = getFramesPy);
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
                       const oscadeval::DebugFramesFn& getFrame) {
    std::vector<oscadeval::DebugFrame> frames = getFrame();
    nb::list allFrameLocals;
    for (const oscadeval::DebugFrame& f : frames) allFrameLocals.append(frameToDict(f));
    errorBreak(line, header, allFrameLocals, callStackToPy(callStack), nb::arg("origin") = origin);
}

// evaluate() with the debugger wired in. echo is delivered live via echoFn
// (not batched), so a paused session can still print. Returns (bodies, [],
// id_to_node). The GIL is released around the C++ evaluate and reacquired
// inside every callback trampoline; the actual pause blocks on the Python
// side (threading.Event) so the GUI thread keeps running.
nb::object debugEvaluate(const std::string& path, nb::dict viewportParams, nb::callable debugHook,
                          nb::callable errorBreak, nb::callable echoFn) {
    std::unordered_map<std::string, oscadeval::Value> vp = toViewportParams(viewportParams);

    std::vector<oscadeval::ColoredBody> bodies;
    std::vector<IdSpan> idSpans;
    std::exception_ptr err;
    {
        nb::gil_scoped_release rel;
        auto echoCpp = [&echoFn](const std::string& m) {
            nb::gil_scoped_acquire g;
            echoFn(m);
        };
        oscadeval::DebugHooks hooks;
        hooks.debugHook = [&](int line, int depth, bool forced, const std::string& origin,
                              const std::vector<oscadeval::CallStackFrame>& cs,
                              const oscadeval::DebugFramesFn& gf) -> oscadeval::DebugAction {
            nb::gil_scoped_acquire g;
            return callPyDebugHook(debugHook, line, depth, forced, origin, cs, gf);
        };
        hooks.errorBreak = [&](int line, const std::string& header, const std::string& origin,
                               const std::vector<oscadeval::CallStackFrame>& cs, const oscadeval::DebugFramesFn& gf) {
            nb::gil_scoped_acquire g;
            callPyErrorBreak(errorBreak, line, header, origin, cs, gf);
        };
        try {
            std::vector<std::unique_ptr<oscad::ASTNode>> ast = oscad::getASTFromFile(path);
            oscadeval::ResolvedUseScopes used = oscadeval::resolveUseScopes(ast, path, echoCpp);
            oscadeval::Evaluator ev(echoCpp, nullptr, nullptr, hooks, false);
            oscadeval::EvalContext ctx = oscadeval::EvalContext::makeRoot(used.rootScope.get());
            bodies = oscadeval::toRenderableBodies(ev.evaluate(used.processedNodes, ctx, vp));
            collectIdSpans(ev, idSpans);
        } catch (...) {
            err = std::current_exception();
        }
    }
    if (err) std::rethrow_exception(err); // re-raised with the GIL held -> Python exception

    return nb::make_tuple(bodiesToList(bodies), nb::list(), idSpansToDict(idSpans));
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
    m.def("evaluate", &evaluate, nb::arg("path"), nb::arg("viewport_params"),
          "Evaluate a .scad file; return (list of body dicts with raw mesh arrays, list of echo strings).");
    m.def("parse_decls", &parseDecls, nb::arg("path"),
          "Parse a .scad file; return top-level declaration (namespace, name, start, end, line, column, origin) tuples.");
    m.def("debug_evaluate", &debugEvaluate, nb::arg("path"), nb::arg("viewport_params"), nb::arg("debug_hook"),
          nb::arg("error_break"), nb::arg("echo_fn"),
          "Evaluate with the debugger wired in; returns (bodies, [], id_to_node). Callbacks fire under the GIL.");
}
