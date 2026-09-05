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
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

#include "openscad_cpp_evaluator/colored_body.hpp"
#include "openscad_cpp_evaluator/csg_node.hpp"
#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/eval_use.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/export.hpp"
#include "openscad_cpp_evaluator/mesh_check.hpp"
#include "openscad_cpp_evaluator/manifold_cache.hpp"
#include "openscad_cpp_evaluator/profile.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <atomic>
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

// Defined in ast_to_py.cpp -- snapshots the parser's AST into plain Python
// dicts. Declared here rather than in a header since these two entry points
// are the whole interface and only this file calls them.
namespace oscadbind {
nb::list parseAstFromFile(const std::string& path, bool includeComments);
nb::list parseAstFromString(const std::string& code, bool includeComments);
std::string formatSourceString(const std::string& code, int indentWidth, bool includeComments);
} // namespace oscadbind

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
    // A display-only body's Manifold is empty by construction, so its
    // GetMeshGL() would hand back nothing -- take the raw triangle soup
    // that failed to build instead. Everything downstream reads plain
    // arrays, so the two cases are indistinguishable from here on.
    manifold::MeshGL mesh = cb.isDisplayOnly() ? *cb.rawMesh : cb.body->GetMeshGL();
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
        // isDisplayOnly() bodies are ALWAYS IsEmpty() -- that's what made
        // an open polyhedron() silently vanish here rather than render.
        if (!cb.isDisplayOnly() && (!cb.body || cb.body->IsEmpty())) continue;
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
        if (!cb.isDisplayOnly() && (!cb.body || cb.body->IsEmpty())) continue;
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

// Exposed so a front end that writes its own files -- BelfrySCAD's GUI has
// its own exporters -- can run the same check the CLI does rather than a
// second, drifting implementation.
// Exposed for the export path: strip zero-area faces and repair the
// T-joints their removal exposes. Returns (verts, tris, report).
nb::tuple stripSliversPy(const std::vector<float>& verts, const std::vector<uint32_t>& tris) {
    manifold::MeshGL m;
    m.numProp = 3;
    m.vertProperties = verts;
    m.triVerts = tris;
    oscadeval::SliverStripReport r;
    manifold::MeshGL out = oscadeval::stripSlivers(m, r);
    nb::dict rep;
    rep["removed"] = r.removed;
    rep["restitched"] = r.restitched;
    rep["needles"] = r.needles;
    rep["left_behind"] = r.leftBehind;
    rep["passes"] = r.passes;
    return nb::make_tuple(out.vertProperties, out.triVerts, rep);
}

nb::dict checkMeshPy(const std::vector<float>& verts, const std::vector<uint32_t>& tris) {
    manifold::MeshGL m;
    m.numProp = 3;
    m.vertProperties = verts;
    m.triVerts = tris;
    const oscadeval::MeshDiagnosis d = oscadeval::checkMesh(m);
    nb::dict out;
    out["boundary_edges"] = d.boundaryEdges;
    out["non_manifold_edges"] = d.nonManifoldEdges;
    out["pinched_vertices"] = d.pinchedVertices;
    out["inconsistent_edges"] = d.inconsistentEdges;
    out["degenerate_faces"] = d.degenerateFaces;
    out["duplicate_faces"] = d.duplicateFaces;
    out["unwelded_vertices"] = d.unweldedVertices;
    out["watertight"] = d.watertight();
    out["manifold"] = d.manifold();
    out["orientable"] = d.orientable();
    out["ok"] = d.ok();
    out["summary"] = d.summary();
    return out;
}

nb::object profileResultToPy(const std::optional<oscadeval::ProfileResult>& pr) {
    if (!pr) return nb::none();
    nb::list sites;
    for (const oscadeval::CallSiteProfile& s : pr->callSites) {
        sites.append(facadeAttr("CallSiteProfile")(s.kind, s.name, s.callerName, s.callOrigin, s.callLine, s.callColumn,
                                                     s.declOrigin, s.declLine, s.callCount, s.selfTime, s.cumulativeTime));
    }
    // The calling-context tree, as plain dicts. Parent/child INDICES, not
    // nested objects: the C++ side is already a flat vector keyed that way,
    // and a self-referential nesting would have to be rebuilt here for no
    // gain -- a consumer walks it by index just as C++ does.
    nb::list paths;
    for (const oscadeval::ProfilePathNode& n : pr->paths) {
        nb::dict d;
        d["parent"] = n.parent;
        nb::list kids;
        for (int c : n.children) kids.append(c);
        d["children"] = kids;
        d["kind"] = n.kind;
        d["name"] = n.name;
        d["call_origin"] = n.callOrigin;
        d["call_line"] = n.callLine;
        d["call_column"] = n.callColumn;
        d["decl_origin"] = n.declOrigin;
        d["decl_line"] = n.declLine;
        d["call_count"] = n.callCount;
        d["self_time"] = n.selfTime;
        d["cumulative_time"] = n.cumulativeTime;
        paths.append(d);
    }
    return facadeAttr("ProfileResult")(sites, pr->resolveTime, pr->generateTime, pr->totalTime, pr->unattributedTime,
                                        paths);
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
// The evaluated bodies, kept on the C++ side.
//
// bodyToDict flattens every Manifold into numpy arrays for the renderer,
// which is all the renderer needs -- but export has to do real CSG (the
// union, the per-colour claim, decompose), and rebuilding Manifolds from
// those arrays to do it costs ~146ms on a 224k-triangle model and throws
// away Manifold's own provenance. Handing back an opaque handle instead
// means geometry never round-trips through Python at all.
struct Geometry {
    std::vector<oscadeval::ColoredBody> bodies;
};

// exportModel, with the path/format/warnings marshalling. Releases the GIL:
// a large export is seconds of Manifold work with no Python involved.
nb::list exportModelPy(const std::string& path, const Geometry& geom, const std::string& format, bool asciiStl,
                        bool stripSlivers) {
    std::vector<std::string> warnings;
    {
        nb::gil_scoped_release rel;
        oscadeval::ExportOptions opts;
        opts.format = format;
        opts.asciiStl = asciiStl;
        opts.stripSlivers = stripSlivers;
        warnings = oscadeval::exportModel(path, geom.bodies, opts);
    }
    nb::list out;
    for (const std::string& w : warnings) out.append(w);
    return out;
}

nb::object evaluate(const std::string& path, nb::dict viewportParams,
                     std::shared_ptr<oscadeval::ManifoldCache> manifoldCache, bool profile,
                     bool generate) {
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
        try {
            std::vector<std::unique_ptr<oscad::ASTNode>> ast = oscad::getASTFromFile(path);
            oscadeval::ResolvedUseScopes used = oscadeval::resolveUseScopes(ast, path, logFn);
            oscadeval::Evaluator ev(logFn, nullptr, manifoldCache, oscadeval::DebugHooks{}, profile);
            oscadeval::EvalContext ctx = oscadeval::EvalContext::makeRoot(used.rootScope.get());
            bodies = oscadeval::toRenderableBodies(ev.evaluate(used.processedNodes, ctx, vp, generate));
            collectIdSpans(ev, idSpans);
            // Only moved out when it will actually be converted below --
            // csgTreeToPy() walks the whole tree building Python objects,
            // which a resolve-only caller has no use for.
            if (generate) csgTree = std::move(ev.csgTree);
            profileResult = std::move(ev.profileResult);
            {
                nb::gil_scoped_acquire g; // building Python objects needs the GIL back
                std::tie(dyn, dynExplicit) = dynStateToPy(ctx);
            }
        } catch (const std::exception& e) {
            // `echoes` is a local, so letting this unwind past here loses
            // every echo and warning the script produced BEFORE it failed --
            // which is exactly the output you want when something fails. The
            // batching that makes them cheap (no GIL round-trip per line) is
            // also what made them disposable.
            //
            // Carry them out on the exception itself: args = (message,
            // [echo, ...]). The facade replays them through echo_fn and then
            // re-raises. Anything that only looks at str(e) is unaffected,
            // since args[0] is still the formatted diagnostic.
            nb::gil_scoped_acquire g;
            nb::list partial;
            for (const std::string& s : echoes) partial.append(s);
            nb::object args = nb::make_tuple(nb::str(e.what()), partial);
            PyErr_SetObject(PyExc_RuntimeError, args.ptr());
            throw nb::python_error();
        }
    }

    nb::list echoList;
    for (const std::string& s : echoes) echoList.append(s);
    auto geom = std::make_shared<Geometry>();
    geom->bodies = std::move(bodies);
    return nb::make_tuple(bodiesToList(geom->bodies), echoList, idSpansToDict(idSpans), csgTreeToPy(csgTree),
                           profileResultToPy(profileResult), dyn, dynExplicit, geom);
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
// side may or may not call" shape. `hookSkippable` mirrors
// setFastContinueBreakpoints's own second parameter exactly -- see its
// doc comment for why this is narrower than "breakpoints is accurate."
using SetFastContinueFn = std::function<void(std::optional<std::unordered_map<std::string, std::set<int>>>, bool)>;

// Wraps a SetFastContinueFn as a Python callable taking either a
// dict[str, set[int]]/dict[str, list[int]] (origin -> breakpoint lines) or
// None, plus a hook_skippable bool (default False). Mirrors
// generatePartialTrampoline's own shape.
nb::object setFastContinueTrampoline(const SetFastContinueFn& setFastContinue) {
    return nb::cpp_function(
        [&setFastContinue](nb::object breakpoints, bool hookSkippable) {
            if (breakpoints.is_none()) {
                setFastContinue(std::nullopt, hookSkippable);
                return;
            }
            std::unordered_map<std::string, std::set<int>> bp;
            for (auto [k, v] : nb::cast<nb::dict>(breakpoints)) {
                std::set<int> lines;
                for (nb::handle line : v) lines.insert(nb::cast<int>(line));
                bp.emplace(nb::cast<std::string>(k), std::move(lines));
            }
            setFastContinue(std::move(bp), hookSkippable);
        },
        // Without .none(), nanobind rejects a Python `None` argument for an
        // `nb::object`-typed parameter registered this ad-hoc way (unlike a
        // `.def()`-declared function parameter, which accepts None by
        // default) -- confirmed via a real call: "incompatible function
        // arguments ... Invoked with types: NoneType" the first time this
        // was called with breakpoints=None (DebugSession's own
        // _apply_fast_continue calls it with None whenever fast-continue
        // mode isn't safe right now -- see debugger.py).
        nb::arg("breakpoints").none(), nb::arg("hook_skippable") = false);
}

// Python-visible handle onto a lock-free, GIL-free interrupt flag -- see
// Evaluator::setFastContinueInterruptFlag's own doc comment (evaluator.hpp)
// for why this exists at all: debug_evaluate() runs as one single blocking
// call with the GIL released for its whole duration, so there is no live,
// Python-callable Evaluator handle DebugSession.pause()/set_breakpoints()
// (running on the MAIN/GUI thread) could otherwise invoke directly to
// interrupt hook-skippable mode. A caller creates ONE of these before
// calling debug_evaluate() (passing it as fast_continue_signal), keeps it
// around for the whole debug session, and calls .request() from the main
// thread any time a hook-skippable checkDebug() call needs to stop
// skipping and actually consult Python again -- Pause, or a breakpoint
// being toggled in an editor tab while a render is mid-flight. Nothing
// else needs to read it back on the Python side; C++ test-and-clears it
// (see checkDebug's own doc comment, debug_profile.cpp) the moment it acts
// on it, so there's no separate "acknowledge" step.
class FastContinueSignal {
public:
    void request() { flag_->store(true, std::memory_order_release); }
    const std::shared_ptr<std::atomic<bool>>& flag() const { return flag_; }

private:
    std::shared_ptr<std::atomic<bool>> flag_ = std::make_shared<std::atomic<bool>>(false);
};

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
                          std::shared_ptr<oscadeval::ManifoldCache> manifoldCache, nb::object returnHook,
                          FastContinueSignal* fastContinueSignal) {
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
            [&evPtr](std::optional<std::unordered_map<std::string, std::set<int>>> breakpoints, bool hookSkippable) {
            evPtr->setFastContinueBreakpoints(std::move(breakpoints), hookSkippable);
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
            if (fastContinueSignal) ev.setFastContinueInterruptFlag(fastContinueSignal->flag());
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

    // See FastContinueSignal's own doc comment, above -- only a
    // constructor and request() are ever called from Python.
    nb::class_<FastContinueSignal>(m, "FastContinueSignal")
        .def(nb::init<>())
        .def("request", &FastContinueSignal::request);

    nb::class_<Geometry>(m, "Geometry",
                          "Opaque handle to the evaluated bodies, kept on the C++ side so export never has to "
                          "rebuild Manifolds from the renderer's flattened arrays. Hand it to export_model().")
        .def("__len__", [](const Geometry& g) { return g.bodies.size(); })
        .def("is_empty", [](const Geometry& g) { return g.bodies.empty(); });

    m.def("export_model", &exportModelPy, nb::arg("path"), nb::arg("geometry"), nb::arg("format") = std::string(),
          nb::arg("ascii_stl") = false, nb::arg("strip_slivers") = true,
          "Write `geometry` to `path`, format taken from the extension unless `format` says otherwise. "
          "Returns the warnings to surface (open shells, mesh problems, slivers removed) rather than "
          "logging them. Raises RuntimeError when there is no geometry, the format is unknown, or the "
          "file cannot be opened.");

    m.def("export_extensions", []() { return oscadeval::exportExtensions(); },
          "The file extensions export_model understands, dot-prefixed. Read from the C++ writer table "
          "rather than restated here, so the Python side cannot drift from what can actually be written.");

    m.def("evaluate", &evaluate, nb::arg("path"), nb::arg("viewport_params"), nb::arg("manifold_cache") = nullptr,
          nb::arg("profile") = false, nb::arg("generate") = true,
          "Evaluate a .scad file; return (bodies, echoes, id_to_node, csg_tree, profile_result, dyn, dyn_explicit).\n"
          "generate=False stops after the resolve pass: the script runs and reports everything "
          "it normally would, but no Manifold geometry is built, and neither bodies nor csg_tree "
          "are populated.");
    m.def("parse_decls", &parseDecls, nb::arg("path"),
          "Parse a .scad file; return top-level declaration (namespace, name, start, end, line, column, origin) tuples.");
    m.def("strip_slivers", &stripSliversPy, nb::arg("verts"), nb::arg("tris"),
          "Remove zero-area faces and repair the T-joints their removal "
          "exposes. Returns (verts, tris, report).");
    m.def("check_mesh", &checkMeshPy, nb::arg("verts"), nb::arg("tris"),
          "Diagnose a triangle mesh against the manifoldness conditions. "
          "verts is a flat [x,y,z,...] list, tris a flat index list. Returns "
          "counts per condition plus watertight/manifold/orientable/ok and a "
          "one-line summary.");
    m.def("parse_ast", &oscadbind::parseAstFromFile, nb::arg("path"), nb::arg("include_comments") = false,
          "Parse a .scad file; return the AST as a list of plain nested dicts "
          "(kind, position, plus that kind's own fields). A snapshot, not a view -- "
          "safe to keep, share across threads, and outlive the parse.");
    m.def("parse_ast_string", &oscadbind::parseAstFromString, nb::arg("code"), nb::arg("include_comments") = false,
          "As parse_ast(), but parses a source string instead of a file.");
    m.def("format_source", &oscadbind::formatSourceString, nb::arg("code"),
          nb::arg("indent") = 4, nb::arg("include_comments") = true,
          "Reformat OpenSCAD source: parse it and print the AST back out. "
          "Wraps over-long argument lists and vectors, puts a modifier's "
          "children on their own indented line, and preserves comments. "
          "Idempotent -- formatting formatted source returns it unchanged. "
          "Raises ParseError if the source does not parse.");
    m.def("debug_evaluate", &debugEvaluate, nb::arg("path"), nb::arg("viewport_params"), nb::arg("debug_hook"),
          nb::arg("error_break"), nb::arg("echo_fn"), nb::arg("manifold_cache") = nullptr,
          nb::arg("return_hook") = nb::none(), nb::arg("fast_continue_signal") = nullptr,
          "Evaluate with the debugger wired in; returns (bodies, [], id_to_node, dyn, dyn_explicit). Callbacks fire under the GIL.");
}
