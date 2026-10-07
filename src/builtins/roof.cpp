#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <manifold/polygon.h>

#include <boost/polygon/voronoi.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

// roof(): a roof over the children's 2D outline.
namespace oscadeval {

namespace {

// The roof solid over `cs` (its floor at z = 0), discretizing curved parts
// by `fa`/`fs`.
// CLEAN-ROOM: reimplement from spec section E1.
manifold::Manifold voronoiRoof(const manifold::CrossSection&, double, double) {
    return manifold::Manifold();
}

} // namespace

// roof(method="voronoi") -- `method` is accepted and validated (unknown
// values warn and fall back to "voronoi") though both values build the same
// roof here (spec section E1).

// Split like computeLinearExtrudeParams (extrude.cpp) -- but UNLIKE that
// group, this one genuinely can't move before evalChildren: the "Unknown
// roof method" warning below is an observable side effect (an echo/warn
// message), and native resolveRoof always evaluates children FIRST, so
// this warning fires AFTER any echo()/warn() a child produces. Op::
// PushBuiltinWrap's own runtime handler (bytecode_vm.cpp) therefore calls
// this at POP time (after children finish -- VmFrame::builtinWrapStack
// retains `args` for exactly this) rather than at PUSH time the way
// computeLinearExtrudeParams/computeTransformParams/computeColorParams are
// called -- see Op::PushBuiltinWrap's own Roof-kind doc comment
// (bytecode.hpp) for the full contract. Takes the already-resolved
// `CallArgs`/`EvalContext` directly (not the raw node+ctx) since by POP
// time the argument expressions have already run once and must not
// re-run (double rands()/side effects).
CSGParams computeRoofParams(Evaluator& ev, const CallArgs& args, EvalContext& effCtx) {
    Value methodArg = getArg(args, std::nullopt, "method", Value{std::string("voronoi")});
    std::string method = std::holds_alternative<std::string>(methodArg) ? std::get<std::string>(methodArg) : "voronoi";
    if (method != "voronoi" && method != "straight") {
        // No location suffix here -- mirrors the reference's own bare
        // echo_fn call for this particular warning (unlike most others).
        ev.warn("Unknown roof method '" + method + "'. Using 'voronoi'.", nullptr);
        method = "voronoi";
    }

    const auto dynOr = [&](const char* name, double fallback) {
        const Value* v = effCtx.dyn->find(name);
        if (!v) return fallback;
        const double* d = std::get_if<double>(v);
        return d ? *d : fallback;
    };

    CSGParams params;
    params["method"] = Value{method};
    params["fa"] = Value{dynOr("$fa", 12.0)};
    params["fs"] = Value{dynOr("$fs", 2.0)};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

CSGParams resolveRoof(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(effCtx);
    ev.evalChildren(node.children, blockCtx);
    return computeRoofParams(ev, args, effCtx);
}

std::vector<ColoredBody> generateRoof(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>& children,
                                       const oscad::ASTNode& node) {
    const std::optional<manifold::CrossSection> cs = toCrossSection(flattenCsgTree(children));
    if (!cs) return {};
    if (cs->ToPolygons().empty()) return {};

    try {
        manifold::Manifold body = voronoiRoof(*cs, std::get<double>(params.at("fa")), std::get<double>(params.at("fs")));
        if (body.IsEmpty()) return {};
        return {ev.tagGenerated(std::move(body), node, params.at("color"))};
    } catch (const std::exception& e) {
        ev.error(std::string("roof: ") + e.what(), node);
        return {};
    }
}

} // namespace oscadeval
