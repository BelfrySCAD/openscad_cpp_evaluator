#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <cstdio>

namespace oscadeval {

// Microseconds below a millisecond, where "0.00 ms" would hide the answer.
std::string fmtMs(double ms) {
    char buf[32];
    if (ms < 1.0) std::snprintf(buf, sizeof buf, "%.1f \u00b5s", ms * 1000.0);
    else std::snprintf(buf, sizeof buf, "%.2f ms", ms);
    return buf;
}

std::string profileLabel(const Value& label, const oscad::Position& pos) {
    if (const std::string* s = std::get_if<std::string>(&label)) return *s;
    if (!std::holds_alternative<std::monostate>(label)) return fmtValue(label);
    std::string where = "line " + std::to_string(pos.line);
    if (!pos.origin.empty()) where += " of " + pos.origin.substr(pos.origin.find_last_of("/\\") + 1);
    return where;
}

// profile_time([label]) { ... } -- resolves its children like render() does.
// The script-time clock is NOT here but around this call in
// Evaluator::evalModularCall, so it covers argument evaluation and every
// child alike; the geometry time is taken in generateTreeImpl. The label
// goes into params (it is constant, so it costs the cache nothing).
CSGParams resolveProfileTime(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    params["label"] = getArg(args, 0, "label");
    EvalContext blockCtx = ev.blockScope(effCtx);
    ev.evalChildren(node.children, blockCtx);
    return params;
}

void Evaluator::reportProfile(const CSGNode& node, double geometryMs, bool cached) {
    if (!echoFn_ || node.node == nullptr) return;
    auto it = node.params.find("label");
    const std::string label = profileLabel(it == node.params.end() ? Value{} : it->second, node.node->position());
    const double scriptMs = node.profileScriptMs < 0 ? 0.0 : node.profileScriptMs;
    std::string msg = "PROFILE: " + label + ": ";
    if (geometryMs < 0) {
        msg += "script " + fmtMs(scriptMs) + " (no geometry built)";
    } else if (cached) {
        msg += fmtMs(scriptMs) + " (script " + fmtMs(scriptMs) + ", geometry cached)";
    } else {
        msg += fmtMs(scriptMs + geometryMs) + " (script " + fmtMs(scriptMs) + ", geometry " + fmtMs(geometryMs) + ")";
    }
    echoFn_(msg);
}

void Evaluator::reportCachedProfiles(const CSGNode& node) {
    for (const std::unique_ptr<CSGNode>& c : node.children) {
        if (c->isBuiltin && c->kind == "profile_time") reportProfile(*c, 0.0, true);
        reportCachedProfiles(*c);
    }
}

} // namespace oscadeval
