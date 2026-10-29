#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <map>
#include <set>

namespace oscadeval {

// export_as("name") { ... } -- names the geometry of its children as one
// object in a file that holds several (3MF, OBJ, AMF); see
// splitBodiesForExport. The name rides on ColoredBody::exportName, which
// every transform, color() and modifier carries along because they copy the
// body. An inner export_as() keeps its own name: only unnamed bodies take
// this one.
CSGParams resolveExportAs(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    const Value name = getArg(args, 0, "name");
    const std::string* s = std::get_if<std::string>(&name);
    if (s && !s->empty()) params["name"] = *s;
    else ev.warn("export_as() needs a non-empty name string; its children are exported unnamed", &node.position());
    EvalContext blockCtx = ev.blockScope(effCtx);
    ev.evalChildren(node.children, blockCtx);
    return params;
}

std::vector<ColoredBody> generateExportAs(Evaluator&, const CSGParams& params,
                                          const std::vector<std::unique_ptr<CSGNode>>& children,
                                          const oscad::ASTNode&) {
    std::vector<ColoredBody> result = flattenCsgTree(children);
    auto it = params.find("name");
    if (it == params.end()) return result;
    const std::string& name = std::get<std::string>(it->second);
    for (ColoredBody& b : result) {
        if (!b.exportName) b.exportName = name;
    }
    return result;
}

namespace {

// The builtins whose result is their children's own bodies, copied (so an
// exportName survives them). Every other builtin builds new bodies from its
// children and drops the name.
bool keepsChildBodies(const CSGNode& n) {
    static const std::set<std::string, std::less<>> kinds = {
        "translate", "rotate", "scale", "mirror", "multmatrix", "resize", "color", "highlight",
        "background", "show_only", "render", "profile_time", "export_as", "children",
    };
    return !n.isBuiltin || kinds.count(n.kind) > 0;
}

void walkExportAs(Evaluator& ev, const std::vector<std::unique_ptr<CSGNode>>& nodes, const CSGNode* merger,
                  std::map<std::string, int>& uses) {
    for (const std::unique_ptr<CSGNode>& c : nodes) {
        const CSGNode* childMerger = merger;
        if (c->isBuiltin && c->kind == "export_as") {
            auto it = c->params.find("name");
            if (it != c->params.end() && c->node) {
                const std::string& name = std::get<std::string>(it->second);
                if (merger) {
                    ev.warn("export_as(\"" + name + "\") is inside " + merger->kind + "(), which merges it with " +
                                "its siblings, so the name is lost",
                            &c->node->position());
                } else if (++uses[name] == 2) {
                    ev.warn("export_as(\"" + name + "\") is used more than once; those parts are exported as one " +
                                "object named \"" + name + "\"",
                            &c->node->position());
                }
            }
        } else if (!merger && !keepsChildBodies(*c)) {
            childMerger = c.get();
        }
        walkExportAs(ev, c->children, childMerger, uses);
    }
}

} // namespace

void Evaluator::checkExportNames(const std::vector<std::unique_ptr<CSGNode>>& tree) {
    std::map<std::string, int> uses;
    walkExportAs(*this, tree, nullptr, uses);
}

} // namespace oscadeval
