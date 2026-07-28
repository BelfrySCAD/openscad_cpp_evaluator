#include "openscad_cpp_evaluator/dispatch.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "builtins/builtins.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>

namespace oscadeval {

void Evaluator::buildTreeNode(const std::string& kind, const oscad::ASTNode& node,
                               const std::function<CSGParams()>& resolveBody) {
    // Push a fresh accumulator for whatever CSGNodes get created while
    // resolving THIS node -- i.e. its own children. Any nested
    // evalModularCall/evalModifier triggered from within `resolveBody`
    // (via evalChildren/evalStatement) appends into this frame, since it's
    // now the top of treeStack_.
    treeStack_.emplace_back();
    const std::uint64_t randsBefore = randsCallCount_;
    CSGParams params;
    try {
        params = resolveBody();
    } catch (...) {
        treeStack_.pop_back();
        throw;
    }
    std::vector<std::unique_ptr<CSGNode>> children = std::move(treeStack_.back());
    treeStack_.pop_back();

    // Taint this node if rands() fired anywhere while resolving it -- own
    // resolve or any descendant's (already reflected in each child's own
    // uncacheable flag). See CSGNode::uncacheable's doc comment.
    const bool uncacheable = (randsCallCount_ != randsBefore) ||
                              std::any_of(children.begin(), children.end(), [](const auto& c) { return c->uncacheable; });

    auto treeNode = std::make_unique<CSGNode>();
    treeNode->kind = kind;
    treeNode->node = &node;
    treeNode->isBuiltin = true;
    treeNode->children = std::move(children);
    treeNode->params = std::move(params);
    treeNode->uncacheable = uncacheable;
    treeStack_.back().push_back(std::move(treeNode));
}

void Evaluator::evalModularCall(const oscad::ModularCall& node, EvalContext& ctx) {
    const std::string& name = node.name->name;

    // is_builtin: false when `name` resolves to a user-defined module (a
    // user module can shadow a builtin name, so name/kind alone isn't a
    // unique discriminator -- matches the reference's own comment on
    // _tree_node_kind).
    const oscad::ASTNode* userModuleDecl = ctx.scope->lookupModule(name);
    const bool isBuiltin = (userModuleDecl == nullptr);

    const auto& dispatch = resolveDispatch();
    auto it = isBuiltin ? dispatch.find(name) : dispatch.end();
    const bool hasResolveFn = (it != dispatch.end());

    treeStack_.emplace_back();
    const std::uint64_t randsBefore = randsCallCount_;
    CSGParams params;
    try {
        if (hasResolveFn) {
            params = it->second(*this, node, ctx);
        } else if (!isBuiltin) {
            evalUserModule(static_cast<const oscad::ModuleDeclaration&>(*userModuleDecl), node, ctx);
        } else {
            // Genuinely unknown module name (a typo, or a builtin not in
            // this port's registry) -- real OpenSCAD issues a WARNING with
            // TRACE lines and simply produces no geometry for this call,
            // continuing evaluation of everything else. Mirrors
            // _eval_builtin's unknown-module fallback exactly (verified
            // against the Python reference directly).
            treeStack_.pop_back();
            std::string msg = "WARNING: Ignoring unknown module '" + name + "'" + locSuffix(&node.position());
            for (const std::string& line : traceLines(&node.position(), callStack_)) msg += "\n" + line;
            if (echoFn_) echoFn_(msg);
            return;
        }
    } catch (...) {
        treeStack_.pop_back();
        throw;
    }
    std::vector<std::unique_ptr<CSGNode>> children = std::move(treeStack_.back());
    treeStack_.pop_back();

    // children() and any user-module call are not themselves geometry --
    // children() is a call-site substitution, and a user module is just a
    // named wrapper around whatever geometry statements its body runs.
    // Splice the resolved subtree directly into the enclosing accumulator
    // instead of wrapping it in its own node, grouping >1 spliced sibling
    // under a synthetic (display-only, is_builtin=false) "union" node so
    // the tree still reads as one shape at this call site. Mirrors
    // _eval_statement's splice branch exactly.
    const bool splice = (name == "children" && isBuiltin) || !isBuiltin;
    if (splice) {
        if (randsCallCount_ != randsBefore) {
            // rands() fired directly during *this* call's own resolve
            // (e.g. an assignment before any geometry statement in a user
            // module's body) rather than inside one of the spliced
            // children's own resolve (which already self-taints via the
            // branch below) -- propagate onto every spliced child so the
            // taint isn't silently dropped by splicing away the node it
            // would otherwise have landed on.
            for (auto& c : children) c->uncacheable = true;
        }
        if (children.size() > 1) {
            auto unionNode = std::make_unique<CSGNode>();
            unionNode->kind = "union";
            unionNode->node = &node;
            unionNode->isBuiltin = false;
            unionNode->uncacheable = std::any_of(children.begin(), children.end(), [](const auto& c) { return c->uncacheable; });
            unionNode->children = std::move(children);
            treeStack_.back().push_back(std::move(unionNode));
        } else {
            for (auto& c : children) treeStack_.back().push_back(std::move(c));
        }
        return;
    }

    const bool uncacheable = (randsCallCount_ != randsBefore) ||
                              std::any_of(children.begin(), children.end(), [](const auto& c) { return c->uncacheable; });
    auto treeNode = std::make_unique<CSGNode>();
    treeNode->kind = name;
    treeNode->node = &node;
    treeNode->isBuiltin = true;
    treeNode->uncacheable = uncacheable;
    treeNode->children = std::move(children);
    treeNode->params = std::move(params);
    treeStack_.back().push_back(std::move(treeNode));
}

void Evaluator::evalModifier(const oscad::ModuleInstantiation& child, const oscad::ASTNode& wrapperNode,
                              const std::string& kind, EvalContext& ctx) {
    // Mirrors _resolve_modifier_child: evaluate the single wrapped child
    // for the side effect of building its own CSGNode(s); params are
    // always empty (role tagging happens entirely in the matching
    // generate function -- generateHighlight/Background/ShowOnly).
    buildTreeNode(kind, wrapperNode, [&]() {
        evalStatement(child, ctx);
        return CSGParams{};
    });
}

void Evaluator::evalIntersectionForNode(const oscad::ModularIntersectionFor& node, EvalContext& ctx) {
    buildTreeNode("intersection_for", node, [&]() { return resolveIntersectionFor(*this, node, ctx); });
}

template <typename NodeList>
std::vector<std::unique_ptr<CSGNode>> Evaluator::resolveTreeImpl(const NodeList& nodes, EvalContext& ctx) {
    idToNode.clear();
    idToColor.clear();
    treeStack_.clear();
    treeStack_.emplace_back();
    rootCtx_ = &ctx;
    lastCtx_ = &ctx;
    evalChildren(nodes, ctx);
    std::vector<std::unique_ptr<CSGNode>> tree = std::move(treeStack_.back());
    treeStack_.pop_back();
    return tree;
}

std::vector<std::unique_ptr<CSGNode>> Evaluator::resolveTree(const std::vector<std::unique_ptr<oscad::ASTNode>>& nodes,
                                                               EvalContext& ctx) {
    return resolveTreeImpl(nodes, ctx);
}

std::vector<std::unique_ptr<CSGNode>> Evaluator::resolveTree(const std::vector<const oscad::ASTNode*>& nodes, EvalContext& ctx) {
    return resolveTreeImpl(nodes, ctx);
}

template <typename NodeList>
std::vector<ColoredBody> Evaluator::evaluateImpl(const NodeList& nodes, EvalContext& ctx,
                                                  const std::unordered_map<std::string, Value>& viewportParams) {
    // Seeds ctx.dyn directly, deliberately not touching ctx.dynExplicit --
    // see this method's own doc comment in evaluator.hpp for why that
    // distinction matters to a caller.
    for (const auto& [k, v] : viewportParams) ctx.dyn->set(k, v);

    profileResult.reset();
    profileSites_.clear();
    profileActive_.clear();
    profileChildTime_.clear();

    using Clock = std::chrono::steady_clock;
    const Clock::time_point resolveStart = profiling_ ? Clock::now() : Clock::time_point{};
    std::vector<std::unique_ptr<CSGNode>> tree = resolveTreeImpl(nodes, ctx);
    const Clock::time_point resolveEnd = profiling_ ? Clock::now() : Clock::time_point{};
    std::vector<ColoredBody> result = generateTree(tree);
    const Clock::time_point generateEnd = profiling_ ? Clock::now() : Clock::time_point{};
    csgTree = std::move(tree); // stash for caller inspection -- see the member's own doc comment

    if (profiling_) {
        const double resolveTime = std::chrono::duration<double>(resolveEnd - resolveStart).count();
        const double generateTime = std::chrono::duration<double>(generateEnd - resolveEnd).count();
        double selfSum = 0.0;
        std::vector<CallSiteProfile> sites;
        sites.reserve(profileSites_.size());
        for (const auto& [key, site] : profileSites_) {
            selfSum += site.selfTime;
            sites.push_back(site);
        }
        profileResult = ProfileResult{
            std::move(sites), resolveTime, generateTime, resolveTime + generateTime, std::max(0.0, resolveTime - selfSum),
        };
    }

    const bool anyShowOnly = std::any_of(result.begin(), result.end(),
                                          [](const ColoredBody& b) { return b.role == BodyRole::ShowOnly; });
    if (anyShowOnly) {
        std::vector<ColoredBody> filtered;
        for (ColoredBody& b : result) {
            if (b.role == BodyRole::ShowOnly || b.role == BodyRole::Highlight) filtered.push_back(std::move(b));
        }
        return filtered;
    }
    return result;
}

std::vector<ColoredBody> Evaluator::evaluate(const std::vector<std::unique_ptr<oscad::ASTNode>>& nodes, EvalContext& ctx,
                                              const std::unordered_map<std::string, Value>& viewportParams) {
    return evaluateImpl(nodes, ctx, viewportParams);
}

std::vector<ColoredBody> Evaluator::evaluate(const std::vector<const oscad::ASTNode*>& nodes, EvalContext& ctx,
                                              const std::unordered_map<std::string, Value>& viewportParams) {
    return evaluateImpl(nodes, ctx, viewportParams);
}

} // namespace oscadeval
