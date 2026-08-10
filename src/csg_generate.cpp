#include "openscad_cpp_evaluator/dispatch.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <algorithm>

namespace oscadeval {

std::vector<ColoredBody> flattenCsgTree(const std::vector<std::unique_ptr<CSGNode>>& tree) {
    std::vector<ColoredBody> result;
    for (const auto& node : tree) {
        for (const ColoredBody& b : node->bodies) result.push_back(b);
    }
    return result;
}

std::vector<ColoredBody> flattenCsgTree(const std::vector<std::unique_ptr<CSGNode>>& tree, size_t begin, size_t count) {
    std::vector<ColoredBody> result;
    const size_t end = std::min(tree.size(), begin + count);
    for (size_t i = begin; i < end; ++i) {
        for (const ColoredBody& b : tree[i]->bodies) result.push_back(b);
    }
    return result;
}

// Shared bottom-up walk behind generateTree() (owning unique_ptr tree) and
// generatePartialTree() (a live, still-being-resolved treeStack_ level,
// which the resolve pass -- not this function -- continues to own): takes
// non-owning CSGNode* so both callers share one implementation without
// either transferring ownership. unique_ptr's constness doesn't propagate
// to the pointee, so mutating node.bodies here is legal even from a const
// vector<unique_ptr<CSGNode>>& (see csg_node.hpp) -- this overload just
// makes that non-owning access explicit for the treeStack_ caller too.
std::vector<ColoredBody> Evaluator::generateTreeImpl(const std::vector<CSGNode*>& tree) {
    std::vector<ColoredBody> topLevelBodies;
    const auto& dispatch = generateDispatch();
    for (CSGNode* nodePtr : tree) {
        CSGNode& node = *nodePtr;

        // ManifoldCache lookup: node.uncacheable (rands() taint, set
        // during resolve -- see csg_resolve.cpp) always forces a miss,
        // never a hit, and never gets stored either, since its resolved
        // params aren't a pure function of its own content. On a hit,
        // children are never generated at all -- no wasted Manifold work
        // re-deriving results that would just be discarded. Mirrors
        // generate_tree()'s own cache check exactly.
        std::optional<std::string> key;
        if (manifoldCache_ && !node.uncacheable) key = cacheKey(node);
        std::optional<std::vector<ColoredBody>> cached = key ? manifoldCache_->get(*key) : std::nullopt;
        if (cached) {
            node.bodies = std::move(*cached);
            if (node.node) {
                auto producer = cacheProducer_.find(*key);
                restampCachedIds(node.bodies, *node.node,
                                 producer == cacheProducer_.end() ? nullptr : producer->second);
            }
        } else {
            // Recurse into children first (bottom-up) -- populates each
            // child's own .bodies in place so flattenCsgTree/a GenerateFn
            // can read them back.
            std::vector<CSGNode*> childPtrs;
            childPtrs.reserve(node.children.size());
            for (const std::unique_ptr<CSGNode>& c : node.children) childPtrs.push_back(c.get());
            generateTreeImpl(childPtrs);

            auto it = dispatch.find(node.kind);
            if (node.isBuiltin && it != dispatch.end()) {
                // Resolve captured which user-level call reached this node;
                // republish it so a warning raised inside the GenerateFn
                // can name that line. Restored after (rather than left set)
                // so a sibling generated at top level doesn't inherit it.
                const oscad::Position* savedWarnEntry = generateWarnEntry;
                generateWarnEntry = node.warnEntry;
                node.bodies = it->second(*this, node.params, node.children, *node.node);
                generateWarnEntry = savedWarnEntry;
            } else {
                // No registered GenerateFn for this kind (a builtin with
                // display-only semantics like render(), or a non-builtin
                // node): default to concatenating children's bodies,
                // matching a module body's plain concatenation semantics.
                // Mirrors generate_tree()'s own fallback.
                node.bodies = flattenCsgTree(node.children);
            }
            if (key) {
                manifoldCache_->put(*key, node.bodies);
                cacheProducer_[*key] = node.node;
            }
        }
        for (const ColoredBody& b : node.bodies) topLevelBodies.push_back(b);
    }
    return topLevelBodies;
}

std::vector<ColoredBody> Evaluator::generateTree(const std::vector<std::unique_ptr<CSGNode>>& tree) {
    std::vector<CSGNode*> ptrs;
    ptrs.reserve(tree.size());
    for (const std::unique_ptr<CSGNode>& n : tree) ptrs.push_back(n.get());
    return generateTreeImpl(ptrs);
}

std::vector<ColoredBody> Evaluator::generatePartialTree() {
    // Flatten treeStack_ across every nesting level (mirrors the reference's
    // `[node for level in ev._tree_stack for node in level]`): a CSGNode
    // only lands in its parent's own accumulator once the parent's resolve
    // finishes, so for a script whose whole geometry is one deeply-nested
    // top-level statement, the *top* of the tree stays empty for the entire
    // time spent stepping through its innermost leaves -- flattening every
    // level picks up already-resolved leaves (e.g. a finished cube() sitting
    // in a still-in-progress union()'s accumulator) regardless of how deep.
    std::vector<CSGNode*> flat;
    for (const std::vector<std::unique_ptr<CSGNode>>& level : treeStack_)
        for (const std::unique_ptr<CSGNode>& node : level) flat.push_back(node.get());
    return generateTreeImpl(flat);
}

void Evaluator::restampCachedIds(std::vector<ColoredBody>& bodies, const oscad::ASTNode& node,
                                 const oscad::ASTNode* producer) {
    // A cache hit hands back the geometry AND the originalIDs of whichever
    // call site first produced it. Those IDs are provenance -- "which node
    // made this" -- not content, so two identical shapes at two call sites
    // came back sharing one identity and selecting either picked both.
    //
    // Each distinct run gets a fresh ID, one per run rather than one per
    // body, so a cached subtree made of several parts stays selectable
    // part by part. Which node a fresh ID points at depends on what the old
    // one stood for:
    //
    //   - the reused node itself (`producer`): two separate pieces of
    //     source wrote the same shape, so this copy belongs to the node
    //     reusing it, not the one that happened to be generated first.
    //   - something deeper inside it -- the cube and sphere inside a module
    //     called twice: both calls really do come from that one line of the
    //     module body, so the old node is the honest answer and re-pointing
    //     it at the call site would collapse every part of the call onto a
    //     single line.
    //
    // Across renders the old IDs predate this render's idToNode, so nothing
    // is known about their structure and everything falls to `node`.
    //
    // ponytail: two subtrees identical down to their transforms still
    // share one attribution, since only the inner node is a cache hit and
    // its producer check keeps the first copy's descendants. They occupy
    // the same space, so there is nothing to select apart.
    for (ColoredBody& cb : bodies) {
        if (!cb.body || cb.body->IsEmpty()) continue;
        manifold::MeshGL mesh = cb.body->GetMeshGL();
        if (mesh.runOriginalID.empty()) continue;
        std::unordered_map<uint32_t, uint32_t> remap;
        for (uint32_t& id : mesh.runOriginalID) {
            auto found = remap.find(id);
            if (found == remap.end()) {
                const uint32_t fresh = manifold::Manifold::ReserveIDs(1);
                auto old = idToNode.find(id);
                idToNode[fresh] =
                    (old != idToNode.end() && old->second != producer) ? old->second : &node;
                auto color = idToColor.find(id);
                if (color != idToColor.end()) idToColor[fresh] = color->second;
                found = remap.emplace(id, fresh).first;
            }
            id = found->second;
        }
        cb.body = manifold::Manifold(mesh);
    }
}

ColoredBody Evaluator::tagGenerated(manifold::Manifold body, const oscad::ASTNode& node, const Value& colorValue) {
    manifold::MeshGL mesh = body.GetMeshGL();
    std::optional<std::array<float, 4>> color = valueToColor(colorValue);
    for (uint32_t originalId : mesh.runOriginalID) {
        idToNode[originalId] = &node;
        idToColor[originalId] = color;
    }
    ColoredBody cb;
    cb.body = std::move(body);
    cb.color = color;
    return cb;
}

ColoredBody Evaluator::tagDisplayOnly(manifold::MeshGL mesh, const oscad::ASTNode& node, const Value& colorValue) {
    const std::optional<std::array<float, 4>> color = valueToColor(colorValue);

    // A Manifold that failed to build carries no run IDs, so reserve one
    // and describe the whole soup as a single run. ReserveIDs() draws from
    // the same global counter Manifold hands to real bodies, so an ID
    // minted here can never collide with one of theirs.
    const uint32_t originalId = manifold::Manifold::ReserveIDs(1);
    mesh.runOriginalID = {originalId};
    mesh.runIndex = {0, static_cast<uint32_t>(mesh.triVerts.size())};
    idToNode[originalId] = &node;
    idToColor[originalId] = color;

    ColoredBody cb;
    // Left set-but-empty rather than nullopt: consumers dereference
    // cb.body unconditionally, and an empty Manifold is also the honest
    // answer to "what solid is this?" -- there isn't one.
    cb.body = manifold::Manifold();
    cb.color = color;
    cb.rawMesh = std::move(mesh);
    return cb;
}

} // namespace oscadeval
