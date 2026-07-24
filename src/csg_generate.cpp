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

std::vector<ColoredBody> Evaluator::generateTree(const std::vector<std::unique_ptr<CSGNode>>& tree) {
    std::vector<ColoredBody> topLevelBodies;
    const auto& dispatch = generateDispatch();
    for (const auto& nodePtr : tree) {
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
        } else {
            // Recurse into children first (bottom-up) -- populates each
            // child's own .bodies in place so flattenCsgTree/a GenerateFn
            // can read them back. unique_ptr's constness doesn't propagate
            // to the pointee, so this mutation is legal even though `tree`
            // is a const& (see csg_node.hpp).
            generateTree(node.children);

            auto it = dispatch.find(node.kind);
            if (node.isBuiltin && it != dispatch.end()) {
                node.bodies = it->second(*this, node.params, node.children, *node.node);
            } else {
                // No registered GenerateFn for this kind (a builtin with
                // display-only semantics like render(), or a non-builtin
                // node): default to concatenating children's bodies,
                // matching a module body's plain concatenation semantics.
                // Mirrors generate_tree()'s own fallback.
                node.bodies = flattenCsgTree(node.children);
            }
            if (key) manifoldCache_->put(*key, node.bodies);
        }
        for (const ColoredBody& b : node.bodies) topLevelBodies.push_back(b);
    }
    return topLevelBodies;
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

} // namespace oscadeval
