#pragma once

#include "openscad_cpp_evaluator/colored_body.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast/ast_node.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace oscadeval {

// Resolved-argument bag for a geometry-producing node, keyed by name.
// Deliberately reuses Value (not a per-builtin struct) so a later phase's
// content-hash cache (ManifoldCache, Phase 8) can walk any node's params
// generically -- see the plan's §5 rationale.
using CSGParams = std::unordered_map<std::string, Value>;

// One node in the persistent CSG tree built by the resolve pass and filled
// in by the generate pass. Mirrors the Python reference's CSGNode dataclass
// -- see openscad_evaluator/docs/evaluator.md's "CSG tree" section for the
// full design. Only ModularCall gets a CSGNode as of Phase 2 (the 3 tag
// modifiers and ModularIntersectionFor -- Phase 3/4 -- get one too, per the
// reference; everything else is transparent, no synthetic node).
struct CSGNode {
    std::string kind;                     // e.g. "cube", "union", or (later phase) a user module's own name
    const oscad::ASTNode* node = nullptr;  // non-owning, AST-lifetime-bound
    std::vector<ColoredBody> bodies;       // empty until generateTree()
    bool isBuiltin = true;
    std::vector<std::unique_ptr<CSGNode>> children; // CSGNode owns its own children (strict tree)
    CSGParams params;                      // resolve step's plain-data output
    bool uncacheable = false;              // set by a later phase (ManifoldCache, Phase 8)

    // Memoizes manifold_cache.cpp's cacheKey(*this) -- that function
    // recurses into every descendant to build its own key, so without this
    // memo a generate-time walk that calls cacheKey() at every node (not
    // just the root) redoes each subtree's serialization once per ancestor
    // that references it (O(n^2) on a chain-shaped tree). Populated lazily
    // on first cacheKey() call; every node here is always reached through a
    // non-const CSGNode&, so no `mutable` is needed to write it.
    std::optional<std::string> cachedKey;
};

} // namespace oscadeval
