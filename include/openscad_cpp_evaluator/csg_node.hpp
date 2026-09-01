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


    // The call site that entered this node's call chain from the top
    // level, captured at RESOLVE time -- non-owning, AST-lifetime-bound
    // like `node`. nullptr for a node resolved at top level.
    //
    // Needed because a GenerateFn runs long after resolve has unwound the
    // call stack, so a warning raised there (an open polyhedron(), a
    // non-manifold import()) has no stack left to walk and could only ever
    // name the library's own line. One pointer per node, deliberately, so
    // this stays affordable on a BOSL2-scale tree; the full frame list a
    // TRACE would need is not worth the per-node cost.
    //
    // Not part of cacheKey() (an explicit allowlist of kind/isBuiltin/
    // params/children), so two identical subtrees reached from different
    // call sites still share a cache entry.
    const oscad::Position* warnEntry = nullptr;

    // 1 + the deepest child's own treeDepth (1 for a leaf) -- set once,
    // at construction, by whichever csg_resolve.cpp site finalizes this
    // node's own `children` (buildTreeNode, evalModularCall's non-splice
    // tail, spliceModuleChildren's union-wrap branch), which also checks
    // it against Evaluator::kMaxCsgTreeDepth right there and throws
    // before an unsafely deep tree can ever exist. See that constant's
    // own doc comment (evaluator.hpp) for why this has to be enforced at
    // construction time, not by a later walk over the finished tree --
    // by the time such a tree exists, std::unique_ptr<CSGNode>'s own
    // default (recursive) destructor is itself an un-guardable native-
    // stack risk the moment this node is ever destroyed.
    int treeDepth = 1;

    // Memoizes manifold_cache.cpp's cacheKey(*this) -- that function
    // recurses into every descendant to build its own key, so without this
    // memo a generate-time walk that calls cacheKey() at every node (not
    // just the root) redoes each subtree's serialization once per ancestor
    // that references it (O(n^2) on a chain-shaped tree). Populated lazily
    // on first cacheKey() call; every node here is always reached through a
    // non-const CSGNode&, so no `mutable` is needed to write it.
    std::optional<std::string> cachedKey;

    // Set by the same cacheKey() pass that fills cachedKey: true if this
    // node's own params, or any descendant's, hold a function literal.
    //
    // canonValue() can only key a closure by its AST node's raw address,
    // and that address is meaningful for exactly one parse: the next
    // render frees that AST, and a fresh closure over different captured
    // values can be allocated at the very same address -- so the key
    // silently hits the PREVIOUS render's geometry. (It also ignores the
    // captured values themselves, so two closures over one literal share a
    // key within a single render.) Keying a closure honestly would mean
    // canonicalising its captured `let_` trail, which is type-erased; until
    // that exists, such a subtree just does not participate in the cache.
    // See csg_generate.cpp's lookup.
    bool keyHasClosure = false;
};


} // namespace oscadeval
