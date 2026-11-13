#pragma once

#include "openscad_cpp_evaluator/csg_node.hpp"

#include <cstdint>
#include <mutex>
#include <vector>
#include <optional>
#include <string>
#include <unordered_map>

namespace oscadeval {

// Content-hash cache of already-generated CSGNode subtrees, so
// Evaluator::generateTree() can skip re-running Manifold work for a subtree
// whose resolved content (kind/params/children, see cacheKey() below)
// hasn't changed since a previous call. Opt-in: a bare Evaluator() has no
// cache (constructor default nullptr), so every existing single-shot
// caller (the CLI, this project's own tests) is unaffected; a long-lived
// host (a GUI issuing repeated re-evaluates against a live-edited script)
// constructs one ManifoldCache and passes it into each new Evaluator() it
// creates, so it survives across the fresh Evaluator/AST/CSGNode objects
// every re-evaluate creates. Thread-safe (mirrors the reference's own
// threading.Lock -- a host may run renders/evaluates on a background
// thread while another operation reads/clears the same cache). Mirrors
// the reference's ManifoldCache class.
// What one cache entry holds. The warnings matter as much as the bodies:
// a hit skips the whole subtree's GenerateFn, and every generate-time
// diagnostic is raised *inside* one of those (polyhedron's "mesh is not
// closed", import()'s failures, the dimension rules). Cached without them,
// a second render of unchanged source silently dropped every warning the
// first one raised, so a real defect looked intermittent and the author
// went hunting for a phantom. A cache has to be transparent; that includes
// what it says, not only what it draws. See BelfrySCAD issue #521.
struct CachedSubtree {
    std::vector<ColoredBody> bodies;
    std::vector<std::string> warnings;  // already formatted, replayed verbatim
    // originalID in `bodies` -> which node of the subtree made it, as its
    // CSGNode::preorder minus the cached node's own. A hit reads it to give
    // each part back its own line and call chain (BelfrySCAD #1123); an ID
    // missing here falls back to the hit node.
    std::vector<std::pair<uint32_t, uint32_t>> attribution;
};

class ManifoldCache {
public:
    // Eviction (BelfrySCAD #1117): an animation makes new keys every frame
    // ($t moves the parts), so a cache that kept everything grew by a
    // frame's geometry per frame until a long animation took all of RAM.
    // With `keepRenders` N > 0, beginRender() -- called once per top-level
    // generateTree() -- drops every entry no get()/put() touched in the
    // last N renders. A part unchanged between frames is hit every frame
    // and stays. 0 keeps everything, as before.
    explicit ManifoldCache(unsigned keepRenders = 0) : keepRenders_(keepRenders) {}
    std::optional<CachedSubtree> get(const std::string& key) const;
    void put(std::string key, std::vector<ColoredBody> bodies, std::vector<std::string> warnings,
             std::vector<std::pair<uint32_t, uint32_t>> attribution = {});
    void clear();
    void touch(const std::string& key);  // marks it used, as get() does
    void beginRender();
    size_t size() const;

private:
    struct Entry {
        CachedSubtree value;
        uint64_t lastUsed;
    };
    mutable std::mutex mutex_;
    mutable std::unordered_map<std::string, Entry> entries_;  // get() stamps lastUsed
    unsigned keepRenders_;
    uint64_t generation_ = 0;
};

// Structural content-hash key for `node`, used by generateTree()'s
// ManifoldCache lookup: encodes kind/isBuiltin/canonicalized params/
// recursively-hashed children into one self-delimiting (length-prefixed,
// so no atom's content can be confused with a structural delimiter)
// string. Deliberately excludes CSGNode::node (the AST pointer) -- keying
// on AST identity would defeat cross-evaluate caching entirely, since a
// fresh AST is built on every parse. Params are canonicalized with each
// Value alternative tagged distinctly (in particular bool is tagged apart
// from double, mirroring the reference's own _canon: a real corruption bug
// there came from Python's `False == 0` colliding two differently-typed
// cached results) so no two semantically different param values can ever
// hash to the same key. Pure function of already-resolved data (never
// touches CSGNode::bodies), so it's always cheap/safe to compute
// speculatively even on a cache miss. Mirrors the reference's
// Evaluator._cache_key/_canon.
//
// Takes a non-const reference because it memoizes the result into
// CSGNode::cachedKey -- without this, a top-down walk that calls
// cacheKey() at every node (not just the root, see generateTree()) would
// re-serialize each subtree once per ancestor that references it.
std::string cacheKey(CSGNode& node);

} // namespace oscadeval
