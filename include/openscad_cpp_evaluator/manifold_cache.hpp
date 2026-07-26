#pragma once

#include "openscad_cpp_evaluator/csg_node.hpp"

#include <mutex>
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
class ManifoldCache {
public:
    std::optional<std::vector<ColoredBody>> get(const std::string& key) const;
    void put(std::string key, std::vector<ColoredBody> bodies);
    void clear();

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::vector<ColoredBody>> entries_;
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
