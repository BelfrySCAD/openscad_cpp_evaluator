#include "openscad_cpp_evaluator/manifold_cache.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace oscadeval {

std::optional<std::vector<ColoredBody>> ManifoldCache::get(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = entries_.find(key);
    if (it == entries_.end()) return std::nullopt;
    return it->second;
}

void ManifoldCache::put(std::string key, std::vector<ColoredBody> bodies) {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_[std::move(key)] = std::move(bodies);
}

void ManifoldCache::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
}

namespace {

// Appends `s`'s length then a ':' then `s` itself -- a netstring-style
// length prefix so concatenating several of these back to back (as every
// composite canonValue()/cacheKey() case below does) can never be
// ambiguous, regardless of what bytes `s` itself contains.
void appendLenPrefixed(std::string& out, std::string_view s) {
    out += std::to_string(s.size());
    out += ':';
    out += s;
}

std::string hexBits(double d) {
    uint64_t bits;
    std::memcpy(&bits, &d, sizeof(bits));
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(bits));
    return buf;
}

std::string canonValue(const Value& v) {
    return std::visit(
        [](const auto& val) -> std::string {
            using T = std::decay_t<decltype(val)>;
            if constexpr (std::is_same_v<T, std::monostate>) {
                return "u";
            } else if constexpr (std::is_same_v<T, bool>) {
                // Tagged distinctly from double -- see the header comment's
                // note on the reference's own False==0 collision bug.
                return val ? "bT" : "bF";
            } else if constexpr (std::is_same_v<T, double>) {
                return "d" + hexBits(val);
            } else if constexpr (std::is_same_v<T, std::string>) {
                std::string out = "s";
                appendLenPrefixed(out, val);
                return out;
            } else if constexpr (std::is_same_v<T, ListPtr>) {
                std::string out = "L" + std::to_string(val ? val->items.size() : 0) + "(";
                if (val) {
                    for (const Value& item : val->items) appendLenPrefixed(out, canonValue(item));
                }
                out += ")";
                return out;
            } else if constexpr (std::is_same_v<T, OscRange>) {
                return "r" + hexBits(val.start) + hexBits(val.step) + hexBits(val.end);
            } else if constexpr (std::is_same_v<T, ObjectPtr>) {
                // Preserves the object's own insertion order rather than
                // sorting -- oscEqual() is itself order-sensitive (doc:
                // object() entry), so two values oscEqual() would call
                // equal already share the same order; sorting here would
                // gain nothing and cost an extra copy/sort per object.
                std::string out = "O" + std::to_string(val ? val->items.size() : 0) + "(";
                if (val) {
                    for (const auto& [k, vv] : val->items) {
                        appendLenPrefixed(out, k);
                        appendLenPrefixed(out, canonValue(vv));
                    }
                }
                out += ")";
                return out;
            } else { // const oscad::FunctionLiteral*
                char buf[32];
                std::snprintf(buf, sizeof(buf), "F%p", static_cast<const void*>(val));
                return buf;
            }
        },
        v);
}

std::string canonParams(const CSGParams& params) {
    std::vector<std::pair<std::string, std::string>> entries;
    entries.reserve(params.size());
    for (const auto& [k, v] : params) entries.emplace_back(k, canonValue(v));
    // params is an unordered_map -- iteration order is arbitrary, so it
    // must be sorted for the key to be a pure function of content (not of
    // hash-bucket layout). Mirrors the reference's own deliberate dict-key
    // sort in _canon (needed there because a params dict commonly mixes
    // int positional-arg keys with str named-arg keys, e.g. cube(10,
    // center=true) -- not an issue here since CSGParams keys are always
    // strings, but the sort is still required for determinism).
    std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    std::string out = "P" + std::to_string(entries.size()) + "(";
    for (const auto& [k, canonV] : entries) {
        appendLenPrefixed(out, k);
        appendLenPrefixed(out, canonV);
    }
    out += ")";
    return out;
}

} // namespace

std::string cacheKey(const CSGNode& node) {
    std::string out = "N(";
    appendLenPrefixed(out, node.kind);
    out += node.isBuiltin ? "1" : "0";
    appendLenPrefixed(out, canonParams(node.params));
    out += std::to_string(node.children.size()) + "[";
    for (const auto& c : node.children) appendLenPrefixed(out, cacheKey(*c));
    out += "])";
    return out;
}

} // namespace oscadeval
