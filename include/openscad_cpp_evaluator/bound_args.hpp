#pragma once

#include "openscad_cpp_evaluator/value.hpp"

#include <string>
#include <utility>
#include <vector>

namespace oscadeval {

// A user function/module call's fully-bound arguments -- every entry
// already resolved to its final parameter NAME (a positional argument is
// mapped to the matching ParameterDeclaration's name before landing here),
// unlike CallArgs (call_args.hpp), which still distinguishes
// positional-by-index from named for a builtin call. Flat vector, not
// unordered_map, for the same reason CallArgs is: real arity is almost
// always a handful of parameters, and this is rebuilt fresh on every
// single user function/module call (and, via the bytecode VM's tail-call
// trampoline, on every tail hop of a recursive one).
class BoundArgs {
public:
    // Insert-or-overwrite by name -- mirrors the map-assignment semantics
    // (`result[name] = value`) every caller here used to rely on, so a
    // repeated name (a positional argument followed by an explicit named
    // override of the same parameter) still resolves to the LAST value,
    // exactly like before, with only one entry ever stored per name.
    void set(const std::string& name, Value value) {
        for (auto& [k, v] : entries_) {
            if (k == name) {
                v = std::move(value);
                return;
            }
        }
        entries_.emplace_back(name, std::move(value));
    }

    const Value* find(const std::string& name) const {
        for (const auto& [k, v] : entries_) {
            if (k == name) return &v;
        }
        return nullptr;
    }

    bool count(const std::string& name) const { return find(name) != nullptr; }
    bool empty() const { return entries_.empty(); }
    size_t size() const { return entries_.size(); }
    void reserve(size_t n) { entries_.reserve(n); }

    auto begin() { return entries_.begin(); }
    auto end() { return entries_.end(); }
    auto begin() const { return entries_.begin(); }
    auto end() const { return entries_.end(); }

private:
    std::vector<std::pair<std::string, Value>> entries_;
};

} // namespace oscadeval
