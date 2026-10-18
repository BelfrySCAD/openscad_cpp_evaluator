#pragma once

#include "openscad_cpp_parser/ast/expression.hpp"
#include "openscad_cpp_parser/position.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace oscadeval {

// One name assigned again in the same statement list.
struct Reassignment {
    std::string name;
    const oscad::Position* first;   // where the variable is evaluated
    const oscad::Position* here;    // the assignment whose value it takes
};

// OpenSCAD's handle_assignment (parser.y): a name assigned twice in one
// statement list is ONE assignment, made where it was FIRST assigned but
// with the LAST expression. `x = 1; y = x * 10; x = 2;` gives y = 20.
// Running them in order gave 10 -- and with include<> spliced into the top
// level, that broke the usual way of configuring a library (include it,
// then override its defaults) for every value the library derives from one
// (BelfrySCAD #809).
//
// Returns the merged list -- the last assignment's node in the first one's
// place, later slots dropped -- or nullopt when nothing repeats. ASTs are
// shared and never modified; only this list of pointers changes. Each merge
// is appended to `found` for the caller to warn about.
inline std::optional<std::vector<const oscad::ASTNode*>> mergeReassignments(
    const std::vector<const oscad::ASTNode*>& stmts, std::vector<Reassignment>* found = nullptr) {
    if (stmts.size() < 2) return std::nullopt;
    std::vector<const oscad::ASTNode*> out;
    out.reserve(stmts.size());
    std::unordered_map<std::string, std::pair<size_t, const oscad::Position*>> seen;
    bool merged = false;
    for (const oscad::ASTNode* node : stmts) {
        if (node->kind() != oscad::NodeKind::Assignment) {
            out.push_back(node);
            continue;
        }
        const auto& a = static_cast<const oscad::Assignment&>(*node);
        const std::string& name = a.name->name;
        auto it = seen.find(name);
        if (it == seen.end()) {
            seen.emplace(name, std::make_pair(out.size(), &a.position()));
            out.push_back(node);
            continue;
        }
        if (found) found->push_back({name, it->second.second, &a.position()});
        out[it->second.first] = node;
        merged = true;
    }
    if (!merged) return std::nullopt;
    return out;
}

} // namespace oscadeval
