#pragma once

// Coverage: which parts of a script (and of the libraries it pulls in) ran.
//
// The unit of coverage is the checkpoint set the debugger already uses --
// Evaluator::checkDebug fires for every statement, every function/module
// body entered, every if/else arm's first statement, every ternary arm and
// list-comprehension if/else arm -- plus two arms the debugger never needed
// to stop at, the right operands of `&&`/`||`. Recording a hit is an array
// increment indexed by the node's (treeId, slot), the dense per-parse
// numbering ScopeTable relies on, so the bytecode VM stays on and a hit
// costs no hashing and no string.
//
// What COULD have run -- the universe the report counts against -- comes
// from a walk over the same trees (collectCoverable) using one predicate
// for both sides, so "covered" and "uncovered" are one vocabulary. A hit
// on a node outside that vocabulary (a let-binding, a plain comprehension
// element) is simply not reported.

#include "openscad_cpp_parser/ast/ast_node.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace oscadeval {

enum class CoverageKind {
    Statement, // a statement in a module body, an if/for/let body, or at top level
    Branch,    // a ternary arm, a comprehension if/else arm, a `&&`/`||` right operand
    Body,      // a function, module or function-literal body: was it ever entered?
};

const char* coverageKindName(CoverageKind kind);

struct CoverageSpan {
    std::string origin;
    int line = 0;
    int column = 0;
    int start_offset = 0;
    int end_offset = 0;
    CoverageKind kind = CoverageKind::Statement;
    // A Statement that is the first of an if/else arm. Branch coverage counts
    // these alongside the Branch kind; they stay Statements so a statement
    // count is still a statement count.
    bool arm = false;
    std::uint32_t hits = 0;
};

// Per-file totals. "branches" counts Branch spans plus arm Statements (an
// if/else arm is a branch too); those arm statements are ALSO in
// "statements", since a statement count should be a statement count.
// "percent" is over every span in the file, whatever its kind.
struct CoverageFileSummary {
    std::string origin;
    std::uint32_t statements = 0, statements_hit = 0;
    std::uint32_t branches = 0, branches_hit = 0;
    std::uint32_t bodies = 0, bodies_hit = 0;
    std::uint32_t spans = 0, spans_hit = 0;
    double percent() const { return spans ? 100.0 * spans_hit / spans : 100.0; }
    double statement_percent() const { return statements ? 100.0 * statements_hit / statements : 100.0; }
    double branch_percent() const { return branches ? 100.0 * branches_hit / branches : 100.0; }
    double body_percent() const { return bodies ? 100.0 * bodies_hit / bodies : 100.0; }
};

struct CoverageResult {
    std::vector<CoverageSpan> spans;         // one per coverable node, source order per tree
    std::vector<CoverageFileSummary> files;  // one per origin, sorted by origin
    CoverageFileSummary total;               // every file together (origin empty)
};

// The per-file and total summaries for `spans`; buildCoverageResult calls
// this, and a merged/aggregated span list (BelfrySCAD's --test --coverage)
// can call it again.
void summarizeCoverage(CoverageResult& result);

class CoverageRecorder {
public:
    void clear() { hits_.clear(); }
    void hit(const oscad::ASTNode& node) {
        const std::uint32_t tree = node.treeId(), slot = node.slot();
        if (tree >= hits_.size()) hits_.resize(tree + 1);
        std::vector<std::uint32_t>& slots = hits_[tree];
        if (slot >= slots.size()) slots.resize(slot + 1, 0);
        ++slots[slot];
    }
    std::uint32_t hitsFor(const oscad::ASTNode& node) const {
        const std::uint32_t tree = node.treeId(), slot = node.slot();
        if (tree >= hits_.size()) return 0;
        const std::vector<std::uint32_t>& slots = hits_[tree];
        return slot < slots.size() ? slots[slot] : 0;
    }

private:
    std::vector<std::vector<std::uint32_t>> hits_;
};

struct Coverable {
    const oscad::ASTNode* node = nullptr;
    CoverageKind kind = CoverageKind::Statement;
    bool arm = false;
};

// Every coverable node under `roots` (a top-level statement list: the main
// file with its includes spliced in and any use-injected declarations) plus
// `extraStatements` (a used file's own top-level assignments, which `use`
// never splices anywhere -- see Evaluator::setUsedFileGlobals). Source
// order within each tree; a node reachable twice is listed once.
std::vector<Coverable> collectCoverable(const std::vector<const oscad::ASTNode*>& roots,
                                        const std::vector<const oscad::ASTNode*>& extraStatements);

CoverageResult buildCoverageResult(const std::vector<const oscad::ASTNode*>& roots,
                                   const std::vector<const oscad::ASTNode*>& extraStatements,
                                   const CoverageRecorder& recorder);

} // namespace oscadeval
