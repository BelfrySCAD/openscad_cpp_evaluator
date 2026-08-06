#pragma once

#include <string>
#include <vector>

namespace oscadeval {

// Aggregated profiling data for one *call site* -- a specific source
// location that calls a specific user module/function, not one
// declaration. Two different calls to the same function get separate
// entries; the same call expression re-executed many times (a loop body,
// recursion) aggregates into one entry with callCount > 1, since the AST
// node (and thus its position) is identical across those invocations.
// Mirrors the reference's CallSiteProfile.
struct CallSiteProfile {
    std::string kind; // "module" | "child" (forwarded via children()) | "function"
    std::string name;
    std::string callerName; // enclosing module/function's own name, or "<toplevel>"
    std::string callOrigin;
    int callLine = 0;
    int callColumn = 0; // distinguishes two calls sharing one line
    std::string declOrigin;
    int declLine = 0;
    int callCount = 0;
    double selfTime = 0.0;       // seconds, own code only, never double-counted
    double cumulativeTime = 0.0; // seconds, includes children; recursion-guarded
};

// One node of the calling-context tree: a call site reached by ONE
// specific path from <toplevel>. Where CallSiteProfile aggregates a site
// over every path that reached it, this keeps them separate -- so
// `cuboid` called from `bracket` and `cuboid` called from `rail` are two
// nodes with their own times, and a consumer can show what a particular
// path actually cost rather than a total across all callers.
//
// Flat vector with parent/child indices rather than pointers: it survives
// the vector reallocating as nodes are appended, and crosses the Python
// binding as plain data with no ownership question.
//
// Recursion is folded rather than unrolled. Re-entering a call site
// already on the current path reuses that node instead of appending a new
// child, so `fib` calling itself 400 deep is one node with callCount 400,
// not a 400-node chain. That bounds the tree by distinct ACYCLIC paths and
// keeps the recursion-guarded cumulative-time rule (only the outermost
// entry contributes) meaningful per node.
struct ProfilePathNode {
    int parent = -1;              // index into ProfileResult::paths; -1 for the root
    std::vector<int> children;    // indices, in first-call order
    std::string kind;             // "module" | "child" | "function"; empty for the root
    std::string name;             // callee; "<toplevel>" for the root
    std::string callOrigin;
    int callLine = 0;
    int callColumn = 0;           // distinguishes two calls sharing one line
    std::string declOrigin;
    int declLine = 0;
    int callCount = 0;
    double selfTime = 0.0;        // seconds, this node's own code on this path
    double cumulativeTime = 0.0;  // seconds, this node and everything under it
};

// Whole-evaluate() profiling summary, built when Evaluator is constructed
// with profiling=true. unattributedTime covers top-level script code and
// anything else not inside a user module/function call (native builtins'
// own resolve work, mostly) -- resolveTime always equals
// sum(s.selfTime for callSites) + unattributedTime, so a UI can show
// percentages that honestly add to 100%. Mirrors the reference's
// ProfileResult.
struct ProfileResult {
    std::vector<CallSiteProfile> callSites;
    // The calling-context tree; paths[0] is always the <toplevel> root.
    // Empty only if profiling was off. callSites stays exactly as it was --
    // the flat per-site view is still the right answer to "what is
    // expensive overall", and is cheaper to scan than walking this.
    std::vector<ProfilePathNode> paths;
    double resolveTime = 0.0;
    double generateTime = 0.0;
    double totalTime = 0.0;
    double unattributedTime = 0.0;
};

} // namespace oscadeval
