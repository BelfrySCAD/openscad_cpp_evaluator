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
    std::string kind; // "module" | "function"
    std::string name;
    std::string callerName; // enclosing module/function's own name, or "<toplevel>"
    std::string callOrigin;
    int callLine = 0;
    std::string declOrigin;
    int declLine = 0;
    int callCount = 0;
    double selfTime = 0.0;       // seconds, own code only, never double-counted
    double cumulativeTime = 0.0; // seconds, includes children; recursion-guarded
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
    double resolveTime = 0.0;
    double generateTime = 0.0;
    double totalTime = 0.0;
    double unattributedTime = 0.0;
};

} // namespace oscadeval
