#pragma once

namespace oscadeval {

// `[start:step:end]` -- a plain 3-slot struct, not an expanded list. Mirrors
// the Python reference's OscRange object exactly: echoed as
// "[start : step : end]", only expanded to a list when iterated (for/list
// comprehension/intersection_for).
struct OscRange {
    double start = 0.0;
    double step = 1.0;
    double end = 0.0;

    friend bool operator==(const OscRange& a, const OscRange& b) {
        return a.start == b.start && a.step == b.step && a.end == b.end;
    }
};

} // namespace oscadeval
