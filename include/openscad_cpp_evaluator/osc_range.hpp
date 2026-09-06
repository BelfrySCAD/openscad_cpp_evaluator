#pragma once

#include <cmath>

namespace oscadeval {

// `[start:step:end]` -- a plain 3-slot struct, not an expanded list. Mirrors
// the Python reference's OscRange object exactly: echoed as
// "[start : step : end]", only expanded to a list when iterated (for/list
// comprehension/intersection_for).
struct OscRange {
    double start = 0.0;
    double step = 1.0;
    double end = 0.0;

    // Yields nothing: a zero step, a direction that disagrees with the
    // bounds, or a NaN anywhere in it.
    bool isEmpty() const {
        if (std::isnan(start) || std::isnan(step) || std::isnan(end)) return true;
        if (step == 0.0) return true;
        return (end - start) / step < -1e-10;
    }

    // Two EMPTY ranges are equal whatever their bounds -- the reference
    // compares element counts first, so `[5:1:0] == [10:1:0]` is true, and
    // so is `[0:nan:inf] == [5:1:0]`. Only if both yield something does it
    // compare start/step/end.
    //
    // This is not a NaN rule, though a NaN range is caught by it: checked
    // against the reference, `nan == nan` is false and `[nan] == [nan]` is
    // false, so nothing else treats NaN as equal to itself. Ranges are
    // special because emptiness is.
    //
    // It matters well beyond range comparison: BOSL2 defines
    // `is_nan(x) = (x != x)`, so a self-unequal range made typeof() answer
    // "nan" for one, where the reference says "invalid".
    friend bool operator==(const OscRange& a, const OscRange& b) {
        const bool ae = a.isEmpty(), be = b.isEmpty();
        if (ae || be) return ae && be;
        return a.start == b.start && a.step == b.step && a.end == b.end;
    }
};

} // namespace oscadeval
