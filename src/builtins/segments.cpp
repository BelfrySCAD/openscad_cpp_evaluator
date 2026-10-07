#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <queue>
#include <vector>

namespace oscadeval {

namespace {

double paramOr(const CSGParams& params, const char* key, double fallback) {
    auto it = params.find(key);
    if (it == params.end()) return fallback;
    const double* d = std::get_if<double>(&it->second);
    return d ? *d : fallback;
}

// The smallest $fa and $fs honoured.
constexpr double kMinFragmentSize = 0.01;
// The smallest radius that gets a segment count, and the smallest $fe used: 2^-20.
constexpr double kMinRadius = 1.0 / 1048576.0;
// At or above this $fe/r a pentagon is already within tolerance.
constexpr double kFeFiveSegmentRatio = 0.1909830056;

} // namespace

// Reads $fn/$fa/$fs/$fe from the context and clamps them, reporting each
// clamp through `warn` when one is given. An unset variable takes its
// default; one set to anything but a number counts as 0.
Discretizer Discretizer::fromCtx(const EvalContext& ctx, const std::function<void(const std::string&)>& warn) {
    auto read = [&](const char* name, double fallback) {
        const Value* v = ctx.dyn.find(name);
        if (!v) return fallback;
        const double* d = std::get_if<double>(v);
        return d ? *d : 0.0;
    };
    auto report = [&](const char* text) {
        if (warn) warn(text);
    };

    Discretizer d;
    d.fn = read("$fn", 0.0);
    d.fe = read("$fe", 0.0);
    d.fa = read("$fa", 12.0);
    d.fs = read("$fs", 2.0);

    if (d.fn < 0.0) {
        report("$fn negative - setting to 0");
        d.fn = 0.0;
    }
    if (d.fe < 0.0) {
        report("$fe negative - setting to 0");
        d.fe = 0.0;
    }
    if (d.fs < kMinFragmentSize) {
        report("$fs too small - clamping to 0.010000");
        d.fs = kMinFragmentSize;
    }
    if (d.fa < kMinFragmentSize) {
        report("$fa too small - clamping to 0.010000");
        d.fa = kMinFragmentSize;
    }
    return d;
}

Discretizer Discretizer::fromParams(const CSGParams& params) {
    Discretizer d;
    d.fn = paramOr(params, "fn", 0.0);
    d.fe = paramOr(params, "fe", 0.0);
    d.fa = paramOr(params, "fa", 12.0);
    d.fs = paramOr(params, "fs", 2.0);
    return d;
}

void Discretizer::store(CSGParams& params) const {
    params["fn"] = Value{fn};
    params["fe"] = Value{fe};
    params["fa"] = Value{fa};
    params["fs"] = Value{fs};
}

std::optional<int> Discretizer::circular(double r, double angleDegrees) const {
    if (r < kMinRadius || !std::isfinite(fn) || !std::isfinite(angleDegrees)) return std::nullopt;
    const double share = std::abs(angleDegrees) / 360.0;

    double full;
    if (fn > 0.0) {
        full = std::ceil(std::max(fn, 3.0));
    } else if (!std::isfinite(fe)) {
        return std::nullopt;
    } else if (fe >= kMinRadius) {
        // Enough segments that no chord strays more than $fe from the arc.
        // Deliberately not rounded before scaling to the arc's share.
        const double ratio = fe / r;
        if (ratio >= kFeFiveSegmentRatio) {
            full = 5.0;
        } else {
            const double cap = std::max(std::min(360.0 / kMinFragmentSize, 2.0 * std::numbers::pi * r / kMinFragmentSize), 5.0);
            full = std::min(cap, std::numbers::pi / std::acos(1.0 - ratio));
        }
    } else {
        full = std::ceil(std::max(std::min(360.0 / fa, 2.0 * std::numbers::pi * r / fs), 5.0));
    }
    const double n = std::max(1.0, std::ceil(full * share));
    return static_cast<int>(std::min(n, static_cast<double>(std::numeric_limits<int>::max())));
}

// CLEAN-ROOM: reimplement from spec section C6.
std::optional<int> Discretizer::helixSlices(double, double, double) const {
    return std::nullopt;
}

// CLEAN-ROOM: reimplement from spec section C6.
std::optional<int> Discretizer::conicalHelixSlices(double, double, double, double) const {
    return std::nullopt;
}

// CLEAN-ROOM: reimplement from spec section C6.
std::optional<int> Discretizer::diagonalSlices(double, double) const {
    return std::nullopt;
}

// CLEAN-ROOM: reimplement from spec section C6.
manifold::SimplePolygon Discretizer::splitOutline(const manifold::SimplePolygon& o, double, double, double,
                                                  unsigned, unsigned) const {
    return o;
}

int fnSegmentsFromCtx(const EvalContext& ctx, double r, const std::function<void(const std::string&)>& warn) {
    return Discretizer::fromCtx(ctx, warn).circular(r).value_or(3);
}

} // namespace oscadeval
