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

namespace {

// Radii (and offsets) below this are treated as a point: no slices.
constexpr double kTinyLength = 1.0 / (1 << 20);

double degToRad(double degrees) { return degrees * std::numbers::pi / 180.0; }

// At least one slice per 120 degrees of twist, so no slice turns far
// enough to fold over itself.
double minTwistSlices(double twistDegrees) { return std::max(std::ceil(twistDegrees / 120.0), 1.0); }

int toCount(double n) { return static_cast<int>(std::min(n, static_cast<double>(std::numeric_limits<int>::max()))); }

} // namespace

// Slices for a twist with no scale: the outermost vertex traces a helix,
// sliced like an arc of that length.
std::optional<int> Discretizer::helixSlices(double rSqr, double height, double twistDegrees) const {
    const double r = std::sqrt(rSqr);
    const double twist = std::fabs(twistDegrees);
    if (r < kTinyLength || !std::isfinite(fn) || std::isnan(height) || std::isnan(twist)) return std::nullopt;
    const double least = minTwistSlices(twist);
    if (fn > 0) return toCount(std::max(std::ceil(twist / 360.0 * fn), least));
    if (fe > 0) {
        if (fe >= r) return toCount(least);
        const double stepRadians = 2.0 * (std::numbers::pi - std::acos(fe / r - 1.0));
        return toCount(std::max(std::ceil(degToRad(twist) / stepRadians), least));
    }
    const double turn = degToRad(twist);
    const double helixLength = turn * std::sqrt(rSqr + (height / turn) * (height / turn));
    return toCount(std::max(std::min(std::ceil(twist / fa), std::ceil(helixLength / fs)), least));
}

// Slices for a twist with a uniform scale: the outermost vertex traces a
// conical spiral, whose length (from the Archimedean spiral arc length)
// stands in for the helix's. $fe is not used here.
std::optional<int> Discretizer::conicalHelixSlices(double rSqr, double height, double twistDegrees,
                                                   double scale) const {
    const double r = std::sqrt(rSqr);
    const double twist = std::fabs(twistDegrees);
    if (r < kTinyLength || !std::isfinite(fn)) return std::nullopt;
    const double least = minTwistSlices(twist);
    if (fn > 0) return toCount(std::max(std::ceil(twist * fn / 360.0), least));
    const double turn = degToRad(twist);
    const double end = scale > 1 ? turn * scale / (scale - 1) : turn / (1 - scale);
    const double begin = end - turn;
    const double a = r / end;
    const auto arcLength = [a](double t) { return 0.5 * a * (t * std::sqrt(1 + t * t) + std::asinh(t)); };
    const double flat = arcLength(end) - arcLength(begin);
    const double length = std::sqrt(flat * flat + height * height);
    return toCount(std::max(std::min(std::ceil(twist / fa), std::ceil(length / fs)), least));
}

// Slices for a non-uniform scale: enough that the straight line from a
// vertex to its scaled copy is split into $fs-sized pieces.
std::optional<int> Discretizer::diagonalSlices(double deltaSqr, double height) const {
    if (std::sqrt(deltaSqr) < kTinyLength || !std::isfinite(fn)) return std::nullopt;
    if (fn > 0) return toCount(std::max(std::trunc(fn), 1.0));
    return toCount(std::max(std::ceil(std::sqrt(deltaSqr + height * height) / fs), 1.0));
}

namespace {

// Pieces per edge: start every edge at one, then repeatedly give one more
// piece to every edge tied for the longest piece, while the total stays
// within `target`.
std::vector<unsigned> piecesForTarget(const std::vector<double>& edgeLengths, unsigned target) {
    std::vector<unsigned> pieces(edgeLengths.size(), 1);
    if (edgeLengths.size() >= target) return pieces;
    size_t total = pieces.size();
    while (true) {
        const auto piece = [&](size_t i) { return edgeLengths[i] / (pieces[i] + 0.5); };
        double longest = 0.0;
        for (size_t i = 0; i < pieces.size(); ++i) longest = std::max(longest, piece(i));
        std::vector<size_t> group;
        for (size_t i = 0; i < pieces.size(); ++i) {
            if (piece(i) >= 0.999 * longest) group.push_back(i);
        }
        if (total + group.size() > target) return pieces;
        for (size_t i : group) ++pieces[i];
        total += group.size();
    }
}

} // namespace

// Split an outline's edges so its twisted / scaled copies stay close to the
// true surface between slices. Each edge's length is its longest over every
// slice's copy of it.
manifold::SimplePolygon Discretizer::splitOutline(const manifold::SimplePolygon& outline, double twistDegrees,
                                                  double scaleX, double scaleY, unsigned slices,
                                                  unsigned segments) const {
    const size_t n = outline.size();
    if (n == 0) return outline;

    std::vector<double> lengths(n);
    for (size_t i = 0; i < n; ++i) {
        const manifold::vec2 edge = outline[(i + 1) % n] - outline[i];
        if (scaleX == scaleY) {
            lengths[i] = manifold::la::length(edge) * std::max(scaleX, 1.0);
            continue;
        }
        double longest = 0.0;
        for (unsigned j = 0; j <= slices; ++j) {
            const double t = slices ? static_cast<double>(j) / slices : 0.0;
            const double angle = degToRad(-twistDegrees * t);
            const double c = std::cos(angle), s = std::sin(angle);
            const manifold::vec2 turned(c * edge.x - s * edge.y, s * edge.x + c * edge.y);
            const manifold::vec2 scaled(turned.x * (1 + (scaleX - 1) * t), turned.y * (1 + (scaleY - 1) * t));
            longest = std::max(longest, manifold::la::length(scaled));
        }
        lengths[i] = longest;
    }

    std::vector<unsigned> pieces;
    if (segments > 0 || fn > 0) {
        const unsigned target = segments > 0 ? segments : static_cast<unsigned>(std::max(std::trunc(fn), 3.0));
        pieces = piecesForTarget(lengths, target);
    } else {
        const double full = std::ceil(360.0 / fa);
        if (n >= full) return outline;
        pieces.resize(n);
        double total = 0.0;
        for (size_t i = 0; i < n; ++i) {
            pieces[i] = static_cast<unsigned>(std::ceil(lengths[i] / fs));
            total += pieces[i];
        }
        if (total >= full) pieces = piecesForTarget(lengths, static_cast<unsigned>(full));
    }

    manifold::SimplePolygon result;
    for (size_t i = 0; i < n; ++i) {
        const manifold::vec2 a = outline[i], b = outline[(i + 1) % n];
        for (unsigned k = 0; k < pieces[i]; ++k) result.push_back(a + (b - a) * (static_cast<double>(k) / pieces[i]));
    }
    return result;
}

int fnSegmentsFromCtx(const EvalContext& ctx, double r, const std::function<void(const std::string&)>& warn) {
    return Discretizer::fromCtx(ctx, warn).circular(r).value_or(3);
}

} // namespace oscadeval
