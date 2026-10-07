#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>
#include <vector>

namespace oscadeval {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kGridFine = 0.00000095367431640625; // 2^-20, upstream's GRID_FINE
constexpr double kFMinimum = 0.01;                   // upstream's F_MINIMUM

double paramOr(const CSGParams& params, const char* key, double fallback) {
    auto it = params.find(key);
    if (it == params.end()) return fallback;
    const double* d = std::get_if<double>(&it->second);
    return d ? *d : fallback;
}

double segmentsGivenFa(double fa) { return 360.0 / fa; }
double segmentsGivenFs(double r, double fs) { return r * 2 * kPi / fs; }

// https://mathworld.wolfram.com/Helix.html -- see upstream for the derivation.
double helixArcLength(double rSqr, double height, double twistDegrees) {
    const double t = twistDegrees * kDeg2Rad;
    const double c = height / t;
    return t * std::sqrt(rSqr + c * c);
}

double archimedesLength(double a, double theta) {
    return 0.5 * a * (theta * std::sqrt(1 + theta * theta) + std::asinh(theta));
}

manifold::vec2 transformAt(const manifold::vec2& v, double twist, double scaleX, double scaleY, double t) {
    // Eigen::Scaling(lerp(1, s, t)) * rotate_degrees(-twist * t), as upstream.
    const double a = -twist * t * kDeg2Rad;
    const double x = v.x * std::cos(a) - v.y * std::sin(a);
    const double y = v.x * std::sin(a) + v.y * std::cos(a);
    return {x * (1 + (scaleX - 1) * t), y * (1 + (scaleY - 1) * t)};
}

// The longest an outline edge gets over every slice of the extrusion.
double maxEdgeLength(const manifold::vec2& v0, const manifold::vec2& v1, double twist, double scaleX,
                     double scaleY, unsigned slices) {
    if (scaleX == scaleY) return manifold::la::length(v1 - v0) * std::max(scaleX, 1.0);
    double longest = 0.0;
    for (unsigned j = 0; j <= slices; ++j) {
        const double t = static_cast<double>(j) / slices;
        longest = std::max(longest, manifold::la::length(transformAt(v1, twist, scaleX, scaleY, t) -
                                               transformAt(v0, twist, scaleX, scaleY, t)));
    }
    return longest;
}

void addSegmentedEdge(manifold::SimplePolygon& out, const manifold::vec2& v0, const manifold::vec2& v1,
                      unsigned segments) {
    for (unsigned j = 0; j < segments; ++j) {
        const double t = static_cast<double>(j) / segments;
        out.push_back((1 - t) * v0 + t * v1);
    }
}

// Upstream's splitOutlineByFn: while the outline has fewer than fn vertices,
// split the edge whose segments are longest, a whole group of near-equal
// edges at a time so the result stays symmetrical.
manifold::SimplePolygon splitByFn(const manifold::SimplePolygon& o, double twist, double scaleX, double scaleY,
                                  double fn, unsigned slices) {
    struct Tracker {
        size_t edge;
        double maxLen;
        unsigned count = 1;
        double metric() const { return maxLen / (count + 0.5); }
        bool operator<(const Tracker& rhs) const { return metric() < rhs.metric(); }
        bool closeMatch(const Tracker& other) const {
            const double l1 = metric(), l2 = other.metric();
            return std::min(l1, l2) / std::max(l1, l2) >= 0.999;
        }
    };
    const size_t n = o.size();
    std::vector<unsigned> counts(n, 1);
    std::priority_queue<Tracker> q;
    for (size_t i = 1; i <= n; ++i) {
        q.push(Tracker{i - 1, maxEdgeLength(o[i - 1], o[i % n], twist, scaleX, scaleY, slices)});
    }
    std::vector<Tracker> group;
    size_t total = n;
    while (total < fn) {
        while (!q.empty() && (group.empty() || q.top().closeMatch(group.front()))) {
            group.push_back(q.top());
            q.pop();
        }
        if (total + group.size() <= fn) {
            while (!group.empty()) {
                Tracker cur = group.back();
                group.pop_back();
                ++cur.count;
                ++counts[cur.edge];
                ++total;
                q.push(cur);
            }
        } else {
            while (!group.empty()) {
                q.push(group.back());
                group.pop_back();
            }
            break;
        }
    }
    manifold::SimplePolygon out;
    for (size_t i = 1; i <= n; ++i) addSegmentedEdge(out, o[i - 1], o[i % n], counts[i - 1]);
    return out;
}

manifold::SimplePolygon splitByFs(const manifold::SimplePolygon& o, double twist, double scaleX, double scaleY,
                                  double fs, unsigned slices) {
    const size_t n = o.size();
    manifold::SimplePolygon out;
    for (size_t i = 1; i <= n; ++i) {
        const double len = maxEdgeLength(o[i - 1], o[i % n], twist, scaleX, scaleY, slices);
        addSegmentedEdge(out, o[i - 1], o[i % n], static_cast<unsigned>(std::ceil(len / fs)));
    }
    return out;
}

} // namespace

// Reads $fn/$fa/$fs/$fe from the context and clamps them, reporting each
// clamp through `warn` when one is given.
// CLEAN-ROOM: reimplement from spec section B7.
Discretizer Discretizer::fromCtx(const EvalContext&, const std::function<void(const std::string&)>&) {
    Discretizer d;
    d.fn = 0.0;
    d.fe = 0.0;
    d.fa = 12.0;
    d.fs = 2.0;
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
    if (r < kGridFine || !std::isfinite(fn) || !std::isfinite(angleDegrees)) return std::nullopt;
    double result;
    if (fn > 0.0) {
        // Ceiled before scaling to the arc, as upstream keeps for compatibility.
        result = std::ceil(std::max(fn, 3.0)) * std::fabs(angleDegrees) / 360.0;
    } else {
        if (!std::isfinite(fe)) return std::nullopt;
        if (fe >= kGridFine) {
            // Apothem r - fe: r*cos(pi/n) = r - fe, so n = pi / acos(1 - fe/r).
            // At the ratio giving exactly 5 (which also covers fe >= r), 5.
            const double maxSegments =
                std::max(std::min(segmentsGivenFa(kFMinimum), segmentsGivenFs(r, kFMinimum)), 5.0);
            const double ratio = fe / r;
            result = ratio >= 0.1909830056 ? 5.0 : std::min(maxSegments, kPi / std::acos(1 - ratio));
            // NOT ceiled before scaling to the arc, unlike the $fa/$fs branch.
        } else {
            result = std::ceil(std::max(std::min(segmentsGivenFa(fa), segmentsGivenFs(r, fs)), 5.0));
        }
        result *= std::fabs(angleDegrees) / 360.0;
    }
    return std::max(1, static_cast<int>(std::ceil(result)));
}

std::optional<int> Discretizer::helixSlices(double rSqr, double height, double twistDegrees) const {
    twistDegrees = std::fabs(twistDegrees);
    // At least 3 slices per full turn: 180 degrees in one slice is never manifold.
    const int minSlices = std::max(static_cast<int>(std::ceil(twistDegrees / 120.0)), 1);
    if (std::sqrt(rSqr) < kGridFine || !std::isfinite(fn) || std::isnan(height) || std::isnan(twistDegrees))
        return std::nullopt;
    if (fn > 0.0) return std::max(static_cast<int>(std::ceil(twistDegrees / 360.0 * fn)), minSlices);
    if (fe > 0.0) {
        // Error of the chord between two slices' furthest vertex, from the
        // helix it approximates; see upstream's helix_slices_given_fe.
        const double r = std::sqrt(rSqr);
        if (fe >= r) return minSlices;
        const double theta = 2 * (kPi - std::acos(fe / r - 1));
        return std::max(static_cast<int>(std::ceil(twistDegrees * kDeg2Rad / theta)), minSlices);
    }
    const int faSlices = static_cast<int>(std::ceil(twistDegrees / fa));
    const int fsSlices = static_cast<int>(std::ceil(helixArcLength(rSqr, height, twistDegrees) / fs));
    return std::max(std::min(faSlices, fsSlices), minSlices);
}

std::optional<int> Discretizer::conicalHelixSlices(double rSqr, double height, double twistDegrees,
                                                   double scale) const {
    twistDegrees = std::fabs(twistDegrees);
    const double r = std::sqrt(rSqr);
    const int minSlices = std::max(static_cast<int>(std::ceil(twistDegrees / 120.0)), 1);
    if (r < kGridFine || !std::isfinite(fn)) return std::nullopt;
    if (fn > 0.0) return std::max(static_cast<int>(std::ceil(twistDegrees * fn / 360)), minSlices);
    // Upstream has no $fe case here: a scaled twist is sized by $fa/$fs.
    // The vertex follows a section of an Archimedes spiral; see upstream.
    const double rads = twistDegrees * kDeg2Rad;
    const double angleEnd = scale > 1 ? rads * scale / (scale - 1) : rads / (1 - scale);
    const double angleStart = angleEnd - rads;
    const double a = r / angleEnd;
    const double spiral = archimedesLength(a, angleEnd) - archimedesLength(a, angleStart);
    const double total = std::sqrt(spiral * spiral + height * height);
    const int fsSlices = static_cast<int>(std::ceil(total / fs));
    const int faSlices = static_cast<int>(std::ceil(twistDegrees / fa));
    return std::max(std::min(faSlices, fsSlices), minSlices);
}

std::optional<int> Discretizer::diagonalSlices(double deltaSqr, double height) const {
    if (std::sqrt(deltaSqr) < kGridFine || !std::isfinite(fn)) return std::nullopt;
    if (fn > 0.0) return std::max(static_cast<int>(fn), 1);
    return std::max(static_cast<int>(std::ceil(std::sqrt(deltaSqr + height * height) / fs)), 1);
}

manifold::SimplePolygon Discretizer::splitOutline(const manifold::SimplePolygon& o, double twist, double scaleX,
                                                  double scaleY, unsigned slices, unsigned segments) const {
    if (o.empty()) return o;
    if (segments > 0 || fn > 0.0) {
        const unsigned minVertices = segments > 0 ? segments : static_cast<unsigned>(std::max(fn, 3.0));
        return o.size() >= minVertices ? o : splitByFn(o, twist, scaleX, scaleY, minVertices, slices);
    }
    // $fs, then check whether $fa gives fewer. (No $fe case upstream.)
    const auto faSegs = static_cast<unsigned>(std::ceil(360.0 / fa));
    if (o.size() >= faSegs) return o;
    manifold::SimplePolygon byFs = splitByFs(o, twist, scaleX, scaleY, fs, slices);
    return byFs.size() >= faSegs ? splitByFn(o, twist, scaleX, scaleY, faSegs, slices) : byFs;
}

int fnSegmentsFromCtx(const EvalContext& ctx, double r, const std::function<void(const std::string&)>& warn) {
    return Discretizer::fromCtx(ctx, warn).circular(r).value_or(3);
}

} // namespace oscadeval
