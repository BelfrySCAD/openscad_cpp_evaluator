#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <cmath>
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

// CLEAN-ROOM: reimplement from spec section B7.
std::optional<int> Discretizer::circular(double, double) const {
    return std::nullopt;
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
