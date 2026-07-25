#include "openscad_cpp_evaluator/segments.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace oscadeval {

int fnSegments(double fn, double fa, double fs, double r) {
    if (fn > 0) return std::max(3, static_cast<int>(fn));
    if (fa <= 0) fa = 12.0;
    if (fs <= 0) fs = 2.0;
    r = std::isfinite(r) ? std::abs(r) : 0.0;
    const double val = std::max(5.0, std::min(360.0 / fa, r * 2.0 * std::numbers::pi / fs));
    return static_cast<int>(std::ceil(val));
}

namespace {
double dynNumberOr(const EvalContext& ctx, const std::string& name, double fallback) {
    const Value* v = ctx.dyn->find(name);
    if (!v) return fallback;
    const double* d = std::get_if<double>(v);
    return d ? *d : fallback;
}
} // namespace

int fnSegmentsFromCtx(const EvalContext& ctx, double r) {
    return fnSegments(dynNumberOr(ctx, "$fn", 0.0), dynNumberOr(ctx, "$fa", 12.0), dynNumberOr(ctx, "$fs", 2.0), r);
}

} // namespace oscadeval
