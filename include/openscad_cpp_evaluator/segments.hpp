#pragma once

#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/csg_node.hpp"

#include <manifold/common.h>

#include <functional>
#include <optional>
#include <string>

namespace oscadeval {

// How curves become polygons: OpenSCAD's CurveDiscretizer (src/core/
// CurveDiscretizer.cc, upstream master cc19000e0), ported. Every count here
// matches the 2026.02.01 binary's.
//
// $fn > $fe > $fa/$fs. $fe is the most a curve's polygon may stray from the
// true circle, measured along a radius to an edge's midpoint; when set (>=
// 2^-20) it replaces $fa/$fs entirely. Upstream reads $fe only behind
// --enable=discretization-by-error; this evaluator always honours it, and
// advertises that as supported_feature("discretization-by-error").
//
// Departures from upstream, kept from before this port: $fa/$fs <= 0 fall
// back to their defaults (12/2) rather than clamping to 0.01 with a warning.
struct Discretizer {
    double fn = 0.0;
    double fe = 0.0;
    double fa = 12.0;
    double fs = 2.0;

    // `warn`, when given, receives upstream's clamping warnings.
    static Discretizer fromCtx(const EvalContext& ctx, const std::function<void(const std::string&)>& warn = {});
    // Stored in, and read back from, a node's params -- they are part of its
    // cache key, and generate has only the params to go on.
    static Discretizer fromParams(const CSGParams& params);
    void store(CSGParams& params) const;

    // Segments for `angle` degrees of arc at radius r. nullopt where
    // upstream has none (r below 2^-20, a non-finite $fn/$fe/angle): every
    // caller supplies its own fallback, as upstream's value_or() does.
    std::optional<int> circular(double r, double angleDegrees = 360.0) const;

    // linear_extrude slice counts: twist without scale, twist with uniform
    // scale, and non-uniform scale (with or without twist).
    std::optional<int> helixSlices(double rSqr, double height, double twistDegrees) const;
    std::optional<int> conicalHelixSlices(double rSqr, double height, double twistDegrees, double scale) const;
    std::optional<int> diagonalSlices(double deltaSqr, double height) const;

    // Subdivide an outline's edges so a twisted or non-uniformly scaled
    // extrusion keeps its shape between slices. `segments` is linear_extrude's
    // own argument (0 = not given).
    manifold::SimplePolygon splitOutline(const manifold::SimplePolygon& outline, double twistDegrees,
                                         double scaleX, double scaleY, unsigned slices, unsigned segments) const;
};

// Full-circle segment count from ctx's $fn/$fe/$fa/$fs at radius r;
// upstream's primitives fall back to 3.
int fnSegmentsFromCtx(const EvalContext& ctx, double r = 0.0,
                      const std::function<void(const std::string&)>& warn = {});

} // namespace oscadeval
