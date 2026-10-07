#pragma once

#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/csg_node.hpp"

#include <manifold/common.h>

#include <functional>
#include <optional>
#include <string>

namespace oscadeval {

// How curves become polygons: segment and slice counts from $fn/$fe/$fa/$fs
// (spec sections B7 and C6). $fe is always honoured here (OpenSCAD reads it
// only behind --enable=discretization-by-error), advertised as
// supported_feature("discretization-by-error").
struct Discretizer {
    double fn = 0.0;
    double fe = 0.0;
    double fa = 12.0;
    double fs = 2.0;

    // `warn`, when given, receives the clamping warnings (spec section B7).
    static Discretizer fromCtx(const EvalContext& ctx, const std::function<void(const std::string&)>& warn = {});
    // Stored in, and read back from, a node's params -- they are part of its
    // cache key, and generate has only the params to go on.
    static Discretizer fromParams(const CSGParams& params);
    void store(CSGParams& params) const;

    // Segments for `angle` degrees of arc at radius r. nullopt where there is
    // no count (spec section B7); every caller supplies its own fallback.
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

// Full-circle segment count from ctx's $fn/$fe/$fa/$fs at radius r, or 3
// when there is none.
int fnSegmentsFromCtx(const EvalContext& ctx, double r = 0.0,
                      const std::function<void(const std::string&)>& warn = {});

} // namespace oscadeval
