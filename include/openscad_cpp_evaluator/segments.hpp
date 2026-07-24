#pragma once

#include "openscad_cpp_evaluator/eval_context.hpp"

namespace oscadeval {

// Circular-shape segment count from $fn/$fa/$fs (and, for $fn<=0, radius
// r). Mirrors Evaluator._fn_segments exactly: $fn>0 wins outright (clamped
// to >=3 segments); otherwise derives from $fa (max angle per segment, in
// degrees) and $fs (max segment length), whichever gives fewer segments,
// clamped to >=5.
int fnSegments(double fn, double fa, double fs, double r = 0.0);

// Reads $fn/$fa/$fs from ctx.dyn (falling back to OpenSCAD's own defaults
// -- 0/12.0/2.0 -- if absent or non-numeric, matching the reference's
// isinstance guards) and calls fnSegments(). Mirrors Evaluator._fn().
int fnSegmentsFromCtx(const EvalContext& ctx, double r = 0.0);

} // namespace oscadeval
