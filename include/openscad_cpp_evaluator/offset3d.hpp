#pragma once

// offset() on a 3D solid: r= (rounded, an inscribed faceted ball), delta=
// (mitered, no limit, as 2D) and chamfer= (corners cut flat), outward for
// d > 0 and inward for d < 0. Pure geometry: no evaluator state. See
// offset3d.cpp for the construction and CLAUDE.md "offset() in 3D".

#include <manifold/manifold.h>

namespace oscadeval {

enum class Offset3DMode { Round, Delta, Chamfer };

manifold::Manifold offset3d(const manifold::Manifold& m, double d, Offset3DMode mode, int segs);

} // namespace oscadeval
