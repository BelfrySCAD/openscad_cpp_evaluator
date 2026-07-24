#pragma once

#include <array>
#include <string>

namespace oscadeval {

// "#rrggbb"/"#rgb" hex, or a CSS/SVG color name (case-insensitive) -> RGBA
// with the given alpha. The name table mirrors the Python reference's
// CSS_COLORS exactly (148 entries generated from a live Qt install, not the
// "standard" 147-entry CSS3/SVG list -- includes British-spelling aliases
// like `grey`/`darkgrey`/`lightgrey` alongside their American-spelling
// equivalents at identical RGB values, and does *not* include
// `rebeccapurple`, a CSS Color Module Level 4 addition Qt never adopted).
// An unrecognized name -> white (1,1,1), matching the reference's own
// fallback.
std::array<double, 4> cssColor(const std::string& name, double alpha = 1.0);

} // namespace oscadeval
