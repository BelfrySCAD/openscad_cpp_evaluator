#pragma once

#include <string>
#include <vector>

namespace oscadeval {

// Height data for surface(), read as upstream's SurfaceNode reads it: a
// PNG (by its signature) as 16-bit luminance scaled to 0-100, negated by
// `invert`, bottom image row at y = 0; anything else as a .dat grid, first
// line at y = 0, short rows padded with 0. Never throws: a file that cannot
// be read gives no heights and upstream's warning, and the script carries
// on. `heights[row * cols + col]`; `minVal` is what the base sits 1 below.
struct SurfaceData {
    std::vector<double> heights;
    int rows = 0;
    int cols = 0;
    double minVal = 0;
    std::vector<std::string> warnings;
};
SurfaceData loadSurface(const std::string& path, bool invert);

} // namespace oscadeval
