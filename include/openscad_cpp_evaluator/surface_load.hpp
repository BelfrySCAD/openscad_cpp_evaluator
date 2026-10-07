#pragma once

#include <string>
#include <vector>

namespace oscadeval {

// Height data for surface() (spec section D3). Never throws: a file that
// cannot be read gives no heights and a warning, and the script carries on.
// `heights[row * cols + col]`; `minVal` is what the base sits 1 below.
struct SurfaceData {
    std::vector<double> heights;
    int rows = 0;
    int cols = 0;
    double minVal = 0;
    std::vector<std::string> warnings;
};
SurfaceData loadSurface(const std::string& path, bool invert);

} // namespace oscadeval
