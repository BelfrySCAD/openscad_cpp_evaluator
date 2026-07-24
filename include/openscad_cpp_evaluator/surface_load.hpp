#pragma once

#include <string>
#include <vector>

namespace oscadeval {

// Loads height data for surface(): a `.dat` text file (whitespace-separated
// numbers per line, `#`-prefixed lines skipped, file order reversed so the
// first line represents the highest Y row -- OpenSCAD's own convention) or
// an image (PNG/JPEG/BMP/GIF via stb_image, grayscale luminance mapped to
// 0-100, optionally inverted). Throws std::runtime_error on failure.
// Mirrors _surface_load/_surface_load_dat/_surface_load_image.
std::vector<std::vector<double>> loadSurfaceHeights(const std::string& path, bool invert);

} // namespace oscadeval
