#include "openscad_cpp_evaluator/surface_load.hpp"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO_ASSERT // not needed; keeps stb's own asserts out of a release build
#include <stb_image.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

namespace oscadeval {

// Reads a surface() height file (a PNG image or a .dat text grid).
// CLEAN-ROOM: reimplement from spec section D3.
SurfaceData loadSurface(const std::string&, bool) {
    return SurfaceData{};
}

} // namespace oscadeval
