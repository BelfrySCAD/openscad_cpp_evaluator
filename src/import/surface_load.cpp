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

namespace {

std::string lowerExt(const std::string& path) {
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

bool hasPngHeader(const std::string& path) {
    static const unsigned char kPng[8] = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};
    std::ifstream in(path, std::ios::binary);
    unsigned char head[8] = {};
    in.read(reinterpret_cast<char*>(head), 8);
    return in.gcount() == 8 && std::memcmp(head, kPng, 8) == 0;
}

// SurfaceNode::read_dat: row i of the file is y = i (the first line is
// y = 0 -- this used to reverse them, mirroring every surface in Y); a
// short row is padded with 0; an unparseable value warns and yields
// nothing. min starts at 1, which with createGeometry's "min - 1" puts the
// base at z = 0 for all-positive data, as it always was.
SurfaceData loadDat(const std::string& path) {
    SurfaceData out;
    std::ifstream in(path);
    if (!in) {
        out.warnings.push_back("Can't open DAT file '" + path + "'.");
        return out;
    }
    std::map<std::pair<int, int>, double> cells;
    int lines = 0, columns = 0;
    double minVal = 1;
    std::string line;
    while (std::getline(in, line)) {
        const size_t b = line.find_first_not_of(" \t\r\n");
        if (b == std::string::npos || line[b] == '#') continue;
        std::istringstream ss(line);
        std::string token;
        int col = 0;
        while (ss >> token) {
            size_t used = 0;
            double v = 0;
            try {
                v = std::stod(token, &used);
            } catch (...) {
                used = 0;
            }
            if (used != token.size()) {
                out.warnings.push_back("Illegal value in '" + path + "': bad lexical cast: source type value could "
                                       "not be interpreted as target");
                return SurfaceData{{}, 0, 0, 0, out.warnings};
            }
            cells[{lines, col++}] = v;
            columns = std::max(columns, col);
            minVal = std::min(minVal, v);
        }
        ++lines;
    }
    out.rows = lines;
    out.cols = columns;
    out.minVal = minVal;
    out.heights.assign(static_cast<size_t>(lines) * columns, 0.0);
    for (const auto& [rc, v] : cells) out.heights[static_cast<size_t>(rc.first) * columns + rc.second] = v;
    return out;
}

// SurfaceNode::convert_image: 16-bit luminance scaled to 0-100, inverted by
// NEGATING it (this used 100 - z, a different solid); the bottom image row
// is y = 0. stb_image widens 8-bit channels to 16 the way lodepng does.
SurfaceData loadImage(const std::string& path, bool invert) {
    SurfaceData out;
    int w = 0, h = 0, channels = 0;
    stbi_us* data = stbi_load_16(path.c_str(), &w, &h, &channels, 3);
    if (!data) {
        out.warnings.push_back("Can't read PNG image '" + path + "'");
        return out;
    }
    out.rows = h;
    out.cols = w;
    out.heights.assign(static_cast<size_t>(w) * h, 0.0);
    double minVal = 200;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const stbi_us* px = data + (static_cast<size_t>(y) * w + x) * 3;
            const double pixel = 0.2126 * px[0] + 0.7152 * px[1] + 0.0722 * px[2];
            const double z = 100.0 / 65535.0 * (invert ? 0.0 - pixel : pixel);
            out.heights[static_cast<size_t>(h - 1 - y) * w + x] = z;
            minVal = std::min(minVal, z);
        }
    }
    out.minVal = minVal;
    stbi_image_free(data);
    return out;
}

} // namespace

SurfaceData loadSurface(const std::string& path, bool invert) {
    if (!std::filesystem::exists(path)) {
        SurfaceData out;
        out.warnings.push_back("The file '" + path + "' couldn't be opened.");
        return out;
    }
    // Upstream decides by the PNG signature, not the name. Other image
    // types (a BelfrySCAD addition) still go by extension.
    const std::string ext = lowerExt(path);
    if (hasPngHeader(path) || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".gif")
        return loadImage(path, invert);
    if (ext == ".png") {
        SurfaceData out;
        out.warnings.push_back("Can't read PNG image '" + path + "'");
        return out;
    }
    return loadDat(path);
}

} // namespace oscadeval
