#include "openscad_cpp_evaluator/surface_load.hpp"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO_ASSERT // not needed; keeps stb's own asserts out of a release build
#include <stb_image.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>

namespace oscadeval {

namespace {

bool hasPngSignature(const std::string& bytes) {
    static const char kSignature[8] = {'\x89', 'P', 'N', 'G', '\r', '\n', '\x1a', '\n'};
    return bytes.size() >= 8 && std::memcmp(bytes.data(), kSignature, 8) == 0;
}

// Image formats besides PNG, recognised by extension only.
bool hasImageExtension(const std::string& path) {
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".gif";
}

// An image's heights: 0..100 by luminance on 16-bit channels, the bottom
// image row at y = 0; negated by `invert`.
void readImage(const std::string& path, const std::string& bytes, bool invert, SurfaceData& out) {
    int w = 0, h = 0, channels = 0;
    stbi_us* px = stbi_load_16_from_memory(reinterpret_cast<const stbi_uc*>(bytes.data()), static_cast<int>(bytes.size()),
                                           &w, &h, &channels, 3);
    if (!px) {
        out.warnings.push_back("Can't read PNG image '" + path + "'");
        return;
    }
    out.rows = h;
    out.cols = w;
    out.minVal = 200;
    out.heights.resize(static_cast<size_t>(w) * static_cast<size_t>(h));
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c) {
            const stbi_us* p = px + 3 * (static_cast<size_t>(r) * w + c);
            const double lum = 0.2126 * p[0] + 0.7152 * p[1] + 0.0722 * p[2];
            double z = 100.0 / 65535.0 * lum;
            if (invert) z = -z;
            out.heights[static_cast<size_t>(h - 1 - r) * w + c] = z;
            out.minVal = std::min(out.minVal, z);
        }
    stbi_image_free(px);
}

bool parseNumber(const std::string& token, double& value) {
    char* end = nullptr;
    value = std::strtod(token.c_str(), &end);
    return !token.empty() && end == token.c_str() + token.size();
}

// A text grid: whitespace-separated numbers, the first data line at y = 0.
// Blank lines and `#` lines are skipped; short rows are padded with 0.
void readGrid(const std::string& path, const std::string& text, SurfaceData& out) {
    std::vector<std::vector<double>> grid;
    double minVal = 1;
    size_t start = 0;
    while (start < text.size()) {
        size_t end = text.find('\n', start);
        const bool lastLineUnterminated = end == std::string::npos;
        if (lastLineUnterminated) end = text.size();
        std::istringstream line(text.substr(start, end - start));
        start = end + 1;

        std::vector<double> row;
        std::string token;
        while (line >> token) {
            if (row.empty() && token[0] == '#') break;
            double v;
            if (!parseNumber(token, v)) {
                if (!lastLineUnterminated)
                    out.warnings.push_back("Illegal value in '" + path +
                                           "': bad lexical cast: source type value could not be interpreted as target");
                return;
            }
            row.push_back(v);
            minVal = std::min(minVal, v);
        }
        if (!row.empty()) grid.push_back(std::move(row));
    }

    size_t cols = 0;
    for (const auto& row : grid) cols = std::max(cols, row.size());
    out.rows = static_cast<int>(grid.size());
    out.cols = static_cast<int>(cols);
    out.minVal = minVal;
    out.heights.assign(grid.size() * cols, 0.0);
    for (size_t r = 0; r < grid.size(); ++r)
        std::copy(grid[r].begin(), grid[r].end(), out.heights.begin() + static_cast<std::ptrdiff_t>(r * cols));
}

} // namespace

// Reads a surface() height file: a PNG (or JPEG/BMP/GIF) image, or a text
// grid of numbers.
SurfaceData loadSurface(const std::string& path, bool invert) {
    SurfaceData out;
    std::ifstream in(path, std::ios::binary);
    if (path.empty() || !std::filesystem::is_regular_file(path) || !in) {
        out.warnings.push_back("The file '" + path + "' couldn't be opened.");
        return out;
    }
    const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (hasPngSignature(bytes) || hasImageExtension(path))
        readImage(path, bytes, invert, out);
    else
        readGrid(path, bytes, out);
    return out;
}

} // namespace oscadeval
