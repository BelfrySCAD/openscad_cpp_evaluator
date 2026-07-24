#include "openscad_cpp_evaluator/surface_load.hpp"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO_ASSERT // not needed; keeps stb's own asserts out of a release build
#include <stb_image.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace oscadeval {

namespace {

std::string lowerExt(const std::string& path) {
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

std::vector<std::vector<double>> loadDat(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open '" + path + "'");
    std::vector<std::vector<double>> heights;
    std::string line;
    while (std::getline(in, line)) {
        const size_t b = line.find_first_not_of(" \t\r\n");
        if (b == std::string::npos || line[b] == '#') continue;
        std::vector<double> row;
        std::istringstream ss(line);
        double v;
        while (ss >> v) row.push_back(v);
        if (!row.empty()) heights.push_back(std::move(row));
    }
    std::reverse(heights.begin(), heights.end()); // first file line = highest Y
    return heights;
}

// stb_image's row order matches PIL's (row 0 = top scanline), same as the
// reference's own `for row in range(h-1, -1, -1)` walk -- bottom image row
// becomes heights[0] (Y=0).
std::vector<std::vector<double>> loadImage(const std::string& path, bool invert) {
    int w = 0, h = 0, channels = 0;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &channels, 3);
    if (!data) throw std::runtime_error(std::string("could not load image: ") + stbi_failure_reason());

    std::vector<std::vector<double>> heights;
    heights.reserve(static_cast<size_t>(h));
    for (int row = h - 1; row >= 0; --row) {
        std::vector<double> rowVals;
        rowVals.reserve(static_cast<size_t>(w));
        for (int col = 0; col < w; ++col) {
            const unsigned char* px = data + (static_cast<size_t>(row) * static_cast<size_t>(w) + static_cast<size_t>(col)) * 3;
            const double gray = 0.2126 * px[0] + 0.7152 * px[1] + 0.0722 * px[2]; // linear luminance
            rowVals.push_back(invert ? (255.0 - gray) / 255.0 * 100.0 : gray / 255.0 * 100.0);
        }
        heights.push_back(std::move(rowVals));
    }
    stbi_image_free(data);
    return heights;
}

} // namespace

std::vector<std::vector<double>> loadSurfaceHeights(const std::string& path, bool invert) {
    const std::string ext = lowerExt(path);
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".gif") {
        return loadImage(path, invert);
    }
    return loadDat(path);
}

} // namespace oscadeval
