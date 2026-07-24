#include "openscad_cpp_evaluator/dxf_svg_import.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace oscadeval {

namespace {

std::string trim(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

struct DxfPair {
    int code;
    std::string value;
};

std::vector<DxfPair> readDxfPairs(const std::string& text) {
    std::vector<DxfPair> pairs;
    std::istringstream ss(text);
    std::string codeLine, valueLine;
    while (std::getline(ss, codeLine) && std::getline(ss, valueLine)) {
        const std::string code = trim(codeLine);
        if (code.empty()) continue;
        pairs.push_back({std::stoi(code), trim(valueLine)});
    }
    return pairs;
}

// Skips to just past the next group-0 marker matching none of the given
// entity names, collecting (code -> value) attribute pairs along the way
// (last-write-wins for repeated codes, fine for the handful this parser
// reads). Leaves `i` pointing at the next group-0 pair.
struct Vertex10_20 {
    bool haveX = false;
    double x = 0, y = 0;
    std::vector<std::array<double, 2>> points;
    void feed(int code, double v) {
        if (code == 10) {
            x = v;
            haveX = true;
        } else if (code == 20 && haveX) {
            points.push_back({x, v});
            haveX = false;
        }
    }
};

} // namespace

std::vector<Contour2d> loadDxfContours(const std::string& path, const std::optional<std::string>& layer) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open '" + path + "'");
    std::stringstream buf;
    buf << in.rdbuf();
    const std::vector<DxfPair> pairs = readDxfPairs(buf.str());

    std::vector<Contour2d> contours;
    size_t i = 0;
    while (i < pairs.size()) {
        if (pairs[i].code != 0) {
            ++i;
            continue;
        }
        if (pairs[i].value == "LWPOLYLINE") {
            ++i;
            std::string entLayer;
            bool closed = false;
            Vertex10_20 verts;
            while (i < pairs.size() && pairs[i].code != 0) {
                const DxfPair& p = pairs[i];
                if (p.code == 8) entLayer = p.value;
                else if (p.code == 70) closed = (std::stoi(p.value) & 1) != 0;
                else if (p.code == 10 || p.code == 20) verts.feed(p.code, std::stod(p.value));
                ++i;
            }
            if (!verts.points.empty() && closed && (!layer || *layer == entLayer)) contours.push_back(verts.points);
        } else if (pairs[i].value == "POLYLINE") {
            ++i;
            std::string entLayer;
            bool closed = false;
            while (i < pairs.size() && pairs[i].code != 0) {
                const DxfPair& p = pairs[i];
                if (p.code == 8) entLayer = p.value;
                else if (p.code == 70) closed = (std::stoi(p.value) & 1) != 0;
                ++i;
            }
            Contour2d points;
            while (i < pairs.size() && !(pairs[i].code == 0 && pairs[i].value == "SEQEND")) {
                if (pairs[i].code == 0 && pairs[i].value == "VERTEX") {
                    ++i;
                    Vertex10_20 v;
                    while (i < pairs.size() && pairs[i].code != 0) {
                        if (pairs[i].code == 10 || pairs[i].code == 20) v.feed(pairs[i].code, std::stod(pairs[i].value));
                        ++i;
                    }
                    if (!v.points.empty()) points.push_back(v.points.front());
                } else {
                    ++i;
                }
            }
            if (!points.empty() && closed && (!layer || *layer == entLayer)) contours.push_back(points);
        } else {
            ++i;
        }
    }
    return contours;
}

} // namespace oscadeval
