#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <cmath>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace oscadeval {

// dxf_dim() / dxf_cross() -- read a measurement straight out of a DXF file
// rather than out of the model. This file has its own DXF reading; it does
// not share src/import/dxf_import.cpp's.

namespace {

constexpr double kPi = 3.14159265358979323846;

// One entity's worth of group codes: only those the two functions read.
struct DxfEntity {
    std::string type, layer, name;
    int flags = 0;
    double angle = 0, endAngle = 0, radius = 0;
    double x[7] = {}, y[7] = {};
};

// The placement both functions take: v -> (v - origin) * scale.
struct Placement {
    double ox = 0, oy = 0, scale = 1;
};

std::string trim(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

double toNumber(const std::string& s) {
    try {
        return std::stod(s);
    } catch (...) {
        return 0;
    }
}

// Every entity in file order, or nothing when the file cannot be opened.
// Coordinates come back already placed; slots 1, 2 and 6 of a DIMENSION are
// offsets rather than positions, so they are only scaled.
std::optional<std::vector<DxfEntity>> readDxf(const std::string& path, const Placement& p) {
    std::ifstream in(path);
    if (!in) return std::nullopt;
    std::vector<DxfEntity> entities;
    std::string codeLine, value;
    while (std::getline(in, codeLine) && std::getline(in, value)) {
        value = trim(value);
        const int code = static_cast<int>(toNumber(trim(codeLine)));
        if (code == 0) {
            entities.push_back(DxfEntity{value});
            continue;
        }
        if (entities.empty()) continue;
        DxfEntity& e = entities.back();
        if (code == 8) e.layer = value;
        else if (code == 1) e.name = value;
        else if (code == 70) e.flags = static_cast<int>(toNumber(value));
        else if (code == 50) e.angle = toNumber(value);
        else if (code == 51) e.endAngle = toNumber(value);
        else if (code == 40) e.radius = toNumber(value) * p.scale;
        else if (code >= 10 && code <= 26 && code % 10 <= 6) {
            const int slot = code % 10;
            const bool isX = code < 20;
            const bool offsetOnly = e.type == "DIMENSION" && (slot == 1 || slot == 2 || slot == 6);
            const double origin = offsetOnly ? 0 : (isX ? p.ox : p.oy);
            (isX ? e.x : e.y)[slot] = (toNumber(value) - origin) * p.scale;
        }
    }
    return entities;
}

// The file as the script wrote it, for messages.
std::string fileLabel(const Value& file) {
    if (std::holds_alternative<std::monostate>(file)) return "";
    if (std::holds_alternative<std::string>(file)) return std::get<std::string>(file);
    return fmtValue(file);
}

std::string stringOr(const Value& v) {
    return std::holds_alternative<std::string>(v) ? std::get<std::string>(v) : std::string();
}

Placement readPlacement(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node, const char* fn) {
    Placement p;
    const Value origin = getArg(args, 2, "origin");
    if (!std::holds_alternative<std::monostate>(origin)) {
        const ListPtr* list = std::get_if<ListPtr>(&origin);
        if (list && (*list)->items.size() == 2 && std::holds_alternative<double>((*list)->items[0]) &&
            std::holds_alternative<double>((*list)->items[1])) {
            p.ox = std::get<double>((*list)->items[0]);
            p.oy = std::get<double>((*list)->items[1]);
        } else {
            ev.warn(std::string(fn) + "(..., origin=" + fmtValue(origin) + ") could not be converted", &node.position());
        }
    }
    const Value scale = getArg(args, 3, "scale");
    if (std::holds_alternative<double>(scale)) p.scale = std::get<double>(scale);
    return p;
}

bool onLayer(const DxfEntity& e, const std::string& layer) { return layer.empty() || e.layer == layer; }

// Direction of (dx, dy) in degrees, measured from +Y towards +X.
double bearing(double dx, double dy) { return std::atan2(dx, dy) * 180 / kPi; }

}  // namespace

// dxf_dim(file, layer, origin, scale, name).
Value builtinDxfDim(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node) {
    const Value file = getArg(args, 0, "file");
    const std::string layer = stringOr(getArg(args, 1, "layer"));
    const std::string name = stringOr(getArg(args, 4, "name"));
    const Placement placement = readPlacement(ev, args, node, "dxf_dim");
    const std::string label = fileLabel(file);

    const auto entities = readDxf(resolveFilePath(file, node), placement);
    if (!entities) {
        ev.warn("Can't open DXF file '" + label + "'!", &node.position());
        return Value{};
    }
    for (const DxfEntity& e : *entities) {
        if (e.type != "DIMENSION" || !onLayer(e, layer) || (!name.empty() && e.name != name)) continue;
        const double dx = e.x[4] - e.x[3], dy = e.y[4] - e.y[3];
        switch (e.flags & 7) {
            case 0: {
                const double a = e.angle * kPi / 180;
                return std::fabs(dx * std::cos(a) + dy * std::sin(a));
            }
            case 1: return std::hypot(dx, dy);
            case 2:
                return std::fabs(bearing(e.x[0] - e.x[5], e.y[0] - e.y[5]) - bearing(dx, dy));
            case 3:
            case 4: return std::hypot(e.x[5] - e.x[0], e.y[5] - e.y[0]);
            case 6: return (e.flags & 64) ? e.x[3] : e.y[3];
            default:
                ev.warn("Dimension '" + name + "' in '" + label + "', layer '" + layer + "' has unsupported type!",
                        &node.position());
                return Value{};
        }
    }
    ev.warn("Can't find dimension '" + name + "' in '" + label + "', layer '" + layer + "'!", &node.position());
    return Value{};
}

// dxf_cross(file, layer, origin, scale): where the first two free-standing
// LINEs cross. A line touching another line or an arc is part of an outline.
Value builtinDxfCross(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node) {
    const Value file = getArg(args, 0, "file");
    const std::string layer = stringOr(getArg(args, 1, "layer"));
    const Placement placement = readPlacement(ev, args, node, "dxf_cross");
    const std::string label = fileLabel(file);

    const auto entities = readDxf(resolveFilePath(file, node), placement);
    if (!entities) {
        ev.warn("Can't open DXF file '" + label + "'!", &node.position());
        return Value{};
    }

    struct Point { double x, y; };
    struct Line { Point a, b; };
    std::vector<Line> lines;
    std::vector<Point> arcEnds;
    for (const DxfEntity& e : *entities) {
        if (!onLayer(e, layer)) continue;
        if (e.type == "LINE") {
            lines.push_back({{e.x[0], e.y[0]}, {e.x[1], e.y[1]}});
        } else if (e.type == "ARC") {
            for (const double deg : {e.angle, e.endAngle}) {
                const double a = deg * kPi / 180;
                arcEnds.push_back({e.x[0] + e.radius * std::cos(a), e.y[0] + e.radius * std::sin(a)});
            }
        }
    }

    const auto same = [](const Point& p, const Point& q) {
        return std::fabs(p.x - q.x) < 1e-6 && std::fabs(p.y - q.y) < 1e-6;
    };
    const auto touches = [&](size_t i) {
        for (const Point& p : {lines[i].a, lines[i].b}) {
            for (size_t j = 0; j < lines.size(); ++j) {
                if (j != i && (same(p, lines[j].a) || same(p, lines[j].b))) return true;
            }
            for (const Point& q : arcEnds) {
                if (same(p, q)) return true;
            }
        }
        return false;
    };
    std::vector<Line> strokes;
    for (size_t i = 0; i < lines.size() && strokes.size() < 2; ++i) {
        if (!touches(i)) strokes.push_back(lines[i]);
    }

    if (strokes.size() == 2) {
        const Line& l = strokes[0];
        const Line& m = strokes[1];
        const double rx = l.b.x - l.a.x, ry = l.b.y - l.a.y;
        const double sx = m.b.x - m.a.x, sy = m.b.y - m.a.y;
        const double denom = rx * sy - ry * sx;
        if (std::fabs(denom) > 1e-12) {
            const double t = ((m.a.x - l.a.x) * sy - (m.a.y - l.a.y) * sx) / denom;
            return Value{makeList({l.a.x + t * rx, l.a.y + t * ry})};
        }
    }
    ev.warn("Can't find cross in '" + label + "', layer '" + layer + "'!", &node.position());
    return Value{};
}

}  // namespace oscadeval
