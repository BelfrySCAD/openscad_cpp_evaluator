#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace oscadeval {

// dxf_dim() / dxf_cross() -- read a measurement straight out of a DXF file
// rather than out of the model. Ported from the reference's io/dxfdim.cc
// plus the DIMENSION/LINE half of io/DxfData.cc.
//
// These share the group-code walk with src/import/dxf_import.cpp but not its
// output: that one only cares about closed contours, while these two need
// the DIMENSION entities' seven coordinate slots and the raw LINE endpoints,
// neither of which a contour list keeps.

namespace {

struct Entity {
    std::string type;                  // "DIMENSION", "LINE", ...
    std::string layer;
    std::string name;                  // group 1: DIMENSION's text override
    int dimType = 0;                   // group 70
    double angle = 0.0;                // group 50
    double coords[7][2] = {};          // groups 10-16 / 20-26
    std::vector<double> xverts, yverts; // groups 10/11 and 20/21, in order
};

std::string trimmed(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

// One entity per group-0 marker. Coordinates carry the origin shift and
// scale the reference applies while parsing, with its own exception: groups
// 11, 12 and 16 (and their 21/22/26 partners) are scaled but NOT shifted,
// since they are extents rather than positions.
std::vector<Entity> readEntities(const std::string& path, double xorigin, double yorigin, double scale) {
    std::ifstream in(path);
    std::vector<Entity> out;
    if (!in) return out;

    std::string codeLine, dataLine;
    Entity cur;
    bool started = false;
    while (std::getline(in, codeLine) && std::getline(in, dataLine)) {
        const std::string codeStr = trimmed(codeLine);
        const std::string data = trimmed(dataLine);
        if (codeStr.empty()) continue;
        int code = 0;
        try {
            code = std::stoi(codeStr);
        } catch (...) {
            break;
        }
        const auto num = [&]() -> double {
            try {
                return std::stod(data);
            } catch (...) {
                return 0.0;
            }
        };

        if (code == 0) {
            if (started) out.push_back(cur);
            cur = Entity{};
            cur.type = data;
            started = true;
            continue;
        }
        if (!started) continue;

        if (code >= 10 && code <= 16) {
            cur.coords[code - 10][0] =
                (code == 11 || code == 12 || code == 16) ? num() * scale : (num() - xorigin) * scale;
        } else if (code >= 20 && code <= 26) {
            cur.coords[code - 20][1] =
                (code == 21 || code == 22 || code == 26) ? num() * scale : (num() - yorigin) * scale;
        }
        switch (code) {
            case 1: cur.name = data; break;
            case 8: cur.layer = data; break;
            case 10:
            case 11: cur.xverts.push_back((num() - xorigin) * scale); break;
            case 20:
            case 21: cur.yverts.push_back((num() - yorigin) * scale); break;
            case 50: cur.angle = num(); break;
            case 70:
                try {
                    cur.dimType = std::stoi(data);
                } catch (...) {
                }
                break;
            default: break;
        }
    }
    if (started) out.push_back(cur);
    return out;
}

double deg(double r) { return r * 180.0 / 3.14159265358979323846; }
double rad(double d) { return d * 3.14159265358979323846 / 180.0; }

struct CommonArgs {
    std::string path;
    std::string rawFile;
    std::string layer;
    double xorigin = 0.0, yorigin = 0.0, scale = 1.0;
};

CommonArgs commonArgs(const CallArgs& args, const oscad::ASTNode& node) {
    CommonArgs c;
    const Value fileArg = getArg(args, 0, "file", Value{});
    c.rawFile = std::holds_alternative<std::string>(fileArg) ? std::get<std::string>(fileArg) : fmtValue(fileArg);
    c.path = resolveFilePath(fileArg, node);
    const Value layerArg = getArg(args, std::nullopt, "layer", Value{});
    if (const std::string* s = std::get_if<std::string>(&layerArg)) c.layer = *s;
    const Value originArg = getArg(args, std::nullopt, "origin", Value{});
    if (const ListPtr* l = std::get_if<ListPtr>(&originArg); l && *l && (*l)->items.size() >= 2) {
        c.xorigin = toDoubleLenient((*l)->items[0]);
        c.yorigin = toDoubleLenient((*l)->items[1]);
    }
    const Value scaleArg = getArg(args, std::nullopt, "scale", Value{1.0});
    if (std::holds_alternative<double>(scaleArg)) c.scale = std::get<double>(scaleArg);
    return c;
}

} // namespace

Value builtinDxfDim(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node) {
    const CommonArgs c = commonArgs(args, node);
    const Value nameArg = getArg(args, std::nullopt, "name", Value{});
    const std::string name = std::holds_alternative<std::string>(nameArg) ? std::get<std::string>(nameArg) : "";

    if (!std::filesystem::exists(c.path)) {
        ev.warn("Can't open DXF file '" + c.rawFile + "'!", &node.position());
        return Value{};
    }
    for (const Entity& e : readEntities(c.path, c.xorigin, c.yorigin, c.scale)) {
        if (e.type != "DIMENSION") continue;
        if (!c.layer.empty() && c.layer != e.layer) continue;
        if (!name.empty() && e.name != name) continue;

        const int type = e.dimType & 7;
        if (type == 0) {   // rotated, horizontal or vertical
            const double x = e.coords[4][0] - e.coords[3][0];
            const double y = e.coords[4][1] - e.coords[3][1];
            return Value{std::fabs(x * std::cos(rad(e.angle)) + y * std::sin(rad(e.angle)))};
        }
        if (type == 1) {   // aligned
            const double x = e.coords[4][0] - e.coords[3][0];
            const double y = e.coords[4][1] - e.coords[3][1];
            return Value{std::sqrt(x * x + y * y)};
        }
        if (type == 2) {   // angular
            const double a1 = deg(std::atan2(e.coords[0][0] - e.coords[5][0], e.coords[0][1] - e.coords[5][1]));
            const double a2 = deg(std::atan2(e.coords[4][0] - e.coords[3][0], e.coords[4][1] - e.coords[3][1]));
            return Value{std::fabs(a1 - a2)};
        }
        if (type == 3 || type == 4) {   // diameter or radius
            const double x = e.coords[5][0] - e.coords[0][0];
            const double y = e.coords[5][1] - e.coords[0][1];
            return Value{std::sqrt(x * x + y * y)};
        }
        if (type == 6) {   // ordinate
            return Value{(e.dimType & 64) ? e.coords[3][0] : e.coords[3][1]};
        }
        // type 5 (angular 3-point) falls through, as in the reference.
        ev.warn("Dimension '" + name + "' in '" + c.rawFile + "', layer '" + c.layer + "' has unsupported type!",
                &node.position());
        return Value{};
    }
    ev.warn("Can't find dimension '" + name + "' in '" + c.rawFile + "', layer '" + c.layer + "'!",
            &node.position());
    return Value{};
}

Value builtinDxfCross(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node) {
    const CommonArgs c = commonArgs(args, node);
    if (!std::filesystem::exists(c.path)) {
        ev.warn("Can't open DXF file '" + c.rawFile + "'!", &node.position());
        return Value{};
    }

    // The first two LINEs on the layer are taken to be the cross. The
    // reference walks every 2-point path instead, which for a real cross is
    // the same set: its two strokes meet in the middle rather than at their
    // endpoints, so neither gets joined into a longer path.
    double p[4][2];
    int found = 0;
    for (const Entity& e : readEntities(c.path, c.xorigin, c.yorigin, c.scale)) {
        if (e.type != "LINE") continue;
        if (!c.layer.empty() && c.layer != e.layer) continue;
        if (e.xverts.size() < 2 || e.yverts.size() < 2) continue;
        p[found][0] = e.xverts[0];
        p[found][1] = e.yverts[0];
        ++found;
        p[found][0] = e.xverts[1];
        p[found][1] = e.yverts[1];
        ++found;
        if (found == 4) {
            const double x1 = p[0][0], y1 = p[0][1], x2 = p[1][0], y2 = p[1][1];
            const double x3 = p[2][0], y3 = p[2][1], x4 = p[3][0], y4 = p[3][1];
            const double dem = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
            if (dem == 0.0) break;   // parallel: no cross, same as the reference
            const double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / dem;
            std::vector<Value> xy{Value{x1 + ua * (x2 - x1)}, Value{y1 + ua * (y2 - y1)}};
            return Value{std::make_shared<const ValueList>(ValueList{std::move(xy)})};
        }
    }
    ev.warn("Can't find cross in '" + c.rawFile + "', layer '" + c.layer + "'!", &node.position());
    return Value{};
}

} // namespace oscadeval
