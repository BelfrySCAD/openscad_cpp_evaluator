#include "openscad_cpp_evaluator/dxf_svg_import.hpp"

#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>

// DXF import, written from the DXF reference (group codes; the ENTITIES and
// BLOCKS sections; LINE, CIRCLE, ARC, ELLIPSE, LWPOLYLINE, POLYLINE/VERTEX,
// INSERT) and from black-box runs of OpenSCAD 2026.02.01, whose behaviour it
// reproduces:
//
// - Every entity becomes straight segments. An arc gets the circle's
//   fragment count scaled to its sweep; an ellipse counts by its major radius.
// - Every point is snapped to a 1/1024 grid, reusing an already-seen point
//   in a neighbouring cell, so near-coincident endpoints join.
// - Segments are chained end to end into paths, and EVERY path is a polygon:
//   an open chain is closed by a straight edge. The polygons are filled
//   even-odd (the caller's CrossSection does that).
// - origin= and scale= map file coordinates to (v - origin) * scale before
//   snapping, so arc fragment counts see the scaled radius.
// - A block's contents are built (and snapped) in the block's own frame and
//   only then moved by each INSERT (scale 41/42, then rotation 50, then the
//   insertion point); the block's base point is not used. layer= filters the
//   INSERT, not what the block holds.
// - An entity missing coordinates it needs is not drawn: OpenSCAD warns "Not
//   enough input values for <the NEXT entity's type>" and keeps collecting
//   into the same entity, so the next one's values complete it.
// - A malformed number warns and is skipped. LWPOLYLINE bulges are ignored.
//
// Deliberately not copied: OpenSCAD shifts an ELLIPSE's major-axis vector
// (11/21, which is relative to the centre) by origin=, and applies scale= to
// its minor axis twice, so a scaled or shifted ellipse comes out distorted.
// Here an ellipse moves and scales like everything else. Kept from before, as
// an extension: POLYLINE/VERTEX/SEQEND, which OpenSCAD reports as
// unsupported and draws nothing for.

namespace oscadeval {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGrid = 1024.0;  // OpenSCAD snaps DXF points to 1/1024

using Pt = std::array<double, 2>;

std::string trim(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// One entity's group values as read. xs/ys collect groups 10/11 and 20/21
// in file order, which is all any supported entity needs.
struct Entity {
    std::string type;
    std::string layer;
    std::string name;  // group 2: INSERT's block, BLOCK's own name, SECTION's
    std::vector<double> xs, ys;
    std::optional<double> g40, g41, g42, g50, g51;
    int flags = 0;  // group 70
};

// Snaps a point to the grid, reusing a point already seen in its cell or a
// neighbouring one.
class Grid {
public:
    Pt align(const Pt& p) {
        const long long kx = std::llround(p[0] * kGrid), ky = std::llround(p[1] * kGrid);
        if (auto it = cells_.find({kx, ky}); it != cells_.end()) return it->second;
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy)
                if (auto it = cells_.find({kx + dx, ky + dy}); it != cells_.end()) return it->second;
        const Pt snapped{kx / kGrid, ky / kGrid};
        cells_[{kx, ky}] = snapped;
        return snapped;
    }

private:
    std::map<std::pair<long long, long long>, Pt> cells_;
};

using Segments = std::vector<std::pair<Pt, Pt>>;

// Where an entity's coordinates land: (v - origin) * scale at top level,
// unchanged inside a block.
struct Frame {
    double ox = 0, oy = 0, s = 1;
    Pt pt(double x, double y) const { return {(x - ox) * s, (y - oy) * s}; }
};

class Builder {
public:
    Builder(const Discretizer& disc, const std::map<std::string, std::vector<Entity>>& blocks)
        : disc_(disc), blocks_(blocks) {}

    void entity(const Entity& e, const Frame& f, Segments& out, int depth = 0) {
        const auto add = [&](Pt a, Pt b) {
            if (!std::isfinite(a[0]) || !std::isfinite(a[1]) || !std::isfinite(b[0]) || !std::isfinite(b[1])) return;
            a = grid_.align(a);
            b = grid_.align(b);
            if (a != b) out.push_back({a, b});
        };
        // c + u*cos(t) + v*sin(t) for t from t0 over `sweep` radians, in as
        // many segments as a circle of radius r gets for that sweep.
        const auto curve = [&](Pt c, Pt u, Pt v, double r, double t0, double sweep) {
            const int n = disc_.circular(r, sweep * 180.0 / kPi).value_or(3);
            const auto at = [&](int i) {
                const double t = t0 + sweep * i / n;
                return Pt{c[0] + u[0] * std::cos(t) + v[0] * std::sin(t), c[1] + u[1] * std::cos(t) + v[1] * std::sin(t)};
            };
            for (int i = 0; i < n; ++i) add(at(i), at(i + 1));
        };

        if (e.type == "LINE") {
            add(f.pt(e.xs[0], e.ys[0]), f.pt(e.xs[1], e.ys[1]));
        } else if (e.type == "CIRCLE" || e.type == "ARC") {
            const double r = e.g40.value_or(0) * f.s;
            double a0 = 0, sweep = 2 * kPi;
            if (e.type == "ARC") {
                a0 = e.g50.value_or(0);
                double a1 = e.g51.value_or(0);
                while (a1 < a0) a1 += 360;
                sweep = (a1 - a0) * kPi / 180;
                a0 *= kPi / 180;
            }
            curve(f.pt(e.xs[0], e.ys[0]), {r, 0}, {0, r}, r, a0, sweep);
        } else if (e.type == "ELLIPSE") {
            const Pt major{e.xs[1] * f.s, e.ys[1] * f.s};
            const double ratio = e.g40.value_or(0);
            const double t0 = e.g41.value_or(0);
            double t1 = e.g42.value_or(0);  // unset 41/42 is an empty arc, as in OpenSCAD
            while (t1 < t0) t1 += 2 * kPi;
            curve(f.pt(e.xs[0], e.ys[0]), major, {-major[1] * ratio, major[0] * ratio}, std::hypot(major[0], major[1]),
                  t0, t1 - t0);
        } else if (e.type == "LWPOLYLINE") {
            const size_t n = e.xs.size();
            for (size_t i = 0; i + 1 < n; ++i) add(f.pt(e.xs[i], e.ys[i]), f.pt(e.xs[i + 1], e.ys[i + 1]));
            if ((e.flags & 1) && n > 1) add(f.pt(e.xs[n - 1], e.ys[n - 1]), f.pt(e.xs[0], e.ys[0]));
        } else if (e.type == "INSERT") {
            // ponytail: a depth cap stands in for cycle detection; a block
            // that inserts itself just stops expanding.
            const auto it = blocks_.find(e.name);
            if (it == blocks_.end() || depth > 32) return;
            Segments inner;
            for (const Entity& be : it->second) entity(be, Frame{}, inner, depth + 1);
            const double sx = e.g41.value_or(1), sy = e.g42.value_or(1);
            const double rot = e.g50.value_or(0) * kPi / 180, c = std::cos(rot), s = std::sin(rot);
            const auto place = [&](const Pt& p) {
                const double x = p[0] * sx, y = p[1] * sy;
                return f.pt(e.xs[0] + c * x - s * y, e.ys[0] + s * x + c * y);
            };
            for (const auto& [a, b] : inner) add(place(a), place(b));
        }
    }

private:
    const Discretizer& disc_;
    const std::map<std::string, std::vector<Entity>>& blocks_;
    Grid grid_;
};

// Whether an entity has the coordinates it needs to be drawn.
bool complete(const Entity& e) {
    if (e.type == "LINE" || e.type == "ELLIPSE") return e.xs.size() >= 2 && e.ys.size() >= 2;
    if (e.type == "CIRCLE" || e.type == "ARC" || e.type == "INSERT" || e.type == "VERTEX")
        return !e.xs.empty() && !e.ys.empty();
    if (e.type == "LWPOLYLINE") return e.xs.size() == e.ys.size();
    return true;
}

// The whole (trimmed) value must be a number, as strtod reads one.
bool parseNumber(const std::string& s, double& out) {
    if (s.empty()) return false;
    char* end = nullptr;
    out = std::strtod(s.c_str(), &end);
    return *end == '\0';
}

// Chains segments end to end, as OpenSCAD does for every case this was
// checked against (all of its own test files and a few hundred random ones):
// first every chain with a loose end, walked from that end; then the rest,
// in file order. A path grows from its end, then from its start, taking the
// first unused segment in file order, until it closes or nothing connects.
// In that second pass a path also may not run back into a point it already
// passes through, other than to close on its start.
//
// ponytail: OpenSCAD's choice where three or more segments meet inside a
// closed network (duplicated edges especially) still differs in some
// orderings; nothing in the black-box runs pinned its rule down.
std::vector<Contour2d> joinSegments(const Segments& segs) {
    std::vector<bool> used(segs.size(), false);
    std::multimap<Pt, size_t> byEnd;
    for (size_t i = 0; i < segs.size(); ++i) {
        byEnd.insert({segs[i].first, i});
        byEnd.insert({segs[i].second, i});
    }
    const auto loose = [&](const Pt& p) { return byEnd.count(p) == 1; };

    std::vector<Contour2d> paths;
    const auto walk = [&](size_t i, const Pt& from, const Pt& to, bool simple) {
        used[i] = true;
        std::deque<Pt> path{from, to};
        std::set<Pt> visited{from, to};
        // Takes the first unused segment from p that may join the path;
        // returns its other end. `other` is the path's opposite end.
        const auto take = [&](const Pt& p, const Pt& other) -> std::optional<Pt> {
            size_t best = segs.size();
            const auto [lo, hi] = byEnd.equal_range(p);
            for (auto it = lo; it != hi; ++it) {
                const auto& [a, b] = segs[it->second];
                const Pt& far = a == p ? b : a;
                if (!used[it->second] && (!simple || far == other || !visited.count(far)))
                    best = std::min(best, it->second);
            }
            if (best == segs.size()) return std::nullopt;
            used[best] = true;
            const Pt far = segs[best].first == p ? segs[best].second : segs[best].first;
            visited.insert(far);
            return far;
        };
        bool closed = false;
        while (const auto next = take(path.back(), path.front())) {
            if (*next == path.front()) {
                closed = true;
                break;
            }
            path.push_back(*next);
        }
        while (!closed) {
            const auto prev = take(path.front(), path.back());
            if (!prev || *prev == path.back()) break;
            path.push_front(*prev);
        }
        paths.emplace_back(path.begin(), path.end());
    };
    for (size_t i = 0; i < segs.size(); ++i) {
        if (used[i]) continue;
        const auto& [a, b] = segs[i];
        if (loose(a)) walk(i, a, b, false);
        else if (loose(b)) walk(i, b, a, false);
    }
    for (size_t i = 0; i < segs.size(); ++i)
        if (!used[i]) walk(i, segs[i].first, segs[i].second, true);
    return paths;
}

} // namespace

DxfImport loadDxf(const std::string& path, const DxfOptions& opts) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open '" + path + "'");

    static const std::set<std::string> kSupported = {"LINE",   "CIRCLE",    "ARC",      "ELLIPSE", "LWPOLYLINE",
                                                     "INSERT", "DIMENSION", "POLYLINE", "VERTEX",  "SEQEND"};
    static const std::set<std::string> kStructure = {"SECTION", "ENDSEC", "BLOCK", "ENDBLK"};

    DxfImport result;
    std::vector<Entity> top;
    std::map<std::string, std::vector<Entity>> blocks;
    // Iterated in hash order, which is the order OpenSCAD reports them in
    // (with the same standard library).
    std::unordered_map<std::string, int> unsupported;
    std::string section;
    std::optional<std::string> block;
    std::optional<Entity> polyline;  // POLYLINE..SEQEND, gathered as one LWPOLYLINE

    // Files a finished entity where it belongs. False, filing nothing, when
    // it is short of the coordinates it needs.
    const auto finish = [&](const Entity& e) {
        if (!complete(e)) return false;
        if (e.type == "SECTION") section = e.name;
        if (e.type == "BLOCK") block = e.name;
        if (kStructure.count(e.type)) return true;
        std::vector<Entity>* dest = nullptr;
        if (section == "ENTITIES") dest = &top;
        else if (section == "BLOCKS" && block) dest = &blocks[*block];
        if (!dest) return true;
        if (e.type == "POLYLINE") {
            polyline = Entity{"LWPOLYLINE", e.layer, "", {}, {}, {}, {}, {}, {}, {}, e.flags};
        } else if (e.type == "VERTEX") {
            if (polyline) {
                polyline->xs.push_back(e.xs[0]);
                polyline->ys.push_back(e.ys[0]);
            }
        } else if (e.type == "SEQEND") {
            if (polyline) dest->push_back(*polyline);
            polyline.reset();
        } else {
            dest->push_back(e);
        }
        return true;
    };

    Entity cur;
    std::string codeLine, valueLine;
    while (std::getline(in, codeLine) && std::getline(in, valueLine)) {
        const std::string codeStr = trim(codeLine), value = trim(valueLine);
        if (codeStr.empty()) continue;
        char* end = nullptr;
        const long code = std::strtol(codeStr.c_str(), &end, 10);
        if (*end != '\0') break;

        if (code == 0) {
            if (!finish(cur)) {
                result.warnings.push_back("Not enough input values for " + value + ". in '" + path + "'");
                continue;
            }
            if (value == "ENDSEC") section.clear();
            if (value == "ENDBLK") block.reset();
            if ((section == "ENTITIES" || (section == "BLOCKS" && block)) && !kSupported.count(value) &&
                !kStructure.count(value))
                ++unsupported[value];
            cur = Entity{};
            cur.type = value;
            continue;
        }

        const bool numeric = (code >= 10 && code <= 16) || (code >= 20 && code <= 26) || (code >= 40 && code <= 42) ||
                             code == 50 || code == 51 || code == 70;
        double v = 0;
        if (numeric && !parseNumber(value, v)) {
            result.warnings.push_back("Illegal value '" + value + "'in `" + path + "'");
            continue;
        }
        switch (code) {
            case 2: cur.name = value; break;
            case 8: cur.layer = value; break;
            case 10:
            case 11: cur.xs.push_back(v); break;
            case 20:
            case 21: cur.ys.push_back(v); break;
            case 40: cur.g40 = v; break;
            case 41: cur.g41 = v; break;
            case 42: cur.g42 = v; break;
            case 50: cur.g50 = v; break;
            case 51: cur.g51 = v; break;
            case 70: cur.flags = static_cast<int>(v); break;
            default: break;
        }
    }
    finish(cur);

    Builder builder(opts.disc, blocks);
    Segments segs;
    const Frame frame{opts.xorigin, opts.yorigin, opts.scale};
    for (const Entity& e : top)
        if (!opts.layer || e.layer == *opts.layer) builder.entity(e, frame, segs);
    result.contours = joinSegments(segs);

    if (!unsupported.empty()) {
        // Named relative to the working directory, as OpenSCAD names it here.
        std::error_code ec;
        const std::string shown = std::filesystem::absolute(path, ec)
                                      .lexically_proximate(std::filesystem::current_path(ec))
                                      .generic_string();
        for (const auto& [type, count] : unsupported)
            result.warnings.push_back("Unsupported DXF Entity '" + type + "' (" + std::to_string(count) + ") in \"" +
                                      shown + "\".");
    }
    return result;
}

} // namespace oscadeval
