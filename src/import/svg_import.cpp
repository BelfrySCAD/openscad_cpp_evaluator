#include "openscad_cpp_evaluator/dxf_svg_import.hpp"

#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

// Hand-rolled SVG reader -- a close port of _load_svg_contours (path/
// transform parsing) plus a minimal recursive-descent XML tree parser
// (Python's version uses stdlib xml.etree.ElementTree for that half; this
// project has no XML dependency, so this file provides just enough of one:
// open/close/self-closing tags, quoted attributes, comments, CDATA,
// prolog/DOCTYPE skipping, and the 5 basic entity references. No namespace
// URI resolution -- a tag's `prefix:` is stripped textually, matching the
// common case (default-namespace or single-prefix SVGs) without full XML
// namespace semantics. Upgrade to pugixml if a real SVG's XML proves too
// irregular for this.

namespace oscadeval {

namespace {

// -- minimal XML tree -----------------------------------------------------

struct XmlNode {
    std::string tag;
    std::vector<std::pair<std::string, std::string>> attrs;
    std::vector<XmlNode> children;

    std::string getAttr(const std::string& name, const std::string& def = "") const {
        for (const auto& [k, v] : attrs) {
            if (k == name) return v;
        }
        return def;
    }
};

std::string decodeEntities(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size();) {
        if (s[i] == '&') {
            if (s.compare(i, 5, "&amp;") == 0) {
                out += '&';
                i += 5;
                continue;
            }
            if (s.compare(i, 4, "&lt;") == 0) {
                out += '<';
                i += 4;
                continue;
            }
            if (s.compare(i, 4, "&gt;") == 0) {
                out += '>';
                i += 4;
                continue;
            }
            if (s.compare(i, 6, "&quot;") == 0) {
                out += '"';
                i += 6;
                continue;
            }
            if (s.compare(i, 6, "&apos;") == 0) {
                out += '\'';
                i += 6;
                continue;
            }
        }
        out += s[i++];
    }
    return out;
}

std::string stripPrefix(const std::string& tag) {
    const size_t colon = tag.find(':');
    return colon == std::string::npos ? tag : tag.substr(colon + 1);
}

void skipWs(const std::string& s, size_t& i) {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
}

// Parses one element (and, recursively, its children) starting at `s[i]`
// (which must be '<' of an opening tag), advancing `i` past its closing
// tag. Returns nullopt for a construct that isn't a real element (comment,
// prolog, DOCTYPE, CDATA-only content) -- the caller skips and retries.
std::optional<XmlNode> parseElement(const std::string& s, size_t& i) {
    if (s.compare(i, 4, "<!--") == 0) {
        const size_t end = s.find("-->", i);
        i = (end == std::string::npos) ? s.size() : end + 3;
        return std::nullopt;
    }
    if (s.compare(i, 9, "<![CDATA[") == 0) {
        const size_t end = s.find("]]>", i);
        i = (end == std::string::npos) ? s.size() : end + 3;
        return std::nullopt;
    }
    if (s.compare(i, 2, "<?") == 0) {
        const size_t end = s.find("?>", i);
        i = (end == std::string::npos) ? s.size() : end + 2;
        return std::nullopt;
    }
    if (s.compare(i, 2, "<!") == 0) {
        const size_t end = s.find('>', i);
        i = (end == std::string::npos) ? s.size() : end + 1;
        return std::nullopt;
    }

    ++i; // '<'
    XmlNode node;
    size_t nameStart = i;
    while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i])) && s[i] != '>' && s[i] != '/') ++i;
    node.tag = stripPrefix(s.substr(nameStart, i - nameStart));

    bool selfClosing = false;
    while (i < s.size()) {
        skipWs(s, i);
        if (i >= s.size()) break;
        if (s[i] == '/' && i + 1 < s.size() && s[i + 1] == '>') {
            selfClosing = true;
            i += 2;
            break;
        }
        if (s[i] == '>') {
            ++i;
            break;
        }
        const size_t attrNameStart = i;
        while (i < s.size() && s[i] != '=' && !std::isspace(static_cast<unsigned char>(s[i])) && s[i] != '>' && s[i] != '/') ++i;
        const std::string attrName = s.substr(attrNameStart, i - attrNameStart);
        skipWs(s, i);
        std::string attrValue;
        if (i < s.size() && s[i] == '=') {
            ++i;
            skipWs(s, i);
            if (i < s.size() && (s[i] == '"' || s[i] == '\'')) {
                const char quote = s[i++];
                const size_t valStart = i;
                while (i < s.size() && s[i] != quote) ++i;
                attrValue = decodeEntities(s.substr(valStart, i - valStart));
                if (i < s.size()) ++i;
            }
        }
        if (!attrName.empty()) node.attrs.emplace_back(attrName, attrValue);
    }

    if (!selfClosing) {
        // Consume children/text until this tag's own closing `</tag>`.
        for (;;) {
            const size_t nextLt = s.find('<', i);
            if (nextLt == std::string::npos) break;
            i = nextLt;
            if (s.compare(i, 2, "</") == 0) {
                const size_t end = s.find('>', i);
                i = (end == std::string::npos) ? s.size() : end + 1;
                break;
            }
            std::optional<XmlNode> child = parseElement(s, i);
            if (child) node.children.push_back(std::move(*child));
        }
    }
    return node;
}

std::optional<XmlNode> parseXmlRoot(const std::string& text) {
    size_t i = 0;
    for (;;) {
        const size_t lt = text.find('<', i);
        if (lt == std::string::npos) return std::nullopt;
        i = lt;
        std::optional<XmlNode> el = parseElement(text, i);
        if (el) return el;
    }
}

// -- 2D affine transform (SVG's own [[a,c,e],[b,d,f],[0,0,1]] convention) --

struct Mat3 {
    double a = 1, b = 0, c = 0, d = 1, e = 0, f = 0;
};

// this * other (SVG transform composition is left-multiplication of the
// new transform onto the accumulated one -- matches `m = new @ m`).
Mat3 compose(const Mat3& m, const Mat3& acc) {
    return Mat3{
        m.a * acc.a + m.c * acc.b, m.b * acc.a + m.d * acc.b, m.a * acc.c + m.c * acc.d,
        m.b * acc.c + m.d * acc.d, m.a * acc.e + m.c * acc.f + m.e, m.b * acc.e + m.d * acc.f + m.f,
    };
}

std::array<double, 2> applyMat(const std::array<double, 2>& pt, const Mat3& m) {
    const double x = m.a * pt[0] + m.c * pt[1] + m.e;
    const double y = m.b * pt[0] + m.d * pt[1] + m.f;
    return {x, -y}; // flip Y: SVG down -> OpenSCAD up
}

std::vector<double> parseNumberList(const std::string& s) {
    std::vector<double> out;
    size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && (std::isspace(static_cast<unsigned char>(s[i])) || s[i] == ',')) ++i;
        const size_t start = i;
        if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
        while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '.')) ++i;
        if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
            ++i;
            if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
            while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
        }
        if (i > start) out.push_back(std::stod(s.substr(start, i - start)));
        else if (i < s.size()) ++i; // stray character, skip
    }
    return out;
}

Mat3 parseTransform(const std::string& tStr) {
    Mat3 acc;
    size_t i = 0;
    while (i < tStr.size()) {
        while (i < tStr.size() && !std::isalpha(static_cast<unsigned char>(tStr[i]))) ++i;
        const size_t nameStart = i;
        while (i < tStr.size() && std::isalpha(static_cast<unsigned char>(tStr[i]))) ++i;
        const std::string cmd = tStr.substr(nameStart, i - nameStart);
        if (cmd.empty()) break;
        const size_t parenOpen = tStr.find('(', i);
        if (parenOpen == std::string::npos) break;
        const size_t parenClose = tStr.find(')', parenOpen);
        if (parenClose == std::string::npos) break;
        const std::vector<double> ns = parseNumberList(tStr.substr(parenOpen + 1, parenClose - parenOpen - 1));
        i = parenClose + 1;

        if (cmd == "matrix" && ns.size() >= 6) {
            acc = compose(Mat3{ns[0], ns[1], ns[2], ns[3], ns[4], ns[5]}, acc);
        } else if (cmd == "translate" && !ns.empty()) {
            const double tx = ns[0], ty = ns.size() > 1 ? ns[1] : 0.0;
            acc = compose(Mat3{1, 0, 0, 1, tx, ty}, acc);
        } else if (cmd == "scale" && !ns.empty()) {
            const double sx = ns[0], sy = ns.size() > 1 ? ns[1] : ns[0];
            acc = compose(Mat3{sx, 0, 0, sy, 0, 0}, acc);
        } else if (cmd == "rotate" && !ns.empty()) {
            const double ang = ns[0] * M_PI / 180.0;
            const double cx = ns.size() > 1 ? ns[1] : 0.0, cy = ns.size() > 2 ? ns[2] : 0.0;
            const double ca = std::cos(ang), sa = std::sin(ang);
            const Mat3 t1{1, 0, 0, 1, -cx, -cy};
            const Mat3 r{ca, sa, -sa, ca, 0, 0};
            const Mat3 t2{1, 0, 0, 1, cx, cy};
            acc = compose(t2, compose(r, compose(t1, acc)));
        }
    }
    return acc;
}

constexpr int kSegs = 32;

std::vector<std::array<double, 2>> cubicPts(const std::array<double, 2>& p0, const std::array<double, 2>& p1,
                                             const std::array<double, 2>& p2, const std::array<double, 2>& p3) {
    std::vector<std::array<double, 2>> pts;
    pts.reserve(kSegs);
    for (int i = 1; i <= kSegs; ++i) {
        const double t = static_cast<double>(i) / kSegs, mt = 1 - t;
        const double a = mt * mt * mt, b = 3 * mt * mt * t, c = 3 * mt * t * t, d = t * t * t;
        pts.push_back({a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0], a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1]});
    }
    return pts;
}

std::vector<std::array<double, 2>> quadPts(const std::array<double, 2>& p0, const std::array<double, 2>& p1,
                                            const std::array<double, 2>& p2) {
    std::vector<std::array<double, 2>> pts;
    pts.reserve(kSegs);
    for (int i = 1; i <= kSegs; ++i) {
        const double t = static_cast<double>(i) / kSegs, mt = 1 - t;
        pts.push_back({mt * mt * p0[0] + 2 * mt * t * p1[0] + t * t * p2[0], mt * mt * p0[1] + 2 * mt * t * p1[1] + t * t * p2[1]});
    }
    return pts;
}

// SVG elliptical arc endpoint-to-center parameterization + sampling.
std::vector<std::array<double, 2>> arcPts(double x1, double y1, double rx, double ry, double xRotDeg, bool large, bool sweep,
                                           double x2, double y2) {
    if (rx == 0 || ry == 0) return {{x2, y2}};
    const double cosR = std::cos(xRotDeg * M_PI / 180.0), sinR = std::sin(xRotDeg * M_PI / 180.0);
    const double dx = (x1 - x2) / 2, dy = (y1 - y2) / 2;
    const double x1p = cosR * dx + sinR * dy, y1p = -sinR * dx + cosR * dy;
    double rxx = rx, ryy = ry;
    const double lam = (x1p / rxx) * (x1p / rxx) + (y1p / ryy) * (y1p / ryy);
    if (lam > 1) {
        rxx *= std::sqrt(lam);
        ryy *= std::sqrt(lam);
    }
    double sq = std::max(0.0, (rxx * ryy) * (rxx * ryy) - (rxx * y1p) * (rxx * y1p) - (ryy * x1p) * (ryy * x1p));
    sq = std::sqrt(sq / std::max(1e-12, (rxx * y1p) * (rxx * y1p) + (ryy * x1p) * (ryy * x1p)));
    if (large == sweep) sq = -sq;
    const double cxp = sq * rxx * y1p / ryy, cyp = -sq * ryy * x1p / rxx;
    const double cx = cosR * cxp - sinR * cyp + (x1 + x2) / 2, cy = sinR * cxp + cosR * cyp + (y1 + y2) / 2;
    const auto angle = [](double ux, double uy, double vx, double vy) { return std::atan2(ux * vy - uy * vx, ux * vx + uy * vy); };
    const double th1 = angle(1, 0, (x1p - cxp) / rxx, (y1p - cyp) / ryy);
    double dth = angle((x1p - cxp) / rxx, (y1p - cyp) / ryy, (-x1p - cxp) / rxx, (-y1p - cyp) / ryy);
    if (!sweep && dth > 0) dth -= 2 * M_PI;
    if (sweep && dth < 0) dth += 2 * M_PI;
    const int n = std::max(4, static_cast<int>(std::fabs(dth) / (2 * M_PI) * kSegs * 4));
    std::vector<std::array<double, 2>> pts;
    pts.reserve(static_cast<size_t>(n));
    for (int i = 1; i <= n; ++i) {
        const double th = th1 + dth * i / n;
        pts.push_back({cosR * rxx * std::cos(th) - sinR * ryy * std::sin(th) + cx, sinR * rxx * std::cos(th) + cosR * ryy * std::sin(th) + cy});
    }
    return pts;
}

// Tokenizes an SVG path `d` attribute into command letters and numbers (in
// source order), mirroring the reference's own regex tokenizer.
std::vector<std::string> tokenizePath(const std::string& d) {
    std::vector<std::string> toks;
    size_t i = 0;
    while (i < d.size()) {
        const char c = d[i];
        if (std::isspace(static_cast<unsigned char>(c)) || c == ',') {
            ++i;
        } else if (std::strchr("MmZzLlHhVvCcSsQqTtAa", c) != nullptr) {
            toks.emplace_back(1, c);
            ++i;
        } else {
            const size_t start = i;
            if (d[i] == '+' || d[i] == '-') ++i;
            while (i < d.size() && (std::isdigit(static_cast<unsigned char>(d[i])) || d[i] == '.')) ++i;
            if (i < d.size() && (d[i] == 'e' || d[i] == 'E')) {
                ++i;
                if (i < d.size() && (d[i] == '+' || d[i] == '-')) ++i;
                while (i < d.size() && std::isdigit(static_cast<unsigned char>(d[i]))) ++i;
            }
            if (i > start) toks.push_back(d.substr(start, i - start));
            else ++i; // stray character
        }
    }
    return toks;
}

std::vector<Contour2d> parsePathD(const std::string& d, const Mat3& mat) {
    const std::vector<std::string> toks = tokenizePath(d);
    std::vector<Contour2d> contours;
    Contour2d contour;
    std::array<double, 2> cur{0, 0}, start{0, 0};
    std::optional<std::array<double, 2>> lastCtrl;
    char cmd = 'M';
    size_t ti = 0;
    const auto nx = [&]() { return std::stod(toks.at(ti++)); };

    while (ti < toks.size()) {
        const std::string& t = toks[ti];
        if (t.size() == 1 && std::strchr("MmZzLlHhVvCcSsQqTtAa", t[0]) != nullptr) {
            cmd = t[0];
            ++ti;
            lastCtrl.reset();
            continue;
        }
        const bool rel = std::islower(static_cast<unsigned char>(cmd));
        const double ox = rel ? cur[0] : 0.0, oy = rel ? cur[1] : 0.0;
        const char lc = static_cast<char>(std::toupper(cmd));
        if (lc == 'M') {
            if (!contour.empty()) contours.push_back(contour);
            cur = {nx() + ox, nx() + oy};
            start = cur;
            contour = {applyMat(cur, mat)};
            cmd = rel ? 'l' : 'L';
        } else if (lc == 'Z') {
            if (!contour.empty()) contours.push_back(contour);
            cur = start;
            contour.clear();
        } else if (lc == 'L') {
            cur = {nx() + ox, nx() + oy};
            contour.push_back(applyMat(cur, mat));
        } else if (lc == 'H') {
            cur = {nx() + ox, cur[1]};
            contour.push_back(applyMat(cur, mat));
        } else if (lc == 'V') {
            cur = {cur[0], nx() + oy};
            contour.push_back(applyMat(cur, mat));
        } else if (lc == 'C') {
            const std::array<double, 2> p1{nx() + ox, nx() + oy}, p2{nx() + ox, nx() + oy}, p3{nx() + ox, nx() + oy};
            lastCtrl = p2;
            for (const auto& pt : cubicPts(cur, p1, p2, p3)) contour.push_back(applyMat(pt, mat));
            cur = p3;
        } else if (lc == 'S') {
            const std::array<double, 2> refl = lastCtrl ? std::array<double, 2>{2 * cur[0] - (*lastCtrl)[0], 2 * cur[1] - (*lastCtrl)[1]} : cur;
            const std::array<double, 2> p2{nx() + ox, nx() + oy}, p3{nx() + ox, nx() + oy};
            lastCtrl = p2;
            for (const auto& pt : cubicPts(cur, refl, p2, p3)) contour.push_back(applyMat(pt, mat));
            cur = p3;
        } else if (lc == 'Q') {
            const std::array<double, 2> p1{nx() + ox, nx() + oy}, p2{nx() + ox, nx() + oy};
            lastCtrl = p1;
            for (const auto& pt : quadPts(cur, p1, p2)) contour.push_back(applyMat(pt, mat));
            cur = p2;
        } else if (lc == 'T') {
            const std::array<double, 2> refl = lastCtrl ? std::array<double, 2>{2 * cur[0] - (*lastCtrl)[0], 2 * cur[1] - (*lastCtrl)[1]} : cur;
            const std::array<double, 2> p2{nx() + ox, nx() + oy};
            lastCtrl = refl;
            for (const auto& pt : quadPts(cur, refl, p2)) contour.push_back(applyMat(pt, mat));
            cur = p2;
        } else if (lc == 'A') {
            const double rx = nx(), ry = nx(), xrot = nx();
            const bool large = nx() != 0, sweep = nx() != 0;
            const double ex = nx() + ox, ey = nx() + oy;
            for (const auto& pt : arcPts(cur[0], cur[1], rx, ry, xrot, large, sweep, ex, ey)) contour.push_back(applyMat(pt, mat));
            cur = {ex, ey};
        }
    }
    if (!contour.empty()) contours.push_back(contour);
    return contours;
}

std::vector<Contour2d> shapeContours(const XmlNode& el, const Mat3& mat) {
    if (el.tag == "path") return parsePathD(el.getAttr("d"), mat);
    if (el.tag == "polygon" || el.tag == "polyline") {
        const std::vector<double> nums = parseNumberList(el.getAttr("points"));
        Contour2d pts;
        for (size_t i = 0; i + 1 < nums.size(); i += 2) pts.push_back(applyMat({nums[i], nums[i + 1]}, mat));
        return pts.empty() ? std::vector<Contour2d>{} : std::vector<Contour2d>{pts};
    }
    if (el.tag == "rect") {
        const double x = std::stod(el.getAttr("x", "0")), y = std::stod(el.getAttr("y", "0"));
        const double w = std::stod(el.getAttr("width", "0")), h = std::stod(el.getAttr("height", "0"));
        Contour2d pts = {applyMat({x, y}, mat), applyMat({x + w, y}, mat), applyMat({x + w, y + h}, mat), applyMat({x, y + h}, mat)};
        return {pts};
    }
    if (el.tag == "circle" || el.tag == "ellipse") {
        const double cx = std::stod(el.getAttr("cx", "0")), cy = std::stod(el.getAttr("cy", "0"));
        const double rx = el.tag == "circle" ? std::stod(el.getAttr("r", "0")) : std::stod(el.getAttr("rx", "0"));
        const double ry = el.tag == "circle" ? rx : std::stod(el.getAttr("ry", "0"));
        Contour2d pts;
        pts.reserve(kSegs);
        for (int i = 0; i < kSegs; ++i) {
            const double th = 2 * M_PI * i / kSegs;
            pts.push_back(applyMat({cx + rx * std::cos(th), cy + ry * std::sin(th)}, mat));
        }
        return {pts};
    }
    return {};
}

void walk(const XmlNode& el, const Mat3& mat, std::vector<Contour2d>& out) {
    if (el.tag == "defs" || el.tag == "symbol") return;
    const Mat3 m = compose(parseTransform(el.getAttr("transform")), mat);
    const std::vector<Contour2d> shapes = shapeContours(el, m);
    out.insert(out.end(), shapes.begin(), shapes.end());
    for (const XmlNode& child : el.children) walk(child, m, out);
}

} // namespace

std::vector<Contour2d> loadSvgContours(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open '" + path + "'");
    std::stringstream buf;
    buf << in.rdbuf();
    std::optional<XmlNode> root = parseXmlRoot(buf.str());
    if (!root) throw std::runtime_error("'" + path + "' is not a well-formed SVG file");

    std::vector<Contour2d> out;
    walk(*root, Mat3{}, out);
    return out;
}

} // namespace oscadeval
