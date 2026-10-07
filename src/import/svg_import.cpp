#include "openscad_cpp_evaluator/dxf_svg_import.hpp"

#include "../builtins/builtins.hpp"

#include <clipper2/clipper.h>
#include <manifold/cross_section.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>

// SVG import, as OpenSCAD 2026.02.01 reads it.
//
// What becomes geometry -- which elements, how a path's `d` is tokenized
// (quirks included), how curves are split, how strokes become outlines,
// which transforms apply, <use>, display:none and Inkscape layers -- is a
// port of OpenSCAD's src/libsvg, which is MIT licensed:
//
//   The MIT License
//
//   Copyright (c) 2016-2018, Torsten Paul <torsten.paul@gmx.de>,
//                            Marius Kintel <marius@kintel.net>
//
//   Permission is hereby granted, free of charge, to any person obtaining
//   a copy of this software and associated documentation files (the
//   "Software"), to deal in the Software without restriction, including
//   without limitation the rights to use, copy, modify, merge, publish,
//   distribute, sublicense, and/or sell copies of the Software, and to
//   permit persons to whom the Software is furnished to do so, subject to
//   the following conditions:
//
//   The above copyright notice and this permission notice shall be
//   included in all copies or substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//   LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//   OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//   WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Everything else -- page placement, which elements a filter selects, how
// shapes are filled and combined -- was worked out from the 2026.02.01
// binary's output, not its (GPL) source.
//
// The XML half is a minimal recursive-descent tree parser: this project has
// no XML dependency, and libsvg's own (libxml2) is not needed for what SVG
// files contain. Tag names keep their prefix, as libxml2 reports them, so
// <svg:rect> is not a rect -- in OpenSCAD either.

namespace oscadeval {

namespace {

// -- minimal XML tree -----------------------------------------------------

struct XmlNode {
    std::string tag;
    std::vector<std::pair<std::string, std::string>> attrs;
    std::vector<XmlNode> children;

    const std::string* findAttr(const std::string& name) const {
        for (const auto& [k, v] : attrs) {
            if (k == name) return &v;
        }
        return nullptr;
    }
    std::string getAttr(const std::string& name, const std::string& def = "") const {
        const std::string* v = findAttr(name);
        return v ? *v : def;
    }
};

void appendUtf8(std::string& out, unsigned long cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

// An attribute value as an XML parser hands it over: literal tabs and line
// breaks become spaces (attribute-value normalisation), then entity and
// character references are decoded -- so `&#10;` survives as a real line
// break, which matters, since libsvg's number parser rejects one.
std::string decodeAttrValue(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    static const std::pair<const char*, char> named[] = {
        {"&amp;", '&'}, {"&lt;", '<'}, {"&gt;", '>'}, {"&quot;", '"'}, {"&apos;", '\''}};
    for (size_t i = 0; i < s.size();) {
        if (s[i] == '&') {
            bool done = false;
            for (const auto& [ent, ch] : named) {
                if (s.compare(i, std::strlen(ent), ent) == 0) {
                    out += ch;
                    i += std::strlen(ent);
                    done = true;
                    break;
                }
            }
            if (done) continue;
            const size_t semi = s.find(';', i);
            if (s.compare(i, 2, "&#") == 0 && semi != std::string::npos) {
                const bool hex = i + 2 < s.size() && (s[i + 2] == 'x' || s[i + 2] == 'X');
                const std::string digits = s.substr(i + (hex ? 3 : 2), semi - i - (hex ? 3 : 2));
                char* end = nullptr;
                const unsigned long cp = std::strtoul(digits.c_str(), &end, hex ? 16 : 10);
                if (!digits.empty() && *end == '\0') {
                    appendUtf8(out, cp);
                    i = semi + 1;
                    continue;
                }
            }
        }
        const char c = s[i++];
        out += (c == '\t' || c == '\n' || c == '\r') ? ' ' : c;
    }
    return out;
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
    node.tag = s.substr(nameStart, i - nameStart);

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
        if (i == attrNameStart) {
            // A character no attribute can start with -- a '/' not followed by
            // '>', a stray '=' -- consumed nothing, and without this the loop
            // came straight back to it forever: a PDF (all "/Type /Catalog")
            // or a malformed `<a / b>` hung import() for good.
            ++i;
            continue;
        }
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
                attrValue = decodeAttrValue(s.substr(valStart, i - valStart));
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

// -- libsvg's number and attribute helpers -----------------------------------

// libsvg's parse_double: the whole string must be one number, else 0 --
// so "10px", " 5" and "5\n" are all 0.
double parseDouble(const std::string& s) {
    if (s.empty() || std::isspace(static_cast<unsigned char>(s[0]))) return 0.0;
    char* end = nullptr;
    const double d = std::strtod(s.c_str(), &end);
    return *end == '\0' ? d : 0.0;
}

// Splits on any of `drop` (discarded) and `keep` (each kept as a one-char
// token), dropping empty tokens: boost::char_separator, as libsvg uses it.
std::vector<std::string> splitTokens(const std::string& s, const char* drop, const char* keep = "") {
    std::vector<std::string> out;
    std::string cur;
    for (const char c : s) {
        const bool isKeep = std::strchr(keep, c) != nullptr;
        if (!isKeep && std::strchr(drop, c) == nullptr) {
            cur += c;
            continue;
        }
        if (!cur.empty()) out.push_back(std::move(cur));
        cur.clear();
        if (isKeep) out.emplace_back(1, c);
    }
    if (!cur.empty()) out.push_back(std::move(cur));
    return out;
}

std::string trim(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

// One property of a `style="a: b; c: d"` attribute, or "".
std::string styleProperty(const XmlNode& el, const std::string& name) {
    for (const std::string& decl : splitTokens(el.getAttr("style"), ";")) {
        const std::vector<std::string> kv = splitTokens(decl, ":");
        if (kv.size() == 2 && trim(kv[0]) == name) return trim(kv[1]);
    }
    return "";
}

// The attribute when present and non-empty, else the style property.
std::string presentation(const XmlNode& el, const std::string& name) {
    const std::string attr = el.getAttr(name);
    return attr.empty() ? styleProperty(el, name) : attr;
}

// -- 2D affine transform (SVG's own [[a,c,e],[b,d,f],[0,0,1]] convention) --

struct Mat3 {
    double a = 1, b = 0, c = 0, d = 1, e = 0, f = 0;
};

// p * q: q applies first.
Mat3 operator*(const Mat3& p, const Mat3& q) {
    return Mat3{p.a * q.a + p.c * q.b, p.b * q.a + p.d * q.b, p.a * q.c + p.c * q.d,
                p.b * q.c + p.d * q.d, p.a * q.e + p.c * q.f + p.e, p.b * q.e + p.d * q.f + p.f};
}

std::array<double, 2> applyMat(const Mat3& m, const std::array<double, 2>& pt) {
    return {m.a * pt[0] + m.c * pt[1] + m.e, m.b * pt[0] + m.d * pt[1] + m.f};
}

// A `transform` attribute, as libsvg reads it: each operation with the
// wrong number of arguments is skipped (it printed a complaint to stdout).
Mat3 parseTransform(const std::string& text) {
    Mat3 acc;
    std::string name;
    std::vector<double> args;
    const auto flush = [&]() {
        const size_t n = args.size();
        Mat3 m;
        if (name == "matrix" && n == 6) {
            m = Mat3{args[0], args[1], args[2], args[3], args[4], args[5]};
        } else if (name == "translate" && (n == 1 || n == 2)) {
            m = Mat3{1, 0, 0, 1, args[0], n == 2 ? args[1] : 0.0};
        } else if (name == "scale" && (n == 1 || n == 2)) {
            m = Mat3{args[0], 0, 0, n == 2 ? args[1] : args[0], 0, 0};
        } else if (name == "rotate" && (n == 1 || n == 3)) {
            const double s = sinDeg(args[0]), c = cosDeg(args[0]);
            const double cx = n == 3 ? args[1] : 0.0, cy = n == 3 ? args[2] : 0.0;
            m = Mat3{1, 0, 0, 1, cx, cy} * Mat3{c, s, -s, c, 0, 0} * Mat3{1, 0, 0, 1, -cx, -cy};
        } else if (name == "skewX" && n == 1) {
            m = Mat3{1, 0, tanDeg(args[0]), 1, 0, 0};
        } else if (name == "skewY" && n == 1) {
            m = Mat3{1, tanDeg(args[0]), 0, 1, 0, 0};
        }
        acc = acc * m;
    };
    for (const std::string& tok : splitTokens(text, " ,()")) {
        if (tok == "matrix" || tok == "translate" || tok == "scale" || tok == "rotate" || tok == "skewX" ||
            tok == "skewY") {
            if (!name.empty()) flush();
            name = tok;
            args.clear();
        } else if (!name.empty()) {
            args.push_back(parseDouble(tok));
        }
    }
    if (!name.empty()) flush();
    return acc;
}

// -- shapes -> outlines (libsvg's shape/path/rect/... set_attrs) -------------

using Path = std::vector<std::array<double, 2>>;
using PathList = std::vector<Path>;

struct Stroke {
    double width;
    Clipper2Lib::JoinType join;
    Clipper2Lib::EndType cap;
};

Stroke strokeOf(const XmlNode& el) {
    const double w = parseDouble(presentation(el, "stroke-width"));
    const std::string cap = presentation(el, "stroke-linecap");
    const std::string join = presentation(el, "stroke-linejoin");
    using Clipper2Lib::EndType;
    using Clipper2Lib::JoinType;
    return Stroke{w < 0.01 ? 1.0 : w,
                  // bevel -> Square is libsvg's choice, kept.
                  join == "bevel" ? JoinType::Square : join == "round" ? JoinType::Round : JoinType::Miter,
                  cap == "round" ? EndType::Round : cap == "square" ? EndType::Square : EndType::Butt};
}

// An open line becomes its stroke's outline. Clipper2 works in integers,
// at 2^27 per user unit as the reference does (measured: an offset
// amplified 1e8 times lands on that grid). It is not cosmetic -- it
// decides which nearly-collinear outline vertices Clipper drops.
void strokeOutline(PathList& out, const Path& line, const Stroke& stroke) {
    const double scale = std::ldexp(1.0, 27);
    Clipper2Lib::Path64 in;
    for (const auto& p : line) in.emplace_back(p[0] * scale, p[1] * scale);
    Clipper2Lib::ClipperOffset co;
    co.AddPath(in, stroke.join, stroke.cap);
    Clipper2Lib::Paths64 result;
    co.Execute(stroke.width * scale / 2, result);
    for (const auto& r : result) {
        Path p;
        for (const auto& pt : r) p.push_back({pt.x / scale, pt.y / scale});
        out.push_back(std::move(p));
    }
}

// SVG arc implementation notes F.6.5, sampled as libsvg does.
void arcTo(Path& path, double x1, double y1, double rx, double ry, double x2, double y2, double angle, bool large,
           bool sweep, const Discretizer& disc) {
    // Out-of-range parameters, as the SVG spec has them. libsvg divides by
    // zero here and draws NaN vertices (which come out at the origin).
    if (x1 == x2 && y1 == y2) return;
    if (rx == 0 || ry == 0) {
        path.push_back({x2, y2});
        return;
    }
    const double cosR = cosDeg(angle), sinR = sinDeg(angle);
    const double dx = (x1 - x2) / 2, dy = (y1 - y2) / 2;
    const double x1_ = cosR * dx + sinR * dy, y1_ = -sinR * dx + cosR * dy;
    const double d = (x1_ * x1_) / (rx * rx) + (y1_ * y1_) / (ry * ry);
    if (d > 1) {
        rx = std::fabs(std::sqrt(d) * rx);
        ry = std::fabs(std::sqrt(d) * ry);
    }
    const double t1 = std::max(0.0, rx * rx * ry * ry - rx * rx * y1_ * y1_ - ry * ry * x1_ * x1_);
    const double t2 = rx * rx * y1_ * y1_ + ry * ry * x1_ * x1_;
    double t3 = std::sqrt(t1 / t2);
    if (large == sweep) t3 = -t3;
    const double cx_ = t3 * rx * y1_ / ry, cy_ = t3 * -ry * x1_ / rx;
    const double cx = cosR * cx_ - sinR * cy_ + (x1 + x2) / 2.0;
    const double cy = sinR * cx_ + cosR * cy_ + (y1 + y2) / 2.0;
    const auto vectorAngle = [](double ux, double uy, double vx, double vy) {
        const double a = atan2Deg(vy, vx) - atan2Deg(uy, ux);
        return a < 0 ? a + 360 : a;
    };
    const double ux = (x1_ - cx_) / rx, uy = (y1_ - cy_) / ry, vx = (-x1_ - cx_) / rx, vy = (-y1_ - cy_) / ry;
    const double theta = vectorAngle(1, 0, ux, uy);
    double delta = vectorAngle(ux, uy, vx, vy);
    if (!sweep) delta -= 360;
    const unsigned fn = static_cast<unsigned>(disc.circular(std::max(rx, ry), delta).value_or(3));
    const unsigned steps = std::max(fn, static_cast<unsigned>(std::fabs(delta) * 10.0 / 180 + 4));
    for (unsigned a = 0; a <= steps; ++a) {
        const double phi = theta + delta * a / steps;
        const double xx = cosR * cosDeg(phi) * rx - sinR * sinDeg(phi) * ry;
        const double yy = sinR * cosDeg(phi) * rx + cosR * sinDeg(phi) * ry;
        path.push_back({xx + cx, yy + cy});
    }
}

// Beziers take $fn segments, never fewer than 20; $fa/$fs play no part.
int bezierSegments(const Discretizer& disc) {
    return std::isfinite(disc.fn) && disc.fn < 1e6 ? std::max(static_cast<int>(disc.fn), 20) : 20;
}

void quadTo(Path& path, double x, double y, double cx1, double cy1, double x2, double y2, const Discretizer& disc) {
    const int fn = bezierSegments(disc);
    for (int idx = 1; idx <= fn; ++idx) {
        const double a = idx * (1.0 / fn), m = 1.0 - a;
        path.push_back({x * m * m + cx1 * 2 * m * a + x2 * a * a, y * m * m + cy1 * 2 * m * a + y2 * a * a});
    }
}

void cubicTo(Path& path, double x, double y, double cx1, double cy1, double cx2, double cy2, double x2, double y2,
             const Discretizer& disc) {
    const int fn = bezierSegments(disc);
    for (int idx = 1; idx <= fn; ++idx) {
        const double a = idx * (1.0 / fn), m = 1.0 - a;
        path.push_back({x * m * m * m + cx1 * 3 * m * m * a + cx2 * 3 * m * a * a + x2 * a * a * a,
                        y * m * m * m + cy1 * 3 * m * m * a + cy2 * 3 * m * a * a + y2 * a * a * a});
    }
}

bool isOpen(const Path& p) {
    return std::hypot(p.front()[0] - p.back()[0], p.front()[1] - p.back()[1]) > 0.1;
}

// "1.5.5" is two numbers, 1.5 and .5.
std::vector<std::string> splitDots(const std::string& str) {
    if (std::count(str.begin(), str.end(), '.') < 2) return {str};
    std::vector<std::string> result;
    std::string text;
    bool dotSeen = false;
    for (const std::string& token : splitTokens(str, "", ".")) {
        text += token;
        if (token == ".") {
            dotSeen = true;
        } else if (dotSeen) {
            result.push_back(text);
            text.clear();
        }
    }
    return result;
}

// A path's `d`, tokenized and interpreted exactly as libsvg does -- a '-'
// is its own token negating the next number, "1e" waits for its exponent,
// and every subpath left open is replaced by its stroke's outline (unless
// the path closed any subpath and this one is its last).
PathList parsePathData(const std::string& data, const Stroke& stroke, const Discretizer& disc) {
    static const char* commands = "-zmlcqahvstZMLCQAHVST";
    std::vector<std::string> tokens;
    for (const std::string& tok : splitTokens(data, " ,", commands)) {
        for (std::string& part : splitDots(tok)) tokens.push_back(std::move(part));
    }

    PathList list(1);
    double x = 0, y = 0, xx = 0, yy = 0, rx = 0, ry = 0, cx1 = 0, cy1 = 0, cx2 = 0, cy2 = 0, angle = 0;
    bool large = false, sweep = false, lastCubic = false, lastQuad = false, negate = false, closed = false;
    char cmd = ' ';
    int point = 0;
    std::string preExp;
    const auto endSegment = [&](bool cubic, bool quad) {
        point = -1;
        lastCubic = cubic;
        lastQuad = quad;
    };
    for (const std::string& v : tokens) {
        double p = 0;
        if (v.size() == 1 && std::strchr(commands, v[0]) != nullptr) {
            if (v[0] == '-') {
                negate = true;
                continue;
            }
            point = -1;
            cmd = v[0];
        } else {
            if (std::tolower(static_cast<unsigned char>(v.back())) == 'e') {
                preExp = negate ? "-" + v : v;
                negate = false;
                continue;
            }
            if (preExp.empty()) {
                p = parseDouble(v);
                if (negate) p = -p;
            } else {
                p = parseDouble(preExp + (negate ? "-" : "") + v);
                preExp.clear();
            }
            negate = false;
        }
        const bool rel = std::islower(static_cast<unsigned char>(cmd));
        switch (std::toupper(static_cast<unsigned char>(cmd))) {
        case 'A':
            switch (point) {
            case 0: rx = std::fabs(p); break;
            case 1: ry = std::fabs(p); break;
            case 2: angle = p; break;
            case 3: large = p > 0.5; break;
            case 4: sweep = p > 0.5; break;
            case 5: xx = rel ? x + p : p; break;
            case 6:
                yy = rel ? y + p : p;
                arcTo(list.back(), x, y, rx, ry, xx, yy, angle, large, sweep, disc);
                x = xx;
                y = yy;
                endSegment(false, false);
                break;
            }
            break;
        case 'L':
            switch (point) {
            case 0: xx = rel ? x + p : p; break;
            case 1:
                yy = rel ? y + p : p;
                list.back().push_back({xx, yy});
                x = xx;
                y = yy;
                endSegment(false, false);
                break;
            }
            break;
        case 'C':
            switch (point) {
            case 0: cx1 = p; break;
            case 1: cy1 = p; break;
            case 2: cx2 = p; break;
            case 3: cy2 = p; break;
            case 4: xx = rel ? x + p : p; break;
            case 5:
                yy = rel ? y + p : p;
                if (rel) {
                    cx1 += x;
                    cy1 += y;
                    cx2 += x;
                    cy2 += y;
                }
                cubicTo(list.back(), x, y, cx1, cy1, cx2, cy2, xx, yy, disc);
                x = xx;
                y = yy;
                endSegment(true, false);
                break;
            }
            break;
        case 'S':
            switch (point) {
            case 0:
                cx1 = lastCubic ? 2 * x - cx2 : x;
                cy1 = lastCubic ? 2 * y - cy2 : y;
                cx2 = p;
                break;
            case 1: cy2 = p; break;
            case 2: xx = rel ? x + p : p; break;
            case 3:
                yy = rel ? y + p : p;
                if (rel) {
                    cx2 += x;
                    cy2 += y;
                }
                cubicTo(list.back(), x, y, cx1, cy1, cx2, cy2, xx, yy, disc);
                x = xx;
                y = yy;
                endSegment(true, false);
                break;
            }
            break;
        case 'Q':
            switch (point) {
            case 0: cx1 = p; break;
            case 1: cy1 = p; break;
            case 2: xx = rel ? x + p : p; break;
            case 3:
                yy = rel ? y + p : p;
                if (rel) {
                    cx1 += x;
                    cy1 += y;
                }
                quadTo(list.back(), x, y, cx1, cy1, xx, yy, disc);
                x = xx;
                y = yy;
                endSegment(false, true);
                break;
            }
            break;
        case 'T':
            switch (point) {
            case 0:
                cx1 = lastQuad ? 2 * x - cx1 : x;
                cy1 = lastQuad ? 2 * y - cy1 : y;
                xx = rel ? x + p : p;
                break;
            case 1:
                yy = rel ? y + p : p;
                quadTo(list.back(), x, y, cx1, cy1, xx, yy, disc);
                x = xx;
                y = yy;
                endSegment(false, true);
                break;
            }
            break;
        case 'M':
            switch (point) {
            case 0: xx = rel ? x + p : p; break;
            case 1:
                yy = rel ? y + p : p;
                cmd = rel ? 'l' : 'L';
                if (!list.back().empty()) {
                    if (isOpen(list.back())) {
                        const Path open = std::move(list.back());
                        list.pop_back();
                        strokeOutline(list, open, stroke);
                    }
                    list.emplace_back();
                }
                list.back().push_back({xx, yy});
                x = xx;
                y = yy;
                endSegment(false, false);
                break;
            }
            break;
        case 'V':
            if (point == 0) {
                y = rel ? y + p : p;
                list.back().push_back({x, y});
                endSegment(false, false);
            }
            break;
        case 'H':
            if (point == 0) {
                x = rel ? x + p : p;
                list.back().push_back({x, y});
                endSegment(false, false);
            }
            break;
        case 'Z':
            if (!list.back().empty()) {
                const auto first = list.back().front();
                list.back().push_back(first);
                x = first[0];
                y = first[1];
            }
            list.emplace_back();
            closed = true;
            lastCubic = lastQuad = false;
            break;
        }
        ++point;
    }

    while (!list.empty() && list.back().empty()) list.pop_back();
    if (!closed && !list.empty() && isOpen(list.back())) {
        const Path open = std::move(list.back());
        list.pop_back();
        strokeOutline(list, open, stroke);
    }
    return list;
}

// "x1,y1 x2,y2 ..." -- an odd number out is dropped.
Path parsePoints(const std::string& text) {
    Path path;
    const std::vector<std::string> toks = splitTokens(text, " ,");
    for (size_t i = 0; i + 1 < toks.size(); i += 2) path.push_back({parseDouble(toks[i]), parseDouble(toks[i + 1])});
    return path;
}

// The outlines one element draws, in its own coordinates.
PathList elementPaths(const XmlNode& el, const Discretizer& disc) {
    const auto num = [&](const char* name) { return parseDouble(el.getAttr(name)); };
    const std::string& tag = el.tag;
    PathList list;
    if (tag == "path") return parsePathData(el.getAttr("d"), strokeOf(el), disc);
    if (tag == "circle" || tag == "ellipse") {
        const double cx = num("cx"), cy = num("cy");
        const double rx = tag == "circle" ? num("r") : num("rx"), ry = tag == "circle" ? rx : num("ry");
        const unsigned long fn = std::max(disc.circular(std::max(rx, ry)).value_or(3), 40);
        Path p;
        for (unsigned long i = 1; i <= fn; ++i) {
            const double a = i * 360.0 / fn;
            p.push_back({rx * sinDeg(a) + cx, ry * cosDeg(a) + cy});
        }
        list.push_back(std::move(p));
    } else if (tag == "line") {
        strokeOutline(list, Path{{num("x1"), num("y1")}, {num("x2"), num("y2")}}, strokeOf(el));
    } else if (tag == "polyline") {
        strokeOutline(list, parsePoints(el.getAttr("points")), strokeOf(el));
    } else if (tag == "polygon") {
        Path p = parsePoints(el.getAttr("points"));
        if (!p.empty()) p.push_back(p.front());
        list.push_back(std::move(p));
    } else if (tag == "rect") {
        const double x = num("x"), y = num("y"), w = num("width"), h = num("height");
        double rx = num("rx"), ry = num("ry");
        const bool hasRx = std::fabs(rx) >= 1e-8, hasRy = std::fabs(ry) >= 1e-8;
        if (!hasRx && !hasRy) {
            list.push_back({{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}, {x, y}});
            return list;
        }
        if (!hasRx) rx = ry;
        if (!hasRy) ry = rx;
        rx = std::min(rx, w / 2);
        ry = std::min(ry, h / 2);
        // A rounded rect is drawn as path data, its numbers printed at the
        // stream's default 6 significant digits -- which libsvg does too.
        std::ostringstream d;
        d << "M " << x + rx << "," << y << " H " << x + w - rx << " A " << rx << "," << ry << " 0 0,1 " << x + w << ","
          << y + ry << " V " << y + h - ry << " A " << rx << "," << ry << " 0 0,1 " << x + w - rx << "," << y + h
          << " H " << x + rx << " A " << rx << "," << ry << " 0 0,1 " << x << "," << y + h - ry << " V " << y + ry
          << " A " << rx << "," << ry << " 0 0,1 " << x + rx << "," << y << " z";
        return parsePathData(d.str(), strokeOf(el), disc);
    }
    return list;
}

// -- the document walk ----------------------------------------------------

// `class` is a space-separated LIST, so this is membership rather than
// equality: class="cut outline" is matched by class="cut".
bool hasClass(const XmlNode& el, const std::string& want) {
    for (const std::string& c : splitTokens(el.getAttr("class"), " \t\r\n")) {
        if (c == want) return true;
    }
    return false;
}

bool isShapeTag(const std::string& tag) {
    static const char* tags[] = {"svg",     "g",        "path", "rect", "circle", "ellipse",
                                 "line",    "polyline", "polygon", "use", "text", "tspan"};
    return std::any_of(std::begin(tags), std::end(tags), [&](const char* t) { return tag == t; });
}

struct Walk {
    const Discretizer& disc;
    const SvgFilter& filter;
    std::map<std::string, const XmlNode*> defs;   // <defs> shapes by id, as seen so far
    std::vector<PathList> shapes;                 // one entry per drawn element
    bool matched = false;

    enum class Mode { Normal, Defs, Clone };

    // Which elements a filter selects (read off the reference): with id
    // (or class), those elements, inside the named layer when one is
    // given too; with layer alone, just the layer's own group.
    bool selects(const XmlNode& el, bool isPage, const std::vector<std::string>& layers) const {
        if (isPage) return false;  // libsvg reads no id or layer for <svg>
        if (filter.id || filter.cls) {
            const bool hit = (filter.id && el.getAttr("id") == *filter.id && el.findAttr("id")) ||
                             (filter.cls && hasClass(el, *filter.cls));
            return hit && (!filter.layer || std::find(layers.begin(), layers.end(), *filter.layer) != layers.end());
        }
        return filter.layer && !layers.empty() && layers.back() == *filter.layer && ownLayer(el);
    }

    static bool ownLayer(const XmlNode& el) {
        return el.getAttr("inkscape:groupmode") == "layer" && el.findAttr("inkscape:label");
    }

    // `included` is libsvg's is_excluded() walked downwards: the nearest
    // selected or display:none ancestor-or-self decides, and with neither,
    // a filtered import leaves the element out and an unfiltered one keeps it.
    void walk(const XmlNode& el, const Mat3& mat, bool included, std::vector<std::string> layers, Mode mode) {
        if (el.tag == "defs") mode = Mode::Defs;
        if (!isShapeTag(el.tag)) {  // not a shape: transparent, transform and all
            for (const XmlNode& child : el.children) walk(child, mat, included, layers, mode);
            return;
        }
        const bool isPage = el.tag == "svg";
        if (!isPage && ownLayer(el)) layers.push_back(el.getAttr("inkscape:label"));
        const bool selected = selects(el, isPage, layers);
        matched = matched || selected;
        std::string display = styleProperty(el, "display");
        if (display.empty()) display = el.getAttr("display");
        const bool hidden = !isPage && display == "none";
        const bool inChain = selected ? true : hidden ? false : included;
        Mat3 m = isPage ? mat : mat * parseTransform(el.getAttr("transform"));

        if (mode == Mode::Defs) {
            if (const std::string* id = el.findAttr("id"); id && !isPage) defs.emplace(*id, &el);
        } else if (inChain) {
            PathList paths = elementPaths(el, disc);
            for (Path& p : paths)
                for (auto& pt : p) pt = applyMat(m, pt);
            if (!paths.empty()) shapes.push_back(std::move(paths));
        }
        if (el.tag == "use" && mode == Mode::Normal) {
            // Only a shape in a <defs> already read can be used, and its
            // copy is placed by the <use>'s own transform and then x/y --
            // which libsvg prints into the transform at 6 digits.
            std::string href = el.getAttr("href");
            if (href.empty()) href = el.getAttr("xlink:href");
            const auto it = href.rfind('#', 0) == 0 ? defs.find(href.substr(1)) : defs.end();
            if (it != defs.end()) {
                std::ostringstream t;
                t << el.getAttr("transform") << " translate(" << parseDouble(el.getAttr("x")) << ","
                  << parseDouble(el.getAttr("y")) << ")";
                walk(*it->second, mat * parseTransform(t.str()), inChain, layers, Mode::Clone);
            }
        }
        const bool container = isPage || el.tag == "g" || el.tag == "text" || el.tag == "tspan";
        for (const XmlNode& child : el.children) {
            if (container) walk(child, m, inChain, layers, mode);
            else walk(child, mat, included, layers, mode);
        }
    }
};

// -- page placement -----------------------------------------------------------

// A page length (the root's width or height): a number with an optional
// unit, whitespace allowed around either. Anything else is no length.
struct PageLength {
    double n;
    std::string unit;  // "", "px", "pt", "pc", "in", "cm", "mm", "%", "em" or "ex"
};

std::optional<PageLength> parsePageLength(const std::string& text) {
    const std::string s = trim(text);
    if (s.empty()) return std::nullopt;
    char* end = nullptr;
    const double n = std::strtod(s.c_str(), &end);
    if (end == s.c_str()) return std::nullopt;
    const std::string unit = trim(end);
    static const char* units[] = {"", "px", "pt", "pc", "in", "cm", "mm", "%", "em", "ex"};
    if (std::none_of(std::begin(units), std::end(units), [&](const char* u) { return unit == u; }))
        return std::nullopt;
    return PageLength{n, unit};
}

// A page length in millimetres. `vbSize` is the viewBox's size along the
// same axis, when there is a viewBox.
double pageLengthMm(const std::optional<PageLength>& len, std::optional<double> vbSize, double dpi) {
    if (!len) return vbSize ? 25.4 * *vbSize / dpi : 0.0;
    const double n = len->n;
    const std::string& u = len->unit;
    if (u.empty()) return 25.4 * n / dpi;
    if (u == "px") return 25.4 * n / 96.0;
    if (u == "pt") return 25.4 * n / 72.0;
    if (u == "pc") return 25.4 * n / 6.0;
    if (u == "in") return 25.4 * n;
    if (u == "cm") return 10.0 * n;
    if (u == "mm") return n;
    if (u == "%") return vbSize ? 25.4 * (n / 100.0) * *vbSize / dpi : 0.0;
    return vbSize ? *vbSize : 0.0;  // em, ex: the viewBox size, whatever the number
}

// viewBox="x y w h", separated by spaces and/or commas; a negative size or
// anything but four numbers is no viewBox.
std::optional<std::array<double, 4>> parseViewBox(const std::string& text) {
    const std::vector<std::string> tok = splitTokens(text, " ,");
    if (tok.size() != 4) return std::nullopt;
    std::array<double, 4> vb{};
    for (int i = 0; i < 4; ++i) {
        char* end = nullptr;
        vb[i] = std::strtod(tok[i].c_str(), &end);
        if (end == tok[i].c_str() || *end != '\0') return std::nullopt;
    }
    if (vb[2] < 0 || vb[3] < 0) return std::nullopt;
    return vb;
}

// Maps the parsed shapes from SVG user units to millimetres, Y up, placing
// them on the page the root <svg> element describes.
void pageMap(const XmlNode& root, double dpi, bool center, std::vector<PathList>& shapes) {
    const auto vb = parseViewBox(root.getAttr("viewBox"));
    const auto width = parsePageLength(root.getAttr("width"));
    const auto height = parsePageLength(root.getAttr("height"));
    const double pageW = pageLengthMm(width, vb ? std::optional<double>((*vb)[2]) : std::nullopt, dpi);
    const double pageH = pageLengthMm(height, vb ? std::optional<double>((*vb)[3]) : std::nullopt, dpi);

    // Without a viewBox one user unit is one millimetre.
    double sx = 1, sy = 1, x0 = 0, y0 = 0, ax = 0, ay = 0;
    if (vb) {
        const auto [vx, vy, vw, vh] = *vb;
        // A percentage page length scales that axis's viewBox origin too.
        x0 = width && width->unit == "%" ? vx * width->n / 100.0 : vx;
        y0 = height && height->unit == "%" ? vy * height->n / 100.0 : vy;
        sx = pageW / vw;  // a zero size gives +inf, which min() below discards
        sy = pageH / vh;

        // preserveAspectRatio="[defer] <align> [meet|slice]", where <align> is
        // none or x<Min|Mid|Max>Y<Min|Mid|Max>. Anything else, in any part,
        // is the default xMidYMid meet.
        auto fraction = [](const std::string& f) {
            return f == "Min" ? 0.0 : f == "Mid" ? 0.5 : f == "Max" ? 1.0 : -1.0;
        };
        std::vector<std::string> par = splitTokens(root.getAttr("preserveAspectRatio"), " ");
        if (!par.empty() && par[0] == "defer") par.erase(par.begin());
        std::string align = par.empty() ? "xMidYMid" : par[0];
        bool slice = par.size() == 2 && par[1] == "slice";
        const bool wellFormed =
            par.size() <= 2 && (par.size() < 2 || par[1] == "meet" || slice) &&
            (align == "none" || (align.size() == 8 && align[0] == 'x' && align[4] == 'Y' &&
                                 fraction(align.substr(1, 3)) >= 0 && fraction(align.substr(5, 3)) >= 0));
        if (!wellFormed) {
            align = "xMidYMid";
            slice = false;
        }
        if (align != "none") {
            const double s = slice ? std::max(sx, sy) : std::min(sx, sy);
            sx = sy = s;
            const double fx = fraction(align.substr(1, 3));
            const double fy = fraction(align.substr(5, 3));
            ax = fx * (pageW - s * vw);
            ay = fy * (pageH - s * vh);
        }
    }

    if (!center) {
        // Y flips about the page. The viewBox's Y origin is added, not
        // subtracted, as the reference places it.
        for (PathList& s : shapes)
            for (Path& p : s)
                for (auto& pt : p) pt = {sx * (pt[0] - x0) + ax, pageH - ay - sy * (pt[1] + y0)};
        return;
    }
    // Centred: the centre of the scaled drawing's bounding box (origin
    // offset not included) goes to the origin; page height and alignment
    // play no part.
    double lo[2] = {INFINITY, INFINITY}, hi[2] = {-INFINITY, -INFINITY};
    for (const PathList& s : shapes)
        for (const Path& p : s)
            for (const auto& pt : p) {
                const double q[2] = {sx * pt[0], sy * pt[1]};
                for (int a = 0; a < 2; ++a) {
                    lo[a] = std::min(lo[a], q[a]);
                    hi[a] = std::max(hi[a], q[a]);
                }
            }
    const double cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2;
    for (PathList& s : shapes)
        for (Path& p : s)
            for (auto& pt : p) pt = {sx * (pt[0] - x0) - cx, cy - sy * (pt[1] + y0)};
}

// Each element fills even-odd on its own -- a path's overlapping subpaths
// cancel -- and the elements then union, as the reference fills them.
std::vector<Contour2d> fill(const std::vector<PathList>& shapes) {
    std::vector<manifold::CrossSection> parts;
    for (const PathList& s : shapes) {
        manifold::Polygons polys;
        for (const Path& c : s) {
            manifold::SimplePolygon poly;
            for (const auto& p : c) poly.push_back({p[0], p[1]});
            polys.push_back(std::move(poly));
        }
        parts.emplace_back(polys, manifold::CrossSection::FillRule::EvenOdd);
    }
    std::vector<Contour2d> out;
    for (const manifold::SimplePolygon& poly :
         manifold::CrossSection::BatchBoolean(parts, manifold::OpType::Add).ToPolygons()) {
        Contour2d c;
        for (const auto& p : poly) c.push_back({p.x, p.y});
        out.push_back(std::move(c));
    }
    return out;
}

} // namespace

std::vector<Contour2d> loadSvgContours(const std::string& path, const SvgFilter& filter, bool* matched, double dpi,
                                       bool center, const Discretizer& disc) {
    if (!(dpi > 0.0)) dpi = 72.0;
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open '" + path + "'");
    std::stringstream buf;
    buf << in.rdbuf();
    std::optional<XmlNode> root = parseXmlRoot(buf.str());
    if (!root) throw std::runtime_error("'" + path + "' is not a well-formed SVG file");

    const bool filtered = filter.id || filter.cls || filter.layer;
    Walk w{disc, filter, {}, {}, false};
    w.walk(*root, Mat3{}, !filtered, {}, Walk::Mode::Normal);
    // A filter that matched nothing imports nothing -- it does NOT fall
    // back to the whole drawing. The caller turns this into the warning.
    if (matched) *matched = !filtered || w.matched;
    pageMap(*root, dpi, center, w.shapes);
    return fill(w.shapes);
}

} // namespace oscadeval
