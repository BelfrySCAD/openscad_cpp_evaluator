// import() for AMF, X3D and VRML97 -- the three multi-object mesh formats
// export writes that nothing could read back.
//
// Geometry only, as every other mesh importer here (colour is not carried
// through import()). Each format's scene structure IS honoured, though:
// X3D/VRML Transforms (translation, rotation, scale, center,
// scaleOrientation) nest and apply, DEF/USE reuse resolves, `ccw FALSE`
// flips winding, and polygons are fan-triangulated. AMF's `unit` scales to
// millimetres, and a zipped AMF is read like 3MF.
//
// What is skipped says so, through LoadedMesh::warnings: X3D/VRML
// primitives (Box, Sphere, ...), Inline references, and AMF constellations.
// None of them is a mesh, and inventing one would be a different model from
// the file.
#include "openscad_cpp_evaluator/mesh_import.hpp"

#include "openscad_cpp_evaluator/zip_stored.hpp"

#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace oscadeval {

namespace {

std::string readText(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("could not open '" + path + "'");
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

// Numbers separated by whitespace and/or commas -- how every numeric list in
// X3D attributes and VRML fields is written.
std::vector<double> parseNumbers(std::string_view text) {
    std::vector<double> out;
    std::string buf(text);
    for (char& c : buf) {
        if (c == ',') c = ' ';
    }
    const char* p = buf.c_str();
    char* end = nullptr;
    while (*p) {
        const double v = std::strtod(p, &end);
        if (end == p) {
            ++p;  // not a number here: skip one character and keep scanning
            continue;
        }
        out.push_back(v);
        p = end;
    }
    return out;
}

// -- 4x4 affine transforms ----------------------------------------------------

using Mat = std::array<std::array<double, 4>, 4>;

Mat identity() {
    Mat m{};
    for (int i = 0; i < 4; ++i) m[i][i] = 1;
    return m;
}

Mat mul(const Mat& a, const Mat& b) {
    Mat m{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k) m[i][j] += a[i][k] * b[k][j];
    return m;
}

Mat translate(double x, double y, double z) {
    Mat m = identity();
    m[0][3] = x;
    m[1][3] = y;
    m[2][3] = z;
    return m;
}

Mat scale(double x, double y, double z) {
    Mat m = identity();
    m[0][0] = x;
    m[1][1] = y;
    m[2][2] = z;
    return m;
}

// X3D/VRML rotation: axis x y z, angle in radians (right-hand rule).
Mat rotate(double x, double y, double z, double angle) {
    const double len = std::sqrt(x * x + y * y + z * z);
    if (len == 0 || angle == 0) return identity();
    x /= len;
    y /= len;
    z /= len;
    const double c = std::cos(angle), s = std::sin(angle), t = 1 - c;
    Mat m = identity();
    m[0] = {t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0};
    m[1] = {t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0};
    m[2] = {t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0};
    return m;
}

std::array<double, 3> apply(const Mat& m, double x, double y, double z) {
    return {m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3], m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
            m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]};
}

// The X3D/VRML Transform node:
//   P' = T * C * R * SR * S * -SR * -C * P
// with each field defaulting to the identity when absent.
Mat transformNode(const std::vector<double>& t, const std::vector<double>& r, const std::vector<double>& s,
                  const std::vector<double>& c, const std::vector<double>& sr) {
    const auto at = [](const std::vector<double>& v, size_t i, double dflt) { return i < v.size() ? v[i] : dflt; };
    const Mat T = translate(at(t, 0, 0), at(t, 1, 0), at(t, 2, 0));
    const Mat C = translate(at(c, 0, 0), at(c, 1, 0), at(c, 2, 0));
    const Mat Ci = translate(-at(c, 0, 0), -at(c, 1, 0), -at(c, 2, 0));
    const Mat R = rotate(at(r, 0, 0), at(r, 1, 0), at(r, 2, 1), at(r, 3, 0));
    const Mat SR = rotate(at(sr, 0, 0), at(sr, 1, 0), at(sr, 2, 1), at(sr, 3, 0));
    const Mat SRi = rotate(at(sr, 0, 0), at(sr, 1, 0), at(sr, 2, 1), -at(sr, 3, 0));
    const Mat S = scale(at(s, 0, 1), at(s, 1, 1), at(s, 2, 1));
    return mul(T, mul(C, mul(R, mul(SR, mul(S, mul(SRi, Ci))))));
}

// -- geometry emission, shared by X3D and VRML ---------------------------------

// Appends `points` (xyz triples, transformed by `m`) and the polygons of
// `index` (-1 separated; a trailing polygon may omit its -1) to `out`,
// fan-triangulated. `ccw false` flips each polygon's winding.
void emitIndexedFaces(LoadedMesh& out, const std::vector<double>& points, const std::vector<double>& index,
                      const Mat& m, bool ccw, const std::string& what) {
    const int n = static_cast<int>(points.size() / 3);
    const int base = static_cast<int>(out.verts.size());
    for (int i = 0; i < n; ++i) out.verts.push_back(apply(m, points[i * 3], points[i * 3 + 1], points[i * 3 + 2]));
    std::vector<int> poly;
    const auto flush = [&]() {
        for (size_t k = 1; k + 1 < poly.size(); ++k) {
            std::array<int, 3> tri{base + poly[0], base + poly[k], base + poly[k + 1]};
            if (!ccw) std::swap(tri[1], tri[2]);
            out.tris.push_back(tri);
        }
        poly.clear();
    };
    for (double d : index) {
        const int i = static_cast<int>(d);
        if (i < 0) {
            flush();
            continue;
        }
        if (i >= n) throw std::runtime_error(what + ": face index " + std::to_string(i) + " is past its " +
                                             std::to_string(n) + " points");
        poly.push_back(i);
    }
    flush();
}

// IndexedTriangleSet / TriangleSet: every three indices (or points) are one
// triangle, no -1 separators.
void emitTriangles(LoadedMesh& out, const std::vector<double>& points, const std::vector<double>* index,
                   const Mat& m, bool ccw, const std::string& what) {
    std::vector<double> faces;
    const size_t count = index ? index->size() : points.size() / 3;
    for (size_t k = 0; k + 2 < count; k += 3) {
        for (size_t j = 0; j < 3; ++j) faces.push_back(index ? (*index)[k + j] : static_cast<double>(k + j));
        faces.push_back(-1);
    }
    emitIndexedFaces(out, points, faces, m, ccw, what);
}

const std::set<std::string>& primitiveNodes() {
    static const std::set<std::string> names = {"Box", "Sphere", "Cylinder", "Cone", "ElevationGrid", "Extrusion",
                                                "Text", "Disk2D", "Rectangle2D", "Circle2D", "Arc2D"};
    return names;
}

std::string skippedWarning(const std::map<std::string, int>& skipped) {
    std::string list;
    for (const auto& [name, count] : skipped) {
        if (!list.empty()) list += ", ";
        list += std::to_string(count) + " " + name;
    }
    return "skipped what is not a mesh: " + list;
}

// -- X3D (XML) ---------------------------------------------------------------

struct XNode {
    std::string name;
    std::map<std::string, std::string> attrs;
    std::vector<std::unique_ptr<XNode>> children;
};

class XmlParser {
public:
    explicit XmlParser(const std::string& text) : s_(text) {}

    std::unique_ptr<XNode> parseDocument() {
        auto root = std::make_unique<XNode>();
        root->name = "#document";
        parseChildren(*root, "");
        return root;
    }

private:
    const std::string& s_;
    size_t p_ = 0;

    bool startsWith(std::string_view w) const { return s_.compare(p_, w.size(), w) == 0; }

    void skipPast(std::string_view w) {
        const size_t at = s_.find(w, p_);
        if (at == std::string::npos) throw std::runtime_error("X3D: unterminated '" + std::string(w) + "'");
        p_ = at + w.size();
    }

    void skipSpace() {
        while (p_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[p_]))) ++p_;
    }

    std::string readName() {
        const size_t b = p_;
        while (p_ < s_.size() && !std::isspace(static_cast<unsigned char>(s_[p_])) && s_[p_] != '>' &&
               s_[p_] != '/' && s_[p_] != '=')
            ++p_;
        return s_.substr(b, p_ - b);
    }

    static std::string decode(std::string v) {
        static const std::pair<const char*, const char*> ents[] = {
            {"&lt;", "<"}, {"&gt;", ">"}, {"&quot;", "\""}, {"&apos;", "'"}, {"&amp;", "&"}};
        for (const auto& [from, to] : ents) {
            for (size_t at = v.find(from); at != std::string::npos; at = v.find(from, at + 1)) v.replace(at, std::strlen(from), to);
        }
        return v;
    }

    // Children of `parent` until the matching close tag (or end of input for
    // the document itself). Text content is ignored: X3D keeps its data in
    // attributes.
    void parseChildren(XNode& parent, const std::string& closeName) {
        while (true) {
            const size_t lt = s_.find('<', p_);
            if (lt == std::string::npos) {
                if (!closeName.empty()) throw std::runtime_error("X3D: <" + closeName + "> is never closed");
                p_ = s_.size();
                return;
            }
            p_ = lt;
            if (startsWith("<!--")) {
                skipPast("-->");
            } else if (startsWith("<![CDATA[")) {
                skipPast("]]>");
            } else if (startsWith("<?") ) {
                skipPast("?>");
            } else if (startsWith("<!")) {
                skipPast(">");  // DOCTYPE, which X3D writes without an internal subset
            } else if (startsWith("</")) {
                p_ += 2;
                const std::string name = readName();
                skipPast(">");
                if (name != closeName) throw std::runtime_error("X3D: </" + name + "> closes <" + closeName + ">");
                return;
            } else {
                ++p_;
                auto node = std::make_unique<XNode>();
                node->name = readName();
                bool selfClosing = false;
                while (true) {
                    skipSpace();
                    if (p_ >= s_.size()) throw std::runtime_error("X3D: unterminated <" + node->name + ">");
                    if (startsWith("/>")) {
                        p_ += 2;
                        selfClosing = true;
                        break;
                    }
                    if (s_[p_] == '>') {
                        ++p_;
                        break;
                    }
                    const std::string key = readName();
                    skipSpace();
                    std::string value;
                    if (p_ < s_.size() && s_[p_] == '=') {
                        ++p_;
                        skipSpace();
                        const char q = s_[p_];
                        if (q != '"' && q != '\'') throw std::runtime_error("X3D: unquoted attribute '" + key + "'");
                        const size_t end = s_.find(q, p_ + 1);
                        if (end == std::string::npos) throw std::runtime_error("X3D: unterminated attribute '" + key + "'");
                        value = decode(s_.substr(p_ + 1, end - p_ - 1));
                        p_ = end + 1;
                    }
                    node->attrs[key] = value;
                }
                if (!selfClosing) parseChildren(*node, node->name);
                parent.children.push_back(std::move(node));
            }
        }
    }
};

class X3dWalker {
public:
    LoadedMesh out;

    void collectDefs(const XNode& n) {
        if (auto it = n.attrs.find("DEF"); it != n.attrs.end()) defs_[it->second] = &n;
        for (const auto& c : n.children) collectDefs(*c);
    }

    void walk(const XNode& raw, const Mat& m) {
        const XNode& n = resolve(raw);
        if (n.name == "Transform") {
            const Mat t = transformNode(nums(n, "translation"), nums(n, "rotation"), nums(n, "scale"),
                                        nums(n, "center"), nums(n, "scaleOrientation"));
            for (const auto& c : n.children) walk(*c, mul(m, t));
        } else if (n.name == "Shape") {
            for (const auto& c : n.children) geometry(resolve(*c), m);
        } else if (n.name == "Inline") {
            ++skipped["Inline"];
        } else {
            for (const auto& c : n.children) walk(*c, m);
        }
    }

    std::map<std::string, int> skipped;

private:
    std::map<std::string, const XNode*> defs_;

    const XNode& resolve(const XNode& n) const {
        auto use = n.attrs.find("USE");
        if (use == n.attrs.end()) return n;
        auto it = defs_.find(use->second);
        if (it == defs_.end()) throw std::runtime_error("X3D: USE='" + use->second + "' has no DEF");
        return *it->second;
    }

    static std::vector<double> nums(const XNode& n, const char* attr) {
        auto it = n.attrs.find(attr);
        return it == n.attrs.end() ? std::vector<double>{} : parseNumbers(it->second);
    }

    static bool ccw(const XNode& n) {
        auto it = n.attrs.find("ccw");
        return it == n.attrs.end() || it->second != "false";
    }

    std::vector<double> points(const XNode& geom) const {
        for (const auto& c : geom.children) {
            const XNode& k = resolve(*c);
            if (k.name == "Coordinate" || k.name == "CoordinateDouble") return nums(k, "point");
        }
        return {};
    }

    void geometry(const XNode& g, const Mat& m) {
        if (g.name == "IndexedFaceSet") {
            emitIndexedFaces(out, points(g), nums(g, "coordIndex"), m, ccw(g), "X3D IndexedFaceSet");
        } else if (g.name == "IndexedTriangleSet") {
            const std::vector<double> index = nums(g, "index");
            emitTriangles(out, points(g), &index, m, ccw(g), "X3D IndexedTriangleSet");
        } else if (g.name == "TriangleSet") {
            emitTriangles(out, points(g), nullptr, m, ccw(g), "X3D TriangleSet");
        } else if (primitiveNodes().count(g.name)) {
            ++skipped[g.name];
        }
    }
};

// -- VRML97 ------------------------------------------------------------------

struct VNode;
using VNodePtr = std::shared_ptr<VNode>;

struct VField {
    std::vector<std::string> scalars;
    std::vector<VNodePtr> nodes;
};

struct VNode {
    std::string type;
    std::map<std::string, VField> fields;
};

class VrmlParser {
public:
    explicit VrmlParser(const std::string& text) { tokenize(text); }

    std::vector<VNodePtr> parseScene() {
        std::vector<VNodePtr> nodes;
        while (i_ < toks_.size()) {
            const std::string& t = toks_[i_];
            if (t == "ROUTE") {
                i_ += 4;  // ROUTE a.b TO c.d
            } else if (t == "PROTO" || t == "EXTERNPROTO") {
                i_ += 2;
                skipBalanced("[", "]");
                if (t == "PROTO") {
                    skipBalanced("{", "}");
                } else if (i_ < toks_.size()) {
                    if (toks_[i_] == "[") skipBalanced("[", "]");
                    else ++i_;  // the url string
                }
            } else {
                if (VNodePtr n = parseNode()) nodes.push_back(n);
            }
        }
        return nodes;
    }

private:
    std::vector<std::string> toks_;
    size_t i_ = 0;
    std::map<std::string, VNodePtr> defs_;

    void tokenize(const std::string& s) {
        size_t p = 0;
        while (p < s.size()) {
            const char c = s[p];
            if (std::isspace(static_cast<unsigned char>(c)) || c == ',') {
                ++p;
            } else if (c == '#') {
                while (p < s.size() && s[p] != '\n' && s[p] != '\r') ++p;
            } else if (c == '{' || c == '}' || c == '[' || c == ']') {
                toks_.emplace_back(1, c);
                ++p;
            } else if (c == '"') {
                size_t e = p + 1;
                while (e < s.size() && s[e] != '"') e += (s[e] == '\\') ? 2 : 1;
                toks_.push_back(s.substr(p, std::min(e + 1, s.size()) - p));
                p = e + 1;
            } else {
                const size_t b = p;
                while (p < s.size() && !std::isspace(static_cast<unsigned char>(s[p])) && s[p] != ',' && s[p] != '#' &&
                       s[p] != '{' && s[p] != '}' && s[p] != '[' && s[p] != ']' && s[p] != '"')
                    ++p;
                toks_.push_back(s.substr(b, p - b));
            }
        }
    }

    const std::string& peek(size_t ahead = 0) const {
        static const std::string eof;
        return i_ + ahead < toks_.size() ? toks_[i_ + ahead] : eof;
    }

    void expect(const std::string& t) {
        if (peek() != t) throw std::runtime_error("VRML: expected '" + t + "', found '" + peek() + "'");
        ++i_;
    }

    void skipBalanced(const std::string& open, const std::string& close) {
        if (peek() != open) return;
        int depth = 0;
        do {
            if (peek() == open) ++depth;
            else if (peek() == close) --depth;
            ++i_;
        } while (depth > 0 && i_ < toks_.size());
    }

    static bool isValueToken(const std::string& t) {
        if (t.empty()) return false;
        const char c = t[0];
        return std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' || c == '.' || c == '"' ||
               t == "TRUE" || t == "FALSE";
    }

    bool atNode() const { return peek() == "DEF" || peek() == "USE" || peek() == "NULL" || peek(1) == "{"; }

    VNodePtr parseNode() {
        if (peek() == "NULL") {
            ++i_;
            return nullptr;
        }
        if (peek() == "USE") {
            const std::string name = peek(1);
            i_ += 2;
            auto it = defs_.find(name);
            if (it == defs_.end()) throw std::runtime_error("VRML: USE " + name + " has no DEF");
            return it->second;
        }
        if (peek() == "DEF") {
            const std::string name = peek(1);
            i_ += 2;
            VNodePtr n = parseNode();
            defs_[name] = n;
            return n;
        }
        auto node = std::make_shared<VNode>();
        node->type = peek();
        ++i_;
        expect("{");
        while (i_ < toks_.size() && peek() != "}") {
            const std::string field = peek();
            ++i_;
            VField& f = node->fields[field];
            if (peek() == "[") {
                ++i_;
                while (i_ < toks_.size() && peek() != "]") {
                    if (atNode()) {
                        if (VNodePtr c = parseNode()) f.nodes.push_back(c);
                    } else {
                        f.scalars.push_back(peek());
                        ++i_;
                    }
                }
                expect("]");
            } else if (atNode()) {
                if (VNodePtr c = parseNode()) f.nodes.push_back(c);
            } else {
                while (i_ < toks_.size() && isValueToken(peek())) {
                    f.scalars.push_back(peek());
                    ++i_;
                }
            }
        }
        expect("}");
        return node;
    }
};

class VrmlWalker {
public:
    LoadedMesh out;
    std::map<std::string, int> skipped;

    void walk(const VNodePtr& n, const Mat& m) {
        if (!n) return;
        if (n->type == "Transform") {
            const Mat t = transformNode(nums(*n, "translation"), nums(*n, "rotation"), nums(*n, "scale"),
                                        nums(*n, "center"), nums(*n, "scaleOrientation"));
            for (const auto& c : kids(*n, "children")) walk(c, mul(m, t));
        } else if (n->type == "Shape") {
            for (const auto& g : kids(*n, "geometry")) geometry(*g, m);
        } else if (n->type == "Inline") {
            ++skipped["Inline"];
        } else {
            // Group, Anchor, Billboard, Collision, Switch (choice), LOD (level)
            // and anything else holding nodes: walk every node-valued field.
            for (const auto& [name, f] : n->fields) {
                for (const auto& c : f.nodes) walk(c, m);
            }
        }
    }

private:
    static std::vector<double> nums(const VNode& n, const char* field) {
        auto it = n.fields.find(field);
        if (it == n.fields.end()) return {};
        std::string joined;
        for (const std::string& s : it->second.scalars) joined += s + " ";
        return parseNumbers(joined);
    }

    static const std::vector<VNodePtr>& kids(const VNode& n, const char* field) {
        static const std::vector<VNodePtr> none;
        auto it = n.fields.find(field);
        return it == n.fields.end() ? none : it->second.nodes;
    }

    void geometry(const VNode& g, const Mat& m) {
        if (g.type == "IndexedFaceSet") {
            std::vector<double> pts;
            for (const auto& c : kids(g, "coord")) {
                if (c) pts = nums(*c, "point");
            }
            bool ccw = true;
            if (auto it = g.fields.find("ccw"); it != g.fields.end() && !it->second.scalars.empty())
                ccw = it->second.scalars[0] != "FALSE";
            emitIndexedFaces(out, pts, nums(g, "coordIndex"), m, ccw, "VRML IndexedFaceSet");
        } else if (primitiveNodes().count(g.type)) {
            ++skipped[g.type];
        }
    }
};

// -- AMF ---------------------------------------------------------------------

// The text between <tag> and </tag>, searched from `from` within `s`;
// npos-safe, returning "" when absent.
std::string_view elementText(std::string_view s, std::string_view tag, size_t from = 0) {
    const std::string open = "<" + std::string(tag) + ">";
    const std::string close = "</" + std::string(tag) + ">";
    const size_t b = s.find(open, from);
    if (b == std::string_view::npos) return {};
    const size_t e = s.find(close, b);
    if (e == std::string_view::npos) return {};
    return s.substr(b + open.size(), e - b - open.size());
}

// OpenSCAD reads an AMF's numbers as millimetres whatever its `unit`
// attribute says (checked, 2026.02.01: a unit="inch" sphere of radius 4.25
// imports at 4.25). Applying the unit made this 25.4 times larger.
double amfUnitScale(std::string_view xml) {
    if (xml.find("<amf") == std::string_view::npos) throw std::runtime_error("AMF: no <amf> element");
    return 1.0;
}

// Each <tag>...</tag> segment inside `s`, in order.
std::vector<std::string_view> segments(std::string_view s, std::string_view tag) {
    std::vector<std::string_view> out;
    const std::string open = "<" + std::string(tag);
    const std::string close = "</" + std::string(tag) + ">";
    for (size_t p = 0;;) {
        size_t b = s.find(open, p);
        // "<vertex" must not match "<vertices".
        while (b != std::string_view::npos && b + open.size() < s.size() &&
               std::isalpha(static_cast<unsigned char>(s[b + open.size()])))
            b = s.find(open, b + 1);
        if (b == std::string_view::npos) break;
        const size_t e = s.find(close, b);
        if (e == std::string_view::npos) break;
        out.push_back(s.substr(b, e + close.size() - b));
        p = e + close.size();
    }
    return out;
}

double amfNumber(std::string_view text, const char* what) {
    const std::string t(text);
    char* end = nullptr;
    const double v = std::strtod(t.c_str(), &end);
    if (end == t.c_str()) throw std::runtime_error(std::string("AMF: missing or bad <") + what + ">");
    return v;
}

} // namespace

LoadedMesh loadAmf(const std::string& path) {
    std::string xml = readText(path);
    // Compressed AMF is a zip holding one .amf -- the spec's own recommendation.
    if (xml.compare(0, 2, "PK") == 0) {
        const std::vector<uint8_t> bytes = readStoredZipEntryBySuffix(path, ".amf");
        xml.assign(bytes.begin(), bytes.end());
    }
    const double unit = amfUnitScale(xml);

    LoadedMesh out;
    for (std::string_view object : segments(xml, "object")) {
        const std::string_view verticesBlock = elementText(object, "vertices");
        const int base = static_cast<int>(out.verts.size());
        for (std::string_view vertex : segments(verticesBlock, "vertex")) {
            const std::string_view c = elementText(vertex, "coordinates");
            out.verts.push_back({amfNumber(elementText(c, "x"), "x") * unit, amfNumber(elementText(c, "y"), "y") * unit,
                                 amfNumber(elementText(c, "z"), "z") * unit});
        }
        const int count = static_cast<int>(out.verts.size()) - base;
        for (std::string_view volume : segments(object, "volume")) {
            for (std::string_view tri : segments(volume, "triangle")) {
                std::array<int, 3> t{};
                const char* names[3] = {"v1", "v2", "v3"};
                for (int k = 0; k < 3; ++k) {
                    const int i = static_cast<int>(amfNumber(elementText(tri, names[k]), names[k]));
                    if (i < 0 || i >= count)
                        throw std::runtime_error("AMF: triangle vertex " + std::to_string(i) + " is past its object's " +
                                                 std::to_string(count) + " vertices");
                    t[k] = base + i;
                }
                out.tris.push_back(t);
            }
        }
    }
    if (xml.find("<constellation") != std::string::npos) {
        out.warnings.push_back("AMF constellations are not applied: each object is imported where it was modeled");
    }
    return out;
}

LoadedMesh loadX3d(const std::string& path) {
    const std::string text = readText(path);
    const std::unique_ptr<XNode> doc = XmlParser(text).parseDocument();
    X3dWalker walker;
    walker.collectDefs(*doc);
    walker.walk(*doc, identity());
    if (!walker.skipped.empty()) walker.out.warnings.push_back(skippedWarning(walker.skipped));
    return std::move(walker.out);
}

LoadedMesh loadVrml(const std::string& path) {
    const std::string text = readText(path);
    if (text.compare(0, 10, "#VRML V1.0") == 0) throw std::runtime_error("VRML 1.0 is not supported, only VRML97 (V2.0)");
    VrmlParser parser(text);
    const std::vector<VNodePtr> scene = parser.parseScene();
    VrmlWalker walker;
    for (const VNodePtr& n : scene) walker.walk(n, identity());
    if (!walker.skipped.empty()) walker.out.warnings.push_back(skippedWarning(walker.skipped));
    return std::move(walker.out);
}

} // namespace oscadeval
