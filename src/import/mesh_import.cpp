#include "openscad_cpp_evaluator/mesh_import.hpp"

#include "openscad_cpp_evaluator/zip_stored.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace oscadeval {

namespace {

std::vector<uint8_t> readWholeFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("could not open '" + path + "'");
    in.seekg(0, std::ios::end);
    const auto size = in.tellg();
    if (size < 0) throw std::runtime_error("could not read '" + path + "'");
    std::vector<uint8_t> buf(static_cast<size_t>(size));
    in.seekg(0);
    if (!buf.empty()) in.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
    return buf;
}

std::string readWholeFileText(const std::string& path) {
    const std::vector<uint8_t> bytes = readWholeFile(path);
    return std::string(bytes.begin(), bytes.end());
}

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> lines;
    std::stringstream ss(text);
    std::string line;
    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t");
    return s.substr(b, e - b + 1);
}

std::vector<std::string> splitWs(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string tok;
    while (ss >> tok) out.push_back(tok);
    return out;
}

std::string attrValue(std::string_view tag, std::string_view attrName) {
    const std::string needle = std::string(attrName) + "=\"";
    const size_t pos = tag.find(needle);
    if (pos == std::string_view::npos) return "0";
    const size_t start = pos + needle.size();
    const size_t end = tag.find('"', start);
    if (end == std::string_view::npos) return "0";
    return std::string(tag.substr(start, end - start));
}

} // namespace

// Exact-match vertex welding: every position keeps its first index. Only
// bit-identical coordinates merge; near-coincident ones stay apart (that is
// repair=true's job). A face index past the vertex list is an error here
// rather than a bad read.
LoadedMesh weldVertices(const std::vector<std::array<double, 3>>& verts, const std::vector<std::array<int, 3>>& tris) {
    std::map<std::array<double, 3>, int> seen;
    LoadedMesh out;
    std::vector<int> remap(verts.size());
    for (size_t i = 0; i < verts.size(); ++i) {
        auto it = seen.find(verts[i]);
        if (it != seen.end()) {
            remap[i] = it->second;
        } else {
            const int idx = static_cast<int>(out.verts.size());
            out.verts.push_back(verts[i]);
            seen.emplace(verts[i], idx);
            remap[i] = idx;
        }
    }
    const auto at = [&](int i) {
        if (i < 0 || static_cast<size_t>(i) >= verts.size())
            throw std::runtime_error("face index " + std::to_string(i) + " is past its " + std::to_string(verts.size()) +
                                     " vertices");
        return remap[static_cast<size_t>(i)];
    };
    out.tris.reserve(tris.size());
    for (const auto& t : tris) out.tris.push_back({at(t[0]), at(t[1]), at(t[2])});
    return out;
}

void appendWelded(LoadedMesh& out, const LoadedMesh& part) {
    const LoadedMesh w = weldVertices(part.verts, part.tris);
    const int base = static_cast<int>(out.verts.size());
    out.verts.insert(out.verts.end(), w.verts.begin(), w.verts.end());
    for (const auto& t : w.tris) out.tris.push_back({base + t[0], base + t[1], base + t[2]});
}

LoadedMesh loadStl(const std::string& path) {
    const std::vector<uint8_t> bytes = readWholeFile(path);
    // OpenSCAD's wording for every file it cannot read as STL.
    const std::string notRecognized = "STL format not recognized in '" + path + "'.";
    if (bytes.size() < 80) throw std::runtime_error(notRecognized);

    const std::string sniff(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(std::min<size_t>(bytes.size(), 336)));
    const bool isAscii = sniff.find("facet normal") != std::string::npos;

    std::vector<std::array<double, 3>> verts;
    std::vector<std::array<int, 3>> tris;

    if (isAscii) {
        const std::string text(bytes.begin(), bytes.end());
        std::vector<std::array<double, 3>> triVerts;
        for (const std::string& raw : splitLines(text)) {
            const std::string line = trim(raw);
            if (line.rfind("vertex ", 0) != 0) continue;
            const std::vector<std::string> parts = splitWs(line);
            if (parts.size() < 4) continue;
            triVerts.push_back({std::stod(parts[1]), std::stod(parts[2]), std::stod(parts[3])});
            if (triVerts.size() == 3) {
                const int base = static_cast<int>(verts.size());
                verts.insert(verts.end(), triVerts.begin(), triVerts.end());
                tris.push_back({base, base + 1, base + 2});
                triVerts.clear();
            }
        }
    } else {
        if (bytes.size() < 84) throw std::runtime_error(notRecognized);
        uint32_t count;
        std::memcpy(&count, &bytes[80], 4);
        const size_t needed = 84 + static_cast<size_t>(count) * 50;
        if (bytes.size() < needed) throw std::runtime_error(notRecognized);
        verts.reserve(static_cast<size_t>(count) * 3);
        tris.reserve(count);
        for (uint32_t t = 0; t < count; ++t) {
            const size_t base = 84 + static_cast<size_t>(t) * 50 + 12; // skip normal
            std::array<float, 9> f{};
            std::memcpy(f.data(), &bytes[base], 9 * sizeof(float));
            const int vBase = static_cast<int>(verts.size());
            verts.push_back({f[0], f[1], f[2]});
            verts.push_back({f[3], f[4], f[5]});
            verts.push_back({f[6], f[7], f[8]});
            tris.push_back({vBase, vBase + 1, vBase + 2});
        }
    }
    return weldVertices(verts, tris);
}

LoadedMesh loadObj(const std::string& path) {
    LoadedMesh out;
    for (const std::string& raw : splitLines(readWholeFileText(path))) {
        const std::string line = trim(raw);
        if (line.rfind("v ", 0) == 0) {
            const std::vector<std::string> p = splitWs(line);
            if (p.size() < 4) continue;
            out.verts.push_back({std::stod(p[1]), std::stod(p[2]), std::stod(p[3])});
        } else if (line.rfind("f ", 0) == 0) {
            const std::vector<std::string> tokens = splitWs(line);
            std::vector<int> idx;
            for (size_t i = 1; i < tokens.size(); ++i) {
                const std::string& tok = tokens[i];
                const size_t slash = tok.find('/');
                const std::string first = slash == std::string::npos ? tok : tok.substr(0, slash);
                idx.push_back(std::stoi(first) - 1);
            }
            for (size_t i = 1; i + 1 < idx.size(); ++i) out.tris.push_back({idx[0], idx[i], idx[i + 1]});
        }
    }
    // OpenSCAD welds an OBJ's exactly-equal vertices across the whole file,
    // `o`/`g` boundaries included (checked, 2026.02.01: two halves of a cube
    // in separate `o` objects, each with its own vertex copies, import as
    // one closed cube).
    return weldVertices(out.verts, out.tris);
}

LoadedMesh loadOff(const std::string& path) {
    std::vector<std::vector<std::string>> lines;
    for (const std::string& raw : splitLines(readWholeFileText(path))) {
        std::string line = raw.substr(0, raw.find('#'));
        std::vector<std::string> tokens = splitWs(trim(line));
        if (!tokens.empty()) lines.push_back(std::move(tokens));
    }
    const auto bad = [&](const std::string& why) { return std::runtime_error("OFF file '" + path + "': " + why); };
    if (lines.empty()) throw bad("empty file");

    // The header keyword may carry prefixes (COFF, NOFF, STOFF, ...), and the
    // counts may follow it on the same line, as OpenSCAD accepts.
    size_t idx = 0;
    std::vector<std::string> counts = lines[0];
    const std::string& kw = counts[0];
    if (kw.size() >= 3 && kw.compare(kw.size() - 3, 3, "OFF") == 0) {
        counts.erase(counts.begin());
        if (counts.empty()) {
            if (lines.size() < 2) throw bad("missing the vertex/face count line");
            counts = lines[1];
            idx = 2;
        } else {
            idx = 1;
        }
    }
    const auto toInt = [&](const std::string& t) {
        try {
            size_t used = 0;
            const int v = std::stoi(t, &used);
            if (used == t.size() && v >= 0) return v;
        } catch (const std::exception&) {
        }
        throw bad("'" + t + "' is not a count or index");
    };
    const auto toNum = [&](const std::string& t) {
        try {
            return std::stod(t);
        } catch (const std::exception&) {
            throw bad("'" + t + "' is not a number");
        }
    };
    if (counts.size() < 2) throw bad("malformed vertex/face count line");
    const int nv = toInt(counts[0]);
    const int nf = toInt(counts[1]);
    if (lines.size() - idx < static_cast<size_t>(nv) + static_cast<size_t>(nf)) throw bad("file is truncated");

    LoadedMesh out;
    out.verts.reserve(static_cast<size_t>(nv));
    for (int i = 0; i < nv; ++i) {
        const std::vector<std::string>& p = lines[idx++];
        if (p.size() < 3) throw bad("a vertex needs three coordinates");
        out.verts.push_back({toNum(p[0]), toNum(p[1]), toNum(p[2])});  // any further values are a colour
    }
    for (int i = 0; i < nf; ++i) {
        const std::vector<std::string>& p = lines[idx++];
        const int cnt = toInt(p[0]);
        if (p.size() < static_cast<size_t>(cnt) + 1) throw bad("a face lists fewer indices than its count");
        std::vector<int> faceIdx;
        faceIdx.reserve(static_cast<size_t>(cnt));
        for (int k = 0; k < cnt; ++k) {
            const int v = toInt(p[static_cast<size_t>(k) + 1]);
            if (v >= nv) throw bad("face index " + std::to_string(v) + " is out of range");
            faceIdx.push_back(v);
        }
        for (int k = 1; k + 1 < cnt; ++k) out.tris.push_back({faceIdx[0], faceIdx[static_cast<size_t>(k)], faceIdx[static_cast<size_t>(k) + 1]});
    }
    // An OFF is one mesh, and OpenSCAD merges its exactly-equal vertices
    // when the mesh becomes a solid (checked, 2026.02.01: a triangle soup
    // or a duplicated vertex list takes part in CSG as a closed solid).
    return weldVertices(out.verts, out.tris);
}

namespace {

// An attribute's value, or nullopt. Requires whitespace before the name, so
// `id` never matches inside `objectid` or `pid`.
std::optional<std::string> xmlAttr(std::string_view tag, std::string_view name) {
    const std::string needle = std::string(name) + "=\"";
    for (size_t pos = tag.find(needle); pos != std::string_view::npos; pos = tag.find(needle, pos + 1)) {
        if (pos == 0 || !std::isspace(static_cast<unsigned char>(tag[pos - 1]))) continue;
        const size_t start = pos + needle.size();
        const size_t end = tag.find('"', start);
        if (end == std::string_view::npos) return std::nullopt;
        return std::string(tag.substr(start, end - start));
    }
    return std::nullopt;
}

// Every `<name ...>` start tag in `text` (not `<names>`), in order.
std::vector<std::string_view> startTags(std::string_view text, std::string_view name) {
    std::vector<std::string_view> tags;
    const std::string open = "<" + std::string(name);
    for (size_t p = text.find(open); p != std::string_view::npos; p = text.find(open, p + 1)) {
        const size_t after = p + open.size();
        if (after >= text.size() || !(std::isspace(static_cast<unsigned char>(text[after])) || text[after] == '/' || text[after] == '>')) continue;
        const size_t close = text.find('>', after);
        if (close == std::string_view::npos) break;
        tags.push_back(text.substr(p, close - p + 1));
    }
    return tags;
}

// A 3MF transform: x' = A x + t. The file writes it as twelve numbers,
// "m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32", for a ROW vector
// [x y z 1] times a 4x3 matrix, so A[r][c] = m[c*3 + r] and t[r] = m[9 + r].
struct Affine3 {
    double a[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double t[3] = {0, 0, 0};
};

Affine3 parseTransform(const std::optional<std::string>& text, const std::string& path) {
    Affine3 m;
    if (!text) return m;
    std::vector<double> v;
    for (const std::string& tok : splitWs(*text)) {
        try {
            v.push_back(std::stod(tok));
        } catch (const std::exception&) {
            throw std::runtime_error("3MF file '" + path + "': transform \"" + *text + "\" is not twelve numbers");
        }
    }
    if (v.size() != 12) throw std::runtime_error("3MF file '" + path + "': transform \"" + *text + "\" is not twelve numbers");
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m.a[r][c] = v[static_cast<size_t>(c * 3 + r)];
        m.t[r] = v[static_cast<size_t>(9 + r)];
    }
    return m;
}

// outer after inner: x -> outer(inner(x)).
Affine3 compose(const Affine3& outer, const Affine3& inner) {
    Affine3 m;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            m.a[r][c] = 0;
            for (int k = 0; k < 3; ++k) m.a[r][c] += outer.a[r][k] * inner.a[k][c];
        }
        m.t[r] = outer.t[r];
        for (int k = 0; k < 3; ++k) m.t[r] += outer.a[r][k] * inner.t[k];
    }
    return m;
}

struct ThreeMfObject {
    LoadedMesh mesh;
    bool hasMesh = false;
    struct Component {
        std::string objectId;
        Affine3 transform;
    };
    std::vector<Component> components;
};

} // namespace

// What is placed is what <build> lists: each <item> instances an object,
// which is a <mesh> or a list of <component>s (each instancing another
// object), every level applying its own `transform`. Objects no item reaches
// are not part of the model. A file whose <build> lists no item (which the
// spec does not allow) imports every mesh object as it stands, as before.
//
// Deliberately NOT OpenSCAD 2026.02.01 or 2026.11-RC1 (both checked
// 2026-11-26, identical results), which compose
// a component's transform on the wrong side of its item's, and shift a
// componented object up by 1 in z: both are bugs, and a slicer's file
// would land off its bed. A mirroring transform reverses the triangles so the
// solid stays right side out (OpenSCAD keeps it inside out until a boolean
// flips it).
LoadedMesh loadThreeMf(const std::string& path) {
    const std::vector<uint8_t> xmlBytes = readStoredZipEntryBySuffix(path, "3dmodel.model");
    const std::string xml(xmlBytes.begin(), xmlBytes.end());
    const std::string_view doc(xml);

    std::map<std::string, ThreeMfObject> objects;
    std::vector<std::string> order;
    LoadedMesh out;
    bool externalPart = false;
    for (const std::string_view objTag : startTags(doc, "object")) {
        const size_t bodyStart = static_cast<size_t>(objTag.data() - doc.data()) + objTag.size();
        const size_t bodyEnd = doc.find("</object>", bodyStart);
        const std::string_view body = doc.substr(bodyStart, bodyEnd == std::string_view::npos ? std::string_view::npos : bodyEnd - bodyStart);
        const std::string id = xmlAttr(objTag, "id").value_or("");
        ThreeMfObject obj;
        if (body.find("<mesh") != std::string_view::npos) {
            obj.hasMesh = true;
            for (const std::string_view tag : startTags(body, "vertex"))
                obj.mesh.verts.push_back({std::stod(attrValue(tag, "x")), std::stod(attrValue(tag, "y")), std::stod(attrValue(tag, "z"))});
            for (const std::string_view tag : startTags(body, "triangle"))
                obj.mesh.tris.push_back({std::stoi(attrValue(tag, "v1")), std::stoi(attrValue(tag, "v2")), std::stoi(attrValue(tag, "v3"))});
        }
        for (const std::string_view tag : startTags(body, "component")) {
            // ponytail: a production-extension component pointing into
            // another model part (p:path, as Bambu Studio writes) is skipped
            // with a warning; read that part when someone needs it.
            if (xmlAttr(tag, "p:path")) {
                externalPart = true;
                continue;
            }
            obj.components.push_back({xmlAttr(tag, "objectid").value_or(""), parseTransform(xmlAttr(tag, "transform"), path)});
        }
        order.push_back(id);
        objects[id] = std::move(obj);
    }
    if (externalPart)
        out.warnings.push_back("this 3MF keeps part of its model in another file of the package (a production-extension p:path component), which is not imported");

    const auto place = [&](const LoadedMesh& mesh, const Affine3& m) {
        const double det = m.a[0][0] * (m.a[1][1] * m.a[2][2] - m.a[1][2] * m.a[2][1]) -
                           m.a[0][1] * (m.a[1][0] * m.a[2][2] - m.a[1][2] * m.a[2][0]) +
                           m.a[0][2] * (m.a[1][0] * m.a[2][1] - m.a[1][1] * m.a[2][0]);
        LoadedMesh placed;
        placed.verts.reserve(mesh.verts.size());
        for (const auto& v : mesh.verts) {
            std::array<double, 3> p{};
            for (int r = 0; r < 3; ++r) p[static_cast<size_t>(r)] = m.a[r][0] * v[0] + m.a[r][1] * v[1] + m.a[r][2] * v[2] + m.t[r];
            placed.verts.push_back(p);
        }
        placed.tris = mesh.tris;
        if (det < 0)
            for (auto& t : placed.tris) std::swap(t[1], t[2]);
        // Welded per placed mesh, as OpenSCAD makes each 3MF object a solid
        // on its own (checked, 2026.02.01): a soup object closes, but two
        // open halves in separate objects stay open.
        appendWelded(out, placed);
    };

    const std::vector<std::string_view> items = startTags(doc, "item");
    if (items.empty()) {
        for (const std::string& id : order)
            if (objects[id].hasMesh) place(objects[id].mesh, Affine3{});
        return out;
    }
    // Depth-limited, so a component cycle (invalid, but possible) ends.
    std::function<void(const std::string&, const Affine3&, int)> instance = [&](const std::string& id, const Affine3& m, int depth) {
        const auto it = objects.find(id);
        if (it == objects.end()) throw std::runtime_error("3MF file '" + path + "': object " + id + " is used but never defined");
        if (depth > 64) throw std::runtime_error("3MF file '" + path + "': components nest more than 64 deep (a cycle?)");
        if (it->second.hasMesh) place(it->second.mesh, m);
        for (const auto& c : it->second.components) instance(c.objectId, compose(m, c.transform), depth + 1);
    };
    for (const std::string_view item : items)
        instance(xmlAttr(item, "objectid").value_or(""), parseTransform(xmlAttr(item, "transform"), path), 0);
    return out;
}

} // namespace oscadeval
