#include "openscad_cpp_evaluator/mesh_import.hpp"

#include "openscad_cpp_evaluator/zip_stored.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
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

LoadedMesh loadThreeMf(const std::string& path) {
    const std::vector<uint8_t> xmlBytes = readStoredZipEntryBySuffix(path, "3dmodel.model");
    const std::string xml(xmlBytes.begin(), xmlBytes.end());

    LoadedMesh out;
    size_t searchPos = 0;
    while (true) {
        const size_t meshStart = xml.find("<mesh", searchPos);
        if (meshStart == std::string::npos) break;
        const size_t meshEnd = xml.find("</mesh>", meshStart);
        if (meshEnd == std::string::npos) break;
        const std::string_view segment(xml.data() + meshStart, meshEnd - meshStart);
        searchPos = meshEnd + 7;

        LoadedMesh obj;
        for (size_t p = 0;;) {
            const size_t vStart = segment.find("<vertex", p);
            if (vStart == std::string_view::npos) break;
            const size_t vEnd = segment.find('>', vStart);
            if (vEnd == std::string_view::npos) break;
            const std::string_view tag = segment.substr(vStart, vEnd - vStart + 1);
            obj.verts.push_back({std::stod(attrValue(tag, "x")), std::stod(attrValue(tag, "y")), std::stod(attrValue(tag, "z"))});
            p = vEnd + 1;
        }
        for (size_t p = 0;;) {
            const size_t tStart = segment.find("<triangle", p);
            if (tStart == std::string_view::npos) break;
            if (tStart + 9 < segment.size() && std::isalpha(static_cast<unsigned char>(segment[tStart + 9]))) {  // <triangles>, not a triangle
                p = tStart + 9;
                continue;
            }
            const size_t tEnd = segment.find('>', tStart);
            if (tEnd == std::string_view::npos) break;
            const std::string_view tag = segment.substr(tStart, tEnd - tStart + 1);
            obj.tris.push_back({std::stoi(attrValue(tag, "v1")), std::stoi(attrValue(tag, "v2")), std::stoi(attrValue(tag, "v3"))});
            p = tEnd + 1;
        }
        // Welded per object, as OpenSCAD makes each 3MF object a solid on
        // its own (checked, 2026.02.01): a soup object closes, but two open
        // halves in separate objects stay open.
        appendWelded(out, obj);
    }
    return out;
}

} // namespace oscadeval
