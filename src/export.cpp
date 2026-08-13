#include "openscad_cpp_evaluator/export.hpp"

#include "openscad_cpp_evaluator/mesh_check.hpp"

#include "openscad_cpp_evaluator/zip_stored.hpp"

#include <manifold/manifold.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <optional>
#include <stdexcept>

namespace oscadeval {

namespace {

std::optional<manifold::MeshGL> composeMesh(const std::vector<ColoredBody>& bodies) {
    std::vector<manifold::Manifold> manifolds;
    for (const auto& b : bodies) {
        if (b.body && !b.body->IsEmpty()) manifolds.push_back(*b.body);
    }
    if (manifolds.empty()) return std::nullopt;
    return manifold::Manifold::BatchBoolean(manifolds, manifold::OpType::Add).GetMeshGL();
}

struct Vec3f {
    float x, y, z;
};

Vec3f sub(const Vec3f& a, const Vec3f& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3f cross(const Vec3f& a, const Vec3f& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3f normalized(const Vec3f& v) {
    const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len <= 0.0f) return {0.0f, 0.0f, 0.0f};
    return {v.x / len, v.y / len, v.z / len};
}

std::string formatG6(float v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", static_cast<double>(v));
    return std::string(buf);
}

// ponytail: writes raw host-endian bytes for float/uint32/uint16, assuming
// a little-endian host -- true for every realistic deployment target
// (x86_64/ARM64 on macOS/Linux/Windows). Upgrade to explicit byte-swapping
// if a big-endian target ever matters.
template <typename T>
void writeRaw(std::ofstream& out, T v) {
    out.write(reinterpret_cast<const char*>(&v), sizeof(v));
}

} // namespace

void writeStl(const std::string& path, const std::vector<ColoredBody>& bodies) {
    std::optional<manifold::MeshGL> mesh = composeMesh(bodies);
    if (!mesh) throw std::runtime_error("No geometry to export");

    const auto vertexAt = [&](uint32_t vertIndex) -> Vec3f {
        const size_t base = static_cast<size_t>(vertIndex) * mesh->numProp;
        return {mesh->vertProperties[base], mesh->vertProperties[base + 1], mesh->vertProperties[base + 2]};
    };

    const size_t triCount = mesh->triVerts.size() / 3;

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");

    char header[80] = {};
    out.write(header, sizeof(header));
    writeRaw(out, static_cast<uint32_t>(triCount));

    for (size_t t = 0; t < triCount; ++t) {
        const uint32_t i0 = mesh->triVerts[t * 3 + 0];
        const uint32_t i1 = mesh->triVerts[t * 3 + 1];
        const uint32_t i2 = mesh->triVerts[t * 3 + 2];
        const Vec3f v0 = vertexAt(i0), v1 = vertexAt(i1), v2 = vertexAt(i2);
        const Vec3f normal = normalized(cross(sub(v1, v0), sub(v2, v0)));

        writeRaw(out, normal.x);
        writeRaw(out, normal.y);
        writeRaw(out, normal.z);
        writeRaw(out, v0.x);
        writeRaw(out, v0.y);
        writeRaw(out, v0.z);
        writeRaw(out, v1.x);
        writeRaw(out, v1.y);
        writeRaw(out, v1.z);
        writeRaw(out, v2.x);
        writeRaw(out, v2.y);
        writeRaw(out, v2.z);
        writeRaw(out, static_cast<uint16_t>(0));
    }
}

void writeObj(const std::string& path, const std::vector<ExportObject>& objects) {
    // OBJ carries no colour of its own: `usemtl` names an entry in a .mtl
    // sitting next to the .obj, so an OBJ export writes TWO files. A reader
    // that ignores the mtllib still gets correct geometry.
    const size_t dot = path.find_last_of('.');
    const size_t slash = path.find_last_of("/\\");
    const bool hasExt = dot != std::string::npos && (slash == std::string::npos || dot > slash);
    const std::string mtlPath = (hasExt ? path.substr(0, dot) : path) + ".mtl";
    std::string mtlName = mtlPath;
    if (slash != std::string::npos) mtlName = mtlPath.substr(slash + 1);

    // One material per distinct colour, first-seen order -- counting every
    // colour a per-triangle object uses, not just its base.
    std::vector<std::array<float, 4>> materials;
    const auto materialFor = [&](const std::array<float, 4>& c) {
        for (size_t i = 0; i < materials.size(); ++i) {
            if (materials[i] == c) return i;
        }
        materials.push_back(c);
        return materials.size() - 1;
    };
    for (const ExportObject& o : objects) {
        if (o.triColors.empty()) {
            materialFor(o.color);
        } else {
            for (const auto& c : o.triColors) materialFor(c);
        }
    }

    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");
    if (!materials.empty()) out << "mtllib " << mtlName << "\n\n";

    size_t offset = 1; // OBJ vertex indices are 1-based and file-global
    size_t objIndex = 0;
    for (const ExportObject& o : objects) {
        ++objIndex;
        out << "o object_" << objIndex << "\n";
        const size_t vertCount = o.verts.size() / 3;
        for (size_t v = 0; v < vertCount; ++v) {
            out << "v " << formatG6(o.verts[v * 3]) << " " << formatG6(o.verts[v * 3 + 1]) << " "
                << formatG6(o.verts[v * 3 + 2]) << "\n";
        }
        const size_t triCount = o.tris.size() / 3;
        if (o.triColors.empty()) {
            out << "usemtl color_" << (materialFor(o.color) + 1) << "\n";
            for (size_t t = 0; t < triCount; ++t) {
                out << "f " << (o.tris[t * 3] + offset) << " " << (o.tris[t * 3 + 1] + offset) << " "
                    << (o.tris[t * 3 + 2] + offset) << "\n";
            }
        } else {
            // A multi-coloured surface becomes runs of faces with a usemtl
            // between them, emitted in triangle order and only when the
            // colour actually changes, so the face order still matches
            // every other format's.
            size_t current = static_cast<size_t>(-1);
            for (size_t t = 0; t < triCount; ++t) {
                const size_t m = materialFor(o.triColors[t]);
                if (m != current) {
                    out << "usemtl color_" << (m + 1) << "\n";
                    current = m;
                }
                out << "f " << (o.tris[t * 3] + offset) << " " << (o.tris[t * 3 + 1] + offset) << " "
                    << (o.tris[t * 3 + 2] + offset) << "\n";
            }
        }
        out << "\n";
        offset += vertCount;
    }
    out.close();

    if (materials.empty()) return;
    std::ofstream mtl(mtlPath);
    if (!mtl) throw std::runtime_error("Could not open '" + mtlPath + "' for writing");
    for (size_t i = 0; i < materials.size(); ++i) {
        const auto& c = materials[i];
        mtl << "newmtl color_" << (i + 1) << "\n";
        mtl << "Kd " << formatG6(std::clamp(c[0], 0.0f, 1.0f)) << " " << formatG6(std::clamp(c[1], 0.0f, 1.0f)) << " "
            << formatG6(std::clamp(c[2], 0.0f, 1.0f)) << "\n";
        // d is opacity, not transparency -- 1 is solid.
        if (c[3] < 1.0f) mtl << "d " << formatG6(std::clamp(c[3], 0.0f, 1.0f)) << "\n";
        mtl << "\n";
    }
}

void writeOff(const std::string& path, const std::vector<ColoredBody>& bodies) {
    std::optional<manifold::MeshGL> mesh = composeMesh(bodies);
    if (!mesh) throw std::runtime_error("No geometry to export");

    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");

    const size_t vertCount = mesh->vertProperties.size() / mesh->numProp;
    const size_t triCount = mesh->triVerts.size() / 3;
    out << "OFF\n" << vertCount << " " << triCount << " 0\n";
    for (size_t v = 0; v < vertCount; ++v) {
        const size_t base = v * mesh->numProp;
        out << formatG6(mesh->vertProperties[base]) << " " << formatG6(mesh->vertProperties[base + 1]) << " "
            << formatG6(mesh->vertProperties[base + 2]) << "\n";
    }
    for (size_t t = 0; t < triCount; ++t) {
        out << "3 " << mesh->triVerts[t * 3 + 0] << " " << mesh->triVerts[t * 3 + 1] << " " << mesh->triVerts[t * 3 + 2]
            << "\n";
    }
}

namespace {

std::string hexColor(const std::array<float, 4>& rgba) {
    const auto clamp255 = [](float c) {
        const int v = static_cast<int>(std::lround(c * 255.0f));
        return static_cast<uint8_t>(std::clamp(v, 0, 255));
    };
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%02X%02X%02X%02X", clamp255(rgba[0]), clamp255(rgba[1]), clamp255(rgba[2]),
                  clamp255(rgba[3]));
    return std::string(buf);
}

std::vector<uint8_t> toBytes(const std::string& s) { return std::vector<uint8_t>(s.begin(), s.end()); }

} // namespace

void writeThreeMf(const std::string& path, const std::vector<ExportObject>& objects) {
    std::string resources;
    std::string build;
    int nextId = 1;
    bool any = false;

    for (const ExportObject& o : objects) {
        const size_t triCount = o.tris.size() / 3;
        if (triCount == 0) continue;

        const int colorGroupId = nextId++;
        std::vector<std::string> palette;      // distinct colours, first-seen
        std::vector<size_t> triPalette;        // one palette index per triangle
        if (o.triColors.empty()) {
            palette.push_back(hexColor(o.color));
        } else {
            // 3MF's own model is per-triangle SURFACE colour -- the spec is
            // explicit that colour describes the surface, not the
            // distribution of material through the volume -- so a body whose
            // surface came out of a multi-colour CSG merge needs no volume
            // split to be written faithfully.
            triPalette.reserve(triCount);
            for (const auto& c : o.triColors) {
                const std::string h = hexColor(c);
                auto it = std::find(palette.begin(), palette.end(), h);
                if (it == palette.end()) {
                    triPalette.push_back(palette.size());
                    palette.push_back(h);
                } else {
                    triPalette.push_back(static_cast<size_t>(it - palette.begin()));
                }
            }
        }

        resources += "<m:colorgroup id=\"" + std::to_string(colorGroupId) + "\">";
        for (const std::string& h : palette) resources += "<m:color color=\"" + h + "\"/>";
        resources += "</m:colorgroup>";

        const int objectId = nextId++;
        resources += "<object id=\"" + std::to_string(objectId) + "\" type=\"model\" pid=\"" +
                     std::to_string(colorGroupId) + "\" pindex=\"0\"><mesh><vertices>";
        const size_t vertCount = o.verts.size() / 3;
        for (size_t v = 0; v < vertCount; ++v) {
            resources += "<vertex x=\"" + formatG6(o.verts[v * 3]) + "\" y=\"" + formatG6(o.verts[v * 3 + 1]) +
                         "\" z=\"" + formatG6(o.verts[v * 3 + 2]) + "\"/>";
        }
        resources += "</vertices><triangles>";
        for (size_t t = 0; t < triCount; ++t) {
            resources += "<triangle v1=\"" + std::to_string(o.tris[t * 3 + 0]) + "\" v2=\"" +
                         std::to_string(o.tris[t * 3 + 1]) + "\" v3=\"" + std::to_string(o.tris[t * 3 + 2]) + "\"";
            // p1 alone applies to the whole triangle -- the spec requires
            // p2/p3 to be either absent or equal to it, so the shorter form
            // is the correct one for a flat-shaded face.
            if (!triPalette.empty()) {
                resources += " pid=\"" + std::to_string(colorGroupId) + "\" p1=\"" + std::to_string(triPalette[t]) + "\"";
            }
            resources += "/>";
        }
        resources += "</triangles></mesh></object>";

        build += "<item objectid=\"" + std::to_string(objectId) + "\"/>";
        any = true;
    }

    if (!any) throw std::runtime_error("No geometry to export");

    const std::string model = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n"
                               "<model unit=\"millimeter\" xml:lang=\"en-US\" "
                               "xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\" "
                               "xmlns:m=\"http://schemas.microsoft.com/3dmanufacturing/material/2015/02\">"
                               "<resources>" +
                               resources + "</resources><build>" + build + "</build></model>";

    static const std::string kContentTypes =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
        "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
        "<Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>"
        "</Types>";
    static const std::string kRels = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                                      "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
                                      "<Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" "
                                      "Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>"
                                      "</Relationships>";

    std::vector<ZipEntry> entries = {
        {"[Content_Types].xml", toBytes(kContentTypes)},
        {"_rels/.rels", toBytes(kRels)},
        {"3D/3dmodel.model", toBytes(model)},
    };
    writeDeflateZip(path, entries);
}

std::vector<std::string> checkExportBodies(const std::vector<ColoredBody>& bodies) {
    std::vector<std::string> out;
    size_t n = 0;
    for (const auto& b : bodies) {
        ++n;
        if (!b.body || b.body->IsEmpty()) continue;
        const MeshDiagnosis d = checkMesh(b.body->GetMeshGL());
        if (d.ok()) continue;
        out.push_back("part " + std::to_string(n) + " is not a closed manifold solid -- "
                      + d.summary());
    }
    return out;
}


// -- the object split -----------------------------------------------------

// Ported from BelfrySCAD's exporters.py, which was where the colour and
// mesh-repair rules had been worked out; that module is now a shim over
// this. Behaviour is meant to be identical, including the parts that look
// like details but are not -- see the bounding-box skip below.

namespace {

bool isExportable(const ColoredBody& b) {
    // `%` is scenery: drawn so other things can be lined up against it, and
    // excluded from booleans upstream, so letting it reach a file would put
    // geometry there that no boolean ever accounted for.
    return b.role != BodyRole::Background;
}

void meshToArrays(const manifold::MeshGL& mesh, std::vector<float>& verts, std::vector<uint32_t>& tris) {
    const size_t numProp = mesh.numProp ? mesh.numProp : 3;
    const size_t vertCount = mesh.vertProperties.size() / numProp;
    verts.clear();
    verts.reserve(vertCount * 3);
    for (size_t v = 0; v < vertCount; ++v) {
        const size_t base = v * numProp;
        verts.push_back(mesh.vertProperties[base]);
        verts.push_back(mesh.vertProperties[base + 1]);
        verts.push_back(mesh.vertProperties[base + 2]);
    }
    tris = mesh.triVerts;
}

bool boxesOverlap(const manifold::Manifold& a, const manifold::Manifold& b) {
    const manifold::Box ba = a.BoundingBox();
    const manifold::Box bb = b.BoundingBox();
    return !(ba.max.x < bb.min.x || bb.max.x < ba.min.x || ba.max.y < bb.min.y || bb.max.y < ba.min.y ||
             ba.max.z < bb.min.z || bb.max.z < ba.min.z);
}

// A colour key that keeps per-triangle-coloured bodies apart from
// everything, including each other. Their colours index their own triangle
// list, and a union would rewrite that list and lose them.
struct ColorKey {
    bool perTriangle = false;
    size_t index = 0; // position in `solids`, for the per-triangle case
    bool hasColor = false;
    std::array<float, 4> color{};

    bool operator==(const ColorKey& o) const {
        if (perTriangle != o.perTriangle) return false;
        if (perTriangle) return index == o.index;
        if (hasColor != o.hasColor) return false;
        return !hasColor || color == o.color;
    }
};

struct Claimed {
    manifold::Manifold man;
    std::optional<std::array<float, 4>> color;
    std::vector<std::array<float, 4>> triColors;
    std::vector<uint32_t> sourceTris; // what triColors was indexed against
    ColorKey key;
};

manifold::Manifold addAll(const std::vector<manifold::Manifold>& parts) {
    if (parts.size() == 1) return parts[0];
    return manifold::Manifold::BatchBoolean(parts, manifold::OpType::Add);
}

// `triColors` if it still lines up with `outTris`, else empty.
//
// A per-triangle colour array indexes the triangle list it was built
// against, and every boolean rewrites that list -- so the array can only
// survive an object whose triangles came through untouched. Rather than
// reason about which paths are no-ops, this checks: BatchBoolean over a
// single operand and Decompose() of a single component both return the
// triangles unchanged, and anything that actually cut geometry will not
// match. Falling back to empty costs the object its per-triangle detail and
// it exports in its base colour, which is what happened before any of this.
std::vector<std::array<float, 4>> carryTriColors(const std::vector<std::array<float, 4>>& triColors,
                                                  const std::vector<uint32_t>& sourceTris,
                                                  const std::vector<uint32_t>& outTris) {
    if (triColors.empty() || sourceTris.empty()) return {};
    if (triColors.size() * 3 != sourceTris.size()) return {};
    if (sourceTris != outTris) return {};
    return triColors;
}

} // namespace

std::vector<ExportObject> splitBodiesForExport(const std::vector<ColoredBody>& bodies, std::vector<int>* openParts) {
    struct Solid {
        manifold::Manifold man;
        std::optional<std::array<float, 4>> color;
        std::vector<std::array<float, 4>> triColors;
        std::vector<uint32_t> tris;
    };

    std::vector<Solid> solids;
    std::vector<ExportObject> loose;

    int index = 0;
    for (const ColoredBody& cb : bodies) {
        ++index;
        if (!isExportable(cb)) continue;
        if (cb.isDisplayOnly()) {
            // Manifold rejected this one -- an open shell is not a solid --
            // so it can join no boolean. Its triangles are real geometry the
            // user can see, so they are written as-is and reported.
            ExportObject obj;
            meshToArrays(*cb.rawMesh, obj.verts, obj.tris);
            if (obj.tris.empty()) continue;
            obj.color = cb.color.value_or(kDefaultExportColor);
            if (cb.triColors) obj.triColors = *cb.triColors;
            loose.push_back(std::move(obj));
            if (openParts) openParts->push_back(index);
            continue;
        }
        if (!cb.body || cb.body->IsEmpty()) continue;
        Solid s;
        s.man = *cb.body;
        s.color = cb.color;
        if (cb.triColors) s.triColors = *cb.triColors;
        s.tris = s.man.GetMeshGL().triVerts;
        if (s.tris.empty()) continue;
        solids.push_back(std::move(s));
    }

    std::vector<Claimed> claimedGroups;
    if (!solids.empty()) {
        std::vector<ColorKey> keys;
        keys.reserve(solids.size());
        for (size_t i = 0; i < solids.size(); ++i) {
            ColorKey k;
            k.perTriangle = !solids[i].triColors.empty();
            k.index = i;
            k.hasColor = solids[i].color.has_value();
            if (k.hasColor) k.color = *solids[i].color;
            keys.push_back(k);
        }
        const bool allSame = std::all_of(keys.begin(), keys.end(), [&](const ColorKey& k) { return k == keys[0]; });

        if (allSame) {
            // The common case by far, and it needs no per-body subtraction
            // at all: one colour cannot overlap itself into a different
            // answer.
            std::vector<manifold::Manifold> parts;
            parts.reserve(solids.size());
            for (const Solid& s : solids) parts.push_back(s.man);
            Claimed c;
            c.man = addAll(parts);
            c.color = solids[0].color;
            c.triColors = solids[0].triColors;
            c.sourceTris = solids[0].tris;
            c.key = keys[0];
            claimedGroups.push_back(std::move(c));
        } else {
            // Reverse order + subtract-what-is-already-claimed is what makes
            // the LATER body win: by the time an earlier one is reached,
            // everything after it has already taken its volume.
            std::optional<manifold::Manifold> claimed;
            std::vector<Claimed> owned;
            for (size_t n = solids.size(); n-- > 0;) {
                Solid& s = solids[n];
                manifold::Manifold piece = s.man;
                if (claimed) {
                    // Skipping the subtraction when the bounding boxes
                    // cannot overlap is not just a shortcut: `A - disjoint
                    // B` returns A's volume but REORDERS its triangle list,
                    // which throws away any per-triangle colours A carried.
                    // Most models are mostly disjoint parts, so without this
                    // a two-tone body lost its colours the moment any other
                    // differently-coloured body existed.
                    if (boxesOverlap(s.man, *claimed)) piece = s.man - *claimed;
                }
                claimed = claimed ? (*claimed + s.man) : s.man;
                if (piece.IsEmpty()) continue;
                Claimed c;
                c.man = std::move(piece);
                c.color = s.color;
                c.triColors = s.triColors;
                c.sourceTris = s.tris;
                c.key = keys[n];
                owned.push_back(std::move(c));
            }
            std::reverse(owned.begin(), owned.end());

            // Same-coloured pieces merge into one object; distinct colours
            // stay apart. Insertion-ordered so object order still follows
            // the source.
            for (Claimed& c : owned) {
                auto it = std::find_if(claimedGroups.begin(), claimedGroups.end(),
                                        [&](const Claimed& g) { return g.key == c.key; });
                if (it == claimedGroups.end()) {
                    claimedGroups.push_back(std::move(c));
                } else {
                    it->man = addAll({it->man, c.man});
                }
            }
        }
    }

    std::vector<ExportObject> out;
    for (const Claimed& g : claimedGroups) {
        // Decompose() is the rule-3 split. A single-component solid comes
        // back as a one-element list, so there is no special case here.
        std::vector<manifold::Manifold> parts = g.man.Decompose();
        if (parts.empty()) parts.push_back(g.man);
        for (const manifold::Manifold& part : parts) {
            if (part.IsEmpty()) continue;
            ExportObject obj;
            meshToArrays(part.GetMeshGL(), obj.verts, obj.tris);
            if (obj.tris.empty()) continue;
            obj.color = g.color.value_or(kDefaultExportColor);
            obj.triColors = carryTriColors(g.triColors, g.sourceTris, obj.tris);
            out.push_back(std::move(obj));
        }
    }
    out.insert(out.end(), std::make_move_iterator(loose.begin()), std::make_move_iterator(loose.end()));
    return out;
}


// -- STL (ASCII), PLY, VRML, X3D ------------------------------------------

void writeStlAscii(const std::string& path, const std::vector<ColoredBody>& bodies) {
    std::optional<manifold::MeshGL> mesh = composeMesh(bodies);
    if (!mesh) throw std::runtime_error("No geometry to export");

    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");

    const auto vertexAt = [&](uint32_t vertIndex) -> Vec3f {
        const size_t base = static_cast<size_t>(vertIndex) * mesh->numProp;
        return {mesh->vertProperties[base], mesh->vertProperties[base + 1], mesh->vertProperties[base + 2]};
    };
    const auto fmt = [](const Vec3f& v) {
        return formatG6(v.x) + " " + formatG6(v.y) + " " + formatG6(v.z);
    };

    out << "solid OpenSCAD_Model\n";
    for (size_t t = 0; t < mesh->triVerts.size() / 3; ++t) {
        const Vec3f v0 = vertexAt(mesh->triVerts[t * 3 + 0]);
        const Vec3f v1 = vertexAt(mesh->triVerts[t * 3 + 1]);
        const Vec3f v2 = vertexAt(mesh->triVerts[t * 3 + 2]);
        out << "  facet normal " << fmt(normalized(cross(sub(v1, v0), sub(v2, v0)))) << "\n";
        out << "    outer loop\n";
        out << "      vertex " << fmt(v0) << "\n";
        out << "      vertex " << fmt(v1) << "\n";
        out << "      vertex " << fmt(v2) << "\n";
        out << "    endloop\n  endfacet\n";
    }
    out << "endsolid OpenSCAD_Model\n";
}

void writePly(const std::string& path, const std::vector<ExportObject>& objects) {
    // Flatten first so the header can state the counts up front.
    std::vector<float> verts;
    std::vector<uint8_t> colors;
    std::vector<int32_t> faces;
    for (const ExportObject& o : objects) {
        const size_t vertCount = o.verts.size() / 3;
        const size_t triCount = o.tris.size() / 3;
        const int32_t base = static_cast<int32_t>(verts.size() / 3);
        const auto rgb = [](const std::array<float, 4>& c, int i) {
            return static_cast<uint8_t>(std::clamp(static_cast<int>(std::lround(c[i] * 255.0f)), 0, 255));
        };
        if (o.triColors.empty()) {
            verts.insert(verts.end(), o.verts.begin(), o.verts.end());
            for (size_t v = 0; v < vertCount; ++v) {
                colors.push_back(rgb(o.color, 0));
                colors.push_back(rgb(o.color, 1));
                colors.push_back(rgb(o.color, 2));
            }
            for (uint32_t idx : o.tris) faces.push_back(base + static_cast<int32_t>(idx));
        } else {
            // PLY puts colour on vertices, and a vertex shared by two
            // differently-coloured triangles has no single answer -- so an
            // object with per-triangle colour is unwelded: three vertices
            // per triangle, each carrying that triangle's colour. Only the
            // objects that need it pay for it.
            for (size_t t = 0; t < triCount; ++t) {
                for (int k = 0; k < 3; ++k) {
                    const uint32_t vi = o.tris[t * 3 + k];
                    verts.push_back(o.verts[vi * 3]);
                    verts.push_back(o.verts[vi * 3 + 1]);
                    verts.push_back(o.verts[vi * 3 + 2]);
                    colors.push_back(rgb(o.triColors[t], 0));
                    colors.push_back(rgb(o.triColors[t], 1));
                    colors.push_back(rgb(o.triColors[t], 2));
                    faces.push_back(static_cast<int32_t>(faces.size()) + base);
                }
            }
        }
    }

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");
    out << "ply\n"
        << "format binary_little_endian 1.0\n"
        << "comment Written by BelfrySCAD\n"
        << "element vertex " << (verts.size() / 3) << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        << "element face " << (faces.size() / 3) << "\n"
        << "property list uchar int vertex_indices\n"
        << "end_header\n";
    for (size_t v = 0; v < verts.size() / 3; ++v) {
        writeRaw(out, verts[v * 3]);
        writeRaw(out, verts[v * 3 + 1]);
        writeRaw(out, verts[v * 3 + 2]);
        out.put(static_cast<char>(colors[v * 3]));
        out.put(static_cast<char>(colors[v * 3 + 1]));
        out.put(static_cast<char>(colors[v * 3 + 2]));
    }
    for (size_t t = 0; t < faces.size() / 3; ++t) {
        out.put(static_cast<char>(3));
        writeRaw(out, faces[t * 3]);
        writeRaw(out, faces[t * 3 + 1]);
        writeRaw(out, faces[t * 3 + 2]);
    }
}

namespace {

// (palette, one index per face) -- empty palette when the object is a
// single flat colour. Shared by VRML and X3D, which are the same scene
// graph in different syntax.
struct FaceColors {
    std::vector<std::array<float, 4>> palette;
    std::vector<size_t> index;
};

FaceColors faceColors(const ExportObject& o) {
    FaceColors fc;
    if (o.triColors.empty()) return fc;
    for (const auto& c : o.triColors) {
        auto it = std::find_if(fc.palette.begin(), fc.palette.end(), [&](const std::array<float, 4>& p) {
            return p[0] == c[0] && p[1] == c[1] && p[2] == c[2];
        });
        if (it == fc.palette.end()) {
            fc.index.push_back(fc.palette.size());
            fc.palette.push_back(c);
        } else {
            fc.index.push_back(static_cast<size_t>(it - fc.palette.begin()));
        }
    }
    return fc;
}

} // namespace

void writeVrml(const std::string& path, const std::vector<ExportObject>& objects) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");
    out << "#VRML V2.0 utf8\n# Written by BelfrySCAD\n\n";
    for (const ExportObject& o : objects) {
        const FaceColors fc = faceColors(o);
        const float transparency = 1.0f - o.color[3];
        out << "Shape {\n  appearance Appearance {\n    material Material {\n";
        out << "      diffuseColor " << formatG6(o.color[0]) << " " << formatG6(o.color[1]) << " "
            << formatG6(o.color[2]) << "\n";
        // Neither VRML nor X3D can carry per-triangle alpha -- a Color node
        // is RGB only -- so the object's base alpha applies to the shape.
        if (transparency > 0.0f) out << "      transparency " << formatG6(transparency) << "\n";
        out << "    }\n  }\n  geometry IndexedFaceSet {\n    solid TRUE\n";
        out << "    coord Coordinate {\n      point [\n";
        for (size_t v = 0; v < o.verts.size() / 3; ++v) {
            out << "        " << formatG6(o.verts[v * 3]) << " " << formatG6(o.verts[v * 3 + 1]) << " "
                << formatG6(o.verts[v * 3 + 2]) << ",\n";
        }
        out << "      ]\n    }\n";
        if (!fc.palette.empty()) {
            out << "    colorPerVertex FALSE\n    color Color {\n      color [\n";
            for (const auto& c : fc.palette) {
                out << "        " << formatG6(c[0]) << " " << formatG6(c[1]) << " " << formatG6(c[2]) << ",\n";
            }
            out << "      ]\n    }\n    colorIndex [\n";
            // colorIndex, unlike coordIndex, may contain no negative
            // entries: -1 terminates a face there and means nothing here.
            for (size_t i : fc.index) out << "      " << i << ",\n";
            out << "    ]\n";
        }
        out << "    coordIndex [\n";
        for (size_t t = 0; t < o.tris.size() / 3; ++t) {
            out << "      " << o.tris[t * 3] << " " << o.tris[t * 3 + 1] << " " << o.tris[t * 3 + 2] << " -1,\n";
        }
        out << "    ]\n  }\n}\n\n";
    }
}

void writeX3d(const std::string& path, const std::vector<ExportObject>& objects) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<!DOCTYPE X3D PUBLIC \"ISO//Web3D//DTD X3D 3.3//EN\" "
           "\"https://www.web3d.org/specifications/x3d-3.3.dtd\">\n";
    out << "<X3D profile=\"Interchange\" version=\"3.3\">\n";
    out << "  <head>\n    <meta name=\"generator\" content=\"BelfrySCAD\" />\n  </head>\n";
    out << "  <Scene>\n";
    for (const ExportObject& o : objects) {
        const FaceColors fc = faceColors(o);
        const float transparency = 1.0f - o.color[3];
        out << "    <Shape>\n      <Appearance>\n        <Material diffuseColor=\"" << formatG6(o.color[0]) << " "
            << formatG6(o.color[1]) << " " << formatG6(o.color[2]) << "\"";
        if (transparency > 0.0f) out << " transparency=\"" << formatG6(transparency) << "\"";
        out << " />\n      </Appearance>\n";
        out << "      <IndexedFaceSet solid=\"true\"";
        if (!fc.palette.empty()) {
            out << " colorPerVertex=\"false\" colorIndex=\"";
            for (size_t i = 0; i < fc.index.size(); ++i) out << (i ? " " : "") << fc.index[i];
            out << "\"";
        }
        out << " coordIndex=\"";
        for (size_t t = 0; t < o.tris.size() / 3; ++t) {
            out << (t ? " " : "") << o.tris[t * 3] << " " << o.tris[t * 3 + 1] << " " << o.tris[t * 3 + 2] << " -1";
        }
        out << "\">\n        <Coordinate point=\"";
        for (size_t v = 0; v < o.verts.size() / 3; ++v) {
            out << (v ? " " : "") << formatG6(o.verts[v * 3]) << " " << formatG6(o.verts[v * 3 + 1]) << " "
                << formatG6(o.verts[v * 3 + 2]);
        }
        out << "\" />\n";
        if (!fc.palette.empty()) {
            out << "        <Color color=\"";
            for (size_t i = 0; i < fc.palette.size(); ++i) {
                out << (i ? " " : "") << formatG6(fc.palette[i][0]) << " " << formatG6(fc.palette[i][1]) << " "
                    << formatG6(fc.palette[i][2]);
            }
            out << "\" />\n";
        }
        out << "      </IndexedFaceSet>\n    </Shape>\n";
    }
    out << "  </Scene>\n</X3D>\n";
}


// -- one entry point ------------------------------------------------------

namespace {

std::string lowerExtension(const std::string& path) {
    const size_t dot = path.find_last_of('.');
    const size_t slash = path.find_last_of("/\\");
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return "";
    std::string ext = path.substr(dot);
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    return ext;
}

bool isMultiObject(const std::string& ext) {
    return ext == ".3mf" || ext == ".obj" || ext == ".ply" || ext == ".wrl" || ext == ".x3d";
}

// The merged single mesh STL/OFF write, with the open shells kept.
//
// A union, not a concatenation: concatenating is right only while the bodies
// are disjoint -- where two touch, each writes its own copy of the shared
// face, and the file ends up with coincident duplicate faces and edges used
// by four triangles. A Menger sponge is 400 abutting cubes at level 2 and
// came out with 1784 non-manifold edges that way: valid-looking in a viewer,
// rejected or silently "repaired" by a slicer.
//
// Bodies Manifold rejected (an open shell is not a solid) cannot join the
// union, but their triangles are real geometry the user can see, so they are
// concatenated on rather than dropped and their index is reported.
std::optional<manifold::MeshGL> mergeBodies(const std::vector<ColoredBody>& bodies, std::vector<int>* openParts) {
    std::vector<manifold::Manifold> solids;
    std::vector<const manifold::MeshGL*> loose;
    int index = 0;
    for (const ColoredBody& b : bodies) {
        ++index;
        if (b.role == BodyRole::Background) continue;
        if (b.isDisplayOnly()) {
            if (!b.rawMesh->triVerts.empty()) {
                loose.push_back(&*b.rawMesh);
                if (openParts) openParts->push_back(index);
            }
            continue;
        }
        if (b.body && !b.body->IsEmpty()) solids.push_back(*b.body);
    }
    if (solids.empty() && loose.empty()) return std::nullopt;

    manifold::MeshGL out;
    out.numProp = 3;
    uint32_t offset = 0;
    const auto append = [&](const manifold::MeshGL& m) {
        const size_t np = m.numProp ? m.numProp : 3;
        const size_t vertCount = m.vertProperties.size() / np;
        for (size_t v = 0; v < vertCount; ++v) {
            out.vertProperties.push_back(m.vertProperties[v * np]);
            out.vertProperties.push_back(m.vertProperties[v * np + 1]);
            out.vertProperties.push_back(m.vertProperties[v * np + 2]);
        }
        for (uint32_t i : m.triVerts) out.triVerts.push_back(i + offset);
        offset += static_cast<uint32_t>(vertCount);
    };
    if (!solids.empty()) {
        append(manifold::Manifold::BatchBoolean(solids, manifold::OpType::Add).GetMeshGL());
    }
    for (const manifold::MeshGL* m : loose) append(*m);
    return out;
}

void writeStlMesh(const std::string& path, const manifold::MeshGL& mesh, bool ascii) {
    const size_t numProp = mesh.numProp ? mesh.numProp : 3;
    const auto vertexAt = [&](uint32_t vertIndex) -> Vec3f {
        const size_t base = static_cast<size_t>(vertIndex) * numProp;
        return {mesh.vertProperties[base], mesh.vertProperties[base + 1], mesh.vertProperties[base + 2]};
    };
    const size_t triCount = mesh.triVerts.size() / 3;

    if (ascii) {
        std::ofstream out(path);
        if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");
        const auto fmt = [](const Vec3f& v) { return formatG6(v.x) + " " + formatG6(v.y) + " " + formatG6(v.z); };
        out << "solid OpenSCAD_Model\n";
        for (size_t t = 0; t < triCount; ++t) {
            const Vec3f v0 = vertexAt(mesh.triVerts[t * 3 + 0]);
            const Vec3f v1 = vertexAt(mesh.triVerts[t * 3 + 1]);
            const Vec3f v2 = vertexAt(mesh.triVerts[t * 3 + 2]);
            out << "  facet normal " << fmt(normalized(cross(sub(v1, v0), sub(v2, v0)))) << "\n";
            out << "    outer loop\n";
            out << "      vertex " << fmt(v0) << "\n      vertex " << fmt(v1) << "\n      vertex " << fmt(v2) << "\n";
            out << "    endloop\n  endfacet\n";
        }
        out << "endsolid OpenSCAD_Model\n";
        return;
    }

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");
    char header[80] = {};
    out.write(header, sizeof(header));
    writeRaw(out, static_cast<uint32_t>(triCount));
    for (size_t t = 0; t < triCount; ++t) {
        const Vec3f v0 = vertexAt(mesh.triVerts[t * 3 + 0]);
        const Vec3f v1 = vertexAt(mesh.triVerts[t * 3 + 1]);
        const Vec3f v2 = vertexAt(mesh.triVerts[t * 3 + 2]);
        const Vec3f n = normalized(cross(sub(v1, v0), sub(v2, v0)));
        writeRaw(out, n.x); writeRaw(out, n.y); writeRaw(out, n.z);
        writeRaw(out, v0.x); writeRaw(out, v0.y); writeRaw(out, v0.z);
        writeRaw(out, v1.x); writeRaw(out, v1.y); writeRaw(out, v1.z);
        writeRaw(out, v2.x); writeRaw(out, v2.y); writeRaw(out, v2.z);
        writeRaw(out, static_cast<uint16_t>(0));
    }
}

void writeOffMesh(const std::string& path, const manifold::MeshGL& mesh) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");
    const size_t numProp = mesh.numProp ? mesh.numProp : 3;
    const size_t vertCount = mesh.vertProperties.size() / numProp;
    const size_t triCount = mesh.triVerts.size() / 3;
    out << "OFF\n" << vertCount << " " << triCount << " 0\n";
    for (size_t v = 0; v < vertCount; ++v) {
        const size_t base = v * numProp;
        out << formatG6(mesh.vertProperties[base]) << " " << formatG6(mesh.vertProperties[base + 1]) << " "
            << formatG6(mesh.vertProperties[base + 2]) << "\n";
    }
    for (size_t t = 0; t < triCount; ++t) {
        out << "3 " << mesh.triVerts[t * 3 + 0] << " " << mesh.triVerts[t * 3 + 1] << " " << mesh.triVerts[t * 3 + 2]
            << "\n";
    }
}

} // namespace

const std::vector<std::string>& exportExtensions() {
    static const std::vector<std::string> exts = {".3mf", ".stl", ".obj", ".off", ".ply", ".wrl", ".x3d"};
    return exts;
}

std::vector<std::string> exportModel(const std::string& path, const std::vector<ColoredBody>& bodies,
                                      const ExportOptions& opts) {
    std::string ext = opts.format.empty() ? lowerExtension(path) : opts.format;
    if (!ext.empty() && ext[0] != '.') ext = "." + ext;
    const std::vector<std::string>& known = exportExtensions();
    if (std::find(known.begin(), known.end(), ext) == known.end()) {
        throw std::runtime_error("Unsupported export format '" + ext + "'");
    }

    std::vector<std::string> warnings;
    const auto reportOpen = [&](const std::vector<int>& openParts) {
        for (int n : openParts) {
            warnings.push_back("part " + std::to_string(n) +
                                " is not a closed solid; its surface is written as-is, and most slicers will reject it.");
        }
    };

    if (isMultiObject(ext)) {
        // These keep the parts as separate objects, so each is checked on
        // its own -- that is what the file contains.
        for (std::string& w : checkExportBodies(bodies)) warnings.push_back(std::move(w));
        std::vector<int> openParts;
        const std::vector<ExportObject> objects = splitBodiesForExport(bodies, &openParts);
        reportOpen(openParts);
        if (objects.empty()) throw std::runtime_error("No geometry to export");
        if (ext == ".3mf") {
            writeThreeMf(path, objects);
        } else if (ext == ".obj") {
            writeObj(path, objects);
        } else if (ext == ".ply") {
            writePly(path, objects);
        } else if (ext == ".wrl") {
            writeVrml(path, objects);
        } else {
            writeX3d(path, objects);
        }
        return warnings;
    }

    std::vector<int> openParts;
    std::optional<manifold::MeshGL> mesh = mergeBodies(bodies, &openParts);
    reportOpen(openParts);
    if (!mesh) throw std::runtime_error("No geometry to export");

    if (opts.stripSlivers) {
        SliverStripReport report;
        manifold::MeshGL stripped = stripSlivers(*mesh, report);
        if (report.removed > 0) {
            warnings.push_back("removed " + std::to_string(report.removed) +
                                " zero-area face(s) before writing.");
            mesh = std::move(stripped);
        }
    }

    // Checked AFTER merging and stripping, because that is what gets
    // written. Checking the parts instead passed a Menger sponge whose
    // 160,000 cubes were each fine and whose file was riddled with
    // duplicate faces.
    const MeshDiagnosis d = checkMesh(*mesh);
    if (!d.ok()) warnings.push_back("exported mesh " + d.summary());

    if (ext == ".off") {
        writeOffMesh(path, *mesh);
    } else {
        writeStlMesh(path, *mesh, opts.asciiStl);
    }
    return warnings;
}

} // namespace oscadeval
