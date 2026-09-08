#include "openscad_cpp_evaluator/export.hpp"

#include "openscad_cpp_evaluator/mesh_check.hpp"

#include "openscad_cpp_evaluator/css_colors.hpp"
#include "openscad_cpp_evaluator/zip_stored.hpp"

#include <manifold/manifold.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
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

std::vector<ExportObject> splitBodiesForExport(const std::vector<ColoredBody>& bodies, std::vector<int>* openParts,
                                                bool splitComponents) {
    struct Solid {
        manifold::Manifold man;
        std::optional<std::array<float, 4>> color;
        std::vector<std::array<float, 4>> triColors;
        std::vector<uint32_t> tris;
    };

    std::vector<Solid> solids;
    std::vector<ExportObject> loose;

    // A `flatPreview` body is a top-level 2D shape thin-extruded so it can be
    // SEEN (toRenderableBodies). The top level keeps 2D and 3D side by side
    // for exactly that reason -- but a mesh export of a mixed script must
    // not smuggle a 1-unit-tall slab in beside the real solids, which is what
    // the reference drops there too. A 2D-ONLY script still exports its
    // slab, unchanged: that is the only geometry there is, and dropping it
    // would leave nothing to write.
    bool hasRealSolid = false;
    for (const ColoredBody& cb : bodies) {
        if (!cb.flatPreview && isExportable(cb)) { hasRealSolid = true; break; }
    }

    int index = 0;
    for (const ColoredBody& cb : bodies) {
        ++index;
        if (!isExportable(cb)) continue;
        if (cb.flatPreview && hasRealSolid) continue;
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
        // Decompose() is the rule-3 split, and it is opt-in: without it this
        // colour group stays one object however many disjoint pieces it is
        // in, which is what OpenSCAD writes. A single-component solid comes
        // back as a one-element list either way, so there is no special case.
        std::vector<manifold::Manifold> parts;
        if (splitComponents) parts = g.man.Decompose();
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

// AMF is XML like X3D, but its colour model is per-VOLUME rather than
// per-face: a mesh carries one <vertices> list and any number of <volume>
// blocks, each naming a material. So an object whose triangles are not all
// one colour is written as one volume per distinct colour -- <color> on an
// individual <triangle> is legal but far less widely read, and slicers are
// the audience here.
//
// Material and object ids start at 1; id 0 is reserved by the spec.
// The material table is global, so two objects sharing a colour share one
// material entry rather than each declaring its own.
void writeAmf(const std::string& path, const std::vector<ExportObject>& objects) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");

    std::vector<std::array<float, 4>> materials;
    const auto materialFor = [&](const std::array<float, 4>& c) -> size_t {
        for (size_t i = 0; i < materials.size(); ++i) {
            if (materials[i] == c) return i + 1;
        }
        materials.push_back(c);
        return materials.size();
    };

    struct Volume {
        size_t material = 1;
        std::vector<uint32_t> tris;
    };
    std::vector<std::vector<Volume>> perObject;
    perObject.reserve(objects.size());
    for (const ExportObject& o : objects) {
        const FaceColors fc = faceColors(o);
        std::vector<Volume> vols;
        if (fc.palette.empty()) {
            Volume v;
            v.material = materialFor(o.color);
            v.tris = o.tris;
            vols.push_back(std::move(v));
        } else {
            vols.resize(fc.palette.size());
            for (size_t i = 0; i < fc.palette.size(); ++i) vols[i].material = materialFor(fc.palette[i]);
            for (size_t t = 0; t < o.tris.size() / 3; ++t) {
                std::vector<uint32_t>& dst = vols[fc.index[t]].tris;
                dst.push_back(o.tris[t * 3]);
                dst.push_back(o.tris[t * 3 + 1]);
                dst.push_back(o.tris[t * 3 + 2]);
            }
        }
        perObject.push_back(std::move(vols));
    }

    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<amf unit=\"millimeter\" version=\"1.1\">\n";
    out << "  <metadata type=\"cad\">BelfrySCAD</metadata>\n";
    for (size_t i = 0; i < materials.size(); ++i) {
        const std::array<float, 4>& c = materials[i];
        out << "  <material id=\"" << (i + 1) << "\">\n";
        out << "    <color><r>" << formatG6(c[0]) << "</r><g>" << formatG6(c[1]) << "</g><b>"
            << formatG6(c[2]) << "</b>";
        if (c[3] < 1.0f) out << "<a>" << formatG6(c[3]) << "</a>";
        out << "</color>\n";
        out << "  </material>\n";
    }
    for (size_t oi = 0; oi < objects.size(); ++oi) {
        const ExportObject& o = objects[oi];
        out << "  <object id=\"" << (oi + 1) << "\">\n    <mesh>\n      <vertices>\n";
        for (size_t v = 0; v < o.verts.size() / 3; ++v) {
            out << "        <vertex><coordinates><x>" << formatG6(o.verts[v * 3]) << "</x><y>"
                << formatG6(o.verts[v * 3 + 1]) << "</y><z>" << formatG6(o.verts[v * 3 + 2])
                << "</z></coordinates></vertex>\n";
        }
        out << "      </vertices>\n";
        for (const Volume& vol : perObject[oi]) {
            if (vol.tris.empty()) continue;
            out << "      <volume materialid=\"" << vol.material << "\">\n";
            for (size_t t = 0; t < vol.tris.size() / 3; ++t) {
                out << "        <triangle><v1>" << vol.tris[t * 3] << "</v1><v2>" << vol.tris[t * 3 + 1]
                    << "</v2><v3>" << vol.tris[t * 3 + 2] << "</v3></triangle>\n";
            }
            out << "      </volume>\n";
        }
        out << "    </mesh>\n  </object>\n";
    }
    out << "</amf>\n";
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
    return ext == ".3mf" || ext == ".amf" || ext == ".obj" || ext == ".ply" || ext == ".wrl" || ext == ".x3d";
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

namespace {

// The contours of one 2D body, projected to XY.
//
// A section can carry a transform a CrossSection itself cannot hold -- a Z
// offset, a rotation out of the plane (ColoredBody::sectionXform). Applying
// it and keeping x/y is the top view, which is the only thing a flat page
// can show; for the overwhelmingly common case (no such transform) it is
// the identity and the points pass through untouched.
manifold::Polygons sectionContours(const ColoredBody& cb) {
    manifold::Polygons polys = cb.section->ToPolygons();
    const manifold::mat3x4& m = cb.sectionXform;
    for (manifold::SimplePolygon& contour : polys) {
        for (manifold::vec2& p : contour) {
            const double x = m[0].x * p.x + m[1].x * p.y + m[3].x;
            const double y = m[0].y * p.x + m[1].y * p.y + m[3].y;
            p = manifold::vec2(x, y);
        }
    }
    return polys;
}

// Every 2D writer's input: the contours of an all-2D model, plus their
// bounding box. Shared so the 2D rule -- and its refusal -- has ONE
// implementation: SVG and PDF must agree about what they will draw.
struct Flat2d {
    std::vector<manifold::Polygons> perBody;
    double minx = 0, miny = 0, maxx = 0, maxy = 0;
};

Flat2d collect2d(const std::vector<ColoredBody>& bodies) {
    // OpenSCAD's own wording, and its rule: 2D export needs an all-2D top
    // level. There is no sane projection to fall back on -- a silhouette
    // would be a different model from the one the script describes -- so
    // refusing is the honest answer.
    const char* kNot2d = "Current top level object is not a 2D object";

    Flat2d out;
    bool any = false;
    for (const ColoredBody& cb : bodies) {
        if (!isExportable(cb)) continue;
        // The section decides, not the Manifold: a body that has been
        // through toRenderableBodies() carries BOTH -- a 1-unit slab
        // extruded so a 2D script has something to show, and the contours
        // it came from. Those contours are the real geometry, and the
        // binding hands export that same converted list, so reading `body`
        // first would make 2D export unreachable for every 2D script.
        if (!cb.section) {
            if (cb.isDisplayOnly()) throw std::runtime_error(kNot2d);
            if (cb.body && !cb.body->IsEmpty()) throw std::runtime_error(kNot2d);
            continue;
        }
        manifold::Polygons polys = sectionContours(cb);
        for (const manifold::SimplePolygon& contour : polys) {
            for (const manifold::vec2& p : contour) {
                if (!any) {
                    out.minx = out.maxx = p.x;
                    out.miny = out.maxy = p.y;
                    any = true;
                } else {
                    out.minx = std::min(out.minx, p.x);
                    out.maxx = std::max(out.maxx, p.x);
                    out.miny = std::min(out.miny, p.y);
                    out.maxy = std::max(out.maxy, p.y);
                }
            }
        }
        out.perBody.push_back(std::move(polys));
    }
    if (!any) throw std::runtime_error("No geometry to export");
    return out;
}

} // namespace

void writeSvg(const std::string& path, const std::vector<ColoredBody>& bodies, const ExportSvgOptions& opts) {
    const Flat2d flat = collect2d(bodies);
    const std::vector<manifold::Polygons>& perBody = flat.perBody;
    const double minx = flat.minx, miny = flat.miny, maxx = flat.maxx, maxy = flat.maxy;

    // Page = bounding box, padded by half the stroke (the stroke straddles
    // the contour, so half of it lies outside), rounded outward to whole
    // millimetres. Note the Y terms use -maxy/-miny: the negation below
    // turns the TOP of the model into the most negative coordinate.
    const double pad = opts.stroke ? opts.strokeWidth / 2.0 : 0.0;
    const long left = static_cast<long>(std::floor(minx - pad));
    const long top = static_cast<long>(std::floor(-maxy - pad));
    const long right = static_cast<long>(std::ceil(maxx + pad));
    const long bottom = static_cast<long>(std::ceil(-miny + pad));
    const long width = right - left;
    const long height = bottom - top;

    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");

    const std::string stroke = opts.stroke ? opts.strokeColor : "none";
    const std::string fill = opts.fill ? opts.fillColor : "none";

    out << "<?xml version=\"1.0\" standalone=\"no\"?>\n"
        << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" "
            "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n"
        << "<svg width=\"" << width << "mm\" height=\"" << height << "mm\" viewBox=\"" << left << " " << top
        << " " << width << " " << height << "\" xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">\n"
        << "<title>OpenSCAD Model</title>\n";

    for (const manifold::Polygons& polys : perBody) {
        out << "<path d=\"\n";
        for (const manifold::SimplePolygon& contour : polys) {
            if (contour.empty()) continue;
            out << "M " << contour[0].x << "," << -contour[0].y;
            for (size_t i = 1; i < contour.size(); ++i) {
                out << " L " << contour[i].x << "," << -contour[i].y;
                if ((i % 6) == 5) out << "\n";
            }
            out << " z\n";
        }
        out << "\" stroke=\"" << stroke << "\" fill=\"" << fill << "\" stroke-width=\"" << opts.strokeWidth
            << "\"/>\n";
    }
    out << "</svg>\n";
}

namespace {

// PDF's own unit is 1/72 inch; models are millimetres.
constexpr double kPtPerMm = 72.0 / 25.4;
// Page margin the ruler is drawn on, in points. OpenSCAD's own MARGIN.
constexpr double kPdfMargin = 30.0;
// Ruler tick length, in millimetres of model space.
constexpr double kTickMm = 5.0;
// Ticks every 10mm, labels on every second one. Hard-coded in OpenSCAD
// too: `grid-size` drives the optional grid, not the ruler.
constexpr double kTickStepMm = 10.0;
constexpr int kLabelEveryNTicks = 2;

// Width x height in points, indexed by ExportPdfOptions::Paper.
constexpr int kPaperDims[7][2] = {
    {298, 420},   // A6
    {420, 595},   // A5
    {595, 842},   // A4
    {842, 1190},  // A3
    {612, 792},   // Letter
    {612, 1008},  // Legal
    {792, 1224},  // Tabloid
};

std::string lowerCopy(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return std::tolower(c); });
    return out;
}

// Numbers in a content stream: short, locale-independent, no exponent.
// PDF has no "1e-05" -- a writer that emits one produces a file readers
// silently mis-draw.
std::string pdfNum(double v) {
    if (!std::isfinite(v)) v = 0.0;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.4f", v);
    std::string s(buf);
    const size_t dot = s.find('.');
    if (dot != std::string::npos) {
        size_t last = s.find_last_not_of('0');
        if (last == dot) last = dot - 1;
        s.erase(last + 1);
    }
    if (s == "-0") s = "0";
    return s;
}

// Escapes a PDF literal string's three special bytes. Anything non-ASCII
// is dropped rather than guessed at: the base-14 fonts are single-byte,
// and a mangled byte would draw a wrong glyph instead of nothing.
std::string pdfString(const std::string& text) {
    std::string out = "(";
    for (unsigned char c : text) {
        if (c == '(' || c == ')' || c == '\\') {
            out += '\\';
            out += static_cast<char>(c);
        } else if (c >= 0x20 && c < 0x7f) {
            out += static_cast<char>(c);
        }
    }
    out += ')';
    return out;
}

// "D:YYYYMMDDHHmmSS+00'00'", the only date syntax PDF defines.
std::string pdfDateNow() {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    char buf[64];
    std::strftime(buf, sizeof(buf), "D:%Y%m%d%H%M%S+00'00'", &utc);
    return buf;
}

// Grey level that reads like OpenSCAD's translucent black on white paper.
// Cairo strokes its ruler at alpha 0.6 and its caption at 0.48; on a white
// page those are indistinguishable from these greys, and a grey needs no
// ExtGState -- so the file stays one content stream with no transparency
// group. Over anything but white they would differ, which no page here has.
constexpr double kAxisGrey = 0.4;   // 1 - 0.6
constexpr double kTextGrey = 0.52;  // 1 - 0.48
constexpr double kGridLightGrey = 0.6;

struct PdfText {
    double x, y, size;
    std::string text;
};

void appendMoveLine(std::string& out, double x0, double y0, double x1, double y1) {
    out += pdfNum(x0) + " " + pdfNum(y0) + " m " + pdfNum(x1) + " " + pdfNum(y1) + " l S\n";
}

void appendText(std::string& out, const PdfText& t) {
    out += "BT /F1 " + pdfNum(t.size) + " Tf 1 0 0 1 " + pdfNum(t.x) + " " + pdfNum(t.y) + " Tm " +
            pdfString(t.text) + " Tj ET\n";
}

// zlib stream for /FlateDecode -- exactly what stbi_zlib_compress emits
// (RFC 1950: 2-byte header, DEFLATE data, Adler-32), already vendored for
// surface()'s PNG writing. Empty on failure, and the caller then writes the
// stream raw: an uncompressed content stream is just as valid, only bigger.
std::vector<uint8_t> zlibStream(const std::string& data) {
    return zlibCompress(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

} // namespace

bool paperFromName(const std::string& name, ExportPdfOptions::Paper& out) {
    const std::string n = lowerCopy(name);
    if (n == "a6") out = ExportPdfOptions::Paper::A6;
    else if (n == "a5") out = ExportPdfOptions::Paper::A5;
    else if (n == "a4") out = ExportPdfOptions::Paper::A4;
    else if (n == "a3") out = ExportPdfOptions::Paper::A3;
    else if (n == "letter") out = ExportPdfOptions::Paper::Letter;
    else if (n == "legal") out = ExportPdfOptions::Paper::Legal;
    else if (n == "tabloid") out = ExportPdfOptions::Paper::Tabloid;
    else return false;
    return true;
}

bool orientationFromName(const std::string& name, ExportPdfOptions::Orientation& out) {
    const std::string n = lowerCopy(name);
    if (n == "portrait") out = ExportPdfOptions::Orientation::Portrait;
    else if (n == "landscape") out = ExportPdfOptions::Orientation::Landscape;
    else if (n == "auto") out = ExportPdfOptions::Orientation::Auto;
    else return false;
    return true;
}

std::vector<std::string> writePdf(const std::string& path, const std::vector<ColoredBody>& bodies,
                                   const ExportPdfOptions& opts) {
    const Flat2d flat = collect2d(bodies);
    std::vector<std::string> warnings;

    // -- page ------------------------------------------------------------
    const int paperIdx = static_cast<int>(opts.paper);
    const double spanXpt = (flat.maxx - flat.minx) * kPtPerMm;
    const double spanYpt = (flat.maxy - flat.miny) * kPtPerMm;
    const bool landscape = opts.orientation == ExportPdfOptions::Orientation::Landscape ||
                            (opts.orientation == ExportPdfOptions::Orientation::Auto && spanXpt > spanYpt);
    const double pageW = kPaperDims[paperIdx][landscape ? 1 : 0];
    const double pageH = kPaperDims[paperIdx][landscape ? 0 : 1];

    if (spanXpt > pageW - 2 * kPdfMargin || spanYpt > pageH - 2 * kPdfMargin) {
        warnings.push_back("geometry is larger than the printable area of the selected paper size; "
                            "it is drawn anyway and will run off the page.");
    }

    // Model mm -> page pt. PDF is Y-up like model space, so this is a
    // scale and a translation in BOTH axes -- no negation anywhere, unlike
    // the SVG writer. The model is centred exactly; OpenSCAD's own centring
    // is off by a fraction of a point because it truncates the span to an
    // int on the way, which is a bug rather than something to reproduce.
    const double tx = pageW / 2.0 - (flat.minx + flat.maxx) / 2.0 * kPtPerMm;
    const double ty = pageH / 2.0 - (flat.miny + flat.maxy) / 2.0 * kPtPerMm;
    const auto X = [&](double mm) { return mm * kPtPerMm + tx; };
    const auto Y = [&](double mm) { return mm * kPtPerMm + ty; };
    // ... and back, for turning a page margin into the model coordinate
    // the ruler has to label there.
    const auto mmAtX = [&](double pt) { return (pt - tx) / kPtPerMm; };
    const auto mmAtY = [&](double pt) { return (pt - ty) / kPtPerMm; };

    const double ml = kPdfMargin, mr = pageW - kPdfMargin;
    const double mb = kPdfMargin, mt = pageH - kPdfMargin;

    // -- content ----------------------------------------------------------
    std::string cs;
    cs.reserve(4096);

    // The model itself.
    for (const manifold::Polygons& polys : flat.perBody) {
        std::string path_;
        for (const manifold::SimplePolygon& contour : polys) {
            if (contour.empty()) continue;
            path_ += pdfNum(X(contour[0].x)) + " " + pdfNum(Y(contour[0].y)) + " m\n";
            for (size_t i = 1; i < contour.size(); ++i) {
                path_ += pdfNum(X(contour[i].x)) + " " + pdfNum(Y(contour[i].y)) + " l\n";
            }
            path_ += "h\n";
        }
        if (path_.empty()) continue;
        cs += "q\n";
        if (opts.fill) {
            const std::array<double, 4> c = cssColor(opts.fillColor);
            cs += pdfNum(c[0]) + " " + pdfNum(c[1]) + " " + pdfNum(c[2]) + " rg\n";
        }
        if (opts.stroke) {
            const std::array<double, 4> c = cssColor(opts.strokeColor);
            cs += pdfNum(c[0]) + " " + pdfNum(c[1]) + " " + pdfNum(c[2]) + " RG\n";
            cs += pdfNum(opts.strokeWidth * kPtPerMm) + " w\n";
        }
        cs += path_;
        // Holes come out right under the nonzero winding rule because
        // Manifold hands back outer contours and holes wound oppositely.
        if (opts.fill && opts.stroke) cs += "B\n";
        else if (opts.fill) cs += "f\n";
        else if (opts.stroke) cs += "S\n";
        else cs += "n\n";
        cs += "Q\n";
    }

    std::vector<PdfText> texts;

    if (opts.showScale) {
        cs += "q\n";
        cs += pdfNum(kAxisGrey) + " G\n0.36 w\n";
        // Two axes, not a frame: the left edge and the bottom edge of the
        // margin box, which is what OpenSCAD draws.
        appendMoveLine(cs, ml, mb, ml, mt);
        appendMoveLine(cs, ml, mb, mr, mb);

        const double tick = kTickMm * kPtPerMm;
        // Ticks land on model multiples of 10mm, NOT on page divisions --
        // that is what makes the printed ruler measure the model. The `0`
        // tick therefore sits exactly on the model origin, and labels run
        // negative wherever the page extends past it.
        const long xFirst = static_cast<long>(std::ceil(mmAtX(ml) / kTickStepMm));
        const long xLast = static_cast<long>(std::floor(mmAtX(mr) / kTickStepMm));
        for (long i = xFirst; i <= xLast; ++i) {
            const double px = X(i * kTickStepMm);
            appendMoveLine(cs, px, mb, px, mb - tick);
            if (i % kLabelEveryNTicks == 0) {
                texts.push_back({px + 1.0, mb - tick + 2.0, 6.0, std::to_string(i * (long)kTickStepMm)});
            }
        }
        const long yFirst = static_cast<long>(std::ceil(mmAtY(mb) / kTickStepMm));
        const long yLast = static_cast<long>(std::floor(mmAtY(mt) / kTickStepMm));
        for (long i = yFirst; i <= yLast; ++i) {
            const double py = Y(i * kTickStepMm);
            appendMoveLine(cs, ml, py, ml - tick, py);
            if (i % kLabelEveryNTicks == 0) {
                texts.push_back({ml - tick, py + 3.0, 6.0, std::to_string(i * (long)kTickStepMm)});
            }
        }

        if (opts.showGrid) {
            // The one place gridSize is used. OpenSCAD's own clamp, and its
            // "major line" rule, which is deliberately asymmetric: below
            // 10mm every (10/gridSize)th line is heavier, above it every
            // gridSize'th.
            double g = opts.gridSize < 1.0 ? 2.0 : opts.gridSize;
            const long major = static_cast<long>(g > 10.0 ? g : static_cast<long>(10.0 / g));
            const long gx0 = static_cast<long>(std::ceil(mmAtX(ml) / g));
            const long gx1 = static_cast<long>(std::floor(mmAtX(mr) / g));
            for (long i = gx0; i <= gx1; ++i) {
                const bool heavy = major > 0 && (i % major) == 0;
                cs += pdfNum(heavy ? kAxisGrey : kGridLightGrey) + " G\n" + (heavy ? "0.36" : "0.24") + " w\n";
                appendMoveLine(cs, X(i * g), mb, X(i * g), mt);
            }
            const long gy0 = static_cast<long>(std::ceil(mmAtY(mb) / g));
            const long gy1 = static_cast<long>(std::floor(mmAtY(mt) / g));
            for (long i = gy0; i <= gy1; ++i) {
                const bool heavy = major > 0 && (i % major) == 0;
                cs += pdfNum(heavy ? kAxisGrey : kGridLightGrey) + " G\n" + (heavy ? "0.36" : "0.24") + " w\n";
                appendMoveLine(cs, ml, Y(i * g), mr, Y(i * g));
            }
        }
        cs += "Q\n";

        if (opts.showScaleMsg) {
            // Without this the ruler is a mystery. It is the instruction
            // for the job the format exists to do.
            texts.push_back({ml + 1.0, mb + 2.0, 5.0,
                              "Scale is to calibrate actual printed dimension. Check both X and Y. "
                              "Measure between tick 0 and last tick"});
        }
    }

    if (opts.showFilename && !opts.designFilename.empty()) {
        texts.push_back({ml, mb - kTickMm * kPtPerMm - 10.0, 10.0, opts.designFilename});
    }

    if (!texts.empty()) {
        cs += "q\n" + pdfNum(kTextGrey) + " g\n";
        for (const PdfText& t : texts) appendText(cs, t);
        cs += "Q\n";
    }

    // -- objects ----------------------------------------------------------
    std::vector<uint8_t> stream = zlibStream(cs);
    const bool deflated = !stream.empty() && stream.size() < cs.size();
    if (!deflated) stream.assign(cs.begin(), cs.end());

    std::string info;
    if (opts.addMetaData) {
        info = "<< /Producer " + pdfString("BelfrySCAD (openscad_cpp_evaluator)") + " /CreationDate " +
                pdfString(pdfDateNow());
        if (!opts.metaTitle.empty()) info += " /Title " + pdfString(opts.metaTitle);
        if (!opts.metaAuthor.empty()) info += " /Author " + pdfString(opts.metaAuthor);
        if (!opts.metaSubject.empty()) info += " /Subject " + pdfString(opts.metaSubject);
        if (!opts.metaKeywords.empty()) info += " /Keywords " + pdfString(opts.metaKeywords);
        info += " >>";
    }

    std::vector<std::string> objects;
    objects.push_back("<< /Type /Catalog /Pages 2 0 R >>");
    objects.push_back("<< /Type /Pages /Kids [3 0 R] /Count 1 >>");
    objects.push_back("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 " + pdfNum(pageW) + " " + pdfNum(pageH) +
                       "] /Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>");
    objects.push_back("");  // 4: the content stream, assembled below
    // Base-14 Helvetica: supplied by every PDF reader, so nothing is
    // embedded and no font metrics are needed to place left-aligned text.
    objects.push_back("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>");
    if (!info.empty()) objects.push_back(info);

    std::string body = "%PDF-1.4\n%\xE2\xE3\xCF\xD3\n";
    std::vector<size_t> offsets(objects.size(), 0);
    for (size_t i = 0; i < objects.size(); ++i) {
        offsets[i] = body.size();
        body += std::to_string(i + 1) + " 0 obj\n";
        if (i == 3) {
            body += "<< /Length " + std::to_string(stream.size()) +
                     (deflated ? " /Filter /FlateDecode" : "") + " >>\nstream\n";
            body.append(reinterpret_cast<const char*>(stream.data()), stream.size());
            body += "\nendstream\n";
        } else {
            body += objects[i] + "\n";
        }
        body += "endobj\n";
    }

    const size_t xrefAt = body.size();
    body += "xref\n0 " + std::to_string(objects.size() + 1) + "\n0000000000 65535 f \n";
    for (size_t off : offsets) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%010zu 00000 n \n", off);
        body += buf;
    }
    body += "trailer\n<< /Size " + std::to_string(objects.size() + 1) + " /Root 1 0 R";
    if (!info.empty()) body += " /Info " + std::to_string(objects.size()) + " 0 R";
    body += " >>\nstartxref\n" + std::to_string(xrefAt) + "\n%%EOF\n";

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");
    out.write(body.data(), static_cast<std::streamsize>(body.size()));
    return warnings;
}

const std::vector<std::string>& exportExtensions() {
    static const std::vector<std::string> exts = {".3mf", ".amf", ".pdf", ".stl", ".obj",
                                                  ".off", ".ply", ".svg", ".wrl", ".x3d"};
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

    if (ext == ".pdf") {
        // 2D like SVG, and equally outside the mesh pipeline.
        return writePdf(path, bodies, opts.pdf);
    }

    if (ext == ".svg") {
        // 2D, and nothing the mesh pipeline below does applies: no merge,
        // no sliver strip, no manifoldness check. Contours are what they
        // are.
        writeSvg(path, bodies, opts.svg);
        return warnings;
    }

    if (isMultiObject(ext)) {
        // These keep the parts as separate objects, so each is checked on
        // its own -- that is what the file contains.
        for (std::string& w : checkExportBodies(bodies)) warnings.push_back(std::move(w));
        std::vector<int> openParts;
        const std::vector<ExportObject> objects = splitBodiesForExport(bodies, &openParts, opts.splitComponents);
        reportOpen(openParts);
        if (objects.empty()) throw std::runtime_error("No geometry to export");
        if (ext == ".3mf") {
            writeThreeMf(path, objects);
        } else if (ext == ".amf") {
            writeAmf(path, objects);
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
