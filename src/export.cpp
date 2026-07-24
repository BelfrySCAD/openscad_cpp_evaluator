#include "openscad_cpp_evaluator/export.hpp"

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

void writeObj(const std::string& path, const std::vector<ColoredBody>& bodies) {
    std::optional<manifold::MeshGL> mesh = composeMesh(bodies);
    if (!mesh) throw std::runtime_error("No geometry to export");

    std::ofstream out(path);
    if (!out) throw std::runtime_error("Could not open '" + path + "' for writing");

    const size_t vertCount = mesh->vertProperties.size() / mesh->numProp;
    for (size_t v = 0; v < vertCount; ++v) {
        const size_t base = v * mesh->numProp;
        out << "v " << formatG6(mesh->vertProperties[base]) << " " << formatG6(mesh->vertProperties[base + 1]) << " "
            << formatG6(mesh->vertProperties[base + 2]) << "\n";
    }
    out << "\n";
    for (size_t t = 0; t < mesh->triVerts.size() / 3; ++t) {
        out << "f " << (mesh->triVerts[t * 3 + 0] + 1) << " " << (mesh->triVerts[t * 3 + 1] + 1) << " "
            << (mesh->triVerts[t * 3 + 2] + 1) << "\n";
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

void writeThreeMf(const std::string& path, const std::vector<ColoredBody>& bodies) {
    std::string resources;
    std::string build;
    int nextId = 1;
    bool any = false;

    for (const auto& b : bodies) {
        if (!b.body || b.body->IsEmpty()) continue;
        const manifold::MeshGL mesh = b.body->GetMeshGL();
        const size_t triCount = mesh.triVerts.size() / 3;
        if (triCount == 0) continue;

        const int colorGroupId = nextId++;
        const std::array<float, 4> rgba = b.color.value_or(std::array<float, 4>{0.8f, 0.8f, 0.8f, 1.0f});
        resources += "<m:colorgroup id=\"" + std::to_string(colorGroupId) + "\"><m:color color=\"" + hexColor(rgba) +
                     "\"/></m:colorgroup>";

        const int objectId = nextId++;
        resources += "<object id=\"" + std::to_string(objectId) + "\" type=\"model\" pid=\"" +
                     std::to_string(colorGroupId) + "\" pindex=\"0\"><mesh><vertices>";
        const size_t vertCount = mesh.vertProperties.size() / mesh.numProp;
        for (size_t v = 0; v < vertCount; ++v) {
            const size_t base = v * mesh.numProp;
            resources += "<vertex x=\"" + formatG6(mesh.vertProperties[base]) + "\" y=\"" +
                         formatG6(mesh.vertProperties[base + 1]) + "\" z=\"" + formatG6(mesh.vertProperties[base + 2]) +
                         "\"/>";
        }
        resources += "</vertices><triangles>";
        for (size_t t = 0; t < triCount; ++t) {
            resources += "<triangle v1=\"" + std::to_string(mesh.triVerts[t * 3 + 0]) + "\" v2=\"" +
                         std::to_string(mesh.triVerts[t * 3 + 1]) + "\" v3=\"" + std::to_string(mesh.triVerts[t * 3 + 2]) +
                         "\"/>";
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

} // namespace oscadeval
