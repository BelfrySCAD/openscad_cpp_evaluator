#pragma once

#include "openscad_cpp_evaluator/value.hpp"

#include <array>
#include <string>
#include <vector>

namespace oscadeval {

// Welded verts + triangle-index list, the shared shape every mesh loader
// below returns. Mirrors the reference's (verts, tris) tuple.
struct LoadedMesh {
    std::vector<std::array<double, 3>> verts;
    std::vector<std::array<int, 3>> tris;
    // What the loader read but could not turn into mesh (X3D/VRML primitives,
    // Inline, AMF constellations), for import() to warn about.
    std::vector<std::string> warnings;
};

// Each throws std::runtime_error with a message suitable for wrapping into
// "import: {what}" on parse/IO failure. Mirrors _load_stl/_load_obj/
// _load_off/_load_3mf exactly (including STL's exact-match vertex welding
// and OBJ/OFF's fan triangulation of >3-gon faces).
// Merges vertices at exactly equal positions (STL's corners, an AMF
// object's duplicated vertices); every position keeps its first index.
LoadedMesh weldVertices(const std::vector<std::array<double, 3>>& verts, const std::vector<std::array<int, 3>>& tris);
LoadedMesh loadStl(const std::string& path);
LoadedMesh loadObj(const std::string& path);
LoadedMesh loadOff(const std::string& path);
LoadedMesh loadThreeMf(const std::string& path);
// scene_import.cpp: the three multi-object formats export writes. AMF (plain
// or zipped, `unit` scaled to mm), X3D and VRML97 (Transforms applied,
// DEF/USE resolved, polygons fan-triangulated). Geometry only, like the rest.
LoadedMesh loadAmf(const std::string& path);
LoadedMesh loadX3d(const std::string& path);
LoadedMesh loadVrml(const std::string& path);

} // namespace oscadeval
