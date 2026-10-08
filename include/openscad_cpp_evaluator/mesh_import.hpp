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
// "import: {what}" on parse/IO failure. OBJ/OFF fan-triangulate >3-gon faces.
//
// Every loader merges vertices at exactly the same position, at the
// granularity OpenSCAD 2026.02.01 does (or, for X3D/VRML, which it cannot
// import, the same as AMF): the whole file for STL, OBJ and OFF; each
// object for 3MF and AMF; each Shape's geometry for X3D and VRML. Where the
// unit is an object, two open halves in separate objects stay open (an
// OBJ's `o` groups are not objects in this sense: OpenSCAD welds across
// them). Near-coincident vertices are never merged.
//
// weldVertices merges exactly-equal positions; every position keeps its
// first index. Throws on a face index past the vertex list.
LoadedMesh weldVertices(const std::vector<std::array<double, 3>>& verts, const std::vector<std::array<int, 3>>& tris);
// Welds `part` and appends it to `out`, keeping `out`'s earlier objects apart.
void appendWelded(LoadedMesh& out, const LoadedMesh& part);
LoadedMesh loadStl(const std::string& path);
LoadedMesh loadObj(const std::string& path);
LoadedMesh loadOff(const std::string& path);
LoadedMesh loadThreeMf(const std::string& path);
// scene_import.cpp: the three multi-object formats export writes. AMF (plain
// or zipped, `unit` ignored as OpenSCAD ignores it), X3D and VRML97
// (Transforms applied, DEF/USE resolved, polygons fan-triangulated).
// Geometry only, like the rest.
LoadedMesh loadAmf(const std::string& path);
LoadedMesh loadX3d(const std::string& path);
LoadedMesh loadVrml(const std::string& path);

} // namespace oscadeval
