#pragma once

#include "openscad_cpp_evaluator/colored_body.hpp"

#include <string>
#include <vector>

namespace oscadeval {

// Writes a binary STL: composes every body's mesh into one solid (bodies
// with no `.body` -- 2D-only sections -- or an empty Manifold are
// skipped), per-triangle normals computed from vertex winding. Mirrors
// export.py's write_stl exactly (80-byte zero header, uint32 LE triangle
// count, 50 bytes/triangle: 3x float32 normal, 3x3 float32 vertices,
// uint16 attr=0). Throws std::runtime_error if there's no 3D geometry to
// export, or if the file can't be opened for writing.
//
// 3MF and format-from-extension dispatch land in Phase 5 per the plan's
// phased build order.
void writeStl(const std::string& path, const std::vector<ColoredBody>& bodies);

// Wavefront OBJ: "v x y z" per vertex then "f i j k" per triangle
// (1-indexed). Mirrors export.py's write_obj (%.6g formatting). Same
// compose-and-throw-if-empty behavior as writeStl.
void writeObj(const std::string& path, const std::vector<ColoredBody>& bodies);

// OFF (Object File Format): header "OFF", "$verts $tris 0", vertex lines,
// then "3 i j k" per triangle (0-indexed, count-prefixed). Mirrors
// export.py's write_off.
void writeOff(const std::string& path, const std::vector<ColoredBody>& bodies);

// 3MF: one mesh object + a base-color colorgroup per body (skipping empty
// bodies), written as a stored (uncompressed) ZIP -- see zip_stored.hpp.
// Mirrors export.py's write_3mf's XML shape exactly (core + material
// namespaces, %.6g vertex formatting); throws std::runtime_error if there's
// no geometry to export.
void writeThreeMf(const std::string& path, const std::vector<ColoredBody>& bodies);

} // namespace oscadeval
