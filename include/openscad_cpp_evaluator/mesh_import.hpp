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
};

// Each throws std::runtime_error with a message suitable for wrapping into
// "import: {what}" on parse/IO failure. Mirrors _load_stl/_load_obj/
// _load_off/_load_3mf exactly (including STL's exact-match vertex welding
// and OBJ/OFF's fan triangulation of >3-gon faces).
LoadedMesh loadStl(const std::string& path);
LoadedMesh loadObj(const std::string& path);
LoadedMesh loadOff(const std::string& path);
LoadedMesh loadThreeMf(const std::string& path);

} // namespace oscadeval
