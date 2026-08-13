#pragma once

#include "openscad_cpp_evaluator/colored_body.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace oscadeval {

// The colour every object falls back to when the script gave none.
inline constexpr std::array<float, 4> kDefaultExportColor = {0.8f, 0.8f, 0.8f, 1.0f};

// One object in a file format that can hold more than one -- 3MF, OBJ,
// VRML, X3D. `triColors` is empty for the ordinary flat-coloured object,
// or one RGBA per triangle when the surface came out of a multi-colour CSG
// merge (see splitBodiesForExport).
struct ExportObject {
    std::vector<float> verts;   // xyz triples
    std::vector<uint32_t> tris; // vertex-index triples
    std::array<float, 4> color = kDefaultExportColor;
    std::vector<std::array<float, 4>> triColors;
};

// The implicit top-level union, cut into objects that never overlap.
//
// Top level in OpenSCAD is an implicit union, so `cube(100); cube(100,
// center=true);` is ONE solid -- writing the bodies as they arrive put both
// cubes in the file separately, overlapping, with their interior faces
// intact. Three rules, in order:
//
//   1. Union, never concatenate. Concatenating is only right while the
//      bodies are disjoint; where two touch, each writes its own copy of
//      the shared face and the result is non-manifold.
//   2. One object per colour, and no two objects share volume. Where
//      differently-coloured solids overlap the LATER one owns the shared
//      volume and the earlier is notched around it -- painter's order, so
//      `color("red") body(); color("blue") detail();` leaves the detail
//      whole. Only the invisible interior is affected: the visible surface
//      is identical either way.
//   3. One object per connected component.
//
// A body Manifold rejected (an open shell is not a solid) can take part in
// none of that: it keeps its own triangles and its own object, and its
// 1-based index is appended to `openParts` for the caller to warn about.
std::vector<ExportObject> splitBodiesForExport(const std::vector<ColoredBody>& bodies,
                                                std::vector<int>* openParts = nullptr);

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
// One message per body that is not a closed manifold solid, ready to print.
// Empty when everything is sound.
//
// Returned rather than logged so each front end can surface it its own way,
// and so export.cpp needs no logging dependency. Nothing here refuses to
// write: a deliberately open surface is a legitimate thing to export, and
// silently blocking a save would be worse than an honest warning.
std::vector<std::string> checkExportBodies(const std::vector<ColoredBody>& bodies);

void writeStl(const std::string& path, const std::vector<ColoredBody>& bodies);

// OpenSCAD-compatible ASCII STL -- "solid OpenSCAD_Model" / one "facet
// normal .. outer loop .. vertex x3 .. endloop endfacet" per triangle /
// "endsolid". Format confirmed against real OpenSCAD's own -o out.stl.
void writeStlAscii(const std::string& path, const std::vector<ColoredBody>& bodies);

// Wavefront OBJ: one `o` group per object, `usemtl` naming a material in a
// companion .mtl written alongside (so an OBJ export produces TWO files).
// A multi-coloured surface becomes runs of faces with a `usemtl` between
// them, in triangle order -- the only per-face colour OBJ has.
void writeObj(const std::string& path, const std::vector<ExportObject>& objects);

// Binary little-endian PLY, one flat mesh with per-vertex colour. PLY has
// no object concept, so the objects are concatenated; an object carrying
// per-triangle colour is unwelded (three vertices per triangle), since a
// vertex shared by two differently-coloured triangles has no single answer.
void writePly(const std::string& path, const std::vector<ExportObject>& objects);

// VRML97 ("#VRML V2.0 utf8"), one Shape per object. Per-face colour rides
// on a Color node with `colorPerVertex FALSE`. Neither this nor X3D can
// carry per-triangle alpha, so the object's base alpha applies to the whole
// shape through Material.transparency (which is 1 - alpha).
void writeVrml(const std::string& path, const std::vector<ExportObject>& objects);

// X3D 3.3, Interchange profile -- the XML encoding of what writeVrml emits,
// node for node. Interchange is the accurate claim: per Annex B it is
// Geometry3D level 2 (IndexedFaceSet), Rendering level 3 (Coordinate,
// Color) and Shape level 1 (Appearance, Material), exactly the nodes used.
void writeX3d(const std::string& path, const std::vector<ExportObject>& objects);

// OFF (Object File Format): header "OFF", "$verts $tris 0", vertex lines,
// then "3 i j k" per triangle (0-indexed, count-prefixed). Mirrors
// export.py's write_off.
void writeOff(const std::string& path, const std::vector<ColoredBody>& bodies);

// 3MF: one mesh object + a base-color colorgroup per body (skipping empty
// bodies), written as a DEFLATE-compressed ZIP (writeDeflateZip, see
// zip_stored.hpp) -- the XML is highly compressible, so storing it would
// make the file several times larger for nothing. An entry that does not
// actually shrink is stored raw, as any ZIP writer would.
// Mirrors export.py's write_3mf's XML shape exactly (core + material
// namespaces, %.6g vertex formatting); throws std::runtime_error if there's
// no geometry to export.
void writeThreeMf(const std::string& path, const std::vector<ExportObject>& objects);

// -- one entry point ------------------------------------------------------

struct ExportOptions {
    // Empty means "decide from the path's extension".
    std::string format;
    // .stl only; binary otherwise.
    bool asciiStl = false;
    // Remove zero-area faces before writing the single-mesh formats. They
    // break no topology, but slicers commonly discard them and are then
    // left with the holes their removal opens.
    bool stripSlivers = true;
};

// Writes `bodies` to `path`, choosing the writer from the extension (or
// opts.format) and applying the same repair/verification policy the GUI
// used to apply itself. Returns the warnings to surface -- open shells,
// mesh problems, sliver removal -- rather than logging them, so each front
// end can present them its own way.
//
// Nothing here refuses to write: a deliberately open surface is a
// legitimate export, and blocking a save the user asked for would be worse
// than saying so. Throws std::runtime_error only when there is no geometry
// at all, the format is unknown, or the file cannot be opened.
std::vector<std::string> exportModel(const std::string& path, const std::vector<ColoredBody>& bodies,
                                      const ExportOptions& opts = {});

// The extensions exportModel understands, lower-case and dot-prefixed.
const std::vector<std::string>& exportExtensions();

} // namespace oscadeval
