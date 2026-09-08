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
//   3. One object per connected component -- OPTIONAL, and off by default.
//      Rules 1 and 2 are about correctness; this one is presentation, and
//      it is the one that diverges from OpenSCAD, which writes a single
//      object however many disjoint pieces the model has. Splitting turned
//      `cube(10); translate([20,0,0]) cube(10);` into two 3MF objects and a
//      multiboard tile into hundreds, which slicers list individually and
//      Prusa Slicer complains about (issue #319). Ask for it when separate
//      pieces are what you want; leave it off to match OpenSCAD.
//
// A body Manifold rejected (an open shell is not a solid) can take part in
// none of that: it keeps its own triangles and its own object, and its
// 1-based index is appended to `openParts` for the caller to warn about.
std::vector<ExportObject> splitBodiesForExport(const std::vector<ColoredBody>& bodies,
                                                std::vector<int>* openParts = nullptr,
                                                bool splitComponents = false);

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

// AMF 1.1. XML like X3D, but its colour lives on the <volume>, not the
// face: a mesh holds one vertex list and one <volume> per material. An
// object whose triangles are not all one colour therefore becomes several
// volumes -- which is what a multimaterial slicer reads as separate
// materials to assign to tools.
void writeAmf(const std::string& path, const std::vector<ExportObject>& objects);

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

// -- 2D ------------------------------------------------------------------

// Options for the SVG writer, named and defaulted after OpenSCAD's own
// `-O export-svg/...` set. `stroke`/`strokeWidth` are not purely cosmetic:
// half the stroke width pads the page, so turning the stroke off shrinks
// the document.
struct ExportSvgOptions {
    bool fill = false;
    std::string fillColor = "white";
    bool stroke = true;
    std::string strokeColor = "black";
    double strokeWidth = 0.35;
};

// SVG 1.1 of a 2D-only model, in millimetres at 1:1 -- so a print of it
// measures what the script says. Every contour of a body, holes included,
// becomes a `M .. L .. z` subpath of that body's single `<path>`.
//
// Model space is Y-up and SVG's is Y-down, so the whole point transform is
// a Y negation. The page is the bounding box padded by half the stroke
// width and rounded OUTWARD to whole millimetres, which is why the margin
// is not a constant (0 to just under 1mm per side, depending where the
// bounds fall). Both rules are OpenSCAD's, read from its export_svg.cc and
// checked against its binary.
//
// Throws std::runtime_error if any top-level body is 3D -- OpenSCAD's rule,
// and the only honest answer, since there is no projection to fall back on.
void writeSvg(const std::string& path, const std::vector<ColoredBody>& bodies,
              const ExportSvgOptions& opts = {});

// Options for the PDF writer, named and defaulted after OpenSCAD's own
// `-O export-pdf/...` set.
struct ExportPdfOptions {
    enum class Paper { A6, A5, A4, A3, Letter, Legal, Tabloid };
    // AUTO picks landscape when the model is wider than it is tall.
    enum class Orientation { Portrait, Landscape, Auto };

    Paper paper = Paper::A4;
    Orientation orientation = Orientation::Portrait;
    // The ruler, its labels and the caption that explains them. This is
    // what makes a printed page a measuring tool rather than a plot, and
    // is why the format was asked for at all.
    bool showScale = true;
    bool showScaleMsg = true;
    bool showGrid = false;
    double gridSize = 10.0;
    // Drawn bottom-left when set. The DESIGN's name -- the writer has no
    // way to know it, so a caller that wants it must pass it.
    bool showFilename = false;
    std::string designFilename;
    bool fill = false;
    std::string fillColor = "black";
    bool stroke = true;
    std::string strokeColor = "black";
    double strokeWidth = 0.35;
    bool addMetaData = true;
    std::string metaTitle;
    std::string metaAuthor;
    std::string metaSubject;
    std::string metaKeywords;
};

// "a4"/"letter"/... (case-insensitive) -> Paper. Returns false for a name
// it does not know, leaving `out` untouched.
bool paperFromName(const std::string& name, ExportPdfOptions::Paper& out);
// "portrait"/"landscape"/"auto" -> Orientation, same contract.
bool orientationFromName(const std::string& name, ExportPdfOptions::Orientation& out);

// PDF 1.4 of a 2D-only model, drawn at 1:1 and CENTRED on a fixed paper
// size -- unlike SVG, where the page is cut to fit the model.
//
// PDF user space is Y-up like model space, so unlike writeSvg there is NO
// Y negation: the placement is `pt = mm * 72/25.4` plus a centring offset,
// symmetric in both axes. (OpenSCAD's own file negates Y only because
// Cairo draws Y-down and then flips the whole page.)
//
// With `showScale` on it also draws OpenSCAD's ruler: a left and bottom
// axis at a 30pt margin, ticks every 10mm of MODEL space (hard-coded --
// `gridSize` belongs to the optional grid, not to this), numeric labels on
// every second tick, and the caption explaining what to measure. That
// ruler is the point of the format: it is what lets a printed page tell
// you how far off your printer's scaling is.
//
// Text uses the base-14 Helvetica every PDF reader supplies, so nothing is
// embedded and no font metrics are needed.
//
// Returns the warnings to surface (a model too large for the page is drawn
// anyway, and said so). Throws std::runtime_error if any top-level body is
// 3D, if there is nothing to draw, or if the file cannot be opened.
std::vector<std::string> writePdf(const std::string& path, const std::vector<ColoredBody>& bodies,
                                   const ExportPdfOptions& opts = {});

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
    // Rule 3 of splitBodiesForExport: give every disconnected piece its own
    // object in the multi-object formats (3MF, AMF, OBJ, PLY, VRML, X3D).
    // Off matches OpenSCAD, which writes one object per colour however many
    // pieces it is in; see that function's own comment for why the default
    // moved here.
    bool splitComponents = false;
    // .svg only.
    ExportSvgOptions svg;
    // .pdf only.
    ExportPdfOptions pdf;
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
