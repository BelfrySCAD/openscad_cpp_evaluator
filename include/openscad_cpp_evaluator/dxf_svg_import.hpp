#pragma once

#include "openscad_cpp_evaluator/segments.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace oscadeval {

using Contour2d = std::vector<std::array<double, 2>>;

// Hand-rolled, dependency-free DXF reader covering only what real
// OpenSCAD's own dxf import (and this port's Python reference, via ezdxf)
// exposes: closed LWPOLYLINE and 2D POLYLINE entities, optionally filtered
// to one layer. Any other entity type is ignored. Throws
// std::runtime_error on I/O failure. Mirrors _load_dxf_contours.
std::vector<Contour2d> loadDxfContours(const std::string& path, const std::optional<std::string>& layer);

// SVG reader, as OpenSCAD 2026.02.01 reads it (a port of its MIT-licensed
// libsvg): <path>, <rect>, <circle>, <ellipse>, <line>, <polygon>,
// <polyline> and <use>, with transforms, display:none, and strokes -- an
// open path, <line> or <polyline> is drawn as its stroke's outline. Curves
// split by `disc`. Y is flipped (SVG's is down, OpenSCAD's is up). Throws
// std::runtime_error on I/O/parse failure.
//: Which part of the drawing to import. All unset means the whole file.
//: `id` matches an element's `id` attribute; `cls` (not an OpenSCAD
//: parameter) one entry of its space-separated `class` list; either takes
//: everything under the element, so naming a <g> means "that group".
//: `layer` names an Inkscape layer (inkscape:groupmode="layer", by its
//: inkscape:label): alone it takes that layer, with id/cls it restricts
//: them to elements inside it.
struct SvgFilter {
    std::optional<std::string> id;
    std::optional<std::string> cls;
    std::optional<std::string> layer;
};

// `matched`, when given, reports whether the filter found anything. A miss
// imports nothing rather than falling back to the whole drawing; the caller
// warns. Always true when no filter was set.
// Placed as OpenSCAD places them: page units to mm (a unitless length at
// `dpi`), the viewBox under preserveAspectRatio, Y flipped about the page
// height -- or about the drawing's centre with `center`. The result is
// filled: overlapping contours have already been combined.
std::vector<Contour2d> loadSvgContours(const std::string& path,
                                       const SvgFilter& filter = {},
                                       bool* matched = nullptr,
                                       double dpi = 72.0,
                                       bool center = false,
                                       const Discretizer& disc = {});

} // namespace oscadeval
