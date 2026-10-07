#pragma once

#include "openscad_cpp_evaluator/segments.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace oscadeval {

using Contour2d = std::vector<std::array<double, 2>>;

// import()'s DXF arguments. `disc` carries $fn/$fa/$fs/$fe for arcs.
struct DxfOptions {
    std::optional<std::string> layer;
    double xorigin = 0.0, yorigin = 0.0, scale = 1.0;
    Discretizer disc;
};

// The outlines, to be filled even-odd, and OpenSCAD's warnings about the
// file, in the order it prints them.
struct DxfImport {
    std::vector<Contour2d> contours;
    std::vector<std::string> warnings;
};

// Hand-rolled DXF reader matching OpenSCAD 2026.02.01's import (see the
// notes atop dxf_import.cpp): LINE, CIRCLE, ARC, ELLIPSE, LWPOLYLINE,
// POLYLINE and INSERT, chained into closed outlines. Throws
// std::runtime_error on I/O failure.
DxfImport loadDxf(const std::string& path, const DxfOptions& opts = {});

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
