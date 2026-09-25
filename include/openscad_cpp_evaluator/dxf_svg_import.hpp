#pragma once

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

// Hand-rolled SVG reader: <path> (M/L/H/V/C/S/Q/T/A commands, both absolute
// and relative), <polygon>/<polyline>, <rect>, <circle>, <ellipse>, plus
// transform="matrix()/translate()/scale()/rotate()" (including nested
// group transforms). Y is flipped (SVG's is down, OpenSCAD's is up).
// Throws std::runtime_error on I/O/parse failure. Mirrors
// _load_svg_contours.
//: Which part of the drawing to import. Both unset means the whole file.
//: `id` matches an element's `id` attribute; `cls` matches one entry of its
//: space-separated `class` list. Matching an element takes everything under
//: it, so a filter naming a <g> means "that group".
struct SvgFilter {
    std::optional<std::string> id;
    std::optional<std::string> cls;
};

// `matched`, when given, reports whether the filter found anything. A miss
// imports nothing rather than falling back to the whole drawing; the caller
// warns. Always true when no filter was set.
// Placed as OpenSCAD places them: page units to mm (a unitless length at
// `dpi`), the viewBox under preserveAspectRatio, Y flipped about the page
// height -- or about the drawing's centre with `center`.
std::vector<Contour2d> loadSvgContours(const std::string& path,
                                       const SvgFilter& filter = {},
                                       bool* matched = nullptr,
                                       double dpi = 72.0,
                                       bool center = false);

} // namespace oscadeval
