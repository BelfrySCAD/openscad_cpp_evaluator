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
std::vector<Contour2d> loadSvgContours(const std::string& path);

} // namespace oscadeval
