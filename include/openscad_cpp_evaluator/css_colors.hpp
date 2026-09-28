#pragma once

#include <array>
#include <optional>
#include <string>

namespace oscadeval {

// A colour string as OpenSCAD's parse_color reads it (ColorUtil.cc):
// a CSS Color Level 4 name or OpenSCAD's "transparent" (alpha 0), an
// "xkcd:<name>" from the xkcd colour survey, all case-insensitive; else
// "#rgb", "#rgba", "#rrggbb" or "#rrggbbaa". Nothing if it is none of
// those -- color() then warns and leaves its children's colour alone.
std::optional<std::array<double, 4>> parseColor(const std::string& text);

// parseColor with the alpha replaced, and white for anything unparseable.
// For export options (SVG/PDF fill and stroke), which have no warning path.
std::array<double, 4> cssColor(const std::string& name, double alpha = 1.0);

} // namespace oscadeval
