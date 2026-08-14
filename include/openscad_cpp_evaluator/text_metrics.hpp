#pragma once

#include "openscad_cpp_evaluator/font_provider.hpp"

#include <string>
#include <utility>
#include <vector>

namespace oscadeval {

// Layout of `text`'s ink-bbox/advance metrics, in OpenSCAD units scaled
// for `size`. `glyphs` is what text() places: each shaped glyph with its
// pen position, already scaled. An inkless glyph (a space) still advances
// the pen and still gets a glyphs entry -- it just contributes nothing to
// ascent/descent/inkMinX/inkMaxX.
//
// The run comes from the shaper, so kerning and ligatures are already
// applied and `spacing` scales the shaped advances rather than raw
// per-character widths.
struct TextMeasurement {
    struct Placed {
        GlyphId glyph = 0;
        double x = 0, y = 0;
    };

    double ascent = 0, descent = 0, inkMinX = 0, inkMaxX = 0;
    double advanceX = 0, advanceY = 0;
    std::vector<Placed> glyphs;
};

TextMeasurement measureText(FontProvider& fp, FontHandle handle, const std::string& text, double size,
                            double spacing, const ShapeOptions& opts = {});

// (offsetX, offsetY) translation for halign/valign, given a
// TextMeasurement. Shared by textmetrics() (reports it) and text()
// (applies it).
std::pair<double, double> textAlignOffset(const std::string& halign, const std::string& valign,
                                          const TextMeasurement& m);

} // namespace oscadeval
