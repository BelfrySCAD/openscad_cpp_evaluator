#include "openscad_cpp_evaluator/text_metrics.hpp"

#include <algorithm>
#include <cctype>

namespace oscadeval {

TextMeasurement measureText(FontProvider& fp, FontHandle handle, const std::string& text, double size,
                            double spacing, const ShapeOptions& opts) {
    const FontMetrics fm = fp.metrics(handle);
    // The 100/72 factor is real OpenSCAD's long-standing text() size bug
    // (its issue #4304: FT_Set_Char_Size is given 100 dpi where 72 is
    // correct, making glyphs ~1.39x the nominal size). It is reproduced
    // deliberately -- a decade of models are drawn to it, and OpenSCAD
    // itself chose to keep the bug and add an `em=` parameter rather than
    // fix it.
    const double scale = size * (100.0 / 72.0) / fm.unitsPerEm;

    TextMeasurement m;
    // hb_direction_from_string reads only the first letter, so this is
    // exactly the set of strings the shaper takes as vertical.
    const char d = opts.direction.empty() ? 'l' : static_cast<char>(std::tolower(opts.direction[0]));
    m.vertical = d == 't' || d == 'b';
    double penX = 0.0, penY = 0.0;

    for (const ShapedGlyph& g : fp.shapeText(handle, text, opts)) {
        const double x = (penX + g.xOffset) * scale;
        const double y = (penY + g.yOffset) * scale;

        if (const std::optional<std::array<double, 4>> b = fp.glyphInkBounds(handle, g.glyph)) {
            const double gl = x + (*b)[0] * scale, gr = x + (*b)[2] * scale;
            const double gb = y + (*b)[1] * scale, gt = y + (*b)[3] * scale;
            const double asc = (*b)[3] * scale, desc = (*b)[1] * scale;
            if (!m.hasInk) {
                m.left = gl, m.right = gr, m.bottom = gb, m.top = gt;
                m.ascent = asc, m.descent = desc;
                m.hasInk = true;
            } else {
                m.left = std::min(m.left, gl);
                m.right = std::max(m.right, gr);
                m.bottom = std::min(m.bottom, gb);
                m.top = std::max(m.top, gt);
                m.ascent = std::max(m.ascent, asc);
                m.descent = std::min(m.descent, desc);
            }
        }

        m.glyphs.push_back(TextMeasurement::Placed{g.glyph, x, y});
        penX += g.xAdvance * spacing;
        penY += g.yAdvance * spacing;
    }

    m.advanceX = penX * scale;
    m.advanceY = penY * scale;
    return m;
}

// OpenSCAD's ShapeResults::calc_offsets_horiz/_vert, warnings included. An
// unknown value, or valign="baseline" on a vertical run, leaves that axis
// where it is. Text with no ink never gets this far there, so it never
// warns either.
std::pair<double, double> textAlignOffset(const std::string& halign, const std::string& valign,
                                          const TextMeasurement& m,
                                          const std::function<void(const std::string&)>& warn) {
    if (!m.hasInk) return {0.0, 0.0};
    auto unknown = [&](const char* param, const char* choices, const std::string& value) {
        if (warn) {
            warn(std::string("Unknown value for the ") + param + " parameter (use " + choices + "): '" + value + "'");
        }
    };
    const char* hChoices = "\"left\", \"right\" or \"center\"";
    const char* vChoices = "\"baseline\", \"bottom\", \"top\" or \"center\"";
    double offsetX = 0.0, offsetY = 0.0;
    if (m.vertical) {
        if (halign == "right") offsetX = -m.right;
        else if (halign == "left") offsetX = -m.left;
        else if (halign != "center" && halign != "default") unknown("halign", hChoices, halign);
        if (valign == "center") offsetY = -m.advanceY / 2;
        else if (valign == "bottom") offsetY = -m.advanceY;
        else if (valign == "baseline") {
            if (warn) warn("Don't use valign=\"baseline\" with vertical layouts");
        } else if (valign != "top" && valign != "default") unknown("valign", vChoices, valign);
    } else {
        if (halign == "center") offsetX = -0.5 * m.advanceX;
        else if (halign == "right") offsetX = -m.advanceX;
        else if (halign != "left" && halign != "default") unknown("halign", hChoices, halign);
        if (valign == "top") offsetY = -m.ascent;
        else if (valign == "center") offsetY = -(m.ascent + m.descent) / 2;
        else if (valign == "bottom") offsetY = -m.descent;
        else if (valign != "baseline" && valign != "default") unknown("valign", vChoices, valign);
    }
    return {offsetX, offsetY};
}

} // namespace oscadeval
