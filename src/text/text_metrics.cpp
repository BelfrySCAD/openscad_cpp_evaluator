#include "openscad_cpp_evaluator/text_metrics.hpp"

#include <algorithm>

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
    double penX = 0.0, penY = 0.0;
    bool hasInk = false;

    for (const ShapedGlyph& g : fp.shapeText(handle, text, opts)) {
        const double x = (penX + g.xOffset) * scale;
        const double y = (penY + g.yOffset) * scale;

        if (const std::optional<std::array<double, 4>> b = fp.glyphInkBounds(handle, g.glyph)) {
            const double left = x + (*b)[0] * scale;
            const double right = x + (*b)[2] * scale;
            const double bottom = y + (*b)[1] * scale;
            const double top = y + (*b)[3] * scale;
            if (!hasInk) {
                m.inkMinX = left;
                m.inkMaxX = right;
                m.ascent = top;
                m.descent = bottom;
                hasInk = true;
            } else {
                m.inkMinX = std::min(m.inkMinX, left);
                m.inkMaxX = std::max(m.inkMaxX, right);
                m.ascent = std::max(m.ascent, top);
                m.descent = std::min(m.descent, bottom);
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

std::pair<double, double> textAlignOffset(const std::string& halign, const std::string& valign,
                                          const TextMeasurement& m) {
    double offsetX = 0.0;
    if (halign == "center") offsetX = -0.5 * m.advanceX;
    else if (halign == "right") offsetX = -1.0 * m.advanceX;

    double offsetY = 0.0;
    if (valign == "top") offsetY = -m.ascent;
    else if (valign == "center") offsetY = -(m.ascent + m.descent) / 2;
    else if (valign == "bottom") offsetY = -m.descent;
    // "baseline" (or anything else) -> 0

    return {offsetX, offsetY};
}

} // namespace oscadeval
