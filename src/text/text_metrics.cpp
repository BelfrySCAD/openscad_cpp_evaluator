#include "openscad_cpp_evaluator/text_metrics.hpp"

#include <algorithm>

namespace oscadeval {

std::vector<char32_t> utf8DecodeAll(const std::string& s) {
    std::vector<char32_t> out;
    size_t i = 0;
    const auto byteAt = [&](size_t j) { return static_cast<unsigned char>(s[j]); };
    while (i < s.size()) {
        const unsigned char c0 = byteAt(i);
        uint32_t cp;
        size_t len;
        if (c0 < 0x80) {
            cp = c0;
            len = 1;
        } else if ((c0 & 0xE0) == 0xC0 && i + 1 < s.size()) {
            cp = static_cast<uint32_t>(((c0 & 0x1F) << 6) | (byteAt(i + 1) & 0x3F));
            len = 2;
        } else if ((c0 & 0xF0) == 0xE0 && i + 2 < s.size()) {
            cp = static_cast<uint32_t>(((c0 & 0x0F) << 12) | ((byteAt(i + 1) & 0x3F) << 6) | (byteAt(i + 2) & 0x3F));
            len = 3;
        } else if ((c0 & 0xF8) == 0xF0 && i + 3 < s.size()) {
            cp = static_cast<uint32_t>(((c0 & 0x07) << 18) | ((byteAt(i + 1) & 0x3F) << 12) | ((byteAt(i + 2) & 0x3F) << 6) | (byteAt(i + 3) & 0x3F));
            len = 4;
        } else {
            cp = c0;
            len = 1;
        }
        out.push_back(static_cast<char32_t>(cp));
        i += len;
    }
    return out;
}

TextMeasurement measureText(FontProvider& fp, FontHandle handle, const std::string& text, double size, double spacing) {
    const FontMetrics fm = fp.metrics(handle);
    const double scale = size * (100.0 / 72.0) / fm.unitsPerEm;

    TextMeasurement m;
    double penX = 0.0;
    bool hasInk = false;
    for (char32_t ch : utf8DecodeAll(text)) {
        const std::optional<GlyphMetrics> gm = fp.glyphMetrics(handle, ch);
        if (!gm) continue;
        if (gm->bounds) {
            const std::array<double, 4>& b = *gm->bounds;
            const double left = penX * scale + b[0] * scale;
            const double right = penX * scale + b[2] * scale;
            const double top = b[3] * scale;
            const double bottom = b[1] * scale;
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
        m.glyphs.emplace_back(ch, penX * scale);
        penX += gm->advanceWidth * spacing;
    }
    m.advanceX = penX * scale;
    return m;
}

std::pair<double, double> textAlignOffset(const std::string& halign, const std::string& valign, const TextMeasurement& m) {
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
