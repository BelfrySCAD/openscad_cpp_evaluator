#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace oscadeval {

using FontHandle = std::uint64_t;

// Global (whole-font) metrics, all in font units (not yet scaled for a
// particular `size=`) -- mirrors fontmetrics()'s "nominal"/"max" split:
// ascent/descent/lineGap come from the font's hhea table (or equivalent),
// yMax/yMin from its head table (or equivalent) -- these commonly differ
// (yMax/yMin bound the tallest/deepest glyph actually in the font; ascent/
// descent are the font designer's nominal line-layout values).
struct FontMetrics {
    double unitsPerEm = 1000.0;
    double ascent = 0, descent = 0, lineGap = 0;
    double yMax = 0, yMin = 0;
    std::string family, style;
};

// One glyph's advance + ink bounding box, in font units. `bounds` is
// nullopt for a glyph with no visible ink (space, or a codepoint the font
// doesn't map) -- mirrors _glyph_bounds returning None for a
// numberOfContours==0 glyph.
struct GlyphMetrics {
    double advanceWidth = 0;
    std::optional<std::array<double, 4>> bounds; // xMin, yMin, xMax, yMax
};

// Abstraction over font resolution/metrics/outline extraction, so
// text()/textmetrics()/fontmetrics() don't hard-depend on a specific font
// backend. This matters because this library is meant to be embedded in a
// cross-platform Qt GUI app -- shelling out to `fc-match` (what the Python
// reference does for system font matching) is Linux-only in practice, not
// something this library can assume is available. A Qt host app implements
// this interface using QFontDatabase/QRawFont/QFontMetricsF for real
// system-font matching; this project ships only the interface plus a
// built-in default (StbFontProvider, font_provider.hpp) that always
// resolves to one bundled font regardless of the requested spec, entirely
// sufficient to keep this repo's own tests/CLI self-contained and to prove
// the injection seam works end-to-end.
class FontProvider {
public:
    virtual ~FontProvider() = default;

    // Resolves an OpenSCAD/fontconfig-style spec (e.g. "Times New
    // Roman:style=Bold", or "" for the default) to an opaque handle.
    virtual FontHandle resolveFont(const std::string& spec) = 0;

    virtual FontMetrics metrics(FontHandle handle) = 0;

    // nullopt if the font doesn't map `codepoint` at all (skip entirely --
    // no glyph, no advance; mirrors cmap.get(ord(ch)) is None). A mapped
    // codepoint with zero ink (e.g. space) still returns a real
    // GlyphMetrics (bounds = nullopt within it, advanceWidth set) -- it
    // still advances the pen, unlike the unmapped case. `codepoint` is a
    // full Unicode scalar value (not UTF-8 code units).
    virtual std::optional<GlyphMetrics> glyphMetrics(FontHandle handle, char32_t codepoint) = 0;

    // The glyph's outline as flattened polyline contours (curves split
    // into `segsPerCurve` segments each), in font units. Only called by
    // text() itself (rendering) -- textmetrics()/fontmetrics() need only
    // glyphMetrics()/metrics() above, no outline extraction.
    virtual std::vector<std::vector<std::array<double, 2>>> glyphOutline(FontHandle handle, char32_t codepoint,
                                                                          int segsPerCurve) = 0;
};

} // namespace oscadeval
