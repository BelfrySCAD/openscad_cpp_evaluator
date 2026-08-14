#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace oscadeval {

using FontHandle = std::uint64_t;

// A font-internal glyph index, NOT a Unicode codepoint. Shaping is what
// turns one into the other, and the mapping is neither 1:1 nor stable
// across fonts: one codepoint can shape to several glyphs, several
// codepoints can shape to one ligature glyph, and the same character has a
// different index in every font.
using GlyphId = std::uint32_t;

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

// One glyph as placed by the shaper, in font units. `xOffset`/`yOffset`
// adjust this glyph's own position without moving the pen (mark
// attachment, cursive joins); `xAdvance`/`yAdvance` move the pen. Both are
// shaping output, not raw hmtx values -- kerning and ligature
// substitutions are already applied by the time this exists.
struct ShapedGlyph {
    GlyphId glyph = 0;
    double xOffset = 0, yOffset = 0;
    double xAdvance = 0, yAdvance = 0;
};

// What OpenSCAD's text()/textmetrics() call direction/language/script.
// Any field left empty is guessed from the text itself by the shaper,
// which is what real OpenSCAD's detect_properties() does by hand.
struct ShapeOptions {
    std::string direction; // "ltr", "rtl", "ttb", "btt"
    std::string language;  // BCP 47, e.g. "en"
    std::string script;    // ISO 15924, e.g. "Latn", "Arab"
};

// Flattened polyline contours (curves already split into straight
// segments), in font units.
using GlyphContours = std::vector<std::vector<std::array<double, 2>>>;

// Abstraction over font resolution/shaping/outline extraction, so
// text()/textmetrics()/fontmetrics() don't hard-depend on a specific font
// backend. The library ships FreetypeFontProvider (HarfBuzz for shaping,
// FreeType for outlines and face metrics, and its own fontconfig-syntax
// matcher for system fonts) as the built-in default, which is what every
// build here actually uses; the interface stays because a host that
// already owns a font stack (a Qt app with QRawFont, say) may prefer to
// answer these five questions itself rather than have a second font
// engine resolving names in the same process.
class FontProvider {
public:
    virtual ~FontProvider() = default;

    // Resolves an OpenSCAD/fontconfig-style spec (e.g. "Times New
    // Roman:style=Bold", or "" for the default) to an opaque handle.
    // Never fails: an unmatched spec falls back to the default font,
    // matching fontconfig's own never-fails matching (and so real
    // OpenSCAD's), where a missing font silently becomes the nearest
    // available one rather than an error.
    virtual FontHandle resolveFont(const std::string& spec) = 0;

    virtual FontMetrics metrics(FontHandle handle) = 0;

    // Lays out a whole string at once -- not per codepoint, because
    // kerning, ligatures, mark positioning and bidi reordering are all
    // properties of the run rather than of any character in it.
    virtual std::vector<ShapedGlyph> shapeText(FontHandle handle, const std::string& utf8,
                                               const ShapeOptions& opts) = 0;

    // The glyph's ink bounding box (xMin, yMin, xMax, yMax) in font units,
    // or nullopt for a glyph that draws nothing (a space, or a .notdef
    // with an empty outline) -- such a glyph still advances the pen, so it
    // affects `advance` while contributing nothing to the ink box that
    // textmetrics() reports as position/size.
    virtual std::optional<std::array<double, 4>> glyphInkBounds(FontHandle handle, GlyphId glyph) = 0;

    // Only called by text() itself (rendering) -- textmetrics()/
    // fontmetrics() need only the two calls above.
    virtual GlyphContours glyphOutline(FontHandle handle, GlyphId glyph, int segsPerCurve) = 0;
};

} // namespace oscadeval
