#pragma once

#include "openscad_cpp_evaluator/font_provider.hpp"

#include <memory>

namespace oscadeval {

// The built-in FontProvider: HarfBuzz shapes, FreeType supplies outlines
// and face metrics, and font_match.cpp resolves `font=` specs against the
// platform's font directories. No fontconfig -- see the CMakeLists note on
// why, and font_match.hpp for what of its behaviour is reimplemented.
//
// The bundled Liberation Sans (resources/fonts/, embedded into the binary
// by cmake/embed_font.cmake) is the default font and the last-resort
// fallback, so `text()` always draws something even on a machine with no
// fonts installed at all -- the same font real OpenSCAD falls back to.
//
// Font files are opened lazily and cached per resolved spec: a script that
// never calls text()/textmetrics()/fontmetrics() opens nothing, and one
// that only uses the default font never scans a system directory.
class FreetypeFontProvider : public FontProvider {
public:
    FreetypeFontProvider();
    ~FreetypeFontProvider() override;

    FontHandle resolveFont(const std::string& spec) override;
    FontMetrics metrics(FontHandle handle) override;
    std::vector<ShapedGlyph> shapeText(FontHandle handle, const std::string& utf8, const ShapeOptions& opts) override;
    std::optional<std::array<double, 4>> glyphInkBounds(FontHandle handle, GlyphId glyph) override;
    GlyphContours glyphOutline(FontHandle handle, GlyphId glyph, int segsPerCurve) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace oscadeval
