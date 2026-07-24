#pragma once

#include "openscad_cpp_evaluator/font_provider.hpp"

#include <memory>

namespace oscadeval {

// The built-in default FontProvider (§6 of the design plan): stb_truetype
// over the bundled Liberation Sans TTF only. resolveFont() always returns
// the same handle regardless of the requested spec -- no system font
// matching, no shell-out (unlike the Python reference's `fc-match`), so
// this stays usable unmodified inside a Qt (or any other) host app; a real
// host supplies its own FontProvider (QFontDatabase/QRawFont-backed) for
// actual system-font resolution instead of using this class at all.
class StbFontProvider : public FontProvider {
public:
    StbFontProvider();
    ~StbFontProvider() override;

    FontHandle resolveFont(const std::string& spec) override;
    FontMetrics metrics(FontHandle handle) override;
    std::optional<GlyphMetrics> glyphMetrics(FontHandle handle, char32_t codepoint) override;
    std::vector<std::vector<std::array<double, 2>>> glyphOutline(FontHandle handle, char32_t codepoint, int segsPerCurve) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace oscadeval
