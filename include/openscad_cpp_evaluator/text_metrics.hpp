#pragma once

#include "openscad_cpp_evaluator/font_provider.hpp"

#include <string>
#include <utility>
#include <vector>

namespace oscadeval {

// Decodes a UTF-8 byte string into full Unicode scalar values. A malformed
// lead byte falls back to its raw byte value rather than throwing --
// matches this codebase's existing chr()/ord() leniency
// (function_builtins.cpp).
std::vector<char32_t> utf8DecodeAll(const std::string& s);

// Left-to-right layout of `text`'s ink-bbox/advance metrics, in OpenSCAD
// units scaled for `size`. `glyphs`: (codepoint, pen-x already scaled) for
// each *mapped* codepoint -- used by text() to actually place outlines. A
// codepoint the font doesn't map at all is skipped entirely (no entry, no
// advance); a mapped-but-inkless codepoint (space) still advances and gets
// a glyphs entry, just doesn't affect ascent/descent/inkMinX/inkMaxX.
// Mirrors _measure_text.
struct TextMeasurement {
    double ascent = 0, descent = 0, inkMinX = 0, inkMaxX = 0, advanceX = 0;
    std::vector<std::pair<char32_t, double>> glyphs;
};

TextMeasurement measureText(FontProvider& fp, FontHandle handle, const std::string& text, double size, double spacing);

// (offsetX, offsetY) translation for halign/valign, given a TextMeasurement.
// Shared by textmetrics() (reports it) and text() (applies it). Mirrors
// _text_align_offset.
std::pair<double, double> textAlignOffset(const std::string& halign, const std::string& valign, const TextMeasurement& m);

} // namespace oscadeval
