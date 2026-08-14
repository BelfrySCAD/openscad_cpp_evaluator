#pragma once

#include <optional>
#include <string>
#include <vector>

namespace oscadeval {

// A parsed fontconfig-style font spec, which is what OpenSCAD's `font=`
// takes: a comma-separated family list, then colon-separated properties.
//
//   "Liberation Sans"
//   "Liberation Sans:style=Bold Italic"
//   "Helvetica,Arial,sans-serif:style=Bold"
//
// Only `style=` is meaningful here; fontconfig accepts a long tail of
// other properties (:size=, :weight=, :lang=) that OpenSCAD models never
// use, and quietly ignoring them beats rejecting a spec that fontconfig
// itself would have accepted. A bare `:Bold` (no `=`) is read as a style
// too, since fontconfig's own parser allows it.
struct FontSpec {
    std::vector<std::string> families;
    std::string style;
};

FontSpec parseFontSpec(const std::string& spec);

// Generic family names, expanded to the concrete families likely to be
// installed. fontconfig resolves these through its own alias config
// (OpenSCAD ships 10-liberation.conf pointing them at the Liberation
// faces); with no fontconfig, the alias list lives here instead.
// Returns an empty vector for a family that isn't generic.
std::vector<std::string> expandGenericFamily(const std::string& family);

// Every directory the platform keeps fonts in, plus anything on
// OPENSCAD_FONT_PATH (same environment variable real OpenSCAD reads, same
// platform path separator). Directories that don't exist are dropped, so
// the result is safe to walk blindly.
std::vector<std::string> systemFontDirs();

// Every font file under those directories: .ttf/.otf/.ttc/.otc, recursive.
// A directory that can't be read is skipped rather than throwing -- a font
// directory with odd permissions must not take down a render.
std::vector<std::string> findFontFiles();

// One face inside one font file. A .ttc collection holds several, hence
// the index; a plain .ttf is always index 0.
struct FontFace {
    std::string path;
    int faceIndex = 0;
    std::string family;
    std::string style;
};

// Best match for `spec` among `faces`, or nullopt if no requested family
// (after generic expansion) is present at all. Family match is
// case-insensitive and required; style is a preference, not a filter,
// resolved in the order: exact style, then substring, then "Regular",
// then the first face of that family.
std::optional<FontFace> matchFace(const FontSpec& spec, const std::vector<FontFace>& faces);

} // namespace oscadeval
