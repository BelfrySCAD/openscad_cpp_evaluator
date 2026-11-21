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
// `style=` names a face outright and wins over everything else, as in
// fontconfig. `weight=` and `slant=` take fontconfig's own scale (weight 80
// regular, 200 bold, 210 black; slant 0 roman, 100 italic, 110 oblique) or
// its constant names, and pick the nearest face (#1161; they were ignored).
// `width=` (fontconfig's scale: 75 condensed, 100 normal) breaks ties after
// weight. `spacing=mono` (100) puts `monospace` in front of the families,
// as OpenSCAD's bundled 48-spacing.conf does -- so it beats a named family.
// `fontfeatures=` ("-liga;+smcp") is handed to HarfBuzz when shaping.
// A bare `:Bold`, `:Italic`, `:condensed` or `:mono` is that constant; any
// other bare word is ignored, as fontconfig ignores it. Every other known
// property (size=, lang=, ...) is accepted and ignored. An UNKNOWN property
// name, or a weight or width that is neither a number nor its own
// property's constant, makes the whole spec invalid: fontconfig refuses to
// parse it, and OpenSCAD then warns "Can't get font" and draws nothing.
struct FontSpec {
    std::vector<std::string> families;
    std::string style;
    std::optional<double> weight;
    std::optional<double> slant;
    std::optional<double> width;
    std::vector<std::string> features;  // HarfBuzz feature strings
    bool valid = true;
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
    double weight = 80;   // fontconfig scale, from the OS/2 table
    double slant = 0;     // 0 roman, 100 italic, 110 oblique
    double width = 100;   // fontconfig scale, from the OS/2 table
};

// fontconfig's width for an OpenType usWidthClass (1-9).
double fontconfigWidth(int openTypeWidth);

// fontconfig's weight for an OpenType usWeightClass (FcWeightFromOpenType).
double fontconfigWeight(int openTypeWeight);

// Best match for `spec` among `faces`, or nullopt if no requested family
// (after generic expansion) is present at all. Family match is
// case-insensitive and required; style is a preference, not a filter,
// resolved in the order: exact style, then substring, then "Regular",
// then the first face of that family. With no style= but a weight= or
// slant= or width=, the nearest face wins instead: slant, then weight,
// then width, each unset one defaulting to roman/regular/normal, a tie
// going to the more slanted, heavier or wider face (as the reference picks).
std::optional<FontFace> matchFace(const FontSpec& spec, const std::vector<FontFace>& faces);

} // namespace oscadeval
