#include "openscad_cpp_evaluator/freetype_font_provider.hpp"

#include "openscad_cpp_evaluator/bundled_font_data.hpp"
#include "openscad_cpp_evaluator/font_match.hpp"

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

#include <hb.h>
#include <hb-ot.h>

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace oscadeval {

namespace {

// Everything crossing the FreeType/HarfBuzz boundary is in font units, so
// HarfBuzz is scaled to the em rather than to pixels: hb_font_set_scale's
// 16.16-ish "units per em" convention means positions come back in exactly
// the same space FT_LOAD_NO_SCALE reports outlines in, and the single
// `scale` factor in text_metrics.cpp converts to OpenSCAD units once, at
// the end.
void scaleHbFontToEm(hb_font_t* font, unsigned int unitsPerEm) {
    hb_font_set_scale(font, static_cast<int>(unitsPerEm), static_cast<int>(unitsPerEm));
}

struct OutlineSink {
    GlyphContours contours;
    std::vector<std::array<double, 2>> current;
    std::array<double, 2> pen{0, 0};
    int segsPerCurve = 8;

    void flush() {
        if (current.size() >= 3) contours.push_back(current);
        current.clear();
    }
};

std::array<double, 2> pt(const FT_Vector* v) {
    return {static_cast<double>(v->x), static_cast<double>(v->y)};
}

int moveToFn(const FT_Vector* to, void* user) {
    auto* s = static_cast<OutlineSink*>(user);
    s->flush();
    s->pen = pt(to);
    s->current.push_back(s->pen);
    return 0;
}

int lineToFn(const FT_Vector* to, void* user) {
    auto* s = static_cast<OutlineSink*>(user);
    s->pen = pt(to);
    s->current.push_back(s->pen);
    return 0;
}

// Uniform in t, not in arc length -- the same approximation real OpenSCAD
// makes (DrawingCallback::curve_to, which notes that a chord-length
// iteration would be "a lot of work, little gain"). Matching it keeps
// glyph vertex counts comparable with the reference.
int conicToFn(const FT_Vector* c, const FT_Vector* to, void* user) {
    auto* s = static_cast<OutlineSink*>(user);
    const std::array<double, 2> p0 = s->pen, p1 = pt(c), p2 = pt(to);
    for (int i = 1; i <= s->segsPerCurve; ++i) {
        const double t = static_cast<double>(i) / s->segsPerCurve;
        const double u = 1.0 - t;
        s->current.push_back({u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0],
                              u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1]});
    }
    s->pen = p2;
    return 0;
}

int cubicToFn(const FT_Vector* c1, const FT_Vector* c2, const FT_Vector* to, void* user) {
    auto* s = static_cast<OutlineSink*>(user);
    const std::array<double, 2> p0 = s->pen, p1 = pt(c1), p2 = pt(c2), p3 = pt(to);
    for (int i = 1; i <= s->segsPerCurve; ++i) {
        const double t = static_cast<double>(i) / s->segsPerCurve;
        const double u = 1.0 - t;
        s->current.push_back(
            {u * u * u * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t * t * t * p3[0],
             u * u * u * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t * t * t * p3[1]});
    }
    s->pen = p3;
    return 0;
}

} // namespace

struct FreetypeFontProvider::Impl {
    FT_Library lib = nullptr;

    struct Face {
        FT_Face ft = nullptr;
        hb_blob_t* blob = nullptr;
        hb_face_t* hbFace = nullptr;
        hb_font_t* hbFont = nullptr;
        FontMetrics metrics;
    };

    std::vector<Face> faces;                                 // handle == index
    std::unordered_map<std::string, FontHandle> bySpec;      // resolved spec -> handle
    std::vector<FontFace> installed;                         // system font index
    bool scanned = false;

    ~Impl() {
        for (Face& f : faces) {
            if (f.hbFont) hb_font_destroy(f.hbFont);
            if (f.hbFace) hb_face_destroy(f.hbFace);
            if (f.blob) hb_blob_destroy(f.blob);
            if (f.ft) FT_Done_Face(f.ft);
        }
        if (lib) FT_Done_FreeType(lib);
    }

    static FontMetrics readMetrics(FT_Face ft) {
        FontMetrics m;
        m.unitsPerEm = ft->units_per_EM ? static_cast<double>(ft->units_per_EM) : 1000.0;
        m.ascent = ft->ascender;
        m.descent = ft->descender;
        // FreeType's `height` is the full baseline-to-baseline distance,
        // i.e. ascent - descent + lineGap, so the gap is what's left over.
        // fontmetrics() adds them back up for `interline`; going through
        // lineGap keeps FontProvider's contract (hhea-style fields) the
        // same for any other implementation.
        m.lineGap = ft->height - (ft->ascender - ft->descender);
        m.yMax = ft->bbox.yMax;
        m.yMin = ft->bbox.yMin;
        m.family = ft->family_name ? ft->family_name : "";
        m.style = ft->style_name ? ft->style_name : "";
        return m;
    }

    // Returns the handle, or nullopt if the file/face can't be used --
    // callers fall back to the bundled font rather than failing the render.
    std::optional<FontHandle> openFile(const std::string& path, int index) {
        Face f;
        if (FT_New_Face(lib, path.c_str(), index, &f.ft) != 0) return std::nullopt;
        if (!FT_IS_SCALABLE(f.ft)) {
            FT_Done_Face(f.ft);
            return std::nullopt;
        }
        f.blob = hb_blob_create_from_file_or_fail(path.c_str());
        if (!f.blob) {
            FT_Done_Face(f.ft);
            return std::nullopt;
        }
        f.hbFace = hb_face_create(f.blob, static_cast<unsigned>(index));
        f.hbFont = hb_font_create(f.hbFace);
        hb_ot_font_set_funcs(f.hbFont);
        f.metrics = readMetrics(f.ft);
        scaleHbFontToEm(f.hbFont, static_cast<unsigned>(f.metrics.unitsPerEm));
        faces.push_back(f);
        return faces.size() - 1;
    }

    void openBundled() {
        Face f;
        if (FT_New_Memory_Face(lib, reinterpret_cast<const FT_Byte*>(kBundledFontData),
                               static_cast<FT_Long>(kBundledFontDataSize), 0, &f.ft) != 0) {
            throw std::runtime_error("FreetypeFontProvider: bundled font failed to load");
        }
        f.blob = hb_blob_create(reinterpret_cast<const char*>(kBundledFontData),
                                static_cast<unsigned>(kBundledFontDataSize), HB_MEMORY_MODE_READONLY, nullptr,
                                nullptr);
        f.hbFace = hb_face_create(f.blob, 0);
        f.hbFont = hb_font_create(f.hbFace);
        hb_ot_font_set_funcs(f.hbFont);
        f.metrics = readMetrics(f.ft);
        scaleHbFontToEm(f.hbFont, static_cast<unsigned>(f.metrics.unitsPerEm));
        faces.push_back(f);
    }

    // Reads family/style out of every face of every installed font file.
    // Deferred until a spec actually names a font: the common case (no
    // font=, or font= for a family already resolved once) never pays for
    // it, and on a machine with a few hundred fonts it is the only part of
    // this provider that costs real time.
    void scanInstalled() {
        if (scanned) return;
        scanned = true;
        for (const std::string& path : findFontFiles()) {
            FT_Face probe = nullptr;
            if (FT_New_Face(lib, path.c_str(), -1, &probe) != 0) continue;
            const long count = probe->num_faces;
            FT_Done_Face(probe);
            for (long i = 0; i < count; ++i) {
                FT_Face f = nullptr;
                if (FT_New_Face(lib, path.c_str(), i, &f) != 0) continue;
                if (FT_IS_SCALABLE(f) && f->family_name) {
                    installed.push_back(FontFace{path, static_cast<int>(i), f->family_name,
                                                 f->style_name ? f->style_name : ""});
                }
                FT_Done_Face(f);
            }
        }
    }
};

FreetypeFontProvider::FreetypeFontProvider() : impl_(std::make_unique<Impl>()) {
    if (FT_Init_FreeType(&impl_->lib) != 0) {
        throw std::runtime_error("FreetypeFontProvider: FreeType failed to initialize");
    }
    impl_->openBundled(); // handle 0, the default and the last-resort fallback
}

FreetypeFontProvider::~FreetypeFontProvider() = default;

FontHandle FreetypeFontProvider::resolveFont(const std::string& spec) {
    const FontSpec parsed = parseFontSpec(spec);
    if (parsed.families.empty()) return 0;

    const auto cached = impl_->bySpec.find(spec);
    if (cached != impl_->bySpec.end()) return cached->second;

    impl_->scanInstalled();
    FontHandle handle = 0;
    if (const std::optional<FontFace> hit = matchFace(parsed, impl_->installed)) {
        handle = impl_->openFile(hit->path, hit->faceIndex).value_or(0);
    }
    impl_->bySpec[spec] = handle;
    return handle;
}

FontMetrics FreetypeFontProvider::metrics(FontHandle handle) {
    if (handle >= impl_->faces.size()) return impl_->faces[0].metrics;
    return impl_->faces[handle].metrics;
}

std::vector<ShapedGlyph> FreetypeFontProvider::shapeText(FontHandle handle, const std::string& utf8,
                                                         const ShapeOptions& opts) {
    if (handle >= impl_->faces.size()) handle = 0;
    hb_font_t* font = impl_->faces[handle].hbFont;

    hb_buffer_t* buf = hb_buffer_create();
    hb_buffer_add_utf8(buf, utf8.c_str(), static_cast<int>(utf8.size()), 0, static_cast<int>(utf8.size()));
    if (!opts.direction.empty()) {
        hb_buffer_set_direction(buf, hb_direction_from_string(opts.direction.c_str(), -1));
    }
    if (!opts.script.empty()) {
        hb_buffer_set_script(buf, hb_script_from_string(opts.script.c_str(), -1));
    }
    if (!opts.language.empty()) {
        hb_buffer_set_language(buf, hb_language_from_string(opts.language.c_str(), -1));
    }
    // Fills in only what wasn't set above, from the text itself -- the
    // same job real OpenSCAD's detect_properties()/detect_script() does by
    // hand before shaping.
    hb_buffer_guess_segment_properties(buf);
    hb_shape(font, buf, nullptr, 0);

    unsigned int count = 0;
    const hb_glyph_info_t* info = hb_buffer_get_glyph_infos(buf, &count);
    const hb_glyph_position_t* pos = hb_buffer_get_glyph_positions(buf, &count);

    std::vector<ShapedGlyph> out;
    out.reserve(count);
    for (unsigned int i = 0; i < count; ++i) {
        out.push_back(ShapedGlyph{info[i].codepoint, static_cast<double>(pos[i].x_offset),
                                  static_cast<double>(pos[i].y_offset), static_cast<double>(pos[i].x_advance),
                                  static_cast<double>(pos[i].y_advance)});
    }
    hb_buffer_destroy(buf);
    return out;
}

std::optional<std::array<double, 4>> FreetypeFontProvider::glyphInkBounds(FontHandle handle, GlyphId glyph) {
    if (handle >= impl_->faces.size()) handle = 0;
    FT_Face ft = impl_->faces[handle].ft;
    if (FT_Load_Glyph(ft, glyph, FT_LOAD_NO_SCALE | FT_LOAD_NO_HINTING) != 0) return std::nullopt;
    if (ft->glyph->outline.n_contours == 0) return std::nullopt;

    FT_BBox box;
    FT_Outline_Get_CBox(&ft->glyph->outline, &box);
    // A contour that encloses nothing draws nothing; real OpenSCAD applies
    // the same xMax>xMin && yMax>yMin test before letting a glyph
    // contribute to the text bounding box.
    if (box.xMax <= box.xMin || box.yMax <= box.yMin) return std::nullopt;
    return std::array<double, 4>{static_cast<double>(box.xMin), static_cast<double>(box.yMin),
                                 static_cast<double>(box.xMax), static_cast<double>(box.yMax)};
}

GlyphContours FreetypeFontProvider::glyphOutline(FontHandle handle, GlyphId glyph, int segsPerCurve) {
    if (handle >= impl_->faces.size()) handle = 0;
    FT_Face ft = impl_->faces[handle].ft;
    if (FT_Load_Glyph(ft, glyph, FT_LOAD_NO_SCALE | FT_LOAD_NO_HINTING) != 0) return {};

    OutlineSink sink;
    sink.segsPerCurve = segsPerCurve < 1 ? 1 : segsPerCurve;

    FT_Outline_Funcs funcs;
    funcs.move_to = moveToFn;
    funcs.line_to = lineToFn;
    funcs.conic_to = conicToFn;
    funcs.cubic_to = cubicToFn;
    funcs.shift = 0;
    funcs.delta = 0;

    if (FT_Outline_Decompose(&ft->glyph->outline, &funcs, &sink) != 0) return {};
    sink.flush();
    return sink.contours;
}

} // namespace oscadeval
