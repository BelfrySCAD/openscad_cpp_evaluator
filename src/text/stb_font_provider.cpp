#include "openscad_cpp_evaluator/stb_font_provider.hpp"

#include "openscad_cpp_evaluator/bundled_font_path.hpp"

#define STB_TRUETYPE_IMPLEMENTATION
#include <stb_truetype.h>

#include <fstream>
#include <stdexcept>

namespace oscadeval {

namespace {

// TrueType `name` table strings on the Microsoft platform are UTF-16BE;
// decode just enough (BMP + basic surrogate pairs) for a family/style name.
std::string decodeUtf16Be(const char* data, int lengthBytes) {
    std::string out;
    const auto byteAt = [&](int i) { return static_cast<unsigned char>(data[i]); };
    for (int i = 0; i + 1 < lengthBytes; i += 2) {
        uint32_t cp = (static_cast<uint32_t>(byteAt(i)) << 8) | byteAt(i + 1);
        if (cp >= 0xD800 && cp <= 0xDBFF && i + 3 < lengthBytes) {
            const uint32_t low = (static_cast<uint32_t>(byteAt(i + 2)) << 8) | byteAt(i + 3);
            if (low >= 0xDC00 && low <= 0xDFFF) {
                cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
                i += 2;
            }
        }
        if (cp <= 0x7F) {
            out.push_back(static_cast<char>(cp));
        } else if (cp <= 0x7FF) {
            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        } else if (cp <= 0xFFFF) {
            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
    }
    return out;
}

std::string fontNameString(const stbtt_fontinfo& info, int nameId) {
    int len = 0;
    const char* s = stbtt_GetFontNameString(&info, &len, STBTT_PLATFORM_ID_MICROSOFT, STBTT_MS_EID_UNICODE_BMP, STBTT_MS_LANG_ENGLISH, nameId);
    if (!s || len == 0) return "";
    return decodeUtf16Be(s, len);
}

// Flattens a quadratic bezier (TrueType `glyf` curves) into segsPerCurve
// line segments, appended to `out` (the moveTo point is assumed already
// present as `out`'s last point). Mirrors _FlattenPen._qCurveToOne.
void flattenQuad(std::vector<std::array<double, 2>>& out, std::array<double, 2> p1, std::array<double, 2> p2, int segs) {
    const std::array<double, 2> p0 = out.back();
    for (int i = 1; i <= segs; ++i) {
        const double t = static_cast<double>(i) / segs, mt = 1 - t;
        out.push_back({mt * mt * p0[0] + 2 * mt * t * p1[0] + t * t * p2[0], mt * mt * p0[1] + 2 * mt * t * p1[1] + t * t * p2[1]});
    }
}

void flattenCubic(std::vector<std::array<double, 2>>& out, std::array<double, 2> p1, std::array<double, 2> p2, std::array<double, 2> p3,
                   int segs) {
    const std::array<double, 2> p0 = out.back();
    for (int i = 1; i <= segs; ++i) {
        const double t = static_cast<double>(i) / segs, mt = 1 - t;
        const double a = mt * mt * mt, b = 3 * mt * mt * t, c = 3 * mt * t * t, d = t * t * t;
        out.push_back({a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0], a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1]});
    }
}

} // namespace

struct StbFontProvider::Impl {
    std::vector<unsigned char> data;
    stbtt_fontinfo info{};
    std::string family, style;
};

StbFontProvider::StbFontProvider() : impl_(std::make_unique<Impl>()) {
    std::ifstream in(kBundledFontPath, std::ios::binary);
    if (!in) throw std::runtime_error(std::string("could not open bundled font '") + kBundledFontPath + "'");
    in.seekg(0, std::ios::end);
    const auto size = in.tellg();
    in.seekg(0);
    impl_->data.resize(static_cast<size_t>(size));
    in.read(reinterpret_cast<char*>(impl_->data.data()), size);

    if (!stbtt_InitFont(&impl_->info, impl_->data.data(), 0)) {
        throw std::runtime_error(std::string("failed to parse bundled font '") + kBundledFontPath + "'");
    }
    impl_->family = fontNameString(impl_->info, 1);
    impl_->style = fontNameString(impl_->info, 2);
    if (impl_->family.empty()) impl_->family = "Liberation Sans";
    if (impl_->style.empty()) impl_->style = "Regular";
}

StbFontProvider::~StbFontProvider() = default;

FontHandle StbFontProvider::resolveFont(const std::string&) {
    return 1; // exactly one bundled font, always
}

FontMetrics StbFontProvider::metrics(FontHandle) {
    FontMetrics m;
    m.unitsPerEm = 1.0 / stbtt_ScaleForMappingEmToPixels(&impl_->info, 1.0f);
    int ascent = 0, descent = 0, lineGap = 0;
    stbtt_GetFontVMetrics(&impl_->info, &ascent, &descent, &lineGap);
    m.ascent = ascent;
    m.descent = descent;
    m.lineGap = lineGap;
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    stbtt_GetFontBoundingBox(&impl_->info, &x0, &y0, &x1, &y1);
    m.yMax = y1;
    m.yMin = y0;
    m.family = impl_->family;
    m.style = impl_->style;
    return m;
}

std::optional<GlyphMetrics> StbFontProvider::glyphMetrics(FontHandle, char32_t codepoint) {
    const int glyphIndex = stbtt_FindGlyphIndex(&impl_->info, static_cast<int>(codepoint));
    if (glyphIndex == 0) return std::nullopt; // not mapped by this font -- mirrors cmap.get() is None
    GlyphMetrics gm;
    int advance = 0, lsb = 0;
    stbtt_GetGlyphHMetrics(&impl_->info, glyphIndex, &advance, &lsb);
    gm.advanceWidth = advance;
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    if (stbtt_GetGlyphBox(&impl_->info, glyphIndex, &x0, &y0, &x1, &y1)) {
        gm.bounds = std::array<double, 4>{static_cast<double>(x0), static_cast<double>(y0), static_cast<double>(x1), static_cast<double>(y1)};
    }
    return gm;
}

std::vector<std::vector<std::array<double, 2>>> StbFontProvider::glyphOutline(FontHandle, char32_t codepoint, int segsPerCurve) {
    std::vector<std::vector<std::array<double, 2>>> contours;
    const int glyphIndex = stbtt_FindGlyphIndex(&impl_->info, static_cast<int>(codepoint));
    if (glyphIndex == 0) return contours;

    stbtt_vertex* verts = nullptr;
    const int n = stbtt_GetGlyphShape(&impl_->info, glyphIndex, &verts);
    std::vector<std::array<double, 2>> contour;
    for (int i = 0; i < n; ++i) {
        const stbtt_vertex& v = verts[i];
        switch (v.type) {
            case STBTT_vmove:
                if (!contour.empty()) contours.push_back(contour);
                contour = {{static_cast<double>(v.x), static_cast<double>(v.y)}};
                break;
            case STBTT_vline:
                contour.push_back({static_cast<double>(v.x), static_cast<double>(v.y)});
                break;
            case STBTT_vcurve:
                flattenQuad(contour, {static_cast<double>(v.cx), static_cast<double>(v.cy)}, {static_cast<double>(v.x), static_cast<double>(v.y)},
                            segsPerCurve);
                break;
            case STBTT_vcubic:
                flattenCubic(contour, {static_cast<double>(v.cx), static_cast<double>(v.cy)}, {static_cast<double>(v.cx1), static_cast<double>(v.cy1)},
                             {static_cast<double>(v.x), static_cast<double>(v.y)}, segsPerCurve);
                break;
            default:
                break;
        }
    }
    if (!contour.empty()) contours.push_back(contour);
    if (verts) stbtt_FreeShape(&impl_->info, verts);
    return contours;
}

} // namespace oscadeval
