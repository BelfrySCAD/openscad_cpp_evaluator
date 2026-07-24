#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"
#include "openscad_cpp_evaluator/text_metrics.hpp"

#include <algorithm>

// text(text=, size=10, font=, halign="left", valign="baseline", spacing=1)
// -- renders `text` as 2D glyph outlines via the injected FontProvider
// (Evaluator::fontProvider(), lazily the built-in StbFontProvider default
// if none was supplied to the constructor). Mirrors _resolve_text/
// _generate_text, minus the `fc-match` system-font resolution the
// reference relies on (FontProvider::resolveFont() is a Qt host's job --
// see font_provider.hpp; the built-in default ignores `font=` entirely and
// always uses the bundled font). `direction`/`language`/`script` are
// accepted (parsed) but unused, matching the reference exactly.

namespace oscadeval {

namespace {
std::string asStringOr(const Value& v, const std::string& fallback) {
    const std::string* s = std::get_if<std::string>(&v);
    return s ? *s : fallback;
}
} // namespace

CSGParams resolveText(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    const std::string text = asStringOr(getArg(args, 0, "text", Value{std::string("")}), "");
    const double size = toDoubleLenient(getArg(args, 1, "size", Value{10.0}));
    const std::string fontSpec = asStringOr(getArg(args, std::nullopt, "font", Value{std::string("")}), "");
    const std::string halign = asStringOr(getArg(args, std::nullopt, "halign", Value{std::string("left")}), "left");
    const std::string valign = asStringOr(getArg(args, std::nullopt, "valign", Value{std::string("baseline")}), "baseline");
    const double spacing = toDoubleLenient(getArg(args, std::nullopt, "spacing", Value{1.0}));

    FontProvider& fp = ev.fontProvider();
    const FontHandle handle = fp.resolveFont(fontSpec);
    const FontMetrics fm = fp.metrics(handle);
    const double scale = size * (100.0 / 72.0) / fm.unitsPerEm;
    const int segs = std::max(2, fnSegmentsFromCtx(effCtx) / 2);
    const TextMeasurement m = measureText(fp, handle, text, size, spacing);
    const auto [offsetX, offsetY] = textAlignOffset(halign, valign, m);

    std::vector<Value> glyphs;
    glyphs.reserve(m.glyphs.size());
    for (const auto& [cp, penX] : m.glyphs) {
        glyphs.push_back(Value{std::make_shared<const ValueList>(ValueList{{Value{static_cast<double>(cp)}, Value{penX}}})});
    }

    CSGParams params;
    params["font_handle"] = Value{static_cast<double>(handle)};
    params["scale"] = Value{scale};
    params["segs"] = Value{static_cast<double>(segs)};
    params["offset_x"] = Value{offsetX};
    params["offset_y"] = Value{offsetY};
    params["glyphs"] = Value{std::make_shared<const ValueList>(ValueList{std::move(glyphs)})};
    params["color"] = colorToValue(effCtx.color);
    return params;
}

std::vector<ColoredBody> generateText(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                       const oscad::ASTNode&) {
    const FontHandle handle = static_cast<FontHandle>(std::get<double>(params.at("font_handle")));
    const double scale = std::get<double>(params.at("scale"));
    const int segs = static_cast<int>(std::get<double>(params.at("segs")));
    const auto& glyphs = std::get<ListPtr>(params.at("glyphs"))->items;

    std::optional<manifold::CrossSection> cs;
    for (const Value& gv : glyphs) {
        const auto& pair = std::get<ListPtr>(gv)->items;
        const char32_t cp = static_cast<char32_t>(std::get<double>(pair[0]));
        const double penX = std::get<double>(pair[1]);

        const std::vector<std::vector<std::array<double, 2>>> contours = ev.fontProvider().glyphOutline(handle, cp, segs);
        if (contours.empty()) continue;
        manifold::Polygons polys;
        polys.reserve(contours.size());
        for (const auto& c : contours) {
            manifold::SimplePolygon poly;
            poly.reserve(c.size());
            for (const auto& p : c) poly.push_back(manifold::vec2(p[0], p[1]));
            polys.push_back(std::move(poly));
        }
        manifold::CrossSection glyphCs(polys, manifold::CrossSection::FillRule::NonZero);
        glyphCs = glyphCs.Scale(manifold::vec2(scale, scale)).Translate(manifold::vec2(penX, 0));
        cs = cs ? (*cs + glyphCs) : glyphCs;
    }

    manifold::CrossSection result = cs.value_or(manifold::CrossSection());
    result = result.Translate(manifold::vec2(std::get<double>(params.at("offset_x")), std::get<double>(params.at("offset_y"))));

    ColoredBody body;
    body.section = std::move(result);
    body.color = valueToColor(params.at("color"));
    return {body};
}

} // namespace oscadeval
