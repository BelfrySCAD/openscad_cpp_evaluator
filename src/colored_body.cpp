#include "openscad_cpp_evaluator/colored_body.hpp"

namespace oscadeval {

Value colorToValue(const std::optional<std::array<double, 4>>& color) {
    if (!color) return Value{};
    std::vector<Value> items = {Value{(*color)[0]}, Value{(*color)[1]}, Value{(*color)[2]}, Value{(*color)[3]}};
    return Value{std::make_shared<const ValueList>(ValueList{std::move(items)})};
}

std::optional<std::array<float, 4>> valueToColor(const Value& v) {
    const ListPtr* l = std::get_if<ListPtr>(&v);
    if (!l || !*l || (*l)->items.size() != 4) return std::nullopt;
    std::array<float, 4> c{};
    for (int i = 0; i < 4; ++i) {
        const double* d = std::get_if<double>(&(*l)->items[static_cast<size_t>(i)]);
        if (!d) return std::nullopt;
        c[static_cast<size_t>(i)] = static_cast<float>(*d);
    }
    return c;
}

namespace {
constexpr double kTopLevel2dHeight = 1e-3;
} // namespace

std::vector<ColoredBody> toRenderableBodies(const std::vector<ColoredBody>& bodies) {
    std::vector<ColoredBody> out;
    out.reserve(bodies.size());
    for (const ColoredBody& cb : bodies) {
        if (!cb.body && cb.section) {
            ColoredBody flat;
            flat.body = manifold::Manifold::Extrude(cb.section->ToPolygons(), kTopLevel2dHeight);
            // Put it back where a 3D translate moved it -- the CrossSection
            // could not hold that. See ColoredBody::sectionZ.
            if (cb.sectionZ != 0.0)
                flat.body = flat.body->Translate(manifold::vec3(0, 0, cb.sectionZ));
            flat.color = cb.color;
            flat.flatPreview = true;
            flat.role = cb.role;
            out.push_back(std::move(flat));
        } else {
            out.push_back(cb);
        }
    }
    return out;
}

} // namespace oscadeval
