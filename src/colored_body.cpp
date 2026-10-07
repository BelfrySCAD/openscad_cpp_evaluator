#include "openscad_cpp_evaluator/colored_body.hpp"

namespace oscadeval {

Value colorToValue(const std::optional<std::array<double, 4>>& color) {
    if (!color) return Value{};
    std::vector<Value> items = {Value{(*color)[0]}, Value{(*color)[1]}, Value{(*color)[2]}, Value{(*color)[3]}};
    return Value{makeList(std::move(items))};
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

// kTopLevel2dHeight (colored_body.hpp), the default slab height: the
// reference extrudes a 2D shape to exactly 1 unit for preview -- measured
// against OpenSCAD at three camera tilts (60/70/80 degrees), which put it at
// 0.92/1.00/0.99, the 60 being pixel-rounding on the shallowest angle. Ours
// was 1e-3, which reads as a flat silhouette with no visible edge and, more
// annoyingly, gives --viewall a different bounding box to fit, so every 2D
// docs image came out at a different scale from the published one.

std::vector<ColoredBody> toRenderableBodies(const std::vector<ColoredBody>& bodies, double flatHeight) {
    std::vector<ColoredBody> out;
    out.reserve(bodies.size());
    for (const ColoredBody& cb : bodies) {
        if (!cb.body && cb.section) {
            ColoredBody flat;
            flat.body = manifold::Manifold::Extrude(cb.section->ToPolygons(), flatHeight);
            if (cb.sectionId && !flat.body->IsEmpty()) {
                // One run carrying the section's own ID, so a click on the
                // slab finds the node that built the shape (see
                // ColoredBody::sectionId). Manifold keeps a MeshGL's
                // runOriginalID when it is given one.
                // MeshGL64: rebuilt below, and float32 would move every vertex.
                manifold::MeshGL64 mesh = flat.body->GetMeshGL64();
                mesh.runOriginalID = {*cb.sectionId};
                mesh.runIndex = {0, mesh.triVerts.size()};
                mesh.runTransform.clear();
                mesh.faceID.clear();
                flat.body = manifold::Manifold(mesh);
            }
            // Put it back where the transforms a CrossSection could not
            // hold would have placed it -- a Z offset, a rotation out of
            // the plane. See ColoredBody::sectionXform.
            flat.body = flat.body->Transform(cb.sectionXform);
            flat.color = cb.color;
            flat.flatPreview = true;
            // Carried, not consumed: the 2D writers (writeSvg) need the
            // contours, and every caller that exports goes through this
            // conversion first -- the Python binding hands ONE body list to
            // both the renderer and export. Dropping it here made an SVG
            // export of any 2D script impossible from Python.
            flat.section = cb.section;
            flat.sectionXform = cb.sectionXform;
            flat.role = cb.role;
            out.push_back(std::move(flat));
        } else {
            out.push_back(cb);
        }
    }
    return out;
}

manifold::mat2x3 projectTo2d(const manifold::mat3x4& m) {
    return manifold::mat2x3(manifold::vec2(m[0].x, m[0].y), manifold::vec2(m[1].x, m[1].y),
                            manifold::vec2(m[3].x, m[3].y));
}

bool transformSection(manifold::CrossSection& cs, const manifold::mat3x4& m) {
    const manifold::mat2x3 p = projectTo2d(m);
    // A singular 2D matrix removes the shape.
    if (p[0].x * p[1].y - p[1].x * p[0].y == 0.0) {
        cs = manifold::CrossSection();
        return false;
    }
    cs = cs.Transform(p);
    return true;
}

bool projectSectionXform(ColoredBody& b) {
    const manifold::mat3x4 ident(manifold::vec3(1, 0, 0), manifold::vec3(0, 1, 0), manifold::vec3(0, 0, 1),
                                 manifold::vec3(0, 0, 0));
    if (!b.section || b.sectionXform == ident) return true;
    const bool kept = transformSection(*b.section, b.sectionXform);
    b.sectionXform = ident;
    return kept;
}

} // namespace oscadeval
