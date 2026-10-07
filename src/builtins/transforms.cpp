#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <array>
#include <cmath>
#include <numbers>

namespace oscadeval {

namespace {

using Mat4 = std::array<std::array<double, 4>, 4>;  // row-major

Mat4 identity4() {
    Mat4 m{};
    for (int i = 0; i < 4; ++i) m[i][i] = 1.0;
    return m;
}

// The row-major 4x4 matrix translate/rotate/scale/mirror/multmatrix
// applies (`name` is the module), with its argument warnings. Identity for
// any other name.
// CLEAN-ROOM: reimplement from spec section D1.
Mat4 transformMatrix(Evaluator&, const std::string&, const CallArgs&, const oscad::Position*) {
    return identity4();
}

// The affine part of a row-major 4x4, as Manifold's column-major 3x4.
// ponytail: a projective bottom row (after the [3][3] division) is dropped;
// neither Manifold nor a CrossSection can hold one.
manifold::mat3x4 toMat3x4(const Mat4& m) {
    return manifold::mat3x4(manifold::vec3(m[0][0], m[1][0], m[2][0]), manifold::vec3(m[0][1], m[1][1], m[2][1]),
                            manifold::vec3(m[0][2], m[1][2], m[2][2]), manifold::vec3(m[0][3], m[1][3], m[2][3]));
}

// `a` applied after `b`, both as Manifold's column-major 3x4 affines.
manifold::mat3x4 compose3x4(const manifold::mat3x4& a, const manifold::mat3x4& b) {
    manifold::mat3x4 out;
    for (int c = 0; c < 3; ++c)
        out[c] = a[0] * b[c].x + a[1] * b[c].y + a[2] * b[c].z;
    out[3] = a[0] * b[3].x + a[1] * b[3].y + a[2] * b[3].z + a[3];
    return out;
}

// True if `m` is a pure translation: nothing has rotated, scaled or
// mirrored the section, so a further translation commutes with it.
bool hasIdentityLinearPart(const manifold::mat3x4& m) {
    for (int c = 0; c < 3; ++c) {
        const manifold::vec3 want(c == 0 ? 1.0 : 0.0, c == 1 ? 1.0 : 0.0, c == 2 ? 1.0 : 0.0);
        if (m[c] != want) return false;
    }
    return true;
}

bool isIdentity3x4(const manifold::mat3x4& m) {
    for (int c = 0; c < 4; ++c) {
        const manifold::vec3 want(c == 0 ? 1.0 : 0.0, c == 1 ? 1.0 : 0.0, c == 2 ? 1.0 : 0.0);
        if (m[c] != want) return false;
    }
    return true;
}

// True if `m` maps the XY plane onto itself, so a CrossSection can hold it:
// nothing leaves the plane (no Z from x or y, no Z offset) and Z maps to Z.
bool preservesXYPlane(const manifold::mat3x4& m) {
    return m[0].z == 0.0 && m[1].z == 0.0 && m[3].z == 0.0 &&
           m[2].x == 0.0 && m[2].y == 0.0;
}

// Applies `m` to a raw mesh's vertex POSITIONS in place. Any further
// per-vertex properties (normals and the like) are left alone: nothing
// reads them off a display-only body, which is drawn from its triangles.
void transformMeshInPlace(manifold::MeshGL& mesh, const manifold::mat3x4& m) {
    const uint32_t stride = mesh.numProp;
    if (stride < 3) return;
    for (size_t i = 0; i + 2 < mesh.vertProperties.size(); i += stride) {
        const double x = mesh.vertProperties[i];
        const double y = mesh.vertProperties[i + 1];
        const double z = mesh.vertProperties[i + 2];
        const manifold::vec3 p = m[0] * x + m[1] * y + m[2] * z + m[3];
        mesh.vertProperties[i] = static_cast<float>(p.x);
        mesh.vertProperties[i + 1] = static_cast<float>(p.y);
        mesh.vertProperties[i + 2] = static_cast<float>(p.z);
    }
}


// resize(newsize, auto): the requested size per axis and which axes are
// auto-scaled.
// CLEAN-ROOM: reimplement from spec section D2.
void resizeArgs(const CallArgs&, double ns[3], bool autoAxes[3]) {
    for (int i = 0; i < 3; ++i) {
        ns[i] = 0;
        autoAxes[i] = false;
    }
}

// The per-axis scale resize applies to a 3D body with bounding box `bbox`.
// CLEAN-ROOM: reimplement from spec section D2.
manifold::vec3 resizeScale3d(const manifold::Box&, const double[3], const bool[3]) {
    return manifold::vec3(1.0, 1.0, 1.0);
}

// The per-axis scale resize applies to a 2D shape with bounding box `bbox`.
// CLEAN-ROOM: reimplement from spec section D2.
manifold::vec2 resizeScale2d(const manifold::Rect&, const double[3], const bool[3]) {
    return manifold::vec2(1.0, 1.0);
}

} // namespace

BuiltinWrapParams computeTransformParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    const std::string& name = node.name->name;
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);

    CSGParams params;
    params["name"] = Value{name};
    // The arguments as written: what a CSG-tree dump shows, and all resize
    // needs (its scale comes from the children's bounding box, so it can
    // only be worked out at generate time).
    params["args"] = callArgsToValue(args);
    if (name != "resize") {
        // Every other transform is one fixed matrix, built here, once, with
        // its warnings -- the same matrix for a solid, a raw mesh and a 2D
        // section.
        const Mat4 m = transformMatrix(ev, name, args, &node.position());
        bool finite = true;
        std::vector<Value> flat;
        for (const auto& row : m)
            for (double e : row) {
                finite &= std::isfinite(e);
                flat.push_back(Value{e});
            }
        if (!finite) {
            ev.warn("Transformation matrix contains Not-a-Number and/or Infinity - removing object.",
                    &node.position());
        }
        params["m"] = Value{makeList(std::move(flat))};
        params["remove"] = Value{!finite};
    }
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveTransform(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeTransformParams(ev, node, ctx);
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

namespace {
manifold::mat3x4 paramsMatrix(const CSGParams& params) {
    const auto& items = std::get<ListPtr>(params.at("m"))->items;
    Mat4 m{};
    for (int i = 0; i < 16; ++i) m[static_cast<size_t>(i / 4)][static_cast<size_t>(i % 4)] = std::get<double>(items[static_cast<size_t>(i)]);
    return toMat3x4(m);
}

std::shared_ptr<const std::vector<ColoredBody>> transformParts(const std::vector<ColoredBody>& parts,
                                                                const manifold::mat3x4& m) {
    std::vector<ColoredBody> out;
    for (ColoredBody part : parts) {
        if (part.body) part.body = part.body->Transform(m);
        if (part.mergedFrom) part.mergedFrom = transformParts(*part.mergedFrom, m);
        out.push_back(std::move(part));
    }
    return std::make_shared<const std::vector<ColoredBody>>(std::move(out));
}
} // namespace

std::vector<ColoredBody> generateTransform(Evaluator& ev, const CSGParams& params,
                                            const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode&) {
    const std::string& name = std::get<std::string>(params.at("name"));

    if (name == "resize") {
        const CallArgs args = valueToCallArgs(params.at("args"));
        double ns[3];
        bool autoAxes[3];
        resizeArgs(args, ns, autoAxes);
        std::vector<ColoredBody> result;
        for (ColoredBody b : flattenCsgTree(children)) {
            if (b.section) {
                // resize measures the shape, so the shape has to be where
                // it really is first. See projectSectionXform.
                if (!projectSectionXform(b)) ev.warn(kZeroScale2dWarning, nullptr);
                const manifold::vec2 k = resizeScale2d(b.section->Bounds(), ns, autoAxes);
                b.section = b.section->Scale(k);
            } else if (b.body) {
                const manifold::vec3 k = resizeScale3d(b.body->BoundingBox(), ns, autoAxes);
                b.body = b.body->Scale(k);
                if (b.mergedFrom) {
                    manifold::mat3x4 s(manifold::vec3(k.x, 0, 0), manifold::vec3(0, k.y, 0), manifold::vec3(0, 0, k.z),
                                       manifold::vec3(0, 0, 0));
                    b.mergedFrom = transformParts(*b.mergedFrom, s);
                }
            }
            result.push_back(std::move(b));
        }
        return result;
    }

    // A matrix with NaN or infinity in it removes its children (warned at
    // resolve).
    if (std::get<bool>(params.at("remove"))) return {};
    const manifold::mat3x4 m = paramsMatrix(params);

    std::vector<ColoredBody> result;
    for (ColoredBody b : flattenCsgTree(children)) {
        if (b.section) {
            // A 2D-representable transform goes into the CrossSection, so
            // 2D booleans and offset() keep operating on real 2D geometry.
            // Anything else -- a Z translation, a rotation out of the XY
            // plane -- has nowhere to go there and rides along on the body
            // until the section is extruded. Once one of those has been
            // seen, every LATER (outer) transform must ride along too, or
            // it would be applied in the wrong order. See
            // ColoredBody::sectionXform.
            if (isIdentity3x4(b.sectionXform) && preservesXYPlane(m)) {
                if (!transformSection(*b.section, m)) ev.warn(kZeroScale2dWarning, nullptr);
            } else if (name == "translate" && hasIdentityLinearPart(b.sectionXform)) {
                // A translate SPLITS while nothing has rotated or scaled the
                // section yet: x/y into the CrossSection, z into the carried
                // transform. Translations commute, so the order is safe --
                // and keeping x/y in the section is what lets a later 2D
                // boolean or offset() still see real 2D geometry after a
                // `down(1) square(10)`.
                b.section = b.section->Translate(manifold::vec2(m[3].x, m[3].y));
                b.sectionXform[3].z += m[3].z;
            } else {
                b.sectionXform = compose3x4(m, b.sectionXform);
            }
        } else if (b.body) {
            b.body = b.body->Transform(m);
            // A display-only body's real geometry is its raw triangles --
            // the Manifold above is empty and moving it moves nothing.
            if (b.rawMesh) transformMeshInPlace(*b.rawMesh, m);
            // The unmerged parts of a union move with it (see
            // ColoredBody::mergedFrom).
            if (b.mergedFrom) b.mergedFrom = transformParts(*b.mergedFrom, m);
        }
        result.push_back(std::move(b));
    }
    return result;
}

} // namespace oscadeval
