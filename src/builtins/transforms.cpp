#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <algorithm>
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

bool isNumber(const Value& v) { return std::holds_alternative<double>(v); }

// Reads `v` into `xyz` as a 2- or 3-vector of numbers, z defaulting to
// `defaultZ` for a 2-vector. Returns false (having written only the
// leading numbers of a 3-vector, or nothing for anything else) when it is
// not one. A 2-vector holding a non-number still succeeds, but leaves x
// and y alone.
bool readVec3(const Value& v, double xyz[3], double defaultZ) {
    if (!std::holds_alternative<ListPtr>(v)) return false;
    const auto& items = std::get<ListPtr>(v)->items;
    if (items.size() == 2) {
        if (isNumber(items[0]) && isNumber(items[1])) {
            xyz[0] = std::get<double>(items[0]);
            xyz[1] = std::get<double>(items[1]);
        }
        xyz[2] = defaultZ;
        return true;
    }
    if (items.size() != 3) return false;
    for (size_t i = 0; i < 3; ++i) {
        if (!isNumber(items[i])) return false;
        xyz[i] = std::get<double>(items[i]);
    }
    return true;
}

Mat4 matMul(const Mat4& a, const Mat4& b) {
    Mat4 out{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k) out[i][j] += a[i][k] * b[k][j];
    return out;
}

// Rotation by `deg` about one coordinate axis (0 = X, 1 = Y, 2 = Z).
Mat4 axisRotation(int axis, double deg) {
    const double c = cosDeg(deg), s = sinDeg(deg);
    const int p = (axis + 1) % 3, q = (axis + 2) % 3;
    Mat4 m = identity4();
    m[p][p] = c;
    m[p][q] = -s;
    m[q][p] = s;
    m[q][q] = c;
    return m;
}

// Rotation by `deg` about the axis (x, y, z); the identity for a zero axis.
Mat4 axisAngleRotation(double deg, const double axis[3]) {
    const double len = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (len == 0.0) return identity4();
    const double k[3] = {axis[0] / len, axis[1] / len, axis[2] / len};
    const double c = cosDeg(deg), s = sinDeg(deg), t = 1.0 - c;
    Mat4 m = identity4();
    m[0][0] = c + t * k[0] * k[0];
    m[0][1] = t * k[0] * k[1] - s * k[2];
    m[0][2] = t * k[0] * k[2] + s * k[1];
    m[1][0] = t * k[1] * k[0] + s * k[2];
    m[1][1] = c + t * k[1] * k[1];
    m[1][2] = t * k[1] * k[2] - s * k[0];
    m[2][0] = t * k[2] * k[0] - s * k[1];
    m[2][1] = t * k[2] * k[1] + s * k[0];
    m[2][2] = c + t * k[2] * k[2];
    return m;
}

Mat4 translateMatrix(Evaluator& ev, const CallArgs& args, const oscad::Position* where) {
    const Value v = getArg(args, 0, "v");
    double t[3] = {0, 0, 0};
    const bool ok = readVec3(v, t, 0.0) && std::isfinite(t[0]) && std::isfinite(t[1]) && std::isfinite(t[2]);
    Mat4 m = identity4();
    if (!ok) {
        ev.warn("Unable to convert translate(" + fmtValue(v) + ") parameter to a vec3 or vec2 of numbers", where);
        return m;
    }
    for (int i = 0; i < 3; ++i) m[i][3] = t[i];
    return m;
}

Mat4 scaleMatrix(Evaluator& ev, const CallArgs& args, const oscad::Position* where) {
    const Value v = getArg(args, 0, "v");
    double k[3] = {1, 1, 1};
    if (!readVec3(v, k, 1.0)) {
        if (isNumber(v)) {
            k[0] = k[1] = k[2] = std::get<double>(v);
        } else {
            ev.warn("Unable to convert scale(" + fmtValue(v) +
                        ") parameter to a number, a vec3 or vec2 of numbers or a number",
                    where);
        }
    }
    Mat4 m = identity4();
    for (int i = 0; i < 3; ++i) m[i][i] = k[i];
    return m;
}

Mat4 mirrorMatrix(Evaluator& ev, const CallArgs& args, const oscad::Position* where) {
    const Value v = getArg(args, 0, "v");
    double n[3] = {1, 0, 0};
    if (!readVec3(v, n, 0.0))
        ev.warn("Unable to convert mirror(" + fmtValue(v) + ") parameter to a vec3 or vec2 of numbers", where);
    const double len2 = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
    Mat4 m = identity4();
    if (len2 == 0.0) return m;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m[i][j] -= 2.0 * n[i] * n[j] / len2;
    return m;
}

Mat4 rotateMatrix(Evaluator& ev, const CallArgs& args, const oscad::Position* where) {
    const Value a = getArg(args, 0, "a");
    const Value v = getArg(args, 1, "v");
    const bool hasV = !std::holds_alternative<std::monostate>(v);
    const std::string aText = "rotate(a=" + fmtValue(a);
    const std::string vText = hasV ? ", v=" + fmtValue(v) : "";

    if (std::holds_alternative<ListPtr>(a)) {
        // Euler angles about X, then Y, then Z. A non-number repeats the
        // angle read just before it, reading from Z back to X.
        const auto& items = std::get<ListPtr>(a)->items;
        const size_t n = std::min<size_t>(items.size(), 3);
        bool problem = items.size() > 3;
        double angles[3] = {0, 0, 0};
        double last = 0;
        for (size_t i = n; i-- > 0;) {
            if (isNumber(items[i])) {
                last = std::get<double>(items[i]);
                problem |= !std::isfinite(last);
            } else {
                problem = true;
            }
            angles[i] = last;
        }
        if (problem)
            ev.warn("Problem converting " + aText + vText + ") parameter", where);
        else if (hasV)
            ev.warn("When parameter a is supplied as vector, v is ignored " + aText + vText + ")", where);
        return matMul(axisRotation(2, angles[2]), matMul(axisRotation(1, angles[1]), axisRotation(0, angles[0])));
    }

    const bool angleOk = isNumber(a) && std::isfinite(std::get<double>(a));
    const double angle = angleOk ? std::get<double>(a) : 0.0;
    double axis[3] = {0, 0, 1};
    const bool axisOk = !hasV || readVec3(v, axis, 0.0);
    if (!axisOk)
        ev.warn(std::string("Problem converting ") + (angleOk ? "rotate(..." : aText) + ", v=" + fmtValue(v) +
                    ") parameter",
                where);
    else if (!angleOk)
        ev.warn("Problem converting " + aText + ") parameter", where);
    return axisAngleRotation(angle, axis);
}

Mat4 multmatrixMatrix(const CallArgs& args) {
    const Value v = getArg(args, 0, "m");
    Mat4 m = identity4();
    if (!std::holds_alternative<ListPtr>(v)) return m;
    const auto& rows = std::get<ListPtr>(v)->items;
    for (size_t i = 0; i < std::min<size_t>(rows.size(), 4); ++i) {
        if (!std::holds_alternative<ListPtr>(rows[i])) continue;
        const auto& row = std::get<ListPtr>(rows[i])->items;
        for (size_t j = 0; j < std::min<size_t>(row.size(), 4); ++j)
            if (isNumber(row[j])) m[i][j] = std::get<double>(row[j]);
    }
    const double w = m[3][3];
    if (w != 1.0)
        for (auto& row : m)
            for (double& e : row) e /= w;
    return m;
}

// The row-major 4x4 matrix translate/rotate/scale/mirror/multmatrix
// applies (`name` is the module), with its argument warnings. Identity for
// any other name.
Mat4 transformMatrix(Evaluator& ev, const std::string& name, const CallArgs& args, const oscad::Position* where) {
    if (name == "translate") return translateMatrix(ev, args, where);
    if (name == "scale") return scaleMatrix(ev, args, where);
    if (name == "mirror") return mirrorMatrix(ev, args, where);
    if (name == "rotate") return rotateMatrix(ev, args, where);
    if (name == "multmatrix") return multmatrixMatrix(args);
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


// resize(newsize, auto): the requested size per axis (0 where none is
// requested) and which axes are auto-scaled.
void resizeArgs(const CallArgs& args, double ns[3], bool autoAxes[3]) {
    for (int i = 0; i < 3; ++i) {
        ns[i] = 0;
        autoAxes[i] = false;
    }
    const Value newsize = getArg(args, 0, "newsize");
    if (std::holds_alternative<ListPtr>(newsize)) {
        const auto& items = std::get<ListPtr>(newsize)->items;
        for (size_t i = 0; i < std::min<size_t>(items.size(), 3); ++i)
            if (isNumber(items[i])) ns[i] = std::get<double>(items[i]);
    }
    const Value autoArg = getArg(args, 1, "auto");
    if (std::holds_alternative<bool>(autoArg)) {
        for (int i = 0; i < 3; ++i) autoAxes[i] = std::get<bool>(autoArg);
    } else if (std::holds_alternative<ListPtr>(autoArg)) {
        const auto& items = std::get<ListPtr>(autoArg)->items;
        for (size_t i = 0; i < std::min<size_t>(items.size(), 3); ++i) autoAxes[i] = truthy(items[i]);
    }
}

// Per-axis scale over the first `n` axes: each requested axis is scaled to
// its size; an unrequested auto axis follows axis `lead`'s scale.
void resizeScale(int n, const double extent[3], const double ns[3], const bool autoAxes[3], int lead, double k[3]) {
    for (int i = 0; i < n; ++i) k[i] = ns[i] > 0 ? ns[i] / extent[i] : 1.0;
    for (int i = 0; i < n; ++i)
        if (ns[i] <= 0 && autoAxes[i]) k[i] = k[lead];
}

// The per-axis scale resize applies to a 3D body with bounding box `bbox`.
manifold::vec3 resizeScale3d(const manifold::Box& bbox, const double ns[3], const bool autoAxes[3]) {
    const manifold::vec3 size = bbox.Size();
    const double extent[3] = {size.x, size.y, size.z};
    int lead = 0;
    for (int i = 1; i < 3; ++i)
        if (ns[i] > ns[lead]) lead = i;
    double k[3];
    resizeScale(3, extent, ns, autoAxes, lead, k);
    return manifold::vec3(k[0], k[1], k[2]);
}

// The per-axis scale resize applies to a 2D shape with bounding box `bbox`.
manifold::vec2 resizeScale2d(const manifold::Rect& bbox, const double ns[3], const bool autoAxes[3]) {
    const manifold::vec2 size = bbox.Size();
    const double extent[3] = {size.x, size.y, 0};
    const int lead = (ns[1] != 0 && ns[1] > ns[0]) ? 1 : 0;
    double k[3];
    resizeScale(2, extent, ns, autoAxes, lead, k);
    return manifold::vec2(k[0], k[1]);
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
