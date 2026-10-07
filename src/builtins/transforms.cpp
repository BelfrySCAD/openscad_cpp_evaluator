#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <array>
#include <cmath>
#include <numbers>

namespace oscadeval {

namespace {

using Mat4 = std::array<std::array<double, 4>, 4>;  // row-major, as OpenSCAD's Matrix4d

Mat4 identity4() {
    Mat4 m{};
    for (int i = 0; i < 4; ++i) m[i][i] = 1.0;
    return m;
}

// Value::getDouble: a number, nothing else.
bool getNum(const Value& v, double& out) {
    if (const double* d = std::get_if<double>(&v)) {
        out = *d;
        return true;
    }
    return false;
}

// Value::getVec3(x, y, z, default), assignments and all: a 2-vector sets
// x and y and the default z; a 3-vector assigns element by element and
// stops at the first that is not a number -- so translate([1,"a",3]) has
// already written its 1 when it fails. Anything else writes nothing.
bool getVec3(const Value& v, double& x, double& y, double& z, double defaultZ) {
    const ListPtr* l = std::get_if<ListPtr>(&v);
    if (!l || !*l) return false;
    const auto& it = (*l)->items;
    if (it.size() == 2) {
        double a, b;
        if (getNum(it[0], a) && getNum(it[1], b)) {
            x = a;
            y = b;
        }
        z = defaultZ;
        return true;
    }
    if (it.size() != 3) return false;
    return getNum(it[0], x) && getNum(it[1], y) && getNum(it[2], z);
}

// OpenSCAD's angle_axis_degrees(): Rodrigues, without normalising v where
// it can be avoided, and the identity for a zero axis.
std::array<std::array<double, 3>, 3> angleAxisDegrees(double a, double vx, double vy, double vz) {
    std::array<std::array<double, 3>, 3> M{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const double s = sinDeg(a), c = cosDeg(a);
    const double m = vx * vx + vy * vy + vz * vz;
    if (m > 0) {
        const double k = (1 - c) / m;
        const double Cv[3] = {vx * k, vy * k, vz * k};
        const double n = std::sqrt(m);
        const double us[3] = {vx / n * s, vy / n * s, vz / n * s};
        const double v[3] = {vx, vy, vz};
        M = {{{Cv[0] * v[0] + c, Cv[1] * v[0] - us[2], Cv[2] * v[0] + us[1]},
              {Cv[0] * v[1] + us[2], Cv[1] * v[1] + c, Cv[2] * v[1] - us[0]},
              {Cv[0] * v[2] - us[1], Cv[1] * v[2] + us[0], Cv[2] * v[2] + c}}};
    }
    return M;
}

Mat4 fromLinear(const std::array<std::array<double, 3>, 3>& r) {
    Mat4 m = identity4();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m[i][j] = r[i][j];
    return m;
}

// The 4x4 matrix a transform module applies, built exactly as upstream's
// builtin_translate/rotate/scale/mirror/multmatrix build it (TransformNode.cc),
// with the same warnings for arguments they cannot use. Previously each
// argument was coerced leniently: translate(5) moved by [5,0,0],
// scale([2]) scaled only x, mirror([0,0,0]) deleted the object, and
// multmatrix filled missing entries with 0 rather than the identity and
// ignored the [3][3] normaliser.
Mat4 transformMatrix(Evaluator& ev, const std::string& name, const CallArgs& args, const oscad::Position* where) {
    Mat4 m = identity4();
    if (name == "translate") {
        const Value v = getArg(args, 0, "v");
        double x = 0, y = 0, z = 0;
        const bool ok = getVec3(v, x, y, z, 0.0) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        if (ok) {
            m[0][3] = x;
            m[1][3] = y;
            m[2][3] = z;
        } else {
            ev.warn("Unable to convert translate(" + fmtValue(v) + ") parameter to a vec3 or vec2 of numbers", where);
        }
    } else if (name == "scale") {
        const Value v = getArg(args, 0, "v");
        double sx = 1, sy = 1, sz = 1;
        if (!getVec3(v, sx, sy, sz, 1.0)) {
            double num;
            if (getNum(v, num)) {
                sx = sy = sz = num;
            } else {
                ev.warn("Unable to convert scale(" + fmtValue(v) +
                            ") parameter to a number, a vec3 or vec2 of numbers or a number",
                        where);
            }
        }
        m[0][0] = sx;
        m[1][1] = sy;
        m[2][2] = sz;
    } else if (name == "mirror") {
        const Value v = getArg(args, 0, "v");
        double x = 1, y = 0, z = 0;
        if (!getVec3(v, x, y, z, 0.0)) {
            ev.warn("Unable to convert mirror(" + fmtValue(v) + ") parameter to a vec3 or vec2 of numbers", where);
        }
        if (x != 0.0 || y != 0.0 || z != 0.0) {
            const double a = x * x + y * y + z * z;
            m = {{{1 - 2 * x * x / a, -2 * y * x / a, -2 * z * x / a, 0},
                  {-2 * x * y / a, 1 - 2 * y * y / a, -2 * z * y / a, 0},
                  {-2 * x * z / a, -2 * y * z / a, 1 - 2 * z * z / a, 0},
                  {0, 0, 0, 1}}};
        }
    } else if (name == "rotate") {
        const Value valA = getArg(args, 0, "a");
        const Value valV = getArg(args, 1, "v");
        const bool vSupplied = !std::holds_alternative<std::monostate>(valV);
        if (const ListPtr* la = std::get_if<ListPtr>(&valA); la && *la) {
            const auto& va = (*la)->items;
            double sx = 0, sy = 0, sz = 0, cx = 1, cy = 1, cz = 1, a = 0;
            bool ok = true;
            const auto axis = [&](size_t i, double& s, double& c) {
                ok &= getNum(va[i], a);
                ok &= std::isfinite(a);
                s = sinDeg(a);
                c = cosDeg(a);
            };
            switch (va.size()) {
            default: ok = false; [[fallthrough]];
            case 3: axis(2, sz, cz); [[fallthrough]];
            case 2: axis(1, sy, cy); [[fallthrough]];
            case 1: axis(0, sx, cx); break;
            case 0: break;
            }
            if (ok) {
                if (vSupplied)
                    ev.warn("When parameter a is supplied as vector, v is ignored rotate(a=" + fmtValue(valA) +
                                ", v=" + fmtValue(valV) + ")",
                            where);
            } else if (vSupplied) {
                ev.warn("Problem converting rotate(a=" + fmtValue(valA) + ", v=" + fmtValue(valV) + ") parameter",
                        where);
            } else {
                ev.warn("Problem converting rotate(a=" + fmtValue(valA) + ") parameter", where);
            }
            m = fromLinear({{{cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz},
                             {cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz},
                             {-sy, cy * sx, cx * cy}}});
        } else {
            double a = 0.0;
            bool aConverted = getNum(valA, a);
            aConverted &= std::isfinite(a);
            double vx = 0, vy = 0, vz = 1;
            const bool vConverted = getVec3(valV, vx, vy, vz, 0.0);
            m = fromLinear(angleAxisDegrees(aConverted ? a : 0, vx, vy, vz));
            if (vSupplied && !vConverted) {
                ev.warn(aConverted ? "Problem converting rotate(..., v=" + fmtValue(valV) + ") parameter"
                                   : "Problem converting rotate(a=" + fmtValue(valA) + ", v=" + fmtValue(valV) +
                                         ") parameter",
                        where);
            } else if (!aConverted) {
                ev.warn("Problem converting rotate(a=" + fmtValue(valA) + ") parameter", where);
            }
        }
    } else if (name == "multmatrix") {
        const Value mv = getArg(args, 0, "m");
        if (const ListPtr* rows = std::get_if<ListPtr>(&mv); rows && *rows) {
            const auto& r = (*rows)->items;
            for (size_t i = 0; i < std::min<size_t>(r.size(), 4); ++i) {
                const ListPtr* row = std::get_if<ListPtr>(&r[i]);
                if (!row || !*row) continue;
                const auto& cols = (*row)->items;
                for (size_t j = 0; j < std::min<size_t>(cols.size(), 4); ++j) getNum(cols[j], m[i][j]);
            }
            const double w = m[3][3];
            if (w != 1.0)
                for (auto& row : m)
                    for (double& e : row) e /= w;
        }
    }
    return m;
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


// upstream's resize newsize/auto parsing (CgalAdvNode.cc): newsize only
// from a vector (resize(10) is ignored), auto from a bool or a vector.
void resizeArgs(const CallArgs& args, double ns[3], bool autoAxes[3]) {
    for (int i = 0; i < 3; ++i) {
        ns[i] = 0;
        autoAxes[i] = false;
    }
    const Value nv = getArg(args, 0, "newsize");
    if (const ListPtr* l = std::get_if<ListPtr>(&nv); l && *l)
        for (size_t i = 0; i < 3 && i < (*l)->items.size(); ++i) {
            const double* d = std::get_if<double>(&(*l)->items[i]);
            ns[i] = d ? *d : 0.0;
        }
    const Value av = getArg(args, 1, "auto");
    if (const bool* b = std::get_if<bool>(&av)) {
        for (int i = 0; i < 3; ++i) autoAxes[i] = *b;
    } else if (const ListPtr* l = std::get_if<ListPtr>(&av); l && *l) {
        for (size_t i = 0; i < 3 && i < (*l)->items.size(); ++i) autoAxes[i] = truthy((*l)->items[i]);
    }
}

// GeometryUtils::getResizeTransform: only a positive newsize scales its
// axis; an auto axis takes the scale of the largest requested one.
manifold::vec3 resizeScale3d(const manifold::Box& bbox, const double ns[3], const bool autoAxes[3]) {
    const manifold::vec3 span = bbox.max - bbox.min;
    const double sp[3] = {span.x, span.y, span.z};
    int maxdim = 0;
    for (int i = 1; i < 3; ++i)
        if (ns[i] > ns[maxdim]) maxdim = i;
    double scale[3] = {1, 1, 1};
    for (int i = 0; i < 3; ++i)
        if (ns[i] > 0) scale[i] = ns[i] / sp[i];
    const double autoscale = scale[maxdim];
    double out[3];
    for (int i = 0; i < 3; ++i) out[i] = (!autoAxes[i] || ns[i] > 0) ? scale[i] : autoscale;
    return manifold::vec3(out[0], out[1], out[2]);
}

// Polygon2d::resize: the same rule over two axes. resize() of a 2D shape
// used to pass it through untouched.
manifold::vec2 resizeScale2d(const manifold::Rect& bbox, const double ns[3], const bool autoAxes[3]) {
    const double sp[2] = {bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y};
    const int maxdim = (ns[1] != 0 && ns[1] > ns[0]) ? 1 : 0;
    const double scale[2] = {ns[0] > 0 ? ns[0] / sp[0] : 1, ns[1] > 0 ? ns[1] / sp[1] : 1};
    const double autoscale = ns[maxdim] > 0 ? ns[maxdim] / sp[maxdim] : 1;
    return manifold::vec2((!autoAxes[0] || ns[0] > 0) ? scale[0] : autoscale,
                          (!autoAxes[1] || ns[1] > 0) ? scale[1] : autoscale);
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

    // A matrix with NaN or infinity in it removes its children, as
    // upstream's GeometryEvaluator does (warned at resolve).
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
