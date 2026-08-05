#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <array>
#include <cmath>
#include <numbers>

namespace oscadeval {

namespace {

manifold::vec3 toVec3(const Value& v, double defaultX = 0.0, double defaultY = 0.0, double defaultZ = 0.0) {
    if (const double* d = std::get_if<double>(&v)) return manifold::vec3(*d, 0.0, 0.0);
    if (const ListPtr* l = std::get_if<ListPtr>(&v); l && *l) {
        const auto& items = (*l)->items;
        const double x = items.size() > 0 ? toDoubleLenient(items[0]) : defaultX;
        const double y = items.size() > 1 ? toDoubleLenient(items[1]) : defaultY;
        const double z = items.size() > 2 ? toDoubleLenient(items[2]) : defaultZ;
        return manifold::vec3(x, y, z);
    }
    return manifold::vec3(defaultX, defaultY, defaultZ);
}

manifold::vec2 toVec2(const Value& v, double defaultX = 0.0, double defaultY = 0.0) {
    if (const ListPtr* l = std::get_if<ListPtr>(&v); l && *l) {
        const auto& items = (*l)->items;
        const double x = items.size() > 0 ? toDoubleLenient(items[0]) : defaultX;
        const double y = items.size() > 1 ? toDoubleLenient(items[1]) : defaultY;
        return manifold::vec2(x, y);
    }
    return manifold::vec2(defaultX, defaultY);
}

// resize()'s `auto` argument: a bare bool applies to all three axes, a
// list applies element-wise (missing elements are false), anything else
// (including undef) disables auto entirely. Mirrors the reference's
// Value::toBool()/vector handling.
std::array<bool, 3> toAutoAxes(const Value& v) {
    if (const bool* b = std::get_if<bool>(&v)) return {*b, *b, *b};
    if (const ListPtr* l = std::get_if<ListPtr>(&v); l && *l) {
        const auto& items = (*l)->items;
        std::array<bool, 3> out{false, false, false};
        for (size_t i = 0; i < 3 && i < items.size(); ++i) out[i] = truthy(items[i]);
        return out;
    }
    return {false, false, false};
}

// Rodrigues' rotation formula as a 3x4 transform (rotate(angle, axis) --
// the axis form, as opposed to rotate([x,y,z]) Euler angles, which uses
// Manifold's own Rotate() directly and never needs this). Mirrors
// _axis_angle_matrix.
manifold::mat3x4 axisAngleMatrix(manifold::vec3 axis, double angleRad) {
    double ax = axis.x, ay = axis.y, az = axis.z;
    const double length = std::sqrt(ax * ax + ay * ay + az * az);
    if (length == 0.0) {
        return manifold::mat3x4(manifold::vec3(1, 0, 0), manifold::vec3(0, 1, 0), manifold::vec3(0, 0, 1),
                                 manifold::vec3(0, 0, 0));
    }
    ax /= length;
    ay /= length;
    az /= length;
    const double c = std::cos(angleRad), s = std::sin(angleRad), t = 1 - c;
    // Row-major in the reference; la::mat is column-major, so each
    // manifold::vec3 below is a COLUMN (transpose of the reference's rows).
    const manifold::vec3 col0(t * ax * ax + c, t * ax * ay + s * az, t * ax * az - s * ay);
    const manifold::vec3 col1(t * ax * ay - s * az, t * ay * ay + c, t * ay * az + s * ax);
    const manifold::vec3 col2(t * ax * az + s * ay, t * ay * az - s * ax, t * az * az + c);
    const manifold::vec3 col3(0, 0, 0);
    return manifold::mat3x4(col0, col1, col2, col3);
}

manifold::Manifold applyRotate(manifold::Manifold body, const Value& aArg, const Value& vArg) {
    if (std::holds_alternative<ListPtr>(aArg)) {
        // rotate([x,y,z]) -- Euler angles in degrees, applied Z then Y then X.
        const manifold::vec3 a = toVec3(aArg);
        return body.Rotate(a.x, a.y, a.z);
    }
    const double angle = toDoubleLenient(aArg);
    const manifold::vec3 axis = std::holds_alternative<std::monostate>(vArg) ? manifold::vec3(0, 0, 1) : toVec3(vArg);
    return body.Transform(axisAngleMatrix(axis, angle * std::numbers::pi / 180.0));
}

// Converts a user-supplied 4x4/4x3 nested-list matrix (row-major, like
// every OpenSCAD-facing representation) to Manifold's column-major 3x4
// transform. Mirrors _to_matrix4x3.
manifold::mat3x4 toMat3x4(const Value& m) {
    const ListPtr* rows = std::get_if<ListPtr>(&m);
    auto rowAt = [&](size_t r, size_t c) -> double {
        if (!rows || !*rows || r >= (*rows)->items.size()) return 0.0;
        const ListPtr* row = std::get_if<ListPtr>(&(*rows)->items[r]);
        if (!row || !*row || c >= (*row)->items.size()) return 0.0;
        return toDoubleLenient((*row)->items[c]);
    };
    const manifold::vec3 col0(rowAt(0, 0), rowAt(1, 0), rowAt(2, 0));
    const manifold::vec3 col1(rowAt(0, 1), rowAt(1, 1), rowAt(2, 1));
    const manifold::vec3 col2(rowAt(0, 2), rowAt(1, 2), rowAt(2, 2));
    const manifold::vec3 col3(rowAt(0, 3), rowAt(1, 3), rowAt(2, 3));
    return manifold::mat3x4(col0, col1, col2, col3);
}

// 2x3 affine sub-matrix extracted from the same 4x4/4x3 input: rows 0,1,
// columns 0,1,3. Mirrors _apply_transform_2d's "multmatrix" branch.
manifold::mat2x3 toMat2x3(const Value& m) {
    const ListPtr* rows = std::get_if<ListPtr>(&m);
    auto rowAt = [&](size_t r, size_t c) -> double {
        if (!rows || !*rows || r >= (*rows)->items.size()) return 0.0;
        const ListPtr* row = std::get_if<ListPtr>(&(*rows)->items[r]);
        if (!row || !*row || c >= (*row)->items.size()) return 0.0;
        return toDoubleLenient((*row)->items[c]);
    };
    const manifold::vec2 col0(rowAt(0, 0), rowAt(1, 0));
    const manifold::vec2 col1(rowAt(0, 1), rowAt(1, 1));
    const manifold::vec2 col2(rowAt(0, 3), rowAt(1, 3));
    return manifold::mat2x3(col0, col1, col2);
}

manifold::Manifold applyTransform3d(const std::string& name, const CallArgs& args, manifold::Manifold body) {
    if (name == "translate") {
        return body.Translate(toVec3(getArg(args, 0, "v", Value{})));
    }
    if (name == "rotate") {
        return applyRotate(std::move(body), getArg(args, 0, "a", Value{0.0}), getArg(args, 1, "v", Value{}));
    }
    if (name == "scale") {
        Value v = getArg(args, 0, "v", Value{1.0});
        if (const double* s = std::get_if<double>(&v)) return body.Scale(manifold::vec3(*s, *s, *s));
        return body.Scale(toVec3(v, 1.0, 1.0, 1.0));
    }
    if (name == "mirror") {
        return body.Mirror(toVec3(getArg(args, 0, "v", Value{}), 1.0, 0.0, 0.0));
    }
    if (name == "resize") {
        const manifold::vec3 newSizeV = toVec3(getArg(args, 0, "newsize", Value{}));
        std::array<bool, 3> autoAxes = toAutoAxes(getArg(args, 1, "auto", Value{}));
        const manifold::Box bbox = body.BoundingBox();
        const manifold::vec3 spanV = bbox.max - bbox.min;
        const double ns[3] = {newSizeV.x, newSizeV.y, newSizeV.z};
        const double span[3] = {spanV.x, spanV.y, spanV.z};

        // An `auto` axis takes the scale factor of whichever axis asked for
        // the LARGEST new size -- not its own (it has none) and not the
        // first specified one. Mirrors the reference's newsizemax_index:
        // ties keep the earlier axis, and an all-zero newsize leaves the
        // factor at 1 so `resize([0,0,0], true)` is a no-op.
        int maxIdx = 0;
        for (int i = 1; i < 3; ++i) {
            if (ns[i] > ns[maxIdx]) maxIdx = i;
        }
        const double autoScale = (ns[maxIdx] != 0.0 && span[maxIdx] != 0.0) ? ns[maxIdx] / span[maxIdx] : 1.0;

        double scale[3];
        for (int i = 0; i < 3; ++i) {
            if (ns[i] != 0.0 && span[i] != 0.0) {
                scale[i] = ns[i] / span[i];
            } else if (ns[i] == 0.0 && autoAxes[static_cast<size_t>(i)]) {
                scale[i] = autoScale;
            } else {
                scale[i] = 1.0;
            }
        }
        return body.Scale(manifold::vec3(scale[0], scale[1], scale[2]));
    }
    if (name == "multmatrix") {
        Value m = getArg(args, 0, "m", Value{});
        if (std::holds_alternative<std::monostate>(m)) return body;
        return body.Transform(toMat3x4(m));
    }
    return body;
}

manifold::CrossSection applyTransform2d(const std::string& name, const CallArgs& args, manifold::CrossSection cs) {
    if (name == "translate") return cs.Translate(toVec2(getArg(args, 0, "v", Value{})));
    if (name == "rotate") {
        Value a = getArg(args, 0, "a", Value{0.0});
        double angle;
        if (const ListPtr* l = std::get_if<ListPtr>(&a); l && *l) {
            angle = (*l)->items.size() > 2 ? toDoubleLenient((*l)->items[2]) : 0.0;
        } else {
            angle = toDoubleLenient(a);
        }
        return cs.Rotate(angle);
    }
    if (name == "scale") {
        Value v = getArg(args, 0, "v", Value{1.0});
        if (const double* s = std::get_if<double>(&v)) return cs.Scale(manifold::vec2(*s, *s));
        return cs.Scale(toVec2(v, 1.0, 1.0));
    }
    if (name == "mirror") return cs.Mirror(toVec2(getArg(args, 0, "v", Value{}), 1.0, 0.0));
    if (name == "multmatrix") {
        Value m = getArg(args, 0, "m", Value{});
        if (std::holds_alternative<std::monostate>(m)) return cs;
        return cs.Transform(toMat2x3(m));
    }
    // "resize" is 3D-only in the reference (2D children pass through
    // unchanged); "translate"/"rotate"/"scale"/"mirror"/"multmatrix" above
    // cover every 2D-applicable name.
    return cs;
}

} // namespace

BuiltinWrapParams computeTransformParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    const std::string& name = node.name->name;
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);

    CSGParams params;
    params["name"] = Value{name};
    // 2D vs. 3D dispatch can't be decided until generate time (only then is
    // each child's actual body/section type known), so the raw arguments
    // are carried through params via callArgsToValue() and re-derived with
    // getArg() again in applyTransform2d/3d below, instead of resolving
    // e.g. "v"/"a" to a concrete vec3/vec2 here.
    params["args"] = callArgsToValue(args);
    return BuiltinWrapParams{std::move(params), std::move(effCtx)};
}

CSGParams resolveTransform(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeTransformParams(ev, node, ctx);
    ev.evalChildren(node.children, result.ctx);
    return std::move(result.params);
}

std::vector<ColoredBody> generateTransform(Evaluator&, const CSGParams& params,
                                            const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode&) {
    const std::string& name = std::get<std::string>(params.at("name"));
    const CallArgs args = valueToCallArgs(params.at("args"));

    std::vector<ColoredBody> result;
    for (ColoredBody b : flattenCsgTree(children)) {
        if (b.section) {
            b.section = applyTransform2d(name, args, std::move(*b.section));
        } else if (b.body) {
            b.body = applyTransform3d(name, args, std::move(*b.body));
        }
        result.push_back(std::move(b));
    }
    return result;
}

} // namespace oscadeval
