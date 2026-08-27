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

// -- warp(f) ---------------------------------------------------------------
//
// Moves every vertex through an OpenSCAD function. Manifold::WarpBatch does
// the work; everything here is about getting an OpenSCAD closure called from
// inside it, and about the one hazard Manifold explicitly refuses to check.
//
// Topology is untouched -- no vertices are added -- so a coarse mesh warps
// coarsely. That is the first thing that surprises people; the docs say so.

namespace {

// numList/allNumericList live in function_builtins.cpp's anonymous
// namespace, so these are the two-line local equivalents rather than
// widening that file's interface for one caller.
Value pointValue(double x, double y, double z) {
    return Value{std::make_shared<const ValueList>(ValueList{{Value{x}, Value{y}, Value{z}}})};
}

std::optional<std::vector<double>> numbersOf(const Value& v) {
    const ListPtr* l = std::get_if<ListPtr>(&v);
    if (!l || !*l) return std::nullopt;
    std::vector<double> out;
    out.reserve((*l)->items.size());
    for (const Value& item : (*l)->items) {
        const double* d = std::get_if<double>(&item);
        if (!d) return std::nullopt;
        out.push_back(*d);
    }
    return out;
}

// NO per-triangle fold check here, and that is deliberate.
//
// The obvious one -- compare each triangle's normal before and after and
// count the ones that inverted -- DOES NOT WORK: GetMeshGL() gives no
// guarantee that triangle t before the warp is triangle t after it, so the
// comparison pairs up unrelated triangles. Measured, not assumed: a rigid
// rotation about Z, which cannot self-intersect and leaves the volume
// identical to the digit, reported 7 inverted triangles on a cylinder and 9
// on a sphere. A warning that fires on provably correct geometry is worse
// than no warning, because people learn to ignore it.
//
// What IS sound is the volume sign: a warp whose Jacobian is negative
// everywhere (any mirroring) turns the solid inside out, and that shows up
// exactly, with no false positives. Genuine self-intersection without
// inversion stays undetected -- Manifold says plainly it does not check
// (manifold.cpp:560-566), and detecting it properly is far too expensive to
// run on every warp. The docs say so rather than implying the check is
// complete.

} // namespace

CSGParams resolveWarp(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    CSGParams params;
    // Position 0 so `warp(function(p) ...)` reads without a keyword.
    params["f"] = getArg(args, 0, "f", Value{});
    ev.evalChildren(node.children, effCtx);
    return params;
}

std::vector<ColoredBody> generateWarp(Evaluator& ev, const CSGParams& params,
                                       const std::vector<std::unique_ptr<CSGNode>>& children,
                                       const oscad::ASTNode& node) {
    std::vector<ColoredBody> bodies = flattenCsgTree(children);
    if (bodies.empty()) return {};

    const Value& fnVal = params.at("f");
    const ClosurePtr* cp = std::get_if<ClosurePtr>(&fnVal);
    if (!cp || !*cp || !(*cp)->node) {
        ev.warn("warp() needs a function, as in warp(function(p) [p.x, p.y, p.z*2])", &node.position());
        return bodies;
    }
    const Closure& closure = **cp;
    const auto& fnParams = closure.node->parameters;
    if (fnParams.empty()) {
        ev.warn("warp()'s function needs one parameter, the point to move", &node.position());
        return bodies;
    }
    const std::string& paramName = fnParams[0]->name->name;

    // The generate pass has no live EvalContext -- the one the call was
    // resolved in is long gone. A closure carries its own scope and its
    // captured lets, and evalFunctionLiteralFromBound only reads ctx as a
    // fallback for the scope, so a fresh root built from that scope is
    // enough. Consequence worth knowing: $-variables are at their defaults
    // inside a warp function, not whatever surrounded the call.
    EvalContext rootCtx = EvalContext::makeRoot(closure.node->scope());

    size_t badPoints = 0, inverted = 0;
    for (ColoredBody& b : bodies) {
        if (b.section) {
            // CrossSection has no Warp, and silently doing nothing would be
            // worse than saying so.
            ev.warn("warp() is 3D only; the 2D shape was left alone", &node.position());
            continue;
        }
        if (!b.body) continue;

        b.body = b.body->WarpBatch([&](manifold::VecView<manifold::vec3> verts) {
            for (manifold::vec3& v : verts) {
                BoundArgs bound;
                bound.set(paramName, pointValue(v.x, v.y, v.z));
                const Value out = ev.evalFunctionLiteralFromBound(closure, std::move(bound), rootCtx,
                                                                   &node.position());
                const std::optional<std::vector<double>> p = numbersOf(out);
                if (!p || p->size() < 3 || !std::isfinite((*p)[0]) || !std::isfinite((*p)[1]) ||
                    !std::isfinite((*p)[2])) {
                    ++badPoints;   // leave the vertex where it is
                    continue;
                }
                v.x = (*p)[0];
                v.y = (*p)[1];
                v.z = (*p)[2];
            }
        });
        if (b.body->Volume() < 0.0) ++inverted;
    }

    if (badPoints > 0) {
        ev.warn("warp(): the function returned something other than a 3-vector for " +
                    std::to_string(badPoints) + " vertex/vertices; those were left in place",
                &node.position());
    }
    if (inverted > 0) {
        ev.warn("warp(): " + std::to_string(inverted) +
                    " body/bodies came out inside-out -- the function mirrors, so it needs an "
                    "even number of sign flips to stay a solid",
                &node.position());
    }
    return bodies;
}

} // namespace oscadeval
