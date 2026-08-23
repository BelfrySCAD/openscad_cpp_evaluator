#include "builtins.hpp"

#include "openscad_cpp_evaluator/dispatch.hpp"
#include "openscad_cpp_evaluator/eval_error.hpp"

namespace oscadeval {

const std::unordered_map<std::string_view, ResolveFn>& resolveDispatch() {
    static const std::unordered_map<std::string_view, ResolveFn> table = {
        {"cube", &resolveCube},
        {"sphere", &resolveSphere},
        {"cylinder", &resolveCylinder},
        {"polyhedron", &resolvePolyhedron},
        {"circle", &resolve2d},
        {"square", &resolve2d},
        {"polygon", &resolve2d},
        {"translate", &resolveTransform},
        {"rotate", &resolveTransform},
        {"scale", &resolveTransform},
        {"mirror", &resolveTransform},
        {"multmatrix", &resolveTransform},
        {"resize", &resolveTransform},
        {"color", &resolveColor},
        {"union", &resolveCsg},
        {"difference", &resolveCsg},
        {"intersection", &resolveCsg},
        {"children", &resolveChildren},
        {"render", &resolveRender},
        {"breakpoint", &resolveBreakpoint},
        {"import", &resolveImport},
        {"hull", &resolveHull},
        {"minkowski", &resolveMinkowski},
        {"linear_extrude", &resolveLinearExtrude},
        {"rotate_extrude", &resolveRotateExtrude},
        {"projection", &resolveProjection},
        {"roof", &resolveRoof},
        {"fill", &resolveFill},
        {"minkowski_difference", &resolveMinkowskiDifference},
        {"simplify", &resolveSimplify},
        {"offset", &resolveOffset},
        {"surface", &resolveSurface},
        {"text", &resolveText},
    };
    return table;
}

const std::unordered_map<std::string_view, GenerateFn>& generateDispatch() {
    static const std::unordered_map<std::string_view, GenerateFn> table = {
        {"cube", &generateCube},
        {"sphere", &generateSphere},
        {"cylinder", &generateCylinder},
        {"polyhedron", &generatePolyhedron},
        {"circle", &generate2d},
        {"square", &generate2d},
        {"polygon", &generate2d},
        {"translate", &generateTransform},
        {"rotate", &generateTransform},
        {"scale", &generateTransform},
        {"mirror", &generateTransform},
        {"multmatrix", &generateTransform},
        {"resize", &generateTransform},
        {"color", &generateColor},
        {"union", &generateCsg},
        {"difference", &generateCsg},
        {"intersection", &generateCsg},
        {"highlight", &generateHighlight},
        {"background", &generateBackground},
        {"show_only", &generateShowOnly},
        {"children", &generateChildren},
        {"intersection_for", &generateIntersectionFor},
        {"import", &generateImport},
        {"hull", &generateHull},
        {"minkowski", &generateMinkowski},
        {"linear_extrude", &generateLinearExtrude},
        {"rotate_extrude", &generateRotateExtrude},
        {"projection", &generateProjection},
        {"roof", &generateRoof},
        {"fill", &generateFill},
        {"minkowski_difference", &generateMinkowskiDifference},
        {"simplify", &generateSimplify},
        {"offset", &generateOffset},
        {"surface", &generateSurface},
        {"text", &generateText},
    };
    return table;
}

const std::vector<std::string>* builtinParamNames(const std::string& name) {
    // Deliberately absent: the builtin *functions* other than the three
    // below. Real OpenSCAD's builtin functions read their arguments
    // positionally without going through Parameters::parse, so `sin(bogus=30)`
    // warns about nothing upstream -- verified against 2022.08.22 -- and
    // warning here would be a divergence, not a fix.
    static const std::unordered_map<std::string, std::vector<std::string>> table = {
        // -- builtin modules ---------------------------------------------
        {"cube", {"size", "center"}},
        {"sphere", {"r", "d", "style"}},
        {"cylinder", {"h", "r1", "r2", "center", "r", "d", "d1", "d2"}},
        {"polyhedron", {"points", "faces", "convexity", "triangles"}},
        {"square", {"size", "center"}},
        {"circle", {"r", "d"}},
        {"polygon", {"points", "paths", "convexity"}},
        {"translate", {"v"}},
        {"rotate", {"a", "v"}},
        {"scale", {"v"}},
        {"mirror", {"v"}},
        {"multmatrix", {"m"}},
        {"resize", {"newsize", "auto", "convexity"}},
        {"color", {"c", "alpha"}},
        {"union", {}},
        {"difference", {}},
        {"intersection", {}},
        {"hull", {}},
        {"minkowski", {"convexity"}},
        {"children", {"index", "separate"}},
        {"render", {"convexity"}},
        // "repair" is this port's own addition, not an upstream parameter.
        {"import",
          {"file", "layer", "convexity", "origin", "scale", "width", "height", "filename", "layername", "center", "dpi",
           "id", "repair"}},
        {"linear_extrude", {"height", "v", "scale", "center", "twist", "slices", "segments", "convexity"}},
        {"rotate_extrude", {"angle", "start", "convexity"}},
        {"projection", {"cut", "convexity"}},
        {"roof", {"method", "convexity"}},
        {"fill", {}},
        {"minkowski_difference", {}},
        {"simplify", {"tolerance"}},
        {"offset", {"r", "delta", "chamfer"}},
        {"surface", {"file", "center", "convexity", "invert"}},
        {"text", {"text", "size", "font", "direction", "language", "script", "halign", "valign", "spacing"}},
        // breakpoint() is this port's own debugger extension, no upstream
        // equivalent to mirror.
        {"breakpoint", {"condition"}},

        // -- the three builtin FUNCTIONS that do use Parameters::parse ----
        {"textmetrics", {"text", "size", "font", "direction", "language", "script", "halign", "valign", "spacing"}},
        {"fontmetrics", {"size", "font"}},
    };
    auto it = table.find(name);
    return it == table.end() ? nullptr : &it->second;
}

void warnUnexpectedBuiltinArgs(Evaluator& ev, const oscad::ASTNode& callNode) {
    if (callNode.kind() != oscad::NodeKind::ModularCall) return;
    const auto& call = static_cast<const oscad::ModularCall&>(callNode);
    if (const std::vector<std::string>* declared = builtinParamNames(call.name->name)) {
        warnUnexpectedArgs(ev, *declared, call.arguments);
    }
}

} // namespace oscadeval
