#include "builtins.hpp"

#include "openscad_cpp_evaluator/dispatch.hpp"

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
        {"offset", &generateOffset},
        {"surface", &generateSurface},
        {"text", &generateText},
    };
    return table;
}

} // namespace oscadeval
