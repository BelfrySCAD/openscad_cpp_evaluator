// `render()` in EXPRESSION position: builds geometry, measures it, and
// throws the geometry away.
//
//     obj = render() { difference() { cube(100); sphere(20); } };
//     echo(obj.volume, obj.genus);
//     polyhedron(obj);
//
// This is the ONLY implementation of "resolved CSG subtree -> object()".
// Both engines call measureCsgSubtree: the interpreter from evalRenderExpr
// (expr_eval.cpp), the VM from Op::PopBuiltinWrap's Kind::Measure branch.
// Keeping it in one place is what stops the two engines drifting.
//
// Nothing here draws. The caller has already popped the subtree off
// treeStack_, so it never reaches the drawn tree, and Evaluator::measuring_
// is set for the whole generate below -- see its doc comment for the four
// provenance writes that suppresses and why the geometry cache stays on.

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "builtins/builtins.hpp"

#include <manifold/manifold.h>

#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace oscadeval {

namespace {

Value listOf(std::vector<Value> items) {
    return Value{std::make_shared<const ValueList>(ValueList{std::move(items)})};
}

Value pointOf(double x, double y, double z) { return listOf({Value{x}, Value{y}, Value{z}}); }
Value pointOf(double x, double y) { return listOf({Value{x}, Value{y}}); }

Value objectOfPairs(std::vector<std::pair<std::string, Value>> items) {
    return Value{std::make_shared<const ValueObject>(ValueObject{std::move(items)})};
}

// Manifold mesh -> VNF halves.
//
// Two things here are load-bearing and both are silent when wrong:
//
// 1. WINDING IS REVERSED. Manifold's triVerts is counter-clockwise seen
//    from outside (mesh.h); VNF and polyhedron() are clockwise
//    (BOSL2/vnf.scad). This is the exact inverse of resolvePolyhedron's
//    intake, which reverses on the way in (primitives_3d.cpp). Getting it
//    wrong does NOT fail a round-trip -- Manifold happily rebuilds a
//    reversed-but-closed mesh -- it just yields inside-out normals that
//    only a slicer or BOSL2's vnf_validate notices.
//
// 2. THE VERTEX WELD IS NOT COSMETIC. Manifold splits property-vertices, so
//    the raw vertex list has duplicates at every seam. Without deduping by
//    exact position the round-tripped polyhedron() is an OPEN mesh and
//    lands in isDrawableFailure. Same approach as import.cpp's meshToVnf.
//
// Not reusing meshToVnf itself: it is file-local to import.cpp, takes a
// LoadedMesh rather than a Manifold, has no winding-reversal option, and
// returns the wrapped [[verts],[faces]] 2-list when what is needed here is
// the two halves as separate object keys. Adapting it would cost more than
// this does.
// ponytail: emits triangles, not merged N-gons. VNF and polyhedron() both
// accept triangles. MeshGL64::faceID would allow merging coplanar triangles
// back into quads, but that needs boundary-loop extraction -- do it only if
// someone actually wants prettier output.
template <typename MeshT>
void meshToVertsAndFaces(const MeshT& mesh, Value& vertsOut, Value& facesOut) {
    std::map<std::array<double, 3>, int> vertMap;
    std::vector<Value> verts;
    std::vector<Value> faces;

    const size_t numProp = mesh.numProp == 0 ? 3 : static_cast<size_t>(mesh.numProp);
    const size_t numVert = mesh.vertProperties.size() / (numProp == 0 ? 1 : numProp);

    const auto indexOf = [&](uint32_t v) -> int {
        if (static_cast<size_t>(v) >= numVert) return -1;
        const size_t base = static_cast<size_t>(v) * numProp;
        const std::array<double, 3> pos = {static_cast<double>(mesh.vertProperties[base + 0]),
                                           static_cast<double>(mesh.vertProperties[base + 1]),
                                           static_cast<double>(mesh.vertProperties[base + 2])};
        auto it = vertMap.find(pos);
        if (it != vertMap.end()) return it->second;
        const int idx = static_cast<int>(verts.size());
        vertMap.emplace(pos, idx);
        verts.push_back(pointOf(pos[0], pos[1], pos[2]));
        return idx;
    };

    for (size_t t = 0; t + 2 < mesh.triVerts.size(); t += 3) {
        const int a = indexOf(mesh.triVerts[t + 0]);
        const int b = indexOf(mesh.triVerts[t + 1]);
        const int c = indexOf(mesh.triVerts[t + 2]);
        if (a < 0 || b < 0 || c < 0) continue;
        // a, c, b -- see (1) above.
        faces.push_back(listOf({Value{static_cast<double>(a)}, Value{static_cast<double>(c)},
                                Value{static_cast<double>(b)}}));
    }

    vertsOut = listOf(std::move(verts));
    facesOut = listOf(std::move(faces));
}

// 2D: CrossSection -> flat `vertices` plus an index `paths` list, matching
// polygon(points=, paths=)'s own argument shape. Deliberately not
// import.cpp's contoursToValue, which emits nested point lists with no
// index split -- the wrong shape for a `paths` key.
void sectionToVertsAndPaths(const manifold::CrossSection& cs, Value& vertsOut, Value& pathsOut, double& perimeter) {
    std::vector<Value> verts;
    std::vector<Value> paths;
    perimeter = 0.0;
    for (const auto& poly : cs.ToPolygons()) {
        std::vector<Value> path;
        path.reserve(poly.size());
        for (size_t i = 0; i < poly.size(); ++i) {
            path.push_back(Value{static_cast<double>(verts.size())});
            verts.push_back(pointOf(poly[i].x, poly[i].y));
            // Closed loop: the last vertex's edge runs back to the first.
            const auto& next = poly[(i + 1) % poly.size()];
            perimeter += std::hypot(next.x - poly[i].x, next.y - poly[i].y);
        }
        paths.push_back(listOf(std::move(path)));
    }
    vertsOut = listOf(std::move(verts));
    pathsOut = listOf(std::move(paths));
}

Value boxValue3d(const manifold::Box& b) {
    return listOf({pointOf(b.min.x, b.min.y, b.min.z), pointOf(b.max.x, b.max.y, b.max.z)});
}

// dim = 0: no geometry at all. Every 3D key is still present so the object's
// shape is stable and `obj.volume` never errors -- but boundingbox is undef,
// NOT Manifold's empty Box, which is {+inf, -inf} and would poison any
// arithmetic a script does with it.
Value emptyMeasurement() {
    return objectOfPairs({
        {"vertices", listOf({})},
        {"faces", listOf({})},
        {"volume", Value{0.0}},
        {"area", Value{0.0}},
        {"genus", Value{0.0}},
        {"boundingbox", Value{}},
        {"dim", Value{0.0}},
        {"vnf", listOf({listOf({}), listOf({})})},
    });
}

} // namespace

// The interpreter half. Resolves the children into their OWN treeStack_
// frame, generates that frame, measures it, and discards it -- the VM's
// Kind::Measure bracket does the same three steps as two opcodes.
Value Evaluator::evalRenderExpr(const oscad::RenderExpression& node, EvalContext& ctx) {
    // evalExpr is also reachable outside a resolve pass (the debug REPL),
    // where there is no frame to push onto and no geometry to speak of.
    if (!inResolvePass_ || treeStack_.empty()) {
        warn("render(): geometry expression is only valid during evaluation", &node.position());
        return Value{};
    }

    // Resolve arguments purely for the $-propagation side effect -- exactly
    // what resolveRender does for the statement form. `convexity` is
    // accepted and ignored; `$fn` and friends reach the children via effCtx.
    auto [args, effCtx] = resolveCallArgs(*this, node.arguments, ctx);
    (void)args;

    // Save/restore rather than set/clear: nesting (a render() inside a
    // render()'s children) then works for free rather than needing an error.
    const bool savedMeasuring = measuring_;
    measuring_ = true;

    treeStack_.emplace_back();
    std::vector<std::unique_ptr<CSGNode>> sub;
    try {
        evalChildren(node.children, effCtx);
        sub = std::move(treeStack_.back());
        treeStack_.pop_back();
    } catch (...) {
        // buildTreeNode's own shape: the frame must come off on the throw
        // path too, or every later resolve accumulates into a dead level.
        treeStack_.pop_back();
        measuring_ = savedMeasuring;
        throw;
    }

    Value result;
    try {
        // measuring_ must still be TRUE here -- it is what suppresses the
        // provenance writes inside the generate.
        result = measureCsgSubtree(std::move(sub), node);
    } catch (...) {
        measuring_ = savedMeasuring;
        throw;
    }
    measuring_ = savedMeasuring;
    return result;
}

Value Evaluator::measureCsgSubtree(std::vector<std::unique_ptr<CSGNode>> sub, const oscad::ASTNode& node) {
    // Callers must already have measuring_ set -- the generate below writes
    // provenance otherwise. Asserting rather than setting it here because
    // the VM's bracket owns the flag's lifetime across two op handlers.
    std::vector<CSGNode*> ptrs;
    ptrs.reserve(sub.size());
    for (const std::unique_ptr<CSGNode>& n : sub) ptrs.push_back(n.get());

    generateTreeImpl(ptrs);
    // Same two lines generateTree() runs over the real top level: a render()
    // block is an implicit union, so it gets the same Group dimension
    // treatment -- which is also what warns about (and drops) mixed 2D/3D.
    // 0 is DimRule::Group; the enum itself is file-local to csg_generate.cpp,
    // which is why applyDimensionRulesTo takes an int (see its declaration).
    applyDimensionRulesTo(ptrs, /*DimRule::Group=*/0);

    std::vector<ColoredBody> bodies;
    for (CSGNode* n : ptrs) {
        for (const ColoredBody& b : n->bodies) bodies.push_back(b);
    }

    const RoleSplit split = splitByRole(bodies);

    // An open surface (polyhedron() with boundary edges) has no Manifold to
    // measure, but the script should still get its mesh back. The producing
    // builtin has already warned with the boundary-edge count, so don't
    // repeat that detail here.
    if (split.foreground.empty() && !split.displayOnly.empty()) {
        const ColoredBody& cb = split.displayOnly.front();
        Value verts, faces;
        meshToVertsAndFaces(*cb.rawMesh, verts, faces);
        warn("render(): result is not a closed solid; volume and genus are unavailable", &node.position());
        // Bounding box by hand: there is no Manifold to ask.
        double lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
        bool any = false;
        const size_t numProp = cb.rawMesh->numProp == 0 ? 3 : static_cast<size_t>(cb.rawMesh->numProp);
        for (size_t i = 0; i + numProp <= cb.rawMesh->vertProperties.size(); i += numProp) {
            for (int k = 0; k < 3; ++k) {
                const double v = static_cast<double>(cb.rawMesh->vertProperties[i + static_cast<size_t>(k)]);
                if (!any || v < lo[k]) lo[k] = v;
                if (!any || v > hi[k]) hi[k] = v;
            }
            any = true;
        }
        return objectOfPairs({
            {"vertices", verts},
            {"faces", faces},
            {"volume", Value{0.0}},
            {"area", Value{0.0}},
            {"genus", Value{}},
            {"boundingbox", any ? listOf({pointOf(lo[0], lo[1], lo[2]), pointOf(hi[0], hi[1], hi[2])}) : Value{}},
            {"dim", Value{3.0}},
            {"vnf", listOf({verts, faces})},
        });
    }

    if (split.foreground.empty()) return emptyMeasurement();

    // Drop operands Manifold considers broken before any union: the same
    // rule generateCsg enforces -- a non-NoError body must never reach
    // operator+, or it silently zeroes the whole result.
    std::vector<ColoredBody> usable;
    manifold::Manifold::Error firstError = manifold::Manifold::Error::NoError;
    for (const ColoredBody& b : split.foreground) {
        if (b.body && b.body->Status() != manifold::Manifold::Error::NoError) {
            if (firstError == manifold::Manifold::Error::NoError) firstError = b.body->Status();
            continue;
        }
        usable.push_back(b);
    }
    if (usable.empty()) {
        if (firstError != manifold::Manifold::Error::NoError) {
            warn("render(): " + manifoldErrorName(firstError) + "; nothing to measure", &node.position());
        }
        return emptyMeasurement();
    }

    const ColoredBody merged = combineBodies(usable);

    if (merged.section) {
        Value verts, paths;
        double perimeter = 0.0;
        sectionToVertsAndPaths(*merged.section, verts, paths, perimeter);
        const manifold::Rect r = merged.section->Bounds();
        const bool empty = merged.section->IsEmpty();
        return objectOfPairs({
            {"vertices", verts},
            {"paths", paths},
            {"area", Value{merged.section->Area()}},
            {"perimeter", Value{perimeter}},
            {"boundingbox", empty ? Value{} : listOf({pointOf(r.min.x, r.min.y), pointOf(r.max.x, r.max.y)})},
            {"dim", Value{2.0}},
        });
    }

    if (!merged.body || merged.body->IsEmpty()) return emptyMeasurement();

    // GetMeshGL64, not GetMeshGL: MeshGL is MeshGLP<float>, so a 100mm cube's
    // vertices would come back float-rounded rather than exact.
    Value verts, faces;
    meshToVertsAndFaces(merged.body->GetMeshGL64(), verts, faces);

    return objectOfPairs({
        {"vertices", verts},
        {"faces", faces},
        {"volume", Value{merged.body->Volume()}},
        {"area", Value{merged.body->SurfaceArea()}},
        {"genus", Value{static_cast<double>(merged.body->Genus())}},
        {"boundingbox", boxValue3d(merged.body->BoundingBox())},
        {"dim", Value{3.0}},
        // Every BOSL2 function takes the 2-list, not two arguments. Shares
        // the same two children, so it costs one ValueList.
        {"vnf", listOf({verts, faces})},
    });
}

} // namespace oscadeval
