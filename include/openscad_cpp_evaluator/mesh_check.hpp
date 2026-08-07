#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <manifold/manifold.h>

namespace oscadeval {

// What is wrong with a triangle mesh, as four independent conditions. They
// are separate because they fail separately and a mesh can satisfy any
// subset: two boxes fused along a face are watertight but have edges with
// four faces; two cones joined tip-to-tip have every edge shared by exactly
// two faces and still pinch at the apex.
struct MeshDiagnosis {
    // Edges used by exactly one face: a hole.
    size_t boundaryEdges = 0;
    // Edges used by three or more faces: surfaces meeting along a seam.
    size_t nonManifoldEdges = 0;
    // Vertices whose incident faces form more than one fan -- the surface
    // pinches to a point there. Passes the edge test, so it needs its own.
    size_t pinchedVertices = 0;
    // Edges whose two faces traverse them the same way round rather than in
    // opposite directions: the winding disagrees across that edge.
    size_t inconsistentEdges = 0;
    // Zero-area triangles, and faces naming the same vertex twice.
    size_t degenerateFaces = 0;
    // Faces with the same three vertices as another face.
    size_t duplicateFaces = 0;
    // Distinct indices at the same position. These split what should be one
    // edge into two boundary edges, which is the usual reason a mesh that
    // looks closed is not.
    size_t unweldedVertices = 0;

    bool watertight() const { return boundaryEdges == 0; }
    bool manifold() const {
        return boundaryEdges == 0 && nonManifoldEdges == 0 && pinchedVertices == 0
               && degenerateFaces == 0;
    }
    bool orientable() const { return inconsistentEdges == 0; }
    bool ok() const { return manifold() && orientable(); }

    // "4 boundary edges, 1 pinched vertex" -- empty when nothing is wrong.
    std::string summary() const;
};

// Diagnose `mesh` against all four conditions. Read-only; nothing is
// modified and no exception is thrown for a broken mesh.
MeshDiagnosis checkMesh(const manifold::MeshGL& mesh);

// What repairMesh did, for reporting. Not a diagnosis: these are actions.
struct MeshRepairReport {
    size_t weldedVertices = 0;
    size_t droppedDegenerate = 0;
    size_t droppedDuplicate = 0;
    size_t reversedFaces = 0;
    size_t filledHoles = 0;
    size_t filledTriangles = 0;
    size_t splitVertices = 0;
    size_t unfilledHoles = 0;      // boundary loops it could not close

    bool didAnything() const {
        return weldedVertices || droppedDegenerate || droppedDuplicate || reversedFaces
               || filledHoles || splitVertices;
    }
    // "welded 12 vertices, filled 1 hole (4 triangles)" -- empty if nothing.
    std::string summary() const;
};

// Best-effort repair, in the only order that works: welding first (it
// changes which edges are shared, so every later test depends on it), then
// degenerate and duplicate removal, then orientation, then hole filling
// last -- filling before orienting would add faces wound against their
// neighbours.
//
// Returns the repaired mesh. A mesh it cannot close is still returned,
// improved as far as it got, with the remainder reported in `report`.
manifold::MeshGL repairMesh(const manifold::MeshGL& mesh, MeshRepairReport& report);

}  // namespace oscadeval
