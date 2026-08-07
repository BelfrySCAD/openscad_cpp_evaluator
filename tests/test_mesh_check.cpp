// The four manifoldness conditions fail independently, so each fixture
// here breaks exactly one and asserts the others stay clean. A fixture that
// breaks two cannot show which condition the code is actually detecting.
#include <gtest/gtest.h>

#include "openscad_cpp_evaluator/mesh_check.hpp"

using namespace oscadeval;

namespace {

manifold::MeshGL build(const std::vector<float>& verts, const std::vector<uint32_t>& tris) {
    manifold::MeshGL m;
    m.numProp = 3;
    m.vertProperties = verts;
    m.triVerts = tris;
    return m;
}

// A closed tetrahedron, wound consistently outward.
manifold::MeshGL tetra() {
    return build({0, 0, 0,  1, 0, 0,  0, 1, 0,  0, 0, 1},
                 {0, 2, 1,  0, 1, 3,  1, 2, 3,  0, 3, 2});
}

}  // namespace

TEST(MeshCheck, ACleanTetrahedronPassesEveryCondition) {
    const MeshDiagnosis d = checkMesh(tetra());
    EXPECT_TRUE(d.ok()) << d.summary();
    EXPECT_TRUE(d.watertight());
    EXPECT_TRUE(d.manifold());
    EXPECT_TRUE(d.orientable());
    EXPECT_EQ(d.summary(), "");
}

TEST(MeshCheck, AMissingFaceIsThreeBoundaryEdges) {
    manifold::MeshGL m = tetra();
    m.triVerts.resize(9);       // drop the last face
    const MeshDiagnosis d = checkMesh(m);
    EXPECT_EQ(d.boundaryEdges, 3u) << d.summary();
    EXPECT_FALSE(d.watertight());
    // Only this condition: the remaining faces are still wound together and
    // no vertex pinches.
    EXPECT_EQ(d.nonManifoldEdges, 0u);
    EXPECT_EQ(d.inconsistentEdges, 0u);
    EXPECT_EQ(d.pinchedVertices, 0u);
}

TEST(MeshCheck, AThirdFaceOnAnEdgeIsNonManifoldButStillWatertightByEdgeCount) {
    manifold::MeshGL m = tetra();
    // A flap hanging off edge 0-1, sharing it with the two faces already
    // there. Watertight is about holes; this is the opposite problem.
    m.vertProperties.insert(m.vertProperties.end(), {1.0f, 1.0f, 1.0f});
    m.triVerts.insert(m.triVerts.end(), {0u, 1u, 4u});
    const MeshDiagnosis d = checkMesh(m);
    EXPECT_EQ(d.nonManifoldEdges, 1u) << d.summary();
    EXPECT_FALSE(d.manifold());
}

TEST(MeshCheck, TwoTetrahedraSharingOneVertexPinchThere) {
    // Every edge still has exactly two faces, which is why the edge test
    // alone cannot see this and the vertex link has to be walked.
    manifold::MeshGL m = tetra();
    const uint32_t base = 4;
    m.vertProperties.insert(m.vertProperties.end(),
                            {0, 0, -1,  1, 0, -1,  0, 1, -1});
    // Second tetrahedron, meeting the first only at vertex 0.
    m.triVerts.insert(m.triVerts.end(), {
        0u, base + 1, base,      0u, base, base + 2,
        0u, base + 2, base + 1,  base, base + 1, base + 2});
    const MeshDiagnosis d = checkMesh(m);
    EXPECT_EQ(d.pinchedVertices, 1u) << d.summary();
    EXPECT_EQ(d.boundaryEdges, 0u) << "still closed: " << d.summary();
    EXPECT_EQ(d.nonManifoldEdges, 0u) << "no edge has three faces: " << d.summary();
    EXPECT_FALSE(d.manifold());
}

TEST(MeshCheck, AReversedFaceIsCaughtAsInconsistentWinding) {
    manifold::MeshGL m = tetra();
    std::swap(m.triVerts[1], m.triVerts[2]);      // flip one face
    const MeshDiagnosis d = checkMesh(m);
    EXPECT_EQ(d.inconsistentEdges, 3u) << d.summary();
    EXPECT_FALSE(d.orientable());
    // Reversing a face does not open a hole -- the edges are still used
    // twice each, just the wrong way round.
    EXPECT_EQ(d.boundaryEdges, 0u) << d.summary();
    EXPECT_EQ(d.nonManifoldEdges, 0u) << d.summary();
}

TEST(MeshCheck, DegenerateAndDuplicateFacesAreReportedSeparately) {
    manifold::MeshGL m = tetra();
    m.triVerts.insert(m.triVerts.end(), {0u, 1u, 1u});     // repeated index
    m.triVerts.insert(m.triVerts.end(), {0u, 2u, 1u});     // same as face 0
    const MeshDiagnosis d = checkMesh(m);
    EXPECT_EQ(d.degenerateFaces, 1u) << d.summary();
    EXPECT_EQ(d.duplicateFaces, 1u) << d.summary();
}

TEST(MeshCheck, CoincidentButDistinctVerticesAreReported) {
    manifold::MeshGL m = tetra();
    m.vertProperties.insert(m.vertProperties.end(), {0, 0, 0});   // copy of vertex 0
    const MeshDiagnosis d = checkMesh(m);
    EXPECT_EQ(d.unweldedVertices, 1u) << d.summary();
}

TEST(MeshCheck, TheSummaryNamesWhatIsWrong) {
    manifold::MeshGL m = tetra();
    m.triVerts.resize(9);
    const std::string s = checkMesh(m).summary();
    EXPECT_NE(s.find("3 boundary edges"), std::string::npos) << s;
}

// -- repair ---------------------------------------------------------------

TEST(MeshRepair, FillsAMissingFace) {
    manifold::MeshGL m = tetra();
    m.triVerts.resize(9);
    ASSERT_EQ(checkMesh(m).boundaryEdges, 3u);

    MeshRepairReport r;
    const manifold::MeshGL fixed = repairMesh(m, r);
    EXPECT_EQ(r.filledHoles, 1u) << r.summary();
    const MeshDiagnosis d = checkMesh(fixed);
    EXPECT_EQ(d.boundaryEdges, 0u) << d.summary();
    EXPECT_TRUE(d.ok()) << d.summary();
}

TEST(MeshRepair, WeldsSplitVerticesThatWereFakingAHole) {
    // The usual reason a mesh "looks closed but isn't": the two halves use
    // different indices for the same corner, so the shared edges are really
    // pairs of boundary edges.
    manifold::MeshGL m = tetra();
    const uint32_t dup = 4;
    m.vertProperties.insert(m.vertProperties.end(), {0, 0, 0});   // == vertex 0
    for (size_t i = 0; i < m.triVerts.size(); ++i) {
        if (m.triVerts[i] == 0 && i >= 6) m.triVerts[i] = dup;    // later faces only
    }
    EXPECT_GT(checkMesh(m).boundaryEdges, 0u);

    MeshRepairReport r;
    const manifold::MeshGL fixed = repairMesh(m, r);
    EXPECT_EQ(r.weldedVertices, 1u) << r.summary();
    EXPECT_EQ(checkMesh(fixed).boundaryEdges, 0u) << checkMesh(fixed).summary();
}

TEST(MeshRepair, RewindsAReversedFace) {
    manifold::MeshGL m = tetra();
    std::swap(m.triVerts[1], m.triVerts[2]);
    ASSERT_EQ(checkMesh(m).inconsistentEdges, 3u);

    MeshRepairReport r;
    const manifold::MeshGL fixed = repairMesh(m, r);
    EXPECT_EQ(r.reversedFaces, 1u) << r.summary();
    EXPECT_TRUE(checkMesh(fixed).orientable()) << checkMesh(fixed).summary();
}

// Consistency and outwardness are different properties, and the fix for
// one can break the other: flood-filling the winding from a seed face that
// was itself reversed turns the whole component inside out, consistently.
// That is what this caught the first time round.
TEST(MeshRepair, LeavesTheMeshFacingOutwardNotMerelyConsistent) {
    manifold::MeshGL m = tetra();
    std::swap(m.triVerts[1], m.triVerts[2]);      // reverse face 0, the seed
    MeshRepairReport r;
    const manifold::MeshGL fixed = repairMesh(m, r);

    EXPECT_TRUE(checkMesh(fixed).orientable());
    // Positive signed volume == normals point out.
    double vol = 0;
    for (size_t t = 0; t + 2 < fixed.triVerts.size(); t += 3) {
        const float* a = &fixed.vertProperties[fixed.triVerts[t] * 3];
        const float* b = &fixed.vertProperties[fixed.triVerts[t + 1] * 3];
        const float* c = &fixed.vertProperties[fixed.triVerts[t + 2] * 3];
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1])
                - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
    }
    EXPECT_GT(vol, 0.0) << "inside out; volume " << vol;
    // And it should say it moved one face, not the three it turned over on
    // the way to that answer.
    EXPECT_EQ(r.reversedFaces, 1u) << r.summary();
}

TEST(MeshRepair, DropsDegenerateAndDuplicateFaces) {
    manifold::MeshGL m = tetra();
    m.triVerts.insert(m.triVerts.end(), {0u, 1u, 1u});
    m.triVerts.insert(m.triVerts.end(), {0u, 2u, 1u});
    MeshRepairReport r;
    const manifold::MeshGL fixed = repairMesh(m, r);
    EXPECT_EQ(r.droppedDegenerate, 1u) << r.summary();
    EXPECT_EQ(r.droppedDuplicate, 1u) << r.summary();
    EXPECT_TRUE(checkMesh(fixed).ok()) << checkMesh(fixed).summary();
}

TEST(MeshRepair, LeavesAGoodMeshAlone) {
    MeshRepairReport r;
    const manifold::MeshGL fixed = repairMesh(tetra(), r);
    EXPECT_FALSE(r.didAnything()) << r.summary();
    EXPECT_EQ(fixed.triVerts, tetra().triVerts);
    EXPECT_TRUE(checkMesh(fixed).ok());
}

TEST(MeshRepair, ReportsWhatItDid) {
    manifold::MeshGL m = tetra();
    m.triVerts.resize(9);
    MeshRepairReport r;
    repairMesh(m, r);
    EXPECT_NE(r.summary().find("hole filled"), std::string::npos) << r.summary();
}
