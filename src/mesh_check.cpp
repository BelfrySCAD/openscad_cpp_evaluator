#include "openscad_cpp_evaluator/mesh_check.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

namespace oscadeval {
namespace {

using Vert = uint32_t;
using Edge = std::pair<Vert, Vert>;

Edge undirected(Vert a, Vert b) { return {std::min(a, b), std::max(a, b)}; }

size_t triCount(const manifold::MeshGL& m) { return m.triVerts.size() / 3; }

void triVerts(const manifold::MeshGL& m, size_t t, Vert out[3]) {
    out[0] = m.triVerts[t * 3];
    out[1] = m.triVerts[t * 3 + 1];
    out[2] = m.triVerts[t * 3 + 2];
}

// Position of vertex v, whatever the stride happens to be.
void pos(const manifold::MeshGL& m, Vert v, double out[3]) {
    const size_t stride = m.numProp ? m.numProp : 3;
    for (int i = 0; i < 3; ++i) out[i] = m.vertProperties[v * stride + i];
}

bool degenerate(const manifold::MeshGL& m, size_t t) {
    Vert v[3];
    triVerts(m, t, v);
    if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) return true;
    double a[3], b[3], c[3];
    pos(m, v[0], a); pos(m, v[1], b); pos(m, v[2], c);
    const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
    const double wx = c[0] - a[0], wy = c[1] - a[1], wz = c[2] - a[2];
    const double cx = uy * wz - uz * wy, cy = uz * wx - ux * wz, cz = ux * wy - uy * wx;
    // Twice the area. Squared, to avoid a sqrt on every face of a big mesh.
    return (cx * cx + cy * cy + cz * cz) < 1e-24;
}

// Vertices that share a position but not an index. Keyed on rounded
// coordinates, matching the welding polyhedron() already does.
size_t countUnwelded(const manifold::MeshGL& m) {
    const size_t stride = m.numProp ? m.numProp : 3;
    const size_t n = stride ? m.vertProperties.size() / stride : 0;
    std::map<std::tuple<long long, long long, long long>, size_t> seen;
    size_t extra = 0;
    for (size_t v = 0; v < n; ++v) {
        double p[3];
        pos(m, static_cast<Vert>(v), p);
        auto key = std::make_tuple(llround(p[0] * 1e6), llround(p[1] * 1e6), llround(p[2] * 1e6));
        if (++seen[key] > 1) ++extra;
    }
    return extra;
}

// The faces around `v`, as (prev, next) pairs on the opposite edge. The link
// is one cycle when following those pairs from any starting corner visits
// all of them; more than one walk means the surface pinches at v.
size_t countPinched(const manifold::MeshGL& m,
                    const std::vector<std::vector<size_t>>& vertTris) {
    size_t pinched = 0;
    for (Vert v = 0; v < vertTris.size(); ++v) {
        const auto& tris = vertTris[v];
        if (tris.size() < 2) continue;
        // next[a] = b for the corner opposite v in each face, undirected so
        // a reversed neighbour still links up -- winding is checked
        // separately and should not show up as a pinch too.
        std::unordered_map<Vert, std::vector<Vert>> link;
        for (size_t t : tris) {
            Vert w[3];
            triVerts(m, t, w);
            Vert a = 0, b = 0;
            int found = 0;
            for (int i = 0; i < 3; ++i) {
                if (w[i] == v) continue;
                (found++ == 0 ? a : b) = w[i];
            }
            if (found < 2) continue;      // degenerate; counted elsewhere
            link[a].push_back(b);
            link[b].push_back(a);
        }
        if (link.empty()) continue;
        std::set<Vert> visited;
        std::vector<Vert> stack{link.begin()->first};
        while (!stack.empty()) {
            Vert cur = stack.back();
            stack.pop_back();
            if (!visited.insert(cur).second) continue;
            for (Vert nxt : link[cur]) {
                if (!visited.count(nxt)) stack.push_back(nxt);
            }
        }
        if (visited.size() < link.size()) ++pinched;
    }
    return pinched;
}

}  // namespace

std::string MeshDiagnosis::summary() const {
    std::vector<std::string> parts;
    auto add = [&](size_t n, const char* one, const char* many) {
        if (n) parts.push_back(std::to_string(n) + " " + (n == 1 ? one : many));
    };
    add(boundaryEdges, "boundary edge", "boundary edges");
    add(nonManifoldEdges, "non-manifold edge", "non-manifold edges");
    add(pinchedVertices, "pinched vertex", "pinched vertices");
    add(inconsistentEdges, "inconsistently wound edge", "inconsistently wound edges");
    add(degenerateFaces, "degenerate face", "degenerate faces");
    add(duplicateFaces, "duplicate face", "duplicate faces");
    add(unweldedVertices, "unwelded vertex", "unwelded vertices");
    std::string out;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i) out += ", ";
        out += parts[i];
    }
    return out;
}

MeshDiagnosis checkMesh(const manifold::MeshGL& mesh) {
    MeshDiagnosis d;
    const size_t tris = triCount(mesh);
    const size_t stride = mesh.numProp ? mesh.numProp : 3;
    const size_t nVerts = stride ? mesh.vertProperties.size() / stride : 0;

    // Directed uses per undirected edge. Directions are what distinguish a
    // consistently wound pair (a->b and b->a) from two faces wound the same
    // way (a->b twice), which an undirected count cannot see.
    std::map<Edge, std::pair<int, int>> uses;   // {forward, backward}
    std::vector<std::vector<size_t>> vertTris(nVerts);
    std::set<std::array<Vert, 3>> seenFaces;

    for (size_t t = 0; t < tris; ++t) {
        if (degenerate(mesh, t)) {
            ++d.degenerateFaces;
            continue;               // its edges are meaningless
        }
        Vert v[3];
        triVerts(mesh, t, v);

        std::array<Vert, 3> sorted{v[0], v[1], v[2]};
        std::sort(sorted.begin(), sorted.end());
        if (!seenFaces.insert(sorted).second) ++d.duplicateFaces;

        for (int i = 0; i < 3; ++i) {
            if (v[i] < nVerts) vertTris[v[i]].push_back(t);
            const Vert a = v[i], b = v[(i + 1) % 3];
            auto& u = uses[undirected(a, b)];
            (a < b ? u.first : u.second)++;
        }
    }

    for (const auto& [edge, u] : uses) {
        const int total = u.first + u.second;
        if (total == 1) {
            ++d.boundaryEdges;
        } else if (total > 2) {
            ++d.nonManifoldEdges;
        } else if (u.first != 1 || u.second != 1) {
            // Two faces, but both traverse the edge the same way round.
            ++d.inconsistentEdges;
        }
    }

    d.pinchedVertices = countPinched(mesh, vertTris);
    d.unweldedVertices = countUnwelded(mesh);
    return d;
}

std::string MeshRepairReport::summary() const {
    std::vector<std::string> parts;
    auto add = [&](size_t n, const std::string& text) {
        if (n) parts.push_back(std::to_string(n) + " " + text);
    };
    add(weldedVertices, weldedVertices == 1 ? "vertex welded" : "vertices welded");
    add(droppedDegenerate, droppedDegenerate == 1 ? "degenerate face dropped"
                                                  : "degenerate faces dropped");
    add(droppedDuplicate, droppedDuplicate == 1 ? "duplicate face dropped"
                                                : "duplicate faces dropped");
    add(reversedFaces, reversedFaces == 1 ? "face re-wound" : "faces re-wound");
    add(splitVertices, splitVertices == 1 ? "pinched vertex split"
                                          : "pinched vertices split");
    if (filledHoles) {
        parts.push_back(std::to_string(filledHoles)
                        + (filledHoles == 1 ? " hole filled (" : " holes filled (")
                        + std::to_string(filledTriangles) + " triangles)");
    }
    add(unfilledHoles, unfilledHoles == 1 ? "hole left open" : "holes left open");
    std::string out;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i) out += ", ";
        out += parts[i];
    }
    return out;
}

namespace {

// Merge vertices that share a position. Everything downstream depends on
// this: unwelded duplicates split one shared edge into two boundary edges,
// so a mesh that is really closed reads as full of holes until it is done.
std::vector<Vert> weldMap(const manifold::MeshGL& m, size_t& welded) {
    const size_t stride = m.numProp ? m.numProp : 3;
    const size_t n = stride ? m.vertProperties.size() / stride : 0;
    std::map<std::tuple<long long, long long, long long>, Vert> first;
    std::vector<Vert> remap(n);
    welded = 0;
    for (size_t v = 0; v < n; ++v) {
        double p[3];
        pos(m, static_cast<Vert>(v), p);
        auto key = std::make_tuple(llround(p[0] * 1e6), llround(p[1] * 1e6), llround(p[2] * 1e6));
        auto [it, fresh] = first.emplace(key, static_cast<Vert>(v));
        remap[v] = it->second;
        if (!fresh) ++welded;
    }
    return remap;
}

// Boundary loops, as ordered vertex rings. Each boundary edge belongs to
// exactly one, so following them from any start returns to it.
std::vector<std::vector<Vert>> boundaryLoops(const std::vector<std::array<Vert, 3>>& tris) {
    std::map<Edge, int> count;
    std::multimap<Vert, Vert> next;      // directed, along the boundary
    for (const auto& t : tris) {
        for (int i = 0; i < 3; ++i) ++count[undirected(t[i], t[(i + 1) % 3])];
    }
    for (const auto& t : tris) {
        for (int i = 0; i < 3; ++i) {
            const Vert a = t[i], b = t[(i + 1) % 3];
            // A boundary edge is walked backwards to close the hole: the
            // filling faces must wind against the face that owns the edge.
            if (count[undirected(a, b)] == 1) next.emplace(b, a);
        }
    }
    std::vector<std::vector<Vert>> loops;
    while (!next.empty()) {
        std::vector<Vert> loop;
        Vert start = next.begin()->first;
        Vert cur = start;
        while (true) {
            auto it = next.find(cur);
            if (it == next.end()) break;      // dangling: not a closed ring
            Vert nxt = it->second;
            next.erase(it);
            loop.push_back(cur);
            cur = nxt;
            if (cur == start) break;
            if (loop.size() > 100000) break;  // runaway guard
        }
        if (loop.size() >= 3) loops.push_back(std::move(loop));
    }
    return loops;
}

}  // namespace

manifold::MeshGL repairMesh(const manifold::MeshGL& mesh, MeshRepairReport& report) {
    report = MeshRepairReport{};
    manifold::MeshGL out = mesh;

    // 1. Weld. Must come first -- it decides which edges are shared, and
    //    every later step reads the edge map.
    size_t welded = 0;
    const std::vector<Vert> remap = weldMap(mesh, welded);
    report.weldedVertices = welded;

    std::vector<std::array<Vert, 3>> tris;
    std::set<std::array<Vert, 3>> seen;
    for (size_t t = 0; t < triCount(mesh); ++t) {
        Vert v[3];
        triVerts(mesh, t, v);
        std::array<Vert, 3> f{remap[v[0]], remap[v[1]], remap[v[2]]};
        // 2. Degenerate faces, now that welding has collapsed slivers into
        //    repeated indices.
        if (f[0] == f[1] || f[1] == f[2] || f[0] == f[2]) {
            ++report.droppedDegenerate;
            continue;
        }
        std::array<Vert, 3> sorted = f;
        std::sort(sorted.begin(), sorted.end());
        if (!seen.insert(sorted).second) {
            ++report.droppedDuplicate;
            continue;
        }
        tris.push_back(f);
    }

    std::vector<size_t> component;
    size_t nComponents = 0;

    // 3. Orientation. Flood-fill from one face per connected component,
    //    reversing any neighbour that shares an edge in the same direction.
    //    Before filling, so the new faces are wound against a settled
    //    surface rather than a mixed one.
    {
        std::map<Edge, std::vector<size_t>> edgeTris;
        for (size_t i = 0; i < tris.size(); ++i) {
            for (int e = 0; e < 3; ++e) {
                edgeTris[undirected(tris[i][e], tris[i][(e + 1) % 3])].push_back(i);
            }
        }
        component.assign(tris.size(), SIZE_MAX);
        std::vector<char> done(tris.size(), 0);
        for (size_t seed = 0; seed < tris.size(); ++seed) {
            if (done[seed]) continue;
            const size_t comp = nComponents++;
            std::vector<size_t> stack{seed};
            done[seed] = 1;
            component[seed] = comp;
            while (!stack.empty()) {
                const size_t i = stack.back();
                stack.pop_back();
                for (int e = 0; e < 3; ++e) {
                    const Vert a = tris[i][e], b = tris[i][(e + 1) % 3];
                    for (size_t j : edgeTris[undirected(a, b)]) {
                        if (j == i || done[j]) continue;
                        bool sameDir = false;
                        for (int f = 0; f < 3; ++f) {
                            if (tris[j][f] == a && tris[j][(f + 1) % 3] == b) sameDir = true;
                        }
                        if (sameDir) {          // agrees with us == wound wrong
                            std::swap(tris[j][1], tris[j][2]);
                        }
                        done[j] = 1;
                        component[j] = component[i];
                        stack.push_back(j);
                    }
                }
            }
        }
    }

    // 4. Fill holes. Ear clipping would respect concavity better, but a fan
    //    from the first vertex closes the ring, which is what watertightness
    //    needs; a hole in a printable part is nearly always small.
    for (const auto& loop : boundaryLoops(tris)) {
        if (loop.size() < 3) {
            ++report.unfilledHoles;
            continue;
        }
        for (size_t i = 1; i + 1 < loop.size(); ++i) {
            tris.push_back({loop[0], loop[i], loop[i + 1]});
            ++report.filledTriangles;
        }
        ++report.filledHoles;
    }

    // 5. Outward. Making the winding *consistent* says nothing about which
    //    side is outside -- seeding the fill on a face that was itself
    //    reversed turns the whole component inside out. Signed volume says
    //    which way it went, and is only meaningful now that the holes are
    //    closed, so this has to come after step 4.
    if (nComponents) {
        component.resize(tris.size(), 0);      // filled faces inherit component 0
        std::vector<double> volume(nComponents, 0.0);
        for (size_t i = 0; i < tris.size(); ++i) {
            const size_t c = component[i] == SIZE_MAX ? 0 : component[i];
            double a[3], b[3], cpt[3];
            pos(mesh, tris[i][0], a); pos(mesh, tris[i][1], b); pos(mesh, tris[i][2], cpt);
            volume[c] += (a[0] * (b[1] * cpt[2] - b[2] * cpt[1])
                          - a[1] * (b[0] * cpt[2] - b[2] * cpt[0])
                          + a[2] * (b[0] * cpt[1] - b[1] * cpt[0])) / 6.0;
        }
        for (size_t i = 0; i < tris.size(); ++i) {
            const size_t c = component[i] == SIZE_MAX ? 0 : component[i];
            if (volume[c] < 0) std::swap(tris[i][1], tris[i][2]);
        }
    }

    // Count what actually changed rather than every intermediate flip: a
    // component that was turned inside out and then back reports nothing,
    // which is what the user sees.
    {
        std::set<std::array<Vert, 3>> before;
        for (size_t t = 0; t < triCount(mesh); ++t) {
            Vert v[3];
            triVerts(mesh, t, v);
            before.insert({remap[v[0]], remap[v[1]], remap[v[2]]});
        }
        auto rotated = [](std::array<Vert, 3> f) {
            // Winding is defined up to rotation, so compare canonically.
            if (f[1] < f[0] && f[1] <= f[2]) return std::array<Vert, 3>{f[1], f[2], f[0]};
            if (f[2] < f[0] && f[2] <= f[1]) return std::array<Vert, 3>{f[2], f[0], f[1]};
            return f;
        };
        std::set<std::array<Vert, 3>> canon;
        for (const auto& f : before) canon.insert(rotated(f));
        for (const auto& f : tris) {
            if (!canon.count(rotated(f)) && canon.count(rotated({f[0], f[2], f[1]}))) {
                ++report.reversedFaces;
            }
        }
    }

    out.triVerts.clear();
    out.triVerts.reserve(tris.size() * 3);
    for (const auto& t : tris) {
        out.triVerts.push_back(t[0]);
        out.triVerts.push_back(t[1]);
        out.triVerts.push_back(t[2]);
    }
    // runIndex/faceID describe the old triangle list; a stale one is worse
    // than none, and Manifold rebuilds what it needs.
    out.runIndex.clear();
    out.runOriginalID.clear();
    out.faceID.clear();
    out.runTransform.clear();
    out.mergeFromVert.clear();
    out.mergeToVert.clear();
    return out;
}

}  // namespace oscadeval
