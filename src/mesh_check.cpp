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

// A face naming the same vertex twice. Its "edges" include a self-edge, so
// it cannot take part in the topology at all.
bool repeatsAVertex(const manifold::MeshGL& m, size_t t) {
    Vert v[3];
    triVerts(m, t, v);
    return v[0] == v[1] || v[1] == v[2] || v[0] == v[2];
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
        if (degenerate(mesh, t)) ++d.degenerateFaces;
        // Counted, but NOT removed from the topology. A zero-area triangle
        // is still a face with three edges, and dropping it leaves those
        // edges with one face each -- which then reads as a hole that is
        // not there. A level-4 Menger sponge is manifold with every face
        // present and reported 722 boundary edges without them.
        //
        // A face naming the same vertex twice is different: its self-edge
        // cannot be matched by anything, so it genuinely has to go.
        if (repeatsAVertex(mesh, t)) continue;
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


namespace {

// Which two corners of a zero-area face sit at the same position, if any.
// Such a face is a needle rather than a T-joint: its two long edges run
// between the same two points, so once it is gone the faces on either side
// already meet along one edge and nothing needs splitting. The coincident
// pair is merged so that is true by index as well as by position.
bool coincidentPair(const manifold::MeshGL& m, const std::array<Vert, 3>& f,
                    Vert& keep, Vert& drop) {
    for (int i = 0; i < 3; ++i) {
        const Vert a = f[i], b = f[(i + 1) % 3];
        if (a == b) { keep = a; drop = b; return true; }
        double p[3], q[3];
        pos(m, a, p); pos(m, b, q);
        if (llround(p[0] * 1e6) == llround(q[0] * 1e6)
            && llround(p[1] * 1e6) == llround(q[1] * 1e6)
            && llround(p[2] * 1e6) == llround(q[2] * 1e6)) {
            keep = std::min(a, b);
            drop = std::max(a, b);
            return keep != drop;
        }
    }
    return false;
}

// Index of the vertex opposite the longest edge -- for three collinear
// points that is the one in the middle.
int middleOfCollinear(const manifold::MeshGL& m, const std::array<Vert, 3>& f) {
    double p[3][3];
    for (int i = 0; i < 3; ++i) pos(m, f[i], p[i]);
    auto d2 = [&](int a, int b) {
        double s = 0;
        for (int k = 0; k < 3; ++k) { const double d = p[a][k] - p[b][k]; s += d * d; }
        return s;
    };
    const double opp[3] = {d2(1, 2), d2(0, 2), d2(0, 1)};
    int best = 0;
    for (int i = 1; i < 3; ++i) if (opp[i] > opp[best]) best = i;
    return best;
}

}  // namespace

manifold::MeshGL stripSlivers(const manifold::MeshGL& mesh, SliverStripReport& report) {
    report = SliverStripReport{};
    std::vector<std::array<Vert, 3>> tris;
    tris.reserve(triCount(mesh));
    for (size_t t = 0; t < triCount(mesh); ++t) {
        Vert v[3];
        triVerts(mesh, t, v);
        tris.push_back({v[0], v[1], v[2]});
    }

    // Removing a sliver can leave its neighbour split into pieces that are
    // themselves slivers, so this repeats. Bounded because each pass must
    // remove at least one face to continue.
    const int kMaxPasses = 12;
    for (int pass = 0; pass < kMaxPasses; ++pass) {
        std::vector<size_t> slivers;
        for (size_t i = 0; i < tris.size(); ++i) {
            const auto& f = tris[i];
            if (f[0] == f[1] || f[1] == f[2] || f[0] == f[2]) continue;  // handled elsewhere
            double a[3], b[3], c[3];
            pos(mesh, f[0], a); pos(mesh, f[1], b); pos(mesh, f[2], c);
            const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
            const double wx = c[0] - a[0], wy = c[1] - a[1], wz = c[2] - a[2];
            const double cx = uy * wz - uz * wy, cy = uz * wx - ux * wz, cz = ux * wy - uy * wx;
            if ((cx * cx + cy * cy + cz * cz) < 1e-24) slivers.push_back(i);
        }
        if (slivers.empty()) break;
        ++report.passes;

        // Which face owns each edge, so a sliver's long-edge neighbour can
        // be found. Rebuilt per pass: splitting changes it.
        std::map<Edge, std::vector<size_t>> owners;
        for (size_t i = 0; i < tris.size(); ++i) {
            for (int e = 0; e < 3; ++e) {
                owners[undirected(tris[i][e], tris[i][(e + 1) % 3])].push_back(i);
            }
        }

        std::vector<char> isSliver(tris.size(), 0);
        for (size_t i : slivers) isSliver[i] = 1;

        std::vector<char> dead(tris.size(), 0);
        std::vector<std::array<Vert, 3>> added;
        std::map<Vert, Vert> merge;      // needle corners to fold together
        for (size_t si : slivers) {
            if (dead[si]) continue;
            const auto f = tris[si];

            // A needle: two corners at one point. Nothing to restitch --
            // the faces on either side already share an edge positionally,
            // and merging the pair makes them share it by index too.
            Vert keep = 0, drop = 0;
            if (coincidentPair(mesh, f, keep, drop)) {
                dead[si] = 1;
                if (keep != drop) merge[drop] = keep;
                ++report.removed;
                ++report.needles;
                continue;
            }

            const int mid = middleOfCollinear(mesh, f);
            const Vert m = f[mid], a = f[(mid + 1) % 3], b = f[(mid + 2) % 3];

            // The neighbour across the long edge a-b, which is the one the
            // middle vertex now sits inside.
            // Prefer a neighbour that is not itself a sliver. Two slivers
            // sharing their long edge are each other's only candidate, and
            // splitting one into the other just moves the problem around --
            // a level-4 Menger sponge has exactly one such pair, and it was
            // what stopped the last two from ever clearing.
            size_t nb = SIZE_MAX, fallback = SIZE_MAX;
            for (size_t cand : owners[undirected(a, b)]) {
                if (cand == si || dead[cand]) continue;
                if (isSliver[cand]) {
                    if (fallback == SIZE_MAX) fallback = cand;
                    continue;
                }
                nb = cand;
                break;
            }
            // A non-sliver neighbour is preferred but not required. Two
            // slivers sharing their long edge are each other's only
            // candidate -- a level-4 Menger sponge has one such pair, and
            // refusing to split into a sliver leaves them forever. Splitting
            // into one still makes progress, because the halves are smaller
            // and the next pass reconsiders them.
            if (nb == SIZE_MAX) nb = fallback;
            if (nb == SIZE_MAX) { ++report.leftBehind; continue; }

            // Split the neighbour at m, keeping its winding: the edge a-b
            // appears in it in some direction, and the two pieces must
            // traverse it the same way round.
            const auto& n = tris[nb];
            int e = -1;
            for (int i = 0; i < 3; ++i) {
                const Vert x = n[i], y = n[(i + 1) % 3];
                if ((x == a && y == b) || (x == b && y == a)) { e = i; break; }
            }
            if (e < 0) { ++report.leftBehind; continue; }
            const Vert x = n[e], y = n[(e + 1) % 3], apex = n[(e + 2) % 3];

            dead[si] = 1;
            dead[nb] = 1;
            added.push_back({x, m, apex});
            added.push_back({m, y, apex});
            ++report.removed;
            ++report.restitched;
        }

        std::vector<std::array<Vert, 3>> next;
        next.reserve(tris.size() + added.size());
        auto resolve = [&](Vert v) {
            for (int hop = 0; hop < 8; ++hop) {      // chains are short
                auto it = merge.find(v);
                if (it == merge.end()) break;
                v = it->second;
            }
            return v;
        };
        for (size_t i = 0; i < tris.size(); ++i) {
            if (dead[i]) continue;
            const auto& f = tris[i];
            next.push_back({resolve(f[0]), resolve(f[1]), resolve(f[2])});
        }
        for (const auto& f : added) {
            next.push_back({resolve(f[0]), resolve(f[1]), resolve(f[2])});
        }
        if (next.size() == tris.size() && added.empty()) break;
        tris.swap(next);
    }

    manifold::MeshGL out = mesh;
    out.triVerts.clear();
    out.triVerts.reserve(tris.size() * 3);
    for (const auto& f : tris) {
        out.triVerts.push_back(f[0]);
        out.triVerts.push_back(f[1]);
        out.triVerts.push_back(f[2]);
    }
    // These describe the old triangle list; a stale one is worse than none.
    out.runIndex.clear();
    out.runOriginalID.clear();
    out.faceID.clear();
    out.runTransform.clear();
    return out;
}

}  // namespace oscadeval
