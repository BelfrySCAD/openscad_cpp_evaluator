# libtess2

Mikko Mononen's libtess2 (SGI Free Software License B 2.0, see LICENSE.txt),
upstream commit c655ececd59c2227b4fb2a1138c8087cd4ac980b (2015-01-13) from
https://github.com/memononen/libtess2, with two changes so that a polyhedron
face is split exactly as OpenSCAD splits it (its own copy, src/ext/libtess2,
carries the same two):

- `tess.c`: a collinear contour's normal uses the shortest axis (`ShortAxis`),
  not the longest, as upstream itself later did.
- `sweep.c`: the sweep's sentinels are spread by at least 0.01, so an empty
  bounding box cannot make them coincide.

Compiled as C with warnings off; see src/CMakeLists.txt.
