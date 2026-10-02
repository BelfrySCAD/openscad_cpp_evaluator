# Patches the fetched Manifold v3.5.2 source; run as its FetchContent
# PATCH_COMMAND, in the source directory.
#
# CsgLeafNode::Compose calls RemoveDegenerates() on the combined mesh and
# then sorts it, and on some inputs the cleanup leaves the face data
# inconsistent: SortGeometry -> GatherFaces then reads past the end of an
# array (a heap-buffer-overflow under AddressSanitizer). Our
# MinkowskiDifference.ShrinksANonConvexBodyOnEverySide test reaches it;
# reproduced with Manifold alone, and bisected upstream to elalish/manifold
# #1789 ("Improved decimation", 969b1417), which fixes it by deleting this
# same call -- the only part of that commit the fix needs. Not in any
# release yet; drop this patch when Manifold is bumped past it.
#
# Idempotent: a re-run on already-patched source changes nothing. Anything
# else -- the call or its surroundings changed -- is an error, so a version
# bump can never silently skip the patch.
set(file src/csg_tree.cpp)
file(READ ${file} src)
set(call "  // required to remove parts that are smaller than the tolerance\n  combined.RemoveDegenerates();\n")
string(FIND "${src}" "${call}" at)
if(at EQUAL -1)
  string(FIND "${src}" "combined.RemoveDegenerates();" any)
  if(NOT any EQUAL -1)
    message(FATAL_ERROR "patch_manifold.cmake: ${file} changed around RemoveDegenerates(); update the patch")
  endif()
  message(STATUS "patch_manifold.cmake: already applied")
  return()
endif()
string(REPLACE "${call}" "" src "${src}")
file(WRITE ${file} "${src}")
message(STATUS "patch_manifold.cmake: removed RemoveDegenerates() from CsgLeafNode::Compose")
