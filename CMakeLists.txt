cmake_minimum_required(VERSION 3.24)
project(openscad_cpp_evaluator CXX)

# On Apple Silicon, warn (don't fail -- a universal or intentionally-x86_64
# build is legitimate) if the effective target architecture is x86_64 while
# the real hardware is ARM64-capable, since that combination almost always
# means the *cmake binary itself* is an Intel-tap Homebrew install
# (/usr/local/bin/cmake) running under Rosetta 2 translation, not a genuine
# choice -- CMake's own architecture detection defaults to matching its own
# host PROCESS architecture when CMAKE_OSX_ARCHITECTURES isn't set
# explicitly, and a Rosetta-translated process reports x86_64 regardless of
# the underlying chip. Caught after this port's own benchmarks measured
# ~2x slower wall-clock time under Rosetta than a native ARM64 build of the
# identical source -- silent enough that four separate profiling sessions
# initially attributed the whole gap to the interpreter itself. `sysctl -n
# hw.optional.arm64` reports genuine hardware capability (unaffected by the
# querying process's own translation state), unlike CMAKE_SYSTEM_PROCESSOR/
# `uname -m`, which under Rosetta report the *translated* x86_64 view
# instead of the real underlying chip.
if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES)
  execute_process(
    COMMAND sysctl -n hw.optional.arm64
    OUTPUT_VARIABLE OSCAD_HW_IS_APPLE_SILICON
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(OSCAD_HW_IS_APPLE_SILICON STREQUAL "1" AND NOT CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    message(WARNING
      "This Mac has Apple Silicon hardware, but CMake is configuring an "
      "'${CMAKE_SYSTEM_PROCESSOR}' build -- almost always means `cmake` "
      "itself (run `file $(which cmake)`) is an Intel-tap Homebrew install "
      "at /usr/local/bin running under Rosetta 2, not a genuine ARM64/"
      "x86_64 choice. This project's own benchmarks measured roughly HALF "
      "the speed under Rosetta vs. a native build of the identical source. "
      "Fix: install/use a native ARM64 cmake (e.g. from an /opt/homebrew "
      "install), or pass -DCMAKE_OSX_ARCHITECTURES=arm64 to force one."
    )
  endif()
endif()

# Default to an optimized build when the caller doesn't ask for a specific
# one -- otherwise the exact command this repo's own README tells users to
# run (`cmake -S . -B build`, no -DCMAKE_BUILD_TYPE) silently produces an
# unoptimized build. CMAKE_CONFIGURATION_TYPES is set instead of
# CMAKE_BUILD_TYPE on multi-config generators (Xcode/MSVC), where this
# guard doesn't apply.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ponytail: MSVC always reports __cplusplus as 199711L regardless of the
# real /std: level unless this flag is passed (a long-standing legacy-
# compatibility quirk) -- set it at the outer-project level too (the
# submodule's own CMakeLists.txt sets it for its own targets, but that
# add_compile_options call is directory-scoped and doesn't reach this
# project's own src/tools/tests targets). Several dependencies here
# (nlohmann_json, Manifold, Boost.Polygon) branch on __cplusplus.
if(MSVC)
  add_compile_options(/Zc:__cplusplus)
  # MSVC's <cmath> only defines M_PI et al. when this is set before the
  # first include -- a command-line define guarantees that regardless of
  # each translation unit's own include order. Unix libcs define these
  # unconditionally (a POSIX extension our code already relies on), so no
  # equivalent is needed there.
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

add_subdirectory(external/openscad_cpp_parser)

# openscad_cpp_parser's own src/CMakeLists.txt exposes its public headers via
# ${CMAKE_SOURCE_DIR}/include -- CMAKE_SOURCE_DIR is fixed to the outermost
# project's root for the whole build, so once this repo is the outer project,
# that expression resolves to *our* include/ instead of the submodule's own.
# Append the real path here rather than patching the vendored submodule.
target_include_directories(openscad_cpp_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/openscad_cpp_parser/include>
)

include(FetchContent)

# MANIFOLD_TEST off: we don't want Manifold's own (large) test/sample/extras
# suite built as part of this project. MANIFOLD_PAR stays at its own default
# (OFF) -- no TBB parallel backend needed yet; revisit if profiling later
# shows single-threaded Boolean ops are a bottleneck. MANIFOLD_CROSS_SECTION
# stays at its own default (ON) -- 2D CrossSection support is needed from
# Phase 3 onward.
#
# BUILD_SHARED_LIBS off: Manifold's own CMakeLists.txt defaults this ON,
# which builds manifold.dll/manifoldc.dll instead of static libs. Harmless
# on Linux/macOS (CMake gives build-tree executables an automatic rpath to
# find a build-tree .so/.dylib), but Windows has no rpath equivalent -- a
# DLL must be either next to the .exe or on PATH, neither of which is true
# for a FetchContent dependency built into its own build/_deps/ tree, so
# every test/CLI executable failed to even start (STATUS_DLL_NOT_FOUND).
# Forcing this off, matching the existing MANIFOLD_TEST override pattern,
# fixes Windows and is a no-op everywhere else this project already worked.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(MANIFOLD_TEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  manifold
  GIT_REPOSITORY https://github.com/elalish/manifold.git
  GIT_TAG v3.5.2
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(manifold)

# Boost.Polygon's Voronoi diagram builder, for roof()'s general (multi-
# contour/hole) case -- fetched as individual per-library repos, not the
# full Boost superproject, to keep this light. Deliberately not CGAL (which
# real OpenSCAD's own "straight" roof method uses) -- see CLAUDE.md's roof()
# gap note for the tradeoff. Additional per-library repos get added here if
# the build reveals more of the header dependency closure than config alone.
FetchContent_Declare(boost_config GIT_REPOSITORY https://github.com/boostorg/config.git GIT_TAG boost-1.86.0 GIT_SHALLOW TRUE)
FetchContent_Declare(boost_polygon GIT_REPOSITORY https://github.com/boostorg/polygon.git GIT_TAG boost-1.86.0 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(boost_config boost_polygon)

# nothings/stb: single-header stb_image.h (surface()'s PNG loader) and
# stb_truetype.h (the built-in FontProvider's TTF parser/rasterizer, Phase
# 7) -- no build system of its own, just headers, so this only needs an
# include path, not add_subdirectory/MakeAvailable's target machinery.
FetchContent_Declare(stb GIT_REPOSITORY https://github.com/nothings/stb.git GIT_TAG master GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(stb)

# yhirose/cpp-linenoise: header-only, BSD-2-Clause C++ port of antirez/
# linenoise with native Windows console support (no ANSI emulation layer,
# unlike GNU readline's Windows story) -- gives the --debug REPL's real
# interactive prompt arrow-key command history. Its own CMakeLists.txt
# already guards its test/example targets behind
# `CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR`, so FetchContent_
# MakeAvailable here only exposes its `linenoise` INTERFACE target, matching
# the existing MANIFOLD_TEST-override pattern's intent for free.
FetchContent_Declare(cpp_linenoise GIT_REPOSITORY https://github.com/yhirose/cpp-linenoise.git GIT_TAG master GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(cpp_linenoise)

# Bakes an absolute path to the bundled default font (resources/fonts/) into
# a generated header, so StbFontProvider's default constructor doesn't need
# a runtime-configurable resource search path.
#
# ponytail: fine for this repo's own tests/CLI (always built from this
# source tree) but not how a packaged/installed build should ship a font --
# a real install needs either the resource file copied alongside the
# installed binary or the font bytes embedded directly into the binary
# (e.g. via a generated byte-array header). Upgrade when packaging is
# actually needed; not worth building now for a resource this project only
# consumes from its own build tree so far.
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openscad_cpp_evaluator/bundled_font_path.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/openscad_cpp_evaluator/bundled_font_path.hpp
)

option(BUILD_TESTING "Build tests" ON)
option(BUILD_EXAMPLES "Build the examples/ programs (e.g. minimal_debugger)" ON)

add_subdirectory(src)
add_subdirectory(tools/cli)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
