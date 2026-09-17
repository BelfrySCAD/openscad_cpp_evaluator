add_library(openscad_cpp_evaluator STATIC
  value.cpp
  utf8.cpp
  format_closure.cpp
  eval_error.cpp
  eval_context.cpp
  expr_eval.cpp
  stmt_eval.cpp
  user_calls.cpp
  bytecode_compiler.cpp
  bytecode_vm.cpp
  colored_body.cpp
  css_colors.cpp
  csg_resolve.cpp
  coverage.cpp
  csg_generate.cpp
  measure_geometry.cpp
  manifold_cache.cpp
  debug_profile.cpp
  eval_use.cpp
  export.cpp
  mesh_check.cpp
  zip_stored.cpp
  import/mesh_import.cpp
  import/surface_load.cpp
  import/dxf_import.cpp
  import/svg_import.cpp
  text/freetype_font_provider.cpp
  text/font_match.cpp
  text/text_metrics.cpp
  debug/debug_repl.cpp
  builtins/text.cpp
  builtins/call_args.cpp
  builtins/segments.cpp
  builtins/registry.cpp
  builtins/dxf_dim.cpp
  builtins/primitives_2d.cpp
  builtins/primitives_3d.cpp
  builtins/transforms.cpp
  builtins/color.cpp
  builtins/booleans.cpp
  builtins/modifiers.cpp
  builtins/control.cpp
  builtins/function_builtins.cpp
  builtins/import.cpp
  builtins/topology.cpp
  builtins/extrude.cpp
  builtins/roof.cpp
  builtins/surface.cpp
)

# Seeds $_BELFRYSCAD (eval_context.cpp). Parsed from pyproject.toml by the
# top-level CMakeLists so there is one source of truth for the version.
target_compile_definitions(openscad_cpp_evaluator PRIVATE
  OSCAD_EVAL_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
  OSCAD_EVAL_VERSION_MINOR=${PROJECT_VERSION_MINOR}
  OSCAD_EVAL_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

target_include_directories(openscad_cpp_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)
target_include_directories(openscad_cpp_evaluator PRIVATE ${stb_SOURCE_DIR} ${CMAKE_BINARY_DIR}/generated)

add_dependencies(openscad_cpp_evaluator generate_bundled_font)

target_link_libraries(openscad_cpp_evaluator PUBLIC openscad_cpp_parser manifold Boost::polygon)
target_link_libraries(openscad_cpp_evaluator PRIVATE linenoise freetype harfbuzz)
target_compile_options(openscad_cpp_evaluator PRIVATE
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall;-Wextra>
)

# -Wall -Wextra above is about OUR code, but two TUs pull a vendored header
# in wholesale and inherit its warnings: 27 from Boost.Polygon's voronoi
# headers and 9 from stb_image_write's implementation, against 5 that were
# real. Silenced per source file, per warning.
#
# Deliberately NOT -isystem, which looks like the tidier fix and is a trap:
# clang searches -isystem AFTER /usr/local/include, so a Homebrew Boost
# 1.90 silently won over the pinned 1.86 (measured, not assumed). Keeping
# -I keeps the fetched copy winning.
include(CheckCXXCompilerFlag)
foreach(_w deprecated-enum-float-conversion missing-field-initializers deprecated-declarations)
  check_cxx_compiler_flag("-Wno-${_w}" OSCAD_HAS_WNO_${_w})
endforeach()

if(OSCAD_HAS_WNO_deprecated-enum-float-conversion)
  # <boost/polygon/voronoi.hpp> does floating-point arithmetic on an
  # unnamed enum, deprecated in C++20.
  set_source_files_properties(builtins/roof.cpp PROPERTIES
    COMPILE_OPTIONS "-Wno-deprecated-enum-float-conversion")
endif()

# zip_stored.cpp defines STB_IMAGE_WRITE_IMPLEMENTATION, so stb's C compiles
# as ours: `= { 0 }` partial initializers, and one sprintf().
set(_stb_quiet "")
if(OSCAD_HAS_WNO_missing-field-initializers)
  list(APPEND _stb_quiet "-Wno-missing-field-initializers")
endif()
if(OSCAD_HAS_WNO_deprecated-declarations)
  list(APPEND _stb_quiet "-Wno-deprecated-declarations")
endif()
if(_stb_quiet)
  set_source_files_properties(zip_stored.cpp PROPERTIES COMPILE_OPTIONS "${_stb_quiet}")
endif()
