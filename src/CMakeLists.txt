add_library(openscad_cpp_evaluator STATIC
  value.cpp
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
  csg_generate.cpp
  manifold_cache.cpp
  debug_profile.cpp
  eval_use.cpp
  export.cpp
  zip_stored.cpp
  import/mesh_import.cpp
  import/surface_load.cpp
  import/dxf_import.cpp
  import/svg_import.cpp
  text/stb_font_provider.cpp
  text/text_metrics.cpp
  debug/debug_repl.cpp
  builtins/text.cpp
  builtins/call_args.cpp
  builtins/segments.cpp
  builtins/registry.cpp
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

target_include_directories(openscad_cpp_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)
target_include_directories(openscad_cpp_evaluator PRIVATE ${stb_SOURCE_DIR} ${CMAKE_BINARY_DIR}/generated)

add_dependencies(openscad_cpp_evaluator generate_bundled_font)

target_link_libraries(openscad_cpp_evaluator PUBLIC openscad_cpp_parser manifold Boost::polygon)
target_link_libraries(openscad_cpp_evaluator PRIVATE linenoise)
target_compile_options(openscad_cpp_evaluator PRIVATE
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall;-Wextra>
)
