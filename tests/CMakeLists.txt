include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(oscad_eval_tests
  test_value.cpp
  test_eval_context.cpp
  test_scoping.cpp
  test_expr_eval.cpp
  test_csg_tree.cpp
  test_export.cpp
  test_primitives.cpp
  test_transforms.cpp
  test_booleans.cpp
  test_control_flow.cpp
  test_function_builtins.cpp
  test_import_export.cpp
  test_zip_stored.cpp
  test_use.cpp
  test_extrude_roof.cpp
  test_surface.cpp
  test_dxf_svg_import.cpp
  test_text.cpp
  test_manifold_cache.cpp
  test_profiling.cpp
  test_debug_hooks.cpp
  test_multi_color_merge.cpp
  test_viewport_params.cpp
  test_cli.cpp
)
target_link_libraries(oscad_eval_tests PRIVATE openscad_cpp_evaluator oscad_cli_lib GTest::gtest_main)
# stb_image_write.h, used only by this test binary to generate a PNG
# fixture for ToRenderableBodies/surface() image-loading tests.
target_include_directories(oscad_eval_tests PRIVATE ${stb_SOURCE_DIR})

include(GoogleTest)
gtest_discover_tests(oscad_eval_tests)
