# A separate static lib (not folded into main.cpp) specifically so
# tests/test_cli.cpp can link against the same argument-handling/--debug
# logic the real executable runs, in-process -- no subprocess spawning.
add_library(oscad_cli_lib STATIC cli_lib.cpp)
target_include_directories(oscad_cli_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(oscad_cli_lib PUBLIC openscad_cpp_evaluator)

add_executable(openscad-cpp-evaluator main.cpp)
target_link_libraries(openscad-cpp-evaluator PRIVATE oscad_cli_lib)

# See tests/CMakeLists.txt's own identical block for the full rationale --
# MSVC's ~1 MiB default thread stack is dwarfed by macOS/Linux's own
# ~8 MiB default; match it instead of running on an unusually small
# stack. Applies here too, not just the test binary -- this is the
# actual shipped CLI a real Windows user runs.
if(MSVC)
  target_link_options(openscad-cpp-evaluator PRIVATE /STACK:8388608)
endif()
