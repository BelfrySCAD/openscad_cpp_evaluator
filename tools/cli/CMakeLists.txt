# A separate static lib (not folded into main.cpp) specifically so
# tests/test_cli.cpp can link against the same argument-handling/--debug
# logic the real executable runs, in-process -- no subprocess spawning.
add_library(oscad_cli_lib STATIC cli_lib.cpp)
target_include_directories(oscad_cli_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(oscad_cli_lib PUBLIC openscad_cpp_evaluator)

add_executable(openscad-cpp-evaluator main.cpp)
target_link_libraries(openscad-cpp-evaluator PRIVATE oscad_cli_lib)
