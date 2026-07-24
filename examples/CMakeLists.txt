add_executable(minimal_debugger minimal_debugger.cpp)
target_link_libraries(minimal_debugger PRIVATE openscad_cpp_evaluator)

add_executable(manifold_cache_reuse manifold_cache_reuse.cpp)
target_link_libraries(manifold_cache_reuse PRIVATE openscad_cpp_evaluator)

# Both examples are self-checking (assert()/nonzero-exit on failure, see
# each one's own main()) -- registering them as plain ctest cases (not a
# gtest wrapper spawning a subprocess, which the project's own testing
# notes reserve for exactly the kind of flakiness this avoids) mirrors the
# reference's own test_examples.py, which just runs each example script
# and checks it doesn't raise/exit nonzero.
if(BUILD_TESTING)
  add_test(NAME ExampleMinimalDebugger COMMAND minimal_debugger)
  add_test(NAME ExampleManifoldCacheReuse COMMAND manifold_cache_reuse)
endif()
