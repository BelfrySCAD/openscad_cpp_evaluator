# Python extension module. Built only when BUILD_PYTHON_BINDINGS=ON (which
# scikit-build-core passes when building the wheel). nanobind + Python are
# resolved from the build environment (build-system.requires in the root
# pyproject.toml), so no FetchContent for them here.
find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(nanobind CONFIG REQUIRED)

# NB_STATIC: link nanobind's own runtime into this module rather than a
# separate shared lib -- one self-contained .so, simpler to install.
nanobind_add_module(_openscad_cpp_evaluator NB_STATIC
  module.cpp
)

# The static evaluator lib PUBLIC-links parser + manifold + Boost::polygon,
# so this one line pulls the whole native closure in.
target_link_libraries(_openscad_cpp_evaluator PRIVATE openscad_cpp_evaluator)

# Land the compiled module inside the import package dir in the wheel
# (python/openscad_cpp_evaluator/ is copied by wheel.packages).
install(TARGETS _openscad_cpp_evaluator LIBRARY DESTINATION openscad_cpp_evaluator)
