# Python extension module. Built only when BUILD_PYTHON_BINDINGS=ON (which
# scikit-build-core passes when building the wheel). nanobind + Python are
# resolved from the build environment (build-system.requires in the root
# pyproject.toml), so no FetchContent for them here.
# Development.SABIModule is required for nanobind's STABLE_ABI below --
# without it, STABLE_ABI silently falls back to a per-version build (no
# error, no warning). CPython >=3.12 is also a hard STABLE_ABI requirement.
find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)
find_package(nanobind CONFIG REQUIRED)

# NB_STATIC: link nanobind's own runtime into this module rather than a
# separate shared lib -- one self-contained .so, simpler to install.
# STABLE_ABI: build against Python's limited API (abi3) so one wheel per
# platform covers every supported Python version, instead of a full
# from-source Manifold/Boost rebuild per interpreter in CI.
nanobind_add_module(_openscad_cpp_evaluator NB_STATIC STABLE_ABI
  module.cpp
  ast_to_py.cpp
)

# The static evaluator lib PUBLIC-links parser + manifold + Boost::polygon,
# so this one line pulls the whole native closure in.
target_link_libraries(_openscad_cpp_evaluator PRIVATE openscad_cpp_evaluator)

# Land the compiled module inside the import package dir in the wheel
# (python/openscad_cpp_evaluator/ is copied by wheel.packages).
install(TARGETS _openscad_cpp_evaluator LIBRARY DESTINATION openscad_cpp_evaluator)
