# openscad_cpp_evaluator

A C++20 port of [`openscad_evaluator`](https://github.com/BelfrySCAD) (Python): an AST-walking
geometry engine that takes a parsed OpenSCAD AST and produces [Manifold](https://github.com/elalish/manifold)
CSG geometry, ready to export to STL/OBJ/OFF/3MF or hand off to a renderer.

Built on:
- [`openscad_cpp_parser`](https://github.com/BelfrySCAD/openscad_cpp_parser) (git submodule) for
  lexing/parsing/AST/scope resolution.
- [Manifold](https://github.com/elalish/manifold) v3.5.2 for the actual CSG boolean/mesh engine.
- [Boost.Polygon](https://github.com/boostorg/polygon) for `roof()`'s Voronoi-diagram construction.
- [`nothings/stb`](https://github.com/nothings/stb) (header-only) for `surface()`'s image loader
  and the built-in font provider's TrueType parsing/rasterization.

All fetched automatically via CMake `FetchContent` — no manual dependency installation beyond the
toolchain below.

## Status

All planned phases (0–9) are complete: the full primitive/transform/boolean/modifier/control-flow
language surface, every builtin function, STL/OBJ/OFF/3MF import and export, `hull()`/`minkowski()`/
`linear_extrude()`/`rotate_extrude()`/`roof()`/`offset()`/`surface()`/DXF & SVG import/`text()`, an
opt-in content-hash geometry cache, per-call-site profiling, and a gdb-style `--debug` REPL. See
`CLAUDE.md` for the full phase-by-phase history and architecture notes.

## Build

Requires:
- CMake ≥ 3.24
- A C++20 compiler (Clang, GCC, or MSVC — all three are covered by CI)
- Bison ≥ 3.8 and Flex
  - macOS: `brew install bison flex` (the system bison is too old; CMake auto-detects Homebrew's)
  - Linux: `sudo apt-get install bison flex`
  - Windows: MSYS2's `bison`/`flex` (`pacman -S bison flex`) — GitHub's `windows-latest` runner
    ships MSYS2 pre-installed at `C:\msys64`; CMake auto-detects it there too

GoogleTest and every other C++ dependency are fetched automatically.

```bash
git clone --recurse-submodules <this-repo>
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

(Omitting `-DCMAKE_BUILD_TYPE=Release` builds unoptimized — pass it explicitly for anything
performance-sensitive.)

On Windows, CMake defaults to the multi-config Visual Studio generator, which ignores
`CMAKE_BUILD_TYPE`; pass `--config Release` to `cmake --build` (and `-C Release` to `ctest`)
instead.

## Test

```bash
ctest --test-dir build --output-on-failure
```

Run a single binary or filter:

```bash
build/tests/oscad_eval_tests
build/tests/oscad_eval_tests --gtest_filter='FormatNumber.*'
```

The suite runs twice in CI — once with the bytecode VM on (the default) and once with it off
(`OSCAD_BYTECODE_VM=0`) — so a regression in either the compiled or interpreted expression-eval
path is caught.

## CLI

```
build/tools/cli/openscad-cpp-evaluator <input.scad> -o <output.{stl,obj,off,3mf}> [--format stl|obj|off|3mf] [--debug]
```

The output format is inferred from `-o`'s extension, or set explicitly with `--format`.
`--debug` drops into a gdb-style REPL (`break`/`run`/`continue`/`step`/`next`/`finish`/`print`/
`backtrace`/`set`/`list`/`quit`/`help`) before and during evaluation.

```bash
build/tools/cli/openscad-cpp-evaluator model.scad -o model.stl
```

## Architecture

See `CLAUDE.md` for the full per-file architecture map, the Python-reference cross-check history,
and the rationale behind every non-obvious design choice (the two-pass resolve/generate CSG
pipeline, the `FontProvider` injection seam, the closure-detection rule, etc.).

## CI

GitHub Actions builds and tests on Linux, macOS, and Windows on every push/PR (`.github/workflows/ci.yml`).
