# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A C++20 port of `openscad_evaluator` (Python): an AST-walking geometry engine that takes a parsed
OpenSCAD AST and produces Manifold CSG geometry. This port builds on two dependencies:

- `openscad_cpp_parser` (git submodule, `external/openscad_cpp_parser`) for lexing/parsing/AST/scope
- Manifold (https://github.com/elalish/manifold) v3.5.2, pulled in via CMake `FetchContent` (pinned
  tag, not vendored) — wired in as of Phase 2
- Boost.Polygon (`boostorg/polygon` + `boostorg/config`, fetched individually — not the full Boost
  superproject) for `roof()`'s Voronoi-diagram construction — wired in as of Phase 6, see below
- `nothings/stb` (`stb_image.h`, `stb_truetype.h`; header-only, no build step) for `surface()`'s
  PNG loader and the built-in `FontProvider` default's TTF parsing/outline extraction — wired in as
  of Phase 7

The full design and phase-by-phase build order live in the plan this repo is being built from:
`/Users/gminette/.claude/plans/please-plan-out-a-twinkly-anchor.md`. Read it before starting a new
phase — it has the rationale for every non-obvious choice (value model, `EvalContext` lifetime,
the `FontProvider` callback abstraction for Qt-hosted cross-platform font support, why Manifold's
own vendored `linalg.h` types and not GLM/Eigen, etc.) and the exit criterion each phase needs to
hit before the next one starts. §6a was corrected during Phase 2 after checking Manifold's real
source (it no longer uses GLM) — a live example of why the plan says to verify API assumptions
against real headers rather than trust general knowledge.

**Current status**: All phases (0–9) complete. Value model; EvalContext/error formatting/expression eval;
Manifold wired in + the resolve/generate CSG-tree pipeline; full primitive/transform/boolean/modifier
coverage + `color()`; STL/OBJ/OFF export; full control flow (`for`/`if`/`if-else`/`let`
statement+expression forms/`echo`/`assert` statement+expression forms/`intersection_for`), all 6 list
comprehension clause kinds, user function calls (recursion, defaults, `FunctionLiteral` values),
user module calls (`children()`/`children(N)`, closure detection via `callCtxFor`), list
indexing/swizzle member access; the full math/string/list/type-check builtin *function* dispatch
(`sin`/`cos`/`pow`/`search`/`lookup`/`chr`/`ord`/`is_*`/`object`/`has_key`/etc, ~43 names — a
completely separate namespace/precedence rule from the builtin *modules* in `registry.cpp`: a
builtin function name always wins over a same-named user function, checked *before* the
user-function lookup), STL/OBJ/OFF/3MF `import()` in both module (geometry) and expression
(VNF/JSON) contexts, and 3MF read/write; (Phase 6) `hull()`/`minkowski()` (transparent-splice
CSG-tree shape like union/difference/intersection, role-split via the shared `splitByRole`),
`linear_extrude()`/`rotate_extrude()`/`projection(cut=)`, and `roof()`; and (Phase 7) `offset()`,
`surface()` (`.dat` text + PNG/JPEG/BMP/GIF via `stb_image`), hand-rolled DXF (`LWPOLYLINE`/2D
`POLYLINE`) and SVG (`path`/`polygon`/`polyline`/`rect`/`circle`/`ellipse`, nested
`transform=`, all path commands including elliptical arcs) 2D contour import in both module
(Region geometry) and expression (Region value) contexts, and the `FontProvider` abstraction
(`font_provider.hpp`) + built-in `StbFontProvider` default + `text()`/`textmetrics()`/
`fontmetrics()`; and (Phase 8) `ManifoldCache` — an opt-in, thread-safe content-hash cache of
already-generated CSGNode subtrees (`manifold_cache.hpp`/`.cpp`), plus `rands()`-taint tracking
(`Evaluator::noteRandsCall()`/`CSGNode::uncacheable`) so a subtree whose resolved content isn't a
pure function of its own params (anything that called `rands()` while resolving) is never served
from or written to the cache; and (Phase 9, the last phase) per-call-site profiling
(`profile.hpp`/`CallSiteProfile`/`ProfileResult`, `Evaluator::profileEnter`/`profileExit` around
every user module/function call, self-vs-cumulative time with a recursion guard) plus a debugging
seam (`debug_hooks.hpp`'s `DebugHookFn`/`ErrorBreakFn`/`ReturnHookFn`, `Evaluator::checkDebug()`
called once per top-level statement via `evalChildren` — a deliberate, documented scope reduction
from the reference's many expr-level call sites, see `DebugHookFn`'s own doc comment) and a
gdb-style REPL built on that seam (`debug_repl.hpp`/`.cpp`, wired into the CLI's `--debug` flag) —
`break`/`delete`/`list`/`run`/`quit`/`help` before running, `continue`/`step`/`next`/`finish`/
`child`/`print`/`backtrace`/`set`/`break`/`delete`/`list`/`quit`/`help` while paused. Cross-checked against
the Python reference CLI for a union/difference/sphere/cylinder/cube script (Phase 3), a
recursion+nested-closure+`children()`+`for`-heavy script (Phase 4), a math-builtin-driven
sizing/import-round-trip script (Phase 5, including a Python→C++ and C++→Python 3MF cross-read), a
hull/minkowski/linear_extrude/rotate_extrude/roof/projection-combined script (Phase 6), a
text()/linear_extrude(text())/DXF-import/SVG-import-combined script (Phase 7, including a
`.dat`/PNG `surface()` round-trip and direct `textmetrics()`/`fontmetrics()` value comparison), and
(Phase 9) an interactive `--debug` session run against both this port's own CLI and the Python
reference's own `openscad-evaluator --debug` side by side on the identical script/breakpoint/command
sequence: byte-for-byte identical REPL transcripts (including a real discrepancy from the *Python
project's own README* — its worked example's transcript turns out to be simplified/idealized
documentation, omitting an incidental break-on-first pause the real implementation, on both sides,
actually shows — see the "debugger" entry below for the full story). Every other cross-check:
identical triangle count and bounding box (or, for `textmetrics()`/`fontmetrics()`, identical
numeric field values) in every case *except* `roof()` on hole-bearing input, a deliberate, verified
improvement over the Python reference — see the `roof()` entry below. Phase 8's own exit criterion
(identical output cache-on vs. cache-off, measurable reuse on an unchanged subtree) is covered
in-repo by `tests/test_manifold_cache.cpp` rather than a CLI cross-check, since `ManifoldCache` is a
library-level feature a long-lived host (not this repo's single-shot CLI) opts into — see below.

**Test-suite parity review, post-Phase-9**: comparing this repo's `ctest` suite against the Python
reference's own `pytest` suite class-by-class (not just a raw test-count comparison, which is
misleading either direction given the two frameworks' different granularity) surfaced four real gaps,
all now closed:
1. Multi-color CSG merge (`attachTriColors`, see the `booleans.cpp` entry below) — a real missing
   *feature*, not just missing tests; ported alongside its tests from the reference's own
   `TestMultiColorCSGMerge`.
2. `Evaluator::evaluate()`'s `viewportParams` parameter (see the CSG-pipeline entry below) — same
   story, ported alongside tests from `TestDynExplicit`.
3. No CLI-level test suite. Closed by extracting the CLI's actual logic out of `main()` into
   `tools/cli/cli_lib.hpp`/`.cpp`'s `runCli(args, in, out, err)` — an in-process testable function
   taking injectable streams, mirroring the reference's own `cli.main(argv)` (testable directly, no
   subprocess, `capsys`/monkeypatched `input()` for output/input capture) — `main.cpp` is now a
   3-line wrapper calling it with `std::cin`/`std::cout`/`std::cerr`. `debug_repl.hpp`/`.cpp` needed
   the same treatment: `DebugRepl`'s constructor now takes `std::istream&`/`std::ostream&` (defaulted
   to `std::cin`/`std::cout` for real use), so a test can drive an entire `--debug` session with an
   `istringstream` of canned commands and assert against an `ostringstream`, exactly like the
   reference's own `_feed_input` pytest fixture. `tests/test_cli.cpp` ports every case from the
   reference's `test_cli.py` this way, including the exact `--debug` breakpoint/step/print/set/
   error-break scenarios previously verified only by hand.
4. No C++ port of `examples/manifold_cache_reuse.py`, and no `ctest` case running the compiled
   `examples/` binaries. Closed: `examples/manifold_cache_reuse.cpp` (registered in `examples/
   CMakeLists.txt`) demonstrates a real ~60x speedup (`minkowski()` of two dense spheres, identical
   subtree reused across a `translate()`-only edit) with the same self-checking-`assert()`/nonzero-
   exit design `minimal_debugger.cpp` already used; both are now registered as plain `add_test()`
   `ctest` cases (not a gtest subprocess wrapper — a bare `add_test(COMMAND <binary>)` is the
   idiomatic CMake shape for "run this self-checking binary, fail if it exits nonzero," matching the
   reference's own `test_examples.py` intent without the flakiness a spawned-subprocess *gtest* case
   would risk).

**Post-Phase-9 unimplemented-surface audit**: a from-scratch audit (complete builtin-*module*-name
cross-check against the reference's `_RESOLVE_DISPATCH`, complete builtin-*function*-name cross-check
against `_math_fns`/`_BUILTIN_FN_NAMES` — both 100% match, no gaps — plus a grep of this port's own
source for "not yet implemented"/TODO/stub markers, since an external API-surface comparison alone
can't find an internal behavioral divergence) surfaced two real gaps, both closed, plus one
doc-only issue:
1. **Unknown module names threw instead of warning-and-continuing.** `evalModularCall`
   (`csg_resolve.cpp`) used to `throw std::logic_error` for any module name that's neither a
   registered builtin nor a user-defined module — verified against the Python reference directly
   (`sphere_typo(1); cube(2);` returns 1 body there, with just a
   `WARNING: Ignoring unknown module 'sphere_typo'...` echo) that real OpenSCAD instead issues a
   WARNING (with TRACE lines walking the call stack, exactly like an error would, when called from
   inside an active user-module/function call — verified with a nested case too) and simply produces
   no geometry for that call, continuing to evaluate everything else. This meant any script
   referencing a typo'd module name, or a builtin genuinely missing from this port's registry (there
   are none currently, but the *language feature* of tolerating one is real), crashed the whole
   evaluation instead of degrading gracefully like real OpenSCAD. Fixed to warn-and-continue,
   matching `_eval_builtin`'s fallback exactly; `tests/test_csg_tree.cpp`'s
   `CsgTree.UnknownModuleWarnsAndContinues`/`UnknownModuleInsideUserModuleWarnsWithTrace` replace the
   old (now-wrong) `UnknownBuiltinThrows` test.
2. **Bitwise operators threw instead of evaluating.** `openscad_cpp_parser`'s grammar accepts
   `|`/`&`/`~`/`<<`/`>>` (5 `NodeKind::Bitwise*Op` kinds); `evalExpr`'s `default:` case threw for
   these. Originally believed to be a vestigial/unused grammar superset (the Python reference had no
   handler at all, confirmed via a zero-hit grep for "Bitwise" in `evaluator.py`) — **corrected**:
   real OpenSCAD actually shipped these in PR #4833 (`openscad/openscad`, merged 2025-03-14, "Fixes
   #3345"), just after this repo's Python reference was last synced with upstream. Implemented for
   real in both places once this came to light — see the "Bitwise/shift operators" entry below for
   the full spec and both ports' details. `function_builtins.hpp`'s header comment claimed
   `textmetrics`/`fontmetrics`/`object()` were "named-only stubs returning undef" pending Phase 7 —
   stale since Phase 7 actually implemented all three; corrected as a doc-only fix alongside this.

**Bitwise/shift operators (`| & ~ << >>`), added after the above audit**: real OpenSCAD's PR #4833
(`openscad/openscad`, merged 2025-03-14) added these as a thin layer over ordinary OpenSCAD
numbers — no new integer type. Full spec pulled directly from the PR's own diff (`Value.cc`) and its
regression test (`tests/data/scad/functions/bitwise-operators.scad`), not re-derived: both operands
truncate-to-int64 (`trunc()` toward zero, cast to `int64_t`; bool is a distinct type, never coerced,
same as every other numeric builtin in this codebase — a bool/string/etc. operand is an "undefined
operation (X op Y)" warning + undef, exactly like the existing comparison operators), operate in
int64 two's-complement arithmetic, then cast back to a plain `double`. `<<`/`>>`'s shift amount must
be `0 <= rhs < 64` or the result is undef with a `"negative shift"`/`"shift too large"` warning (no
operand-type names in *that* particular message, unlike the generic undefined-operation one — ported
verbatim from the PR's own warning text). Right shift of a negative left-hand side is arithmetic
(sign-propagating): in the C++ port this falls out of `int64_t operator>>`, well-defined as
arithmetic shift by the C++20 standard — the same underlying operation real OpenSCAD's own compiled
C++ produces, so no separate sign-fixup was needed to match it; the Python port masks/re-signs into
the 64-bit two's-complement domain explicitly (`Evaluator._to_bitwise_int64`, `evaluator.py`), since
Python ints are arbitrary-precision and don't overflow/wrap on their own — needed specifically to
reproduce int64's *bounded* overflow behavior (e.g. `1 << 32 << 32 == 0`, verified against the real
PR's own test case) that Python's native arbitrary-precision `<<` wouldn't otherwise produce. Real
OpenSCAD's own grammar also gained hex literals (`0xFF`) in the same PR — **not implemented here**:
out of scope for this fix (the user's ask was specifically bitwise operators), and moot besides,
since `openscad_cpp_parser`'s lexer already accepts `0xFF`-style literals independently (confirmed:
`x = 0xff;` already parses to `NumberLiteral{val: 255.0}`) and Python's own float literal parsing
handles `0x` today too. Operator *precedence* needed no grammar changes on the C++ side —
`openscad_cpp_parser`'s existing `%left`/`%right` declarations for these 5 tokens were verified
(via the parser CLI's `-j` JSON-AST-dump mode, checking real parse-tree shape for the PR's own
precedence test cases: shift-over-and, and-over-or, or-over-comparison, add-over-shift) to already
exactly match real OpenSCAD's own grammar, including the deliberate C-divergence that puts binary-
and/binary-or *above* comparison (avoiding `x & 1 == 0` parsing as `x & (1 == 0)`) — a pre-existing,
independently-correct choice by whoever built `openscad_cpp_parser`, not something this fix touched.
Cross-checked directly against the real PR's own regression test
(`tests/data/scad/functions/bitwise-operators.scad`, ported into both `tests/test_expr_eval.cpp`'s
`ExprEvalBitwiseOps` suite here and `openscad_evaluator/tests/test_evaluator.py`'s `TestBitwiseOps`
suite on the Python side) — basic operations, all four precedence orderings, negative-number/
fraction-truncation cases, the `1 << 32 << 32 == 0` overflow-wraparound case, and every undef/warning
case (out-of-range shift, non-numeric operand). All pass on both ports.

**`roof()`'s real algorithm, corrected mid-Phase-6**: this was originally ported from the Python
reference's own simplified approach (a self-rolled single-contour straight-skeleton "Tier 1" +
an SDF/`Manifold::LevelSet` approximation "Tier 2" fallback for anything else), which itself
deliberately avoids CGAL. Prompted by the user pointing at real OpenSCAD's own source
(`openscad/src/geometry/roof_ss.cc`/`roof_vd.cc`), it turned out real OpenSCAD's **default**
`method="voronoi"` isn't a straight skeleton at all — it's a `boost::polygon::voronoi_diagram`
(Fortune's sweep line) segment-Voronoi-diagram construction, which handles holes/multi-contour input
natively and exactly; only the *non-default* `method="straight"` uses CGAL. Given the user wanted to
avoid CGAL, and research confirmed hand-rolling a general straight skeleton is a well-documented hard
problem (the original Felkel & Obdržálek paper CGAL's own implementation is based on was itself
flawed and needed years of fixes — see the SoCG 2020 paper "On Implementing Straight Skeletons:
Challenges and Experiences"), `roof.cpp` was rewritten from scratch as a close port of
`roof_vd.cc`'s `voronoi_diagram_roof`/`vd_inner_faces`/`discretize_arc` (CGAL/Eigen/Clipper2Lib
swapped for `manifold::vec2` arithmetic and `manifold::Triangulate`), on top of Boost.Polygon's
Voronoi builder (fetched as just `boostorg/polygon` + `boostorg/config`, confirmed to need nothing
else from Boost). Both `method` values dispatch to this same construction (no CGAL-based `"straight"`
path exists in this port). **Verified independently**, not just "it compiles": a brute-force
distance-transform grid search over a square-frame-with-a-hole test case found the same ridge height
(≈1.756) this implementation produces — the Python reference's own SDF fallback gets this case
measurably wrong (≈1.5, ~15% low) since it was never more than a documented approximation for
exactly this shape of input. See `roof.cpp`'s own file header comment for the full rationale.

**Known gap closed in Phase 6, worth remembering going forward**: the CLI (`tools/cli/main.cpp`) had
silently written STL bytes to *any* output path regardless of its extension since Phase 2 — passing
`-o foo.3mf` produced a mislabeled STL file, undetected because every prior phase's manual
cross-checks happened to only ever pass `-o *.stl`. Caught by this phase's own 3MF cross-check (the
"3MF" the CLI had just written wouldn't parse as a ZIP at all). Fixed by adding real extension/
`--format`-based dispatch (mirroring the Python CLI's own `format_for_path`). Lesson: a manual
cross-check only exercises the exact path it's given — vary the output format/extension across
phases' smoke tests, don't reuse the same `-o out.stl` every time.

**Known gap closed in Phase 6, also worth remembering**: top-level 2D-only results (a bare
`circle();`, or Phase 6's own `projection()`) couldn't be mesh-exported at all —
`writeStl`/`writeObj`/`writeOff`/`writeThreeMf` only ever look at `ColoredBody::body` (3D), and a
2D-only result only ever sets `.section`. The reference's CLI calls `to_renderable_bodies()` right
before export specifically to paper over this (thin-extrudes any 2D-only top-level body to a
1e-3-unit-tall Manifold, tagged `flat_preview`); this port hadn't ported that step at all, so *any*
top-level 2D primitive/projection() script had been silently unexportable via the CLI since Phase 3
-- caught only now via this phase's cross-check on a `projection()`-containing combined script.
Fixed by porting `toRenderableBodies()` (`colored_body.hpp`/`.cpp`) and calling it from the CLI right
before export, matching the reference's own call site exactly.

**Font support, Phase 7**: mirrors the plan's §6 design exactly — a `FontProvider` abstract
interface (`font_provider.hpp`: `resolveFont`/`metrics`/`glyphMetrics`/`glyphOutline`) rather than
hard-depending on the Python reference's `fc-match` shell-out (Linux-only, unusable inside a
cross-platform Qt host app). `Evaluator`'s constructor takes an optional
`shared_ptr<FontProvider>`; if unset, `Evaluator::fontProvider()` lazily constructs the built-in
`StbFontProvider` default on first use (not eagerly in the constructor, so a script that never
calls `text()`/`textmetrics()`/`fontmetrics()` never pays for font-file parsing).
`StbFontProvider::resolveFont()` always returns the one bundled font (`resources/fonts/
LiberationSans-Regular.ttf`, the *exact same* font file as the Python reference's own bundled
default, copied over so cross-checks are apples-to-apples) regardless of the requested spec — no
system font matching, matching the plan's documented limitation for the built-in default; a real Qt
host app supplies its own `FontProvider` (`QFontDatabase`/`QRawFont`-backed) instead of this class
for actual system-font resolution. `text_metrics.hpp`/`.cpp` holds the shared, backend-agnostic
layout math (`measureText`/`textAlignOffset`, mirroring `_measure_text`/`_text_align_offset`
exactly) used by both `text()` (`builtins/text.cpp`) and `textmetrics()`/`fontmetrics()`
(`function_builtins.cpp`) — cross-checked bit-for-bit against the Python reference's fonttools-based
metrics on the bundled font (identical `advance`/`ascent`/`descent`/family name values) and against
its glyph-outline mesh output (identical triangle count and bounding box for `text()`).

**Known gap carried from Phase 7**: `StbFontProvider::resolveFont()` ignores its `spec` argument
entirely (always the one bundled font) — this is the documented, intentional shape of the *built-in
default* (see above), not a bug; a script that requests a specific system font (`font="Times New
Roman:style=Bold"`) silently gets Liberation Sans instead under this default provider. No upgrade
path needed in this repo: the fix is a Qt host application supplying its own `FontProvider`, which
is exactly what the injection seam exists for.

**Debugger/profiler, Phase 9 (the last phase)**: `checkDebug()` fires from exactly one call site
(`evalChildren`'s `runAll`, plus one explicit call each at user-function/function-literal body entry
and inside `breakpoint()`'s resolve) rather than the reference's several dozen `_check_debug` call
sites scattered across nearly every statement/expression-eval function — a deliberate scope
reduction (see `debug_hooks.hpp`'s `DebugHookFn` doc comment for the full reasoning): the reference's
extra sites exist for `expr_level` sub-statement stepping (individual list-comprehension clauses,
ternary branches) that neither this port's own `debug_repl.cpp` REPL nor the reference's own
`_debug_repl.py` REPL ever actually uses (both only ever pause/step at statement granularity in
practice). Verified this is a faithful, not just simplified, port by running an *identical*
`--debug` session (same script, same `break`/`run`/`print`/`continue`/`step`/`backtrace`/`set`
sequence) against both this port's CLI and the real Python reference's own `openscad-evaluator
--debug` side by side: transcripts matched exactly, **including** an unexpected extra pause neither
transcript "should" have had per the Python project's own README worked example (which shows `run`
landing directly on a `break 12` breakpoint with no earlier stop) — turns out `_break_on_first`
unconditionally pauses at the very first statement reached in the main file regardless of any
already-set breakpoint, so a real run always shows *two* pauses (first-statement, then the
breakpoint) unless they happen to coincide. The README's transcript is simplified documentation, not
literal output — confirmed by actually running the real Python CLI, not just reading its source.
This port's `debug_repl.cpp` reproduces the exact same two-pause behavior, proving the port is
faithful to real behavior rather than to an idealized transcript. `Evaluator::lastCtx_` (updated
once, in `evalChildren`'s per-statement loop, plus at user-function/function-literal body entry)
is `error()`'s only way to reach a locals snapshot for its own `errorBreak` hook call, since `error()`
itself — called from dozens of builtin resolve functions — doesn't take an `EvalContext` parameter;
mirrors the reference's own `self._last_ctx` fallback, including the same "only as fresh as the last
statement boundary" caveat. Profiling's `ProfileSiteKey` is a `std::tuple` used as a `std::map` key
(free `operator<` from comparable fields) rather than requiring a custom `unordered_map` hash
function — a table that's never more than a few thousand entries doesn't need hashing to stay fast.

**Known gap closed (post-audit, three gaps fixed together)**: three previously-documented gaps —
`object()`'s argument merge order, 3MF DEFLATE reading, and `use <file>` resolution — were all
closed in one pass after a fresh "anything still unimplemented?" audit. Each below:

1. **`object()`'s argument merge order.** The reference applies positional/named arguments to the
   result dict in their exact call-site interleaved order (its single `_resolve_args` dict mixes
   int/str keys); `builtinObject` used to apply every positional merge first, then every named
   override, via the shared `CallArgs` (which splits positional/named into two separate containers
   and can't represent that interleaving). Fixed by giving `builtinObject` its own entry point
   (`function_builtins.hpp`) that takes the *raw* argument list and evaluates it itself, called
   directly from `evalFunctionCall` (`user_calls.cpp`) — bypassing `resolveArgs`/`evalBuiltinFunction`
   entirely for this one builtin, specifically so arguments are evaluated exactly once (resolving
   into a `CallArgs` first and then re-evaluating from the raw list would double-evaluate every
   argument expression, corrupting anything with a side effect, e.g. `rands()`) — see
   `tests/test_function_builtins.cpp`'s `ObjectBuiltin.ArgumentsEvaluatedExactlyOnce` for the
   regression test that specifically guards this. `PositionalAfterNamedOverridesInCallSiteOrder`/
   `NamedAfterPositionalOverridesInCallSiteOrder` cover the actual interleaving fix.
2. **3MF DEFLATE reading, later extended to writing too.** `zip_stored.cpp`'s `extractEntry` used
   to throw for any non-STORED entry. Fixed by reusing `stb_image.h`'s own raw-inflate decoder
   (`stbi_zlib_decode_noheader_buffer`, already a project dependency for `surface()`'s PNG loader —
   ZIP's "method 8" stream is exactly the same raw-DEFLATE-no-wrapper format PNG's IDAT chunks use,
   so no new dependency was needed) rather than linking `miniz` as the plan's §9 table originally
   assumed. `tests/test_zip_stored.cpp` covers the raw ZIP-level round trip (a hand-built DEFLATE
   entry with precomputed compressed bytes, since at the time no DEFLATE-capable ZIP *writer*
   existed in this project to generate a fixture from); manually cross-checked end-to-end too,
   importing a real `zipfile.ZIP_DEFLATED`-written 3MF (Python stdlib) through the full CLI
   pipeline. **Writing followed shortly after** (a separate, later ask): `writeDeflateZip()`
   (`zip_stored.hpp`/`.cpp`), reusing `stb_image_write.h`'s `stbi_zlib_compress()` (already vendored
   for surface()'s own PNG-writing test fixture, so again zero new dependency) — its output is a
   full zlib stream (2-byte header + raw DEFLATE + 4-byte Adler-32 trailer), so `deflateRaw()` just
   strips both ends to get the raw DEFLATE bytes ZIP's method 8 wants, mirroring the exact inverse
   trick `stbi_zlib_decode_noheader_*` already uses on the read side. Falls back to STORED per-entry
   when compression doesn't actually shrink the entry (tiny/incompressible data), matching what any
   real ZIP writer does. `writeStoredZip`/`writeDeflateZip` now share one `writeZipEntries()` header/
   central-directory/EOCD writer, parameterized by a `PlacedEntry` (method + already-compressed-or-
   not data + crc + uncompressed size) rather than duplicating that ~70-line body.
   `writeThreeMf` (`export.cpp`) switched from `writeStoredZip` to `writeDeflateZip`, bringing 3MF
   output to parity with the Python reference (whose own `export.py` already used
   `zipfile.ZIP_DEFLATED` — this had been a C++-port-only gap, not a real behavioral divergence from
   the reference). `STB_IMAGE_WRITE_IMPLEMENTATION` needed relocating: previously only defined in
   `tests/test_surface.cpp` (for a PNG test fixture), now defined once in `zip_stored.cpp` instead
   (part of the actual library, needed by `writeDeflateZip` itself) with `test_surface.cpp` reduced
   to a plain `#include` of the header — two translation units both defining
   `STB_IMAGE_WRITE_IMPLEMENTATION` in the same link target would have been an ODR violation.
   `tests/test_zip_stored.cpp` gained `WriteDeflateRoundTrip`/`WriteDeflateActuallyShrinksCompressibleData`/
   `WriteDeflateFallsBackToStoredForTinyEntry`/`WriteDeflateMultipleEntries`; manually cross-checked
   a real exported 3MF's compressed entries read back correctly (and are meaningfully smaller) via
   Python's stdlib `zipfile` too, not just this project's own reader.
3. **`use <file>` statement resolution.** Needed genuinely new infrastructure, not just a bug fix:
   `include/openscad_cpp_evaluator/eval_use.hpp`+`src/eval_use.cpp`'s `resolveUseScopes()`, a
   line-by-line port of the reference's `resolve_use_scopes` (each top-level `UseStatement` is
   replaced by the used file's own module/function declarations only — not its top-level
   geometry/variables, keeping the current file's variable namespace isolated from the used file's
   globals; nested `use`s aren't re-exported; a missing `use` target is silently skipped exactly
   like real OpenSCAD, while any other failure, e.g. a syntax error in the used file, is logged and
   evaluation continues). The real design problem was ownership, not logic: Python's dynamic typing
   lets injected declaration nodes be shared by reference across an unbounded `use`-chain for free;
   C++'s `unique_ptr`-owned AST can't represent that without either cloning nodes (a large surface
   this port doesn't have -- no `clone()` exists for any of the 66 AST node kinds) or tracking
   ownership explicitly. Solved with `ResolvedUseScopes`: a struct that owns every used file's AST
   and its own root `Scope` in flat pools (recursively absorbing a nested `use`'s own pools), while
   `.processedNodes`/`.ownNodesFiltered` are plain non-owning `const ASTNode*` lists over those pools
   plus the caller's own already-owned AST — the caller keeps `ResolvedUseScopes` alive for as long
   as it evaluates against `.rootScope`/`.processedNodes`, mirroring Python's GC-backed reference
   lifetime with an explicit ownership pool instead. This needed two small, purely-additive raw-
   pointer overloads added to the `openscad_cpp_parser` submodule itself (`buildScopes`/
   `collectHoistedDeclarations`, `scope_builder.hpp`/`.cpp` + `api.hpp`/`scope.cpp` — existing
   `unique_ptr`-vector overloads untouched, both call a shared raw-pointer core) since scope-building
   there is hard-wired to `vector<unique_ptr<ASTNode>>` ownership throughout; verified with the
   parser's own standalone `ctest` suite (extra `ScopeBuilderBasics.RawPointerOverloadCombinesTwoOwningVectors`
   case) before touching the evaluator side. `Evaluator::evaluate()`/`resolveTree()` similarly
   gained raw-pointer overloads (mirroring `evalChildren`'s existing dual-overload shape exactly,
   sharing one body via a private `resolveTreeImpl`/`evaluateImpl` template so the ~40-line
   `evaluate()` body isn't duplicated) so a caller can evaluate `ResolvedUseScopes::processedNodes`
   directly. Wired into the CLI (`tools/cli/cli_lib.cpp`): `getASTFromFile()` → `resolveUseScopes()`
   → `evaluate(used.processedNodes, ctx)`, `use`-error messages routed through the same `out` stream
   as `echo_fn`, matching the reference's `cli.py` (`resolve_use_scopes(nodes, args.input, print)`)
   exactly. Cross-checked byte-for-byte against the Python reference on real fixture files (isolation,
   nested-non-reexport, missing-file-silent-skip, syntax-error-logs-and-continues) before writing
   `tests/test_use.cpp`'s permanent versions — no existing Python test suite covered this feature at
   all (`resolve_use_scopes`/`UseStatement` had zero hits in `openscad_evaluator`'s own `tests/`), so
   these were verified against real reference *behavior*, not ported from an existing test.

**Known gap closed in Phase 3, worth remembering for Phase 5+**: every builtin resolve function must
call `resolveCallArgs` (call_args.hpp), never bare `resolveArgs`, or a `$`-prefixed named call
argument (`circle(r=2, $fn=64)`) silently fails to reach `ctx.dyn`. Caught by a geometric-correctness
test (actual triangle/area/volume, not just "does it throw"); a tree-shape-only test would not have.
Keep writing that kind of test.

## Build & Test

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Requires the same toolchain as `openscad_cpp_parser`: CMake ≥3.24, a C++20 compiler, Bison ≥3.5
(macOS: `brew install bison`, the submodule's own CMakeLists auto-detects Homebrew's), Flex.
GoogleTest is fetched automatically via `FetchContent`.

Run a single test binary or filter:

```bash
build/tests/oscad_eval_tests
build/tests/oscad_eval_tests --gtest_filter='FormatNumber.*'
```

## Repo-specific CMake note

`openscad_cpp_parser/src/CMakeLists.txt` exposes its public headers via
`${CMAKE_SOURCE_DIR}/include`. `CMAKE_SOURCE_DIR` is fixed to the *outermost* project's root for
the whole build — once that submodule is vendored as our subdirectory, the expression resolves to
**this** repo's `include/`, not the submodule's own. Don't "fix" this by editing the submodule;
the root `CMakeLists.txt` here already works around it by appending the correct path to the
`openscad_cpp_parser` target's `PUBLIC` include directories after `add_subdirectory()`.

## Architecture

Every C++ file under `src/`/`include/openscad_cpp_evaluator/` is a port of a well-delimited region
of the Python reference's `evaluator.py` — see the plan file's §6 (Project layout) for the mapping,
and `/Users/gminette/dev/git-repos/openscad_evaluator/docs/evaluator.md` for the authoritative
behavior reference (scope rules, the two-pass resolve/generate CSG-tree design, every builtin's
exact semantics, error-message formats). When porting a function, check whether it's meant to
mirror the Python reference exactly — most of the fiddlier logic (number formatting, nan/inf/bool
handling in arithmetic, the closure-detection rule for `call_ctx` vs `child_ctx`) is a deliberate
verbatim port, not an independent design. Deliberate divergences are called out in code comments —
grep for `ponytail:`.

- `include/openscad_cpp_evaluator/value.hpp`, `src/value.cpp` — the `Value` variant (OpenSCAD's
  dynamic value type) and its free-function arithmetic/comparison helpers. See the plan's §1 for
  the type-mapping table and why there's no numpy-equivalent library.
- `include/openscad_cpp_evaluator/eval_context.hpp`, `src/eval_context.cpp` — `EvalContext`, the
  mutable state threaded through recursive evaluation. `dyn`/`let_`/`dynPositions`/`dynExplicit`
  are `shared_ptr`-wrapped, not plain value members — `withScope()` relies on that aliasing to
  give sibling statements in the same block correct assignment visibility (see the header comment
  before reworking this; it's not an arbitrary choice).
- `include/openscad_cpp_evaluator/eval_error.hpp`, `src/eval_error.cpp` — `EvalError` and the
  exact ERROR/TRACE string formatting real OpenSCAD produces.
- `include/openscad_cpp_evaluator/evaluator.hpp`, `src/expr_eval.cpp`, `src/stmt_eval.cpp`,
  `src/user_calls.cpp` — the `Evaluator` class. Expression/statement dispatch is a `switch` on
  `NodeKind` (matches `openscad_cpp_parser`'s own dispatch convention, not a visitor).
  `user_calls.cpp` holds the function/module call machinery: `bindArgs`, `applyDefaults`,
  `callCtxFor` (the closure-detection rule — walks the live call stack for "is this declaration
  lexically nested inside an already-active call," picking `childCtx()` vs `callCtx()`
  accordingly; read its doc comment before touching it, it's the second-trickiest mechanism in this
  codebase after the CSG tree stack), `evalUserModule`/`evalUserFunction`/`evalFunctionLiteral`,
  and `builtinChildren` (children()/children(N), deferred evaluation). `evalFunctionCall`'s
  precedence order (checked in this exact sequence): `import` (special-cased, its own
  module/expression-context split) → `isBuiltinFunctionName` (function_builtins.hpp — always wins
  if true) → user function lookup → function-literal *value* probe → "unknown function" warning.
- `include/openscad_cpp_evaluator/{colored_body,csg_node,dispatch}.hpp`, `src/csg_resolve.cpp`,
  `src/csg_generate.cpp` — the two-pass CSG pipeline. `Evaluator::resolveTree()` walks the AST with
  zero Manifold calls, building a persistent `CSGNode` tree via a push/call/pop stack
  (`Evaluator::evalModularCall`, mirroring the reference's `self._tree_stack` exactly — read its
  comment before touching the mechanism, it's the trickiest part of this pipeline);
  `Evaluator::generateTree()` then walks that tree bottom-up doing the actual Manifold work,
  consulting `ManifoldCache` (`manifold_cache.hpp`/`.cpp`, Phase 8) first when one is attached: a
  cache hit reuses `CSGNode::bodies` without recursing into that node's children at all (no wasted
  Manifold work re-deriving results that would just be discarded), a miss generates normally and
  stores the result keyed by `cacheKey()` — a self-delimiting, length-prefixed string encoding of
  `kind`/`isBuiltin`/canonicalized `params`/recursively-hashed `children` (deliberately excluding
  the AST pointer, so the same script re-parsed into a brand-new AST still hits). `CSGNode::
  uncacheable` (set during *resolve*, in `evalModularCall`/`buildTreeNode` — a before/after
  comparison of `Evaluator::noteRandsCall()`'s counter, propagated to parents via `any(child.
  uncacheable)` and, for a spliced children()/user-module call, explicitly onto every spliced
  sibling if the taint came from the splicing node's own resolve rather than a child's) forces a
  permanent cache miss for any subtree that called `rands()` while resolving, since such a
  subtree's resolved params aren't a pure function of its own content anymore (the actual `rands()`
  output also depends on every earlier `rands()` call's position in the whole evaluation's order).
  Builtins are registered in `src/builtins/registry.cpp` as `(name string) -> (ResolveFn,
  GenerateFn)` pairs — `dispatch.hpp` has the exact reasoning for `GenerateFn`'s `Evaluator&`
  parameter (provenance-table writes only, no `EvalContext`/dynamic-scope access, so the
  content-hash cache above stays sound: a generate call's output depends only on its own
  arguments/children, never on ambient evaluation state a cache key wouldn't capture).
  `Evaluator::evaluate()` also takes an optional `viewportParams` map (added after a test-suite
  parity review found it missing entirely): seeds arbitrary `$`-prefixed entries into `ctx.dyn`
  (e.g. `$vpt`/`$vpr`/`$vpd`/`$vpf` from a GUI host's current camera, or `$t` for animation time)
  before evaluation starts, via a direct `ctx.dyn` write that deliberately never touches
  `ctx.dynExplicit` — lets a caller tell "the script itself assigned this `$`-var"
  (`dynExplicit`) apart from "merely present because the caller seeded it" after `evaluate()`
  returns. Unlike the reference (whose own `ctx` is constructed *inside* `evaluate()`, forcing it to
  expose a separate `_root_ctx` attribute purely so callers/tests can inspect this after the fact),
  this port's `ctx` is always caller-owned already, so no equivalent escape hatch is needed — a
  caller just inspects the same `ctx.dyn`/`ctx.dynExplicit` it passed in.
- `include/openscad_cpp_evaluator/call_args.hpp`, `src/builtins/call_args.cpp` — shared
  positional/named argument resolution for builtins. **Every builtin resolve function must call
  `resolveCallArgs` (not bare `resolveArgs`)** and use its returned `.ctx` for anything evaluated
  afterward (children, `$fn` lookups) — that's what applies a `$`-prefixed named call argument
  (`sphere(r=2, $fn=64)`) to `ctx.dyn`. See the header comment; this was a real bug caught by a
  geometric-correctness test in Phase 3, not a hypothetical.
- `include/openscad_cpp_evaluator/segments.hpp`, `src/builtins/segments.cpp` — `$fn`/`$fa`/`$fs` ->
  circular-shape segment count (`fnSegments`/`fnSegmentsFromCtx`), used by sphere/cylinder/circle.
- `include/openscad_cpp_evaluator/css_colors.hpp`, `src/css_colors.cpp` — the 148-entry CSS/SVG
  color-name table (mechanically ported from the Python reference's own generated table) plus hex
  parsing, backing `color()`'s string-name argument.
- `src/builtins/{primitives_2d,primitives_3d,transforms,color,booleans,modifiers,control}.cpp` —
  one file per builtin group, each a `ResolveFn`/`GenerateFn` pair (or a few, where several builtin
  names share one pair — circle/square/polygon, translate/rotate/scale/mirror/multmatrix/resize,
  union/difference/intersection — matching the reference's own name-dispatch-inside-one-function
  shape). `transforms.cpp` decides 2D vs. 3D dispatch per-body at *generate* time (a transform's
  child type isn't known until then), carrying raw arguments through `CSGParams` via
  `callArgsToValue`/`valueToCallArgs` rather than resolving them early. `control.cpp` holds
  `children()`/`render()`/`breakpoint()` (thin wrappers around `Evaluator` methods) and
  `intersection_for` (the one control-flow-shaped construct that is *not* transparent in the CSG
  tree — its iterations combine via `^`, so it needs a real resolve/generate pair like a boolean
  op, unlike `for`/`if`/`let`/`echo`/`assert`, which `stmt_eval.cpp` handles directly since they
  never need their own `CSGNode`). `booleans.cpp` also hosts two small helpers shared well beyond
  boolean ops — `splitByRole`/`RoleSplit` (background/highlight/show_only splitting, declared in
  `builtins.hpp`, used by `topology.cpp`'s hull/minkowski too) and `toCrossSection` (unions every 2D
  child into one `CrossSection`, used by `extrude.cpp`/`roof.cpp`); `control.cpp` hosts
  `combineBodies` (3D-union-else-2D-union-else-empty, `intersection_for`'s own per-iteration combine
  step, reused by `extrude.cpp`'s `projection()`). `booleans.cpp`'s `generateCsg` also calls
  `attachTriColors` (a local helper, same file) on a real 3D boolean-merge result before returning:
  recovers each triangle's real originating color from `Evaluator::idToColor` (populated by
  `tagGenerated()` when each child was first generated, before being merged away) via Manifold's own
  per-triangle `runOriginalID`/`runIndex` provenance, filling `ColoredBody::triColors` only when the
  merged children's colors actually differ (the common single-color case stays a no-op, so it costs
  nothing and keeps following live color-theme changes the way an unset `.color` always has) — added
  after a test-suite parity review found this feature (the reference's `_attach_tri_colors`) had been
  missed entirely: `union()`-ing an opaque and a translucent body was silently collapsing to one flat
  color instead of preserving each part's own, matching real OpenSCAD's actual behavior once fixed
  (cross-checked triangle-color-count-and-values against the Python reference directly, not just
  “does it run”).
- `src/builtins/topology.cpp` — `hull()`/`minkowski()`. Both splice their children transparently in
  the CSG tree exactly like union/difference/intersection (resolve just evaluates them for the
  side effect); `hull()` picks an all-3D (`Manifold::Hull`) or all-2D (`CrossSection::Hull`) hull of
  the *foreground* (role-split) children, falling back to sections only when no child has a body;
  `minkowski()` only ever operates on 3D bodies (a 2D sibling among the foreground children is
  silently ignored, matching the reference exactly — no 2D `minkowski_sum` fallback the way hull
  has one). Both pass background/highlight/show_only bodies through untouched.
- `src/builtins/extrude.cpp` — `linear_extrude()`/`rotate_extrude()`/`projection(cut=)`. All three
  union every 2D child into one `CrossSection` via `toCrossSection` first. `rotate_extrude`'s
  segment count can't be resolved until generate time (it depends on the merged children's bounds,
  unknown at resolve) — resolve only caches `$fn`/`$fa`/`$fs` for generate to call `fnSegments()`
  with once the real max-x bound exists, mirroring the reference's own resolve/generate split here.
  `projection`'s non-cut path re-fills `Manifold::Project()`'s output with `FillRule::Positive` to
  clean up self-intersections Manifold's own projection can produce; the cut path uses
  `Manifold::Slice(0.0)` instead. `offset(r=)`/`offset(delta=, chamfer=)` (`resolveOffset`/
  `generateOffset`, also in this file) is the other CrossSection-transform-shaped builtin sharing
  `toCrossSection`; no children/generate-time-bounds dependency, so its resolve/generate split is
  the simple kind (segment count for `r=` IS resolvable early here, unlike `rotate_extrude`'s,
  since `offset`'s own radius argument is already known at resolve time).
- `src/builtins/roof.cpp` — `roof()`, built on Boost.Polygon's segment Voronoi diagram (see the
  `roof()` note above for the full story of why this replaced an earlier straight-skeleton
  approximation mid-Phase-6). `voronoiRoof` scales the merged cross-section's polygons into signed
  32-bit integer coordinates (`chooseScale`, headroom below 2^31 for Boost.Polygon's own internal
  robustness margin), constructs the Voronoi diagram over the resulting segments, walks it
  (`vdInnerFaces`, discretizing parabolic point-vs-segment bisector edges via `discretizeArc`) to get
  the roof surface as small polygon faces with a per-vertex height, then triangulates both those
  faces and the flat floor via `manifold::Triangulate` (handles holes/multi-contour natively — no
  hand-rolled ear-clipping in this file). The floor and every roof face are built from the *same*
  scaled-then-divided-back-by-`scale` coordinates specifically so their shared boundary vertices land
  on bit-identical doubles (a real subtlety ported faithfully from `roof_vd.cc`'s own comment on this
  — skip it and the mesh comes out non-manifold at the seam). Self-contained 2D geometry math here
  (`dot2`/`len2`/`norm2`, the parabola-canonical-frame rotation in `discretizeArc`) exists nowhere
  else in this codebase.
- `include/openscad_cpp_evaluator/export.hpp`, `src/export.cpp` — binary STL, OBJ, OFF, and 3MF
  writers (compose every body into one solid first; 3MF via `zip_stored.hpp`'s `writeDeflateZip`).
  `colored_body.hpp`/`.cpp`'s `toRenderableBodies()` (thin-extrudes any top-level 2D-only result to
  a 1e-3-unit-tall `flatPreview` Manifold) must run on a body list before handing it to any of these
  writers — none of them look at `ColoredBody::section` at all, only `.body`. `tools/cli/cli_lib.cpp`
  is the only current caller; a future GUI/renderer caller needs the same call before passing
  results to its own mesh consumer.
- `include/openscad_cpp_evaluator/function_builtins.hpp`, `src/builtins/function_builtins.cpp` —
  the math/string/list/type-check builtin *function* dispatch (`isBuiltinFunctionName`/
  `evalBuiltinFunction`), a completely separate namespace/table from `dispatch.hpp`'s builtin
  *modules*. One `NUMERIC_ONLY` bool-rejection guard shared by every function that needs it
  (`abs`/`sin`/`max`/`norm`/`cross`/... — bool is a distinct type from number, never silently
  coerced, unlike Python's bool-is-int); `object()` (an ordered-map constructor, needing no font
  support unlike its two `_BUILTIN_FN_NAMES` neighbors) and (Phase 7) `textmetrics()`/
  `fontmetrics()` (`builtinTextmetrics`/`builtinFontmetrics`, both thin wrappers over
  `text_metrics.hpp`'s `measureText`/`textAlignOffset` plus `Evaluator::fontProvider()`) are
  implemented here too.
- `include/openscad_cpp_evaluator/mesh_import.hpp`, `src/import/mesh_import.cpp` — STL (ASCII +
  binary, exact-match vertex welding)/OBJ/OFF/3MF mesh loaders, returning plain `(verts, tris)` —
  no `Evaluator`/`Value` dependency, so these are reusable from both `import()` contexts below.
- `include/openscad_cpp_evaluator/import_builtin.hpp`, `include/openscad_cpp_evaluator/
  zip_stored.hpp` (+ `src/zip_stored.cpp`), `src/builtins/import.cpp` — `import()`'s two faces:
  `resolveImport`/`generateImport` (registered in `registry.cpp` like any other builtin *module* —
  a geometry statement, STL/OBJ/OFF/3MF/DXF/SVG) and `importAsValue` (called directly from
  `user_calls.cpp`'s `evalFunctionCall`, special-cased *before* the `isBuiltinFunctionName` gate
  since `import` isn't in that table — returns a VNF `[[verts],[faces]]` for a mesh file, a Region
  `[[[x,y],...],...]` for DXF/SVG, or the JSON file's content as native values).
  `resolveFilePath()` (also declared here, defined in import.cpp) — resolves a file argument
  relative to the *source .scad file's own directory* — is shared by every file-reading builtin,
  not just import() (`surface.cpp` uses it too). `zip_stored.hpp` is a hand-rolled, dependency-free
  ZIP reader/writer (CRC32 + central-directory parsing) supporting both STORED and DEFLATE in both
  directions — `writeStoredZip`/`writeDeflateZip` (writing) and `extractEntry`'s method-0/method-8
  branches (reading) all via `stb_image.h`/`stb_image_write.h`'s own raw-inflate/deflate, no new
  dependency needed for either — see the "3MF DEFLATE reading, later extended to writing too" fix
  above for the full story.
- `include/openscad_cpp_evaluator/eval_use.hpp`, `src/eval_use.cpp` — `resolveUseScopes()`, `use
  <file>` statement resolution. A free function (not an `Evaluator` method), called by a caller
  (the CLI) *before* building an `EvalContext`/calling `evaluate()` — mirrors the reference's own
  `resolve_use_scopes` being outside the `Evaluator` class entirely, for the same reason: it
  operates on the raw AST/scope-building step, not evaluation. Returns a `ResolvedUseScopes` the
  caller must keep alive for the whole evaluation (owns every used file's AST + own root `Scope` in
  flat pools; `.processedNodes`/`.rootScope` are what the caller actually evaluates against). See
  the "`use <file>` statement resolution" entry above for the full design story (why this needed
  two small additive raw-pointer overloads in the `openscad_cpp_parser` submodule itself, and
  matching raw-pointer overloads of `Evaluator::evaluate()`/`resolveTree()`).
- `include/openscad_cpp_evaluator/surface_load.hpp`, `src/import/surface_load.cpp`,
  `src/builtins/surface.cpp` — `surface()`. `loadSurfaceHeights()` dispatches by extension to a
  `.dat` text parser (whitespace-separated numbers, `#` comments, file order reversed — first line
  = highest Y, matching OpenSCAD's own convention) or `stb_image.h` for PNG/JPEG/BMP/GIF (grayscale
  luminance mapped to 0-100, `invert=` flips the mapping); `resolveSurface`/`generateSurface`
  (`builtins/surface.cpp`) build the terrain solid (grid of top vertices at each height + a flat
  bottom cap + side walls) with vertex/triangle indexing kept identical to the reference's own for
  triangle-count parity.
- `include/openscad_cpp_evaluator/dxf_svg_import.hpp`, `src/import/dxf_import.cpp`,
  `src/import/svg_import.cpp` — hand-rolled, dependency-free 2D-contour loaders (no XML/DXF library
  pulled in, matching the plan's §9 dependency table). `loadDxfContours()` reads DXF's group-code
  text format directly (closed `LWPOLYLINE`/2D `POLYLINE` entities only, optional layer filter) --
  covers the same narrow entity set the reference's own `ezdxf`-based loader does, nothing more.
  `loadSvgContours()` pairs a minimal recursive-descent XML tree parser (open/close/self-closing
  tags, quoted attributes, comments/CDATA/prolog skipping, entity decoding — no namespace URI
  resolution, a tag's `prefix:` is stripped textually) with a close port of the reference's own
  path-command tokenizer/flattener (`M`/`L`/`H`/`V`/`C`/`S`/`Q`/`T`/`A`, both absolute and relative,
  cubic/quadratic Bezier flattening, full elliptical-arc endpoint-to-center parameterization) plus
  `rect`/`circle`/`ellipse`/`polygon`/`polyline` and nested `transform="matrix()/translate()/
  scale()/rotate()"` composition (including through `<g>` groups) — Y is flipped (SVG down →
  OpenSCAD up) at the point-transform step, `applyMat()` (named to avoid colliding with
  `std::apply` in an ADL lookup that broke the build once — see its own comment).
- `include/openscad_cpp_evaluator/font_provider.hpp` — the `FontProvider` abstract interface
  (`resolveFont`/`metrics`/`glyphMetrics`/`glyphOutline`), the injection seam that keeps
  `text()`/`textmetrics()`/`fontmetrics()` decoupled from any specific font backend (the plan's §6
  design — avoids hard-depending on the Python reference's Linux-only `fc-match` shell-out, since
  this library is meant to be embedded in a cross-platform Qt GUI app). `Evaluator`'s constructor
  takes an optional injected `shared_ptr<FontProvider>`; `Evaluator::fontProvider()` lazily
  constructs the built-in default (below) on first use otherwise.
- `include/openscad_cpp_evaluator/stb_font_provider.hpp`, `src/text/stb_font_provider.cpp` — the
  built-in `FontProvider` default: `stb_truetype.h` over the one bundled font
  (`resources/fonts/LiberationSans-Regular.ttf`, byte-identical to the Python reference's own
  bundled font). `resolveFont()` ignores its `spec` argument entirely and always returns that one
  font (see the "known gap" note above) — a real Qt host app supplies its own `QFontDatabase`/
  `QRawFont`-backed `FontProvider` instead of this class for real system-font matching.
- `include/openscad_cpp_evaluator/text_metrics.hpp`, `src/text/text_metrics.cpp` — the
  backend-agnostic text-layout math shared by `text()` and `textmetrics()`/`fontmetrics()`:
  `measureText()` (left-to-right glyph layout, ink-bbox/advance aggregation — mirrors
  `_measure_text` exactly, including the "unmapped codepoint skipped entirely, mapped-but-inkless
  codepoint still advances" distinction) and `textAlignOffset()` (halign/valign →
  translation offset, mirrors `_text_align_offset`). `utf8DecodeAll()` here decodes a UTF-8 byte
  string to full Unicode scalar values for per-codepoint `FontProvider` calls.
- `src/builtins/text.cpp` — `text()` itself: resolves layout via `text_metrics.hpp` at resolve
  time (so `resolveText`'s params carry only plain data — per-glyph codepoint+pen-x, scale, segment
  count, alignment offset — no `FontProvider`/`EvalContext` reference crossing into generate),
  then at generate time calls `FontProvider::glyphOutline()` per glyph, unions the resulting
  `CrossSection`s, and applies the alignment offset.
- `include/openscad_cpp_evaluator/debug_hooks.hpp`, `include/openscad_cpp_evaluator/profile.hpp`,
  `src/debug_profile.cpp` — the debugging/profiling injection seam. `DebugHookFn`/`ErrorBreakFn`/
  `ReturnHookFn` (bundled into one `DebugHooks` constructor parameter) mirror the reference's
  `debug_hook`/`error_break_fn`/`return_hook` callback contract; `Evaluator::checkDebug()` (called
  from `evalChildren`, `evalUserFunction`/`evalFunctionLiteral` body entry, and `breakpoint()`'s
  resolve) is the single choke point that invokes `debugHook` — see the "debugger" note above for
  why this port needs far fewer call sites than the reference's own dozens. `CallSiteProfile`/
  `ProfileResult` (`profile.hpp`) and `Evaluator::profileEnter`/`profileExit` (wrapped around every
  user module/function/function-literal call in `user_calls.cpp`) implement per-call-site
  self/cumulative timing with the recursion guard (`profileActive_`) and child-time propagation
  (`profileChildTime_`) that keep self-time slices disjoint and cumulative-time from ballooning past
  total wall time on a self-recursive call site.
- `include/openscad_cpp_evaluator/debug_repl.hpp`, `src/debug/debug_repl.cpp` — `DebugRepl`, a
  gdb-style REPL built entirely on the `DebugHooks` seam above (no special access to `Evaluator`
  internals): `break`/`delete`/`info breakpoints`/`list`/`run`/`quit`/`help` before running,
  `continue`/`step`/`next`/`finish`/`child`/`print`/`backtrace`/`list`/`break`/`delete`/`set`/`quit`/
  `help` while paused — a close, line-by-line port of the reference's own `_debug_repl.py`, not
  re-derived, since its step-into/step-over/step-out semantics already went through several rounds
  of real-world fixes there. Blocks synchronously on its own `std::istream&` from inside the hook
  itself (no worker thread) since the CLI's `evaluate()` call and the prompt are on the same thread.
  Its constructor takes `std::istream&`/`std::ostream&` (defaulted to `std::cin`/`std::cout` for
  real interactive use, stored as `in_`/`out_` member references) specifically so
  `tests/test_cli.cpp` can drive a whole `--debug` session with an `istringstream` of canned
  commands and assert against an `ostringstream` — no subprocess, no real terminal.
  `sourceLinesByOrigin_` (`mutable`, lazily populated via `linesFor()`) caches source lines per
  origin file, not just `sourcePath_` — a breakpoint/step landing inside a `use <file>`-injected
  function/module's own body is paused in a *different* file, and `listSource()` (the automatic
  display on a pause, and the explicit `list` command) must read *that* file's lines. Was a real
  bug (`listSource()` used to always read `sourcePath_`'s own lines regardless of where the
  debugger was actually paused) found during a fresh audit, and confirmed to be an identical bug in
  the Python reference's own `_debug_repl.py` — fixed in both places the same way; see
  `tests/test_cli.cpp`'s `CliDebugRepl.ListShowsSourceFromUseInjectedFileWhenPausedThere` and the
  reference's own `tests/test_cli.py`'s
  `test_list_shows_source_from_use_injected_file_when_paused_there`. Manually cross-checked byte-
  for-byte identical `--debug` transcripts between both CLIs on the same fixture afterward, same as
  every other debugger-behavior claim in this file.

**`child` (step-to-child), added after Phase 9**: neither this port's own `_debug_repl.py`-derived
REPL nor the Python reference's `_debug_repl.py` itself originally had this command — it turned out
to exist only one layer further upstream, in BelfrySCAD's own GUI debugger
(`src/belfryscad/window/debugger.py`'s `DebugSession`, "Step to Child" button/`⌃F11`), which
`_debug_repl.py`'s own module docstring already cites as the source its into/over/out semantics are
ported from, but `_debug_repl.py` had never picked up this one. Confirmed via the Python
reference's own `evaluator.py` first: `Evaluator._child_statement_positions`/
`_last_children_positions` (the evaluator-side plumbing `to_child`'s hit-test needs — "(origin,
line) for each top-level, non-declaration child of the currently-checked node") already existed
there, docstring-labeled for exactly this command, just never wired to a REPL command. Added to
both `_debug_repl.py` and this port the same way: pauses the first time control reaches one of the
paused call's own `{ ... }` children (wherever `children()`/`children(N)` forwards to them), or —
if the call never invokes `children()` at all — falls back to the same "call returned" depth-drop
safety net `finish`/step-out already uses, so it can never hang. This port's own equivalent plumbing
is `Evaluator::lastChildrenPositions()` (`evaluator.hpp`, computed by `checkDebug()`,
`debug_profile.cpp`, on every check — mirrors `_last_children_positions` being recomputed
unconditionally, not just when a debugger happens to be attached) and
`DebugRepl::attachEvaluator()` (`debug_repl.hpp`/`.cpp` — `DebugRepl` is constructed, and its
pre-run prompt run, *before* the `Evaluator` it'll be wired into exists, so this can't happen at
construction time the way the other hooks do; `cli_lib.cpp` calls it right after constructing the
`Evaluator`). Both ports needed the same non-obvious fix once actually tested end to end: the
snapshotted target positions must be run through the same origin-normalization
(`_resolve`/`resolveOrigin`, i.e. realpath) the hit-test's own `resolved` already goes through — a
raw, un-normalized origin (as `_last_children_positions` naturally stores it) can disagree with the
realpath'd form purely from a symlink (e.g. macOS's `/var` → `/private/var`) even for the
objectively same file, which silently broke the very first end-to-end test of this on both sides
before the fix. See `tests/test_cli.cpp`'s `CliDebugRepl.ChildStepsToChildrenCallForwardedStatement`/
`ChildFallsBackToCallReturnWhenChildrenNeverInvoked` and the reference's own `tests/test_cli.py`'s
`test_child_steps_to_children_call_forwarded_statement`/
`test_child_falls_back_to_call_return_when_children_never_invoked`.

**Ctrl+C-interrupts-a-running-eval, added right after `child`**: neither port had ANY SIGINT
handling before this — Ctrl+C during a running `evaluate()` just killed the process (C++'s
default `SIGINT` disposition) or raised an unhandled `KeyboardInterrupt` mid-AST-walk (Python's
own default), in both cases with no chance to inspect state. BelfrySCAD's own `DebugSession` has
the analogous "pause a running eval" feature already, but triggered by a GUI button (F5), not
Ctrl+C — its `_pause_requested`/`pause()` is the proven precedent this ports: a plain flag,
read-and-cleared inside the same hook, folded into the exact same `should_pause`/`shouldPause`
OR-chain breakpoints/steps already use (so the result behaves identically to hitting a
breakpoint — `print`/`backtrace`/`continue`/`step`/`child` all just work once paused this way).
This port's own piece: `DebugRepl::installInterruptHandler()` (`debug_repl.hpp`/`.cpp`) installs a
process-wide `std::signal(SIGINT, ...)` whose handler does nothing but
`pauseRequested_.store(true, ...)` on an `std::atomic<bool>` -- the async-signal-safe minimum,
since a signal handler can't safely do anything else (no I/O, no locks, no throwing) and can't
capture `this` either (a single "currently active `DebugRepl`" pointer bridges the free-function
handler back to the real instance; fine for this CLI's own single-`--debug`-session-per-process
usage, not something a hypothetical multi-instance embedder could rely on as-is).
`DebugRepl::requestPause()` is the same flag-setter, exposed directly and separately so
`tests/test_cli.cpp` can simulate "the user just pressed Ctrl+C" without a real OS signal (the
existing in-process istream/ostream test harness has no subprocess to deliver one to) — see
`CliDebugRepl.RequestPauseCausesNextDebugHookCallToPauseLikeABreakpoint`, which constructs a
`DebugRepl` directly and drives `debugHook()` by hand, isolated from `breakOnFirst_`/
`breakpoints_`/`stepHit` via a deliberately different origin, to prove specifically that
`requestPause()`'s own contribution to the OR-chain is what causes the pause. Manually verified
end to end with a *real* `SIGINT` (`kill -INT <pid>` against a running `--debug` session, both
this port's CLI and the Python reference's own) — both correctly print `"Interrupted at
<file>:<line>"` (a distinct message from `"Breakpoint hit at ..."`, so the user can tell an
unprompted interrupt apart from a breakpoint they set themselves) and the REPL keeps working
normally afterward. One accepted, documented quirk: a `SIGINT` that arrives while already blocked
on stdin at a prompt (not mid-`evaluate()`) still sets the flag, which then gets consumed on the
very next statement check once the user resumes, causing an immediate re-pause — harmless, not a
hang or crash, and not worth a more elaborate "only listen while genuinely running" state machine
for.

**Arrow-key command history in the debug REPL, added after Ctrl+C**: neither port's `DebugRepl`
had any line-editing before this — `std::getline`/`input()` on a raw terminal means the up/down
arrow keys just insert their own ANSI escape bytes into the line instead of recalling a previous
command. The Python reference's fix is a one-liner: `import readline` at the top of
`_debug_repl.py` hooks stdlib `input()` into GNU readline (or libedit, what Python's own `readline`
module actually links against on macOS) for arrow-key history/editing for the rest of the process,
wrapped in `try`/`except ImportError` since the module doesn't exist on Windows. This port has no
stdlib equivalent, so it vendors `yhirose/cpp-linenoise` (`linenoise.hpp`, header-only, BSD-2-
Clause, fetched via `CMakeLists.txt`'s existing `FetchContent` pattern) — a C++ port of
`antirez/linenoise` with *native* Windows console support (no ANSI-emulation layer needed, unlike
GNU readline's own Windows story), matching this project's three-platform CI matrix. Its own
`CMakeLists.txt` already guards its test/example targets behind
`CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR`, so `FetchContent_MakeAvailable` here only
exposes its `linenoise` INTERFACE target, no override needed (unlike Manifold's `MANIFOLD_TEST`).
`DebugRepl::readCommandLine()` (`debug_repl.cpp`) is the one seam both `runPrompt()`/`interact()`
now share: when `enableLineEditing()` was called, it uses `linenoise::Readline()` (note: returns
`true` on *quit* — Ctrl-C/Ctrl-D/EOF — inverted from `std::getline`'s own "true means got a line"
convention, so `readCommandLine()` flips it right there rather than leaking that surprise to
callers) plus `linenoise::AddHistory()` for non-empty lines; otherwise it falls back to the
existing `out_`/`std::getline(in_, ...)` path unchanged. `enableLineEditing()` is opt-in, called
only by `cli_lib.cpp` and only when `&in == &std::cin && &out == &std::cout` — linenoise reads/
writes the real process stdin/stdout file descriptors directly, not the injected `in_`/`out_`
themselves, so it would silently do nothing useful (or worse, fight over the real terminal while a
test expects a scripted istringstream) if ever turned on for `tests/test_cli.cpp`'s injected-stream
harness; that harness is untouched, still exercising the plain `getline` path exclusively. Verified
with a real interactive session, not just "it compiles": since neither port's own in-process test
harness can simulate real keystrokes/arrow-key bytes hitting an actual raw terminal, both were
checked end to end with a Python `pty.openpty()`-driven harness (writes raw bytes to a real pty
master/slave pair, not a piped file) sending `print width`, an up-arrow (`\x1b[A`), and Enter,
confirming the recalled line actually re-submits (`$1 = 10` then `$2 = 10` on the Python side,
matching `print width` re-echoed on this port's side before Enter). One real pitfall the
verification harness itself hit, worth remembering for any future terminal-facing test: `pty.
openpty()` leaves the pty's window size zeroed, which sends linenoise's own `get_columns()` down
its `ioctl(TIOCGWINSZ)`-failed fallback path (querying the terminal with a cursor-position escape
sequence and blocking on the response) — a real terminal always reports a nonzero size so never
takes this path, but the synthetic pty hung indefinitely until the harness explicitly set a window
size via `TIOCSWINSZ`. Not a bug in this port or in the vendored library, purely an artifact of
under-specifying a fake terminal.

**`--profile FILENAME`, added after arrow-key history**: `Evaluator`'s `profiling`/`profileResult`
instrumentation (Phase 9) had no CLI surface at all before this — neither port's CLI ever
constructed `Evaluator` with profiling on, nor printed anything from the result. `cli_lib.cpp` now
parses `--profile FILENAME` the same space-separated way as `-o`/`--format` (not `--profile=X` --
this hand-rolled parser doesn't support `=`-embedded values for any flag, so adding one just for
this flag would be inconsistent with every other flag here; Python's `argparse` accepts both forms
for free, a harmless cross-CLI syntax asymmetry that falls out of each side's own pre-existing
parser, not a deliberate design choice). Passing `--profile` sets `Evaluator`'s `profiling` ctor
argument to `true` and, once `evaluate()` returns, writes a plain-text report to `FILENAME` via
`formatProfileReport()` (a local helper in `cli_lib.cpp`'s anonymous namespace) — a summary
(`resolveTime`/`generateTime`/`totalTime`/`unattributedTime`) followed by one row per call site,
sorted by `selfTime` descending with a deterministic tie-break (`callOrigin`, then `callLine`, then
`name` — `ProfileResult::callSites`' own storage order isn't self-time order, so this needs an
explicit sort). Ported identically, same column layout and tie-break rule, to the Python
reference's own `_format_profile_report()` in `cli.py`, so a report generated by either CLI on the
same script has the same shape (the actual timing numbers naturally differ run to run — real
wall-clock measurement, not something either port tries to make deterministic). Opening the output
path for writing can fail (bad directory, permissions); this returns exit code 1 with an
`error: ...` message on `err`, matching every other file-write failure this CLI already handles
(mesh export's own `ValueError`/`ImportError` catch).
- `examples/minimal_debugger.cpp` — a self-checking, runnable demonstration of the `DebugHookFn`
  seam alone (trace every statement, stop at a chosen line, override a variable via the hook's
  returned `mods`), a close port of the reference's own `examples/minimal_debugger.py`.
  `examples/manifold_cache_reuse.cpp` similarly ports `examples/manifold_cache_reuse.py`: two
  `evaluate()` calls sharing one `ManifoldCache`, `minkowski()` of two dense spheres wrapped in a
  `translate()` whose own offset is the only thing that changes between them, asserting the second
  (cache-warm) call is faster — measured ~60x on this machine, since the whole expensive `minkowski`
  subtree is served from cache untouched. Both built by default (`BUILD_EXAMPLES` CMake option) and
  registered as plain `add_test(COMMAND <binary>)` `ctest` cases (self-checking via `assert()`/
  nonzero exit, no gtest wrapper needed) — `build/examples/minimal_debugger`,
  `build/examples/manifold_cache_reuse`.
- `tools/cli/cli_lib.hpp`, `tools/cli/cli_lib.cpp` — `runCli(args, in, out, err)`, the CLI's actual
  argument-parsing/evaluate/export/`--debug`-wiring logic, extracted out of `main()` into an
  in-process testable function (injectable streams, `args` excludes argv[0]) so
  `tests/test_cli.cpp` can call it directly — mirrors the reference's own `cli.main(argv)` shape
  (a plain function pytest calls directly with `capsys`/monkeypatched `input()`, not a
  subprocess-spawning test). Built as its own small static lib (`tools/cli/CMakeLists.txt`'s
  `oscad_cli_lib` target) specifically so the test binary can link it without pulling in `main()`
  itself. `openscad-cpp-evaluator <file.scad> -o <file.{stl,obj,off,3mf}> [--format ...]
  [--debug]`, format inferred from the output extension (or `--format`) like the Python reference's
  own CLI. `--debug` constructs a `DebugRepl` (above) over the same `in`/`out` streams `runCli`
  itself was given, runs its pre-run prompt, and (if the user didn't quit before `run`) wires its
  three methods into `Evaluator`'s `DebugHooks`. Uses `Evaluator::evaluate()` (not a direct
  `resolveTree()`+`generateTree()` call, a fix made alongside Phase 9 — the direct-call form the
  CLI used through Phase 8 skipped `evaluate()`'s own top-level `!`/show_only filter) so
  profiling/debugging's own bookkeeping resets happen at the right point too.
- `tools/cli/main.cpp` — a 3-line wrapper: builds `args` from `argv`, calls `runCli(args)` with the
  real `std::cin`/`std::cout`/`std::cerr`, returns its exit code.
