"""Tests for the Python-binding surface added on top of the already-tested
C++ engine (ManifoldCache/csg_tree, profiling, return_hook, dyn/dyn_explicit
readback -- see bindings/module.cpp). The underlying algorithms have full
gtest coverage (tests/test_manifold_cache.cpp, test_profiling.cpp,
test_debug_hooks.cpp, test_viewport_params.cpp); this file only exercises the
nanobind plumbing that exposes them to Python, which has no coverage of its
own otherwise.

No test framework: plain functions + assert, run via `python
tests/test_python_bindings.py`. This project has no existing Python test
infrastructure (pure C++/gtest via CMake/ctest) -- adding pytest just for
this would be a new dependency for a handful of checks; wired into
wheels.yml's CIBW_TEST_COMMAND instead, which already builds+installs the
package on every release platform.
"""
import sys
import tempfile
from pathlib import Path

from openscad_cpp_evaluator import Evaluator, ManifoldCache, format_csg_tree


def _write(src: str) -> str:
    f = tempfile.NamedTemporaryFile(suffix=".scad", mode="w", encoding="utf-8", delete=False)
    f.write(src)
    f.close()
    return f.name


def test_manifold_cache_reuse_across_evaluates():
    path = _write("cube([10, 20, 30]);")
    cache = ManifoldCache()
    ev1 = Evaluator(manifold_cache=cache)
    bodies1, _ = ev1.evaluate(path)
    ev2 = Evaluator(manifold_cache=cache)
    bodies2, _ = ev2.evaluate(path)
    assert len(bodies1) == len(bodies2) == 1
    m1, m2 = bodies1[0].body.to_mesh(), bodies2[0].body.to_mesh()
    assert m1.vert_properties.shape == m2.vert_properties.shape
    assert m1.tri_verts.shape == m2.tri_verts.shape
    cache.clear()  # must not raise


def test_csg_tree_shape_and_format():
    path = _write("""
        difference() {
            union() { square(100); circle(100); }
            union() { square(50); circle(50); }
        }
    """)
    ev = Evaluator()
    ev.evaluate(path)
    assert len(ev.csg_tree) == 1
    root = ev.csg_tree[0]
    assert root.kind == "difference"
    assert len(root.children) == 2
    assert root.children[0].kind == "union"
    assert {c.kind for c in root.children[0].children} == {"square", "circle"}

    dump = format_csg_tree(ev.csg_tree)
    assert dump.startswith("difference()")
    assert "square(" in dump and "circle(" in dump
    assert "union()" in dump


def test_csg_tree_empty_for_direct_evaluate_bypass_is_not_applicable():
    # evaluate() always populates csg_tree (it's resolveTree()+generateTree()
    # internally) -- a fresh Evaluator() starts with an empty csg_tree before
    # any evaluate() call, mirroring the reference's own "populated only
    # after evaluate() runs" contract.
    ev = Evaluator()
    assert ev.csg_tree == []


def test_profiling_records_user_function_call_site():
    path = _write("function double(x) = x * 2; y = double(21); cube(y);")
    ev = Evaluator(profile=True)
    ev.evaluate(path)
    assert ev.profile_result is not None
    sites = {s.name: s for s in ev.profile_result.call_sites}
    assert "double" in sites
    assert sites["double"].call_count == 1
    assert sites["double"].kind == "function"
    assert ev.profile_result.resolve_time >= 0.0
    assert ev.profile_result.total_time >= ev.profile_result.resolve_time


def test_profiling_off_by_default():
    path = _write("cube(1);")
    ev = Evaluator()
    ev.evaluate(path)
    assert ev.profile_result is None


def test_return_hook_fires_for_user_function():
    path = _write("function double(x) = x * 2; y = double(21); cube(y);")
    returns = []

    def debug_hook(line, depth, forced=False, expr_level=False, expr_depth=0, origin=None, get_frames=None,
                   generate_partial=None, get_children_positions=None):
        return ("continue", {})

    def return_hook(name, value, depth):
        returns.append((name, value, depth))

    ev = Evaluator(debug_hook=debug_hook, return_hook=return_hook)
    ev.evaluate(path)
    assert returns == [("double", 42.0, 1)]


def test_generate_partial_during_a_live_pause():
    # Two statements: by the time the hook fires on line 2 (forced=True,
    # the breakpoint() call), cube(1) has already fully resolved and
    # generated once via the normal resolve pass -- but sphere(5) on the
    # next line hasn't been reached yet. generate_partial() should return
    # geometry from whatever's ALREADY been resolved at that exact moment
    # (just the cube), proving it reads live in-progress state, not a
    # snapshot from a completed evaluate().
    path = _write("cube(1);\nbreakpoint();\nsphere(5);")
    seen_partial_counts = []

    def debug_hook(line, depth, forced=False, expr_level=False, expr_depth=0, origin=None, get_frames=None,
                   generate_partial=None, get_children_positions=None):
        if forced:
            bodies = generate_partial()
            seen_partial_counts.append(len(bodies))
        return ("continue", {})

    ev = Evaluator(debug_hook=debug_hook)
    bodies, _ = ev.evaluate(path)
    assert seen_partial_counts == [1]  # only the cube, at the moment breakpoint() paused
    assert len(bodies) == 2  # both cube and sphere in the final, complete result


def test_generate_partial_on_an_empty_tree_returns_empty_not_an_error():
    # At the very first statement's forced pause, nothing has resolved yet --
    # generate_partial() must handle an empty tree gracefully (empty list),
    # not raise. (A genuine GenerateFn exception mid-partial-render is real
    # error-propagation behavior too -- see generatePartialTrampoline's own
    # doc comment in module.cpp -- but isn't exercised here: this backend's
    # builtins are permissive about invalid dimensions, same as real OpenSCAD,
    # so there's no reliable way to force one from a script.)
    path = _write("breakpoint();\ncube(1);")
    seen = []

    def debug_hook(line, depth, forced=False, expr_level=False, expr_depth=0, origin=None, get_frames=None,
                   generate_partial=None, get_children_positions=None):
        if forced:
            seen.append(generate_partial())
        return ("continue", {})

    ev = Evaluator(debug_hook=debug_hook)
    ev.evaluate(path)
    assert seen == [[]]


def test_get_children_positions_reports_module_call_block_children():
    # wrapper()'s own { cube(1); } block is what children() inside wrapper's
    # body would forward control to -- get_children_positions() should
    # report that block's own child statement's (origin, line) the moment
    # the debug hook checks the wrapper() call node itself (forced=False,
    # first hook call at line 4 -- the top-level ModularCall).
    path = _write("module wrapper() { children(); }\nwrapper() {\n  cube(1);\n}")
    seen = []

    def debug_hook(line, depth, forced=False, expr_level=False, expr_depth=0, origin=None, get_frames=None,
                   generate_partial=None, get_children_positions=None):
        positions = get_children_positions()
        if positions:
            seen.append((line, positions))
        return ("continue", {})

    ev = Evaluator(debug_hook=debug_hook)
    ev.evaluate(path)
    assert seen, "expected at least one hook call where get_children_positions() was non-empty"
    line, positions = seen[0]
    assert line == 2  # the wrapper() call itself
    assert any(p[1] == 3 for p in positions)  # cube(1)'s own line inside the block


def test_dyn_explicit_distinguishes_seeded_from_script_assigned():
    path = _write("$fn = 72; cube(1);")
    ev = Evaluator()
    ev.evaluate(path, {"$vpt": [1.0, 2.0, 3.0]})
    assert ev.dyn["$fn"] == 72.0
    assert "$fn" in ev.dyn_explicit
    # $vpt was seeded via viewport_params, not assigned by the script itself.
    assert ev.dyn.get("$vpt") == [1.0, 2.0, 3.0]
    assert "$vpt" not in ev.dyn_explicit
    # $fa was never touched at all -- still present (default-seeded), still not explicit.
    assert "$fa" in ev.dyn
    assert "$fa" not in ev.dyn_explicit


def test_tri_colors_populated_for_a_real_multicolor_csg_merge():
    # Found via BelfrySCAD's own renderer crashing on ColoredBody.tri_colors
    # missing entirely -- the C++ core (Evaluator::attachTriColors,
    # booleans.cpp) already computes this, bodyToDict() just never exposed
    # it. union()-ing an opaque cube with a translucent sphere forces a real
    # merge with two distinct triangle colors.
    path = _write("union() { color([1,0,0,1]) cube(10); "
                   "color([0,1,0,0.5]) translate([5,5,5]) sphere(8); }")
    ev = Evaluator()
    bodies, _ = ev.evaluate(path)
    assert len(bodies) == 1
    tri_colors = bodies[0].tri_colors
    assert tri_colors is not None
    num_tris = bodies[0].body.to_mesh().tri_verts.shape[0]
    assert tri_colors.shape == (num_tris, 4)
    # Two distinct colors actually present, not e.g. all-default filler.
    assert len({tuple(row) for row in tri_colors}) >= 2


def test_tri_colors_is_none_for_an_ordinary_single_color_body():
    path = _write("cube(1);")
    ev = Evaluator()
    bodies, _ = ev.evaluate(path)
    assert len(bodies) == 1
    assert bodies[0].tri_colors is None


def test_role_defaults_to_normal_and_reflects_modifiers():
    # Found in the same pass as tri_colors: bodyToDict() never exposed
    # ColoredBody.role either, crashing the renderer on any real render
    # (it reads cb.role for every body, not just modified ones).
    path = _write("cube(1);\n#sphere(1);\n%cylinder(h=1,r=1);\n!cube(2);\n")
    ev = Evaluator()
    bodies, _ = ev.evaluate(path)
    roles = sorted(b.role for b in bodies)
    # show_only (!) present anywhere -> only show_only + highlight bodies
    # survive evaluate()'s own filter (see anyShowOnly in csg_resolve.cpp).
    assert roles == ["highlight", "show_only"]


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failures = []
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except Exception as e:
            failures.append((t.__name__, e))
            print(f"FAIL {t.__name__}: {e}")
    print(f"\n{len(tests) - len(failures)}/{len(tests)} passed")
    if failures:
        sys.exit(1)


if __name__ == "__main__":
    main()
