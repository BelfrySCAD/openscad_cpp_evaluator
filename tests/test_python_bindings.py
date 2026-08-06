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
                   generate_partial=None, get_children_positions=None, set_fast_continue=None):
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
                   generate_partial=None, get_children_positions=None, set_fast_continue=None):
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
                   generate_partial=None, get_children_positions=None, set_fast_continue=None):
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
                   generate_partial=None, get_children_positions=None, set_fast_continue=None):
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


# -- parse_ast: AST snapshot ---------------------------------------------


def test_parse_ast_spans_recover_original_number_text():
    # The whole point of exposing spans: a parsed NumberLiteral is a double,
    # so the source spelling is gone from `val` -- but the span still points
    # at it, letting a rewriter reuse the author's own text verbatim.
    from openscad_cpp_evaluator import parse_ast_string
    src = "translate([1.500, 2.250, 1e3]) cube(2);"
    vec = parse_ast_string(src)[0]["arguments"][0]["expr"]
    spellings = [src[e["position"]["start_offset"]:e["position"]["end_offset"]] for e in vec["elements"]]
    assert spellings == ["1.500", "2.250", "1e3"], spellings
    assert [e["val"] for e in vec["elements"]] == [1.5, 2.25, 1000.0]


def test_parse_ast_node_shapes():
    from openscad_cpp_evaluator import parse_ast_string
    ast = parse_ast_string("module m(a, b=2) { cube(a); } m(1, b=3) sphere(2);")
    decl, call = ast
    assert decl["kind"] == "ModuleDeclaration"
    assert decl["name"]["name"] == "m"
    assert [p["name"]["name"] for p in decl["parameters"]] == ["a", "b"]
    # An absent optional child is None, not a missing key.
    assert decl["parameters"][0]["default_value"] is None
    assert decl["parameters"][1]["default_value"]["val"] == 2.0
    assert call["kind"] == "ModularCall"
    assert [a["kind"] for a in call["arguments"]] == ["PositionalArgument", "NamedArgument"]
    assert call["arguments"][1]["name"]["name"] == "b"
    assert [c["kind"] for c in call["children"]] == ["ModularCall"]


def test_parse_ast_covers_every_node_kind_reachable_from_source():
    # Guards the exhaustive switch in ast_to_py.cpp: a node kind added to
    # the parser without a case here would produce a dict with only
    # kind/position, silently losing its fields.
    from openscad_cpp_evaluator import parse_ast_string
    src = """
    include <x.scad>
    use <y.scad>
    A = 1; B = "s"; C = true; D = undef; E = [0:2:10]; F = -A; G = !C; H = ~1;
    I = A+1-2*3/4%5^2; J = A<1 && A>2 || A<=3 && A>=4 && A==5 && A!=6;
    K = A&1 | 2 << 3 >> 4;
    L = C ? A : B; M = [1,2,3][0]; N = let(x=1) x; O = echo("e") 1;
    P = assert(true) 2; Q = function(a) a; R = [for (i=[0:3]) i*2];
    S = [for (i=[0:3]) if (i%2==0) i else -i];
    S2 = [for (i=[0:3]) if (i%2==0) i];              // bare if -> ListCompIf
    T = [for (i=0; i<3; i=i+1) i]; U = [for (v=[[1,2]]) each v];
    V = [let(z=2) for (i=[0:1]) z*i]; W = object(a=1).a;
    function fn(p) = p; module mod(m) { cube(m); children(); }
    mod(1) { sphere(2); }
    for (i=[0:1]) cube(1);
    intersection_for (i=[0:1]) cube(2);
    let (t=1) cube(t); echo("hi") cube(1); assert(true) cube(1);
    if (A>0) cube(1); else sphere(1);
    if (A>0) cube(1);                                 // bare if -> ModularIf
    !cube(1); #cube(1); %cube(1); *cube(1);
    """
    seen = set()

    def walk(n):
        if isinstance(n, dict) and "kind" in n:
            seen.add(n["kind"])
            pos = n["position"]
            assert pos["end_offset"] >= pos["start_offset"], n["kind"]
            for k, v in n.items():
                if k not in ("kind", "position"):
                    walk(v)
        elif isinstance(n, list):
            for x in n:
                walk(x)

    for node in parse_ast_string(src, True):
        walk(node)
    # Every kind above, minus the three only produced by comment
    # preservation on a file with comments in the right places.
    expected = {
        "IncludeStatement", "UseStatement", "Assignment", "Identifier", "NumberLiteral", "StringLiteral",
        "BooleanLiteral", "UndefinedLiteral", "RangeLiteral", "UnaryMinusOp", "LogicalNotOp", "BitwiseNotOp",
        "AdditionOp", "SubtractionOp", "MultiplicationOp", "DivisionOp", "ModuloOp", "ExponentOp",
        "BitwiseAndOp", "BitwiseOrOp", "BitwiseShiftLeftOp", "BitwiseShiftRightOp", "LogicalAndOp",
        "LogicalOrOp", "EqualityOp", "InequalityOp", "GreaterThanOp", "GreaterThanOrEqualOp", "LessThanOp",
        "LessThanOrEqualOp", "TernaryOp", "PrimaryCall", "PrimaryIndex", "PrimaryMember", "LetOp", "EchoOp",
        "AssertOp", "FunctionLiteral", "ListComprehension", "ListCompFor", "ListCompIf", "ListCompIfElse",
        "ListCompCFor", "ListCompEach", "ListCompLet", "FunctionDeclaration", "ModuleDeclaration",
        "ParameterDeclaration", "PositionalArgument", "NamedArgument", "ModularCall", "ModularFor",
        "ModularIntersectionFor", "ModularLet", "ModularEcho", "ModularAssert", "ModularIf", "ModularIfElse",
        "ModularModifierShowOnly", "ModularModifierHighlight", "ModularModifierBackground",
        "ModularModifierDisable",
    }
    missing = expected - seen
    assert not missing, f"node kinds not emitted: {sorted(missing)}"


def test_parse_ast_snapshot_outlives_the_parse():
    # It is a copy, not a view into parser memory -- the point of the
    # snapshot design. Holding it after many further parses (which reuse
    # and free the same arenas) must not corrupt it.
    from openscad_cpp_evaluator import parse_ast_string
    held = parse_ast_string("a = 1.500;")
    for i in range(200):
        parse_ast_string(f"b{i} = [for (j=[0:20]) j*{i}];")
    assert held[0]["expr"]["val"] == 1.5
    assert held[0]["name"]["name"] == "a"
    held[0]["expr"]["val"] = 99.0  # plain dicts: mutable, owned by the caller
    assert held[0]["expr"]["val"] == 99.0


def test_parse_ast_raises_parse_error_on_bad_syntax():
    from openscad_cpp_evaluator import ParseError, parse_ast_string
    try:
        parse_ast_string("cube(")
    except ParseError:
        return
    raise AssertionError("expected ParseError")


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
