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
    # Without a `!` anywhere, every role passes through.
    path = _write("cube(1);\n#sphere(1);\n%cylinder(h=1,r=1);\n")
    bodies, _ = Evaluator().evaluate(path)
    assert sorted(b.role for b in bodies) == ["background", "highlight", "normal"]

    # With one, its subtree IS the model: the siblings go, highlighted ones
    # included (see rerootAtShowOnly in csg_resolve.cpp).
    path = _write("cube(1);\n#sphere(1);\n%cylinder(h=1,r=1);\n!cube(2);\n")
    bodies, _ = Evaluator().evaluate(path)
    assert [b.role for b in bodies] == ["show_only"]


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


def test_format_source_round_trips_and_is_idempotent():
    from openscad_cpp_evaluator import format_source

    out = format_source("module thing(a=1,b=[1,2,3]){translate([0,0,1])cube([a,a,a]);}\n")
    assert "module thing(a=1, b=[1, 2, 3])" in out, out
    # A modifier's children go on their own indented line.
    assert "translate([0, 0, 1])\n        cube([a, a, a]);" in out, out
    # Formatting formatted source changes nothing: the printer is the
    # parser's own, so its output parses back to the same tree.
    assert format_source(out) == out


def test_format_source_keeps_comments():
    from openscad_cpp_evaluator import format_source

    out = format_source("// keep me\ncube(1);\n")
    assert "// keep me" in out, out
    # ...and can be told not to.
    assert "// keep me" not in format_source("// keep me\ncube(1);\n", include_comments=False)


def test_format_source_honours_the_indent_width():
    from openscad_cpp_evaluator import format_source

    two = format_source("module m(){cube(1);}\n", indent=2)
    assert "\n  cube(1);" in two, two


def test_format_source_raises_parse_error_on_bad_input():
    from openscad_cpp_evaluator import ParseError, format_source
    try:
        format_source("module {{{")
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


def test_evaluate_generate_false_skips_geometry(tmp_path):
    """generate=False runs the script but builds no Manifold geometry.

    The point of the flag is a docs/smoke check that answers "does this
    script run?" without paying for the solids, matching what the reference
    does for `openscad -o out.term`.
    """
    from openscad_cpp_evaluator import Evaluator

    src = tmp_path / "m.scad"
    src.write_text("echo(\"ran\");\ndifference() { cube(10, center=true); sphere(6.4); }\n")

    echoes = []
    ev = Evaluator(echo_fn=echoes.append)
    bodies, _ids = ev.evaluate(str(src), {}, generate=False)

    # The language ran: the echo came out.
    assert any("ran" in e for e in echoes)
    # The geometry did not.
    assert list(bodies) == []
    assert ev.csg_tree == []

    # And the same file with generate left alone still produces geometry,
    # so the flag is what made the difference, not the script.
    echoes2 = []
    ev2 = Evaluator(echo_fn=echoes2.append)
    bodies2, _ids2 = ev2.evaluate(str(src), {})
    assert any("ran" in e for e in echoes2)
    assert len(list(bodies2)) > 0
    assert ev2.csg_tree != []


def test_evaluate_generate_false_still_reports_script_errors(tmp_path):
    """Resolve-only is still a real check: a script that fails, fails."""
    import pytest
    from openscad_cpp_evaluator import Evaluator, EvalError

    src = tmp_path / "bad.scad"
    src.write_text("assert(false, \"nope\");\ncube(1);\n")

    with pytest.raises(EvalError):
        Evaluator(echo_fn=lambda _m: None).evaluate(str(src), {}, generate=False)


def test_str_of_a_function_literal_is_its_source(tmp_path):
    """The reference prints a function literal as its own source, with every
    binary operator and ternary parenthesised. BOSL2's fnliterals tests
    compare these strings exactly, so each case here is a shape whose
    expected text was copied from the reference's own echo output.
    """
    from openscad_cpp_evaluator import Evaluator

    cases = [
        ("function(x) x",                          "function(x) x"),
        ("function(x, y) x + y",                   "function(x, y) (x + y)"),
        ("function() 42",                          "function() 42"),
        ("function(x) x + 1 * 2",                  "function(x) (x + (1 * 2))"),
        ("function(x) (x + 1) * 2",                "function(x) ((x + 1) * 2)"),
        # Unary is never wrapped, though its operand still is.
        ("function(x) -x",                         "function(x) -x"),
        ("function(a, b) -(a + b)",                "function(a, b) -(a + b)"),
        # Even as a call argument.
        ("function(a, b) f(a + b)",                "function(a, b) f((a + b))"),
        ("function(x) x[0]",                       "function(x) x[0]"),
        ("function(x) x.y",                        "function(x) x.y"),
        ("function(x) [1, 2, x]",                  "function(x) [1, 2, x]"),
        ("function(x) x ? 1 : 2",                  "function(x) (x ? 1 : 2)"),
        ("function(x, y = 3) x",                   "function(x, y = 3) x"),
        ("function(x) f(x, y = 2)",                "function(x) f(x, y = 2)"),
        ("function(x) let(a = 1) a + x",           "function(x) let(a = 1) (a + x)"),
        # A written step prints; an implicit one does not.
        ("function(x) [1:2:9]",                    "function(x) [1 : 2 : 9]"),
        ("function(a) [0:a]",                      "function(a) [0 : a]"),
        # A comprehension body is wrapped; a plain vector element is not.
        ("function(a) [for (i = [0:a]) i + 1]",    "function(a) [for(i = [0 : a]) ((i + 1))]"),
        ("function(a) [1, 2 + 3, a]",              "function(a) [1, (2 + 3), a]"),
        ("function(a) [each [a, 1]]",              "function(a) [each ([a, 1])]"),
        ("function(a) [for (i = [0:a]) if (i > 1) i else -i]",
         "function(a) [for(i = [0 : a]) (if((i > 1)) (i) else (-i))]"),
        # The C-style for is the odd one out: no space after the semicolons,
        # and its body is NOT wrapped.
        ("function(a) [for (i = 0; i < a; i = i + 1) i]",
         "function(a) [for(i = 0;(i < a);i = (i + 1)) i]"),
        ("function(a) assert(a > 0) a",            "function(a) assert((a > 0)) a"),
        ("function(a) echo(a) a",                  "function(a) echo(a) a"),
    ]
    src = tmp_path / "f.scad"
    src.write_text("\n".join(f"echo(str({expr}));" for expr, _ in cases) + "\n")

    echoes = []
    Evaluator(echo_fn=echoes.append).evaluate(str(src), {})
    got = [e[len("ECHO: "):] if e.startswith("ECHO: ") else e for e in echoes]

    assert len(got) == len(cases)
    for (expr, want), actual in zip(cases, got):
        assert actual == f'"{want}"', f"{expr}\n  want {want!r}\n  got  {actual!r}"


def _echoes(tmp_path, script):
    from openscad_cpp_evaluator import Evaluator
    src = tmp_path / "s.scad"
    src.write_text(script)
    out = []
    Evaluator(echo_fn=out.append).evaluate(str(src), {})
    return [e[len("ECHO: "):] if e.startswith("ECHO: ") else e for e in out]


def test_search_uses_the_documented_parameter_names(tmp_path):
    """BOSL2's in_list() passes num_returns_per_match and index_col_num --
    the names the manual documents. These used to be num_returns/index_col,
    which nothing binds, so both silently took their defaults."""
    got = _echoes(tmp_path, '''
        t = [[2,"foo"],[4,"bar"],[3,"baz"]];
        echo(search(["bar"], t, num_returns_per_match=1, index_col_num=1));
        echo(search(["bar"], t, 1, 1));
    ''')
    assert got == ["[1]", "[1]"], got


def test_two_empty_ranges_are_equal(tmp_path):
    """The reference compares element counts first, so any two empty ranges
    are equal whatever their bounds, and a NaN-stepped range counts as
    empty. BOSL2 defines is_nan(x) = (x != x), so a range that was not equal
    to itself made typeof() answer "nan" instead of "invalid".

    NaN itself is untouched: it is still unequal to itself, bare or in a
    list, exactly as the reference has it.
    """
    got = _echoes(tmp_path, '''
        n = 0/0;
        echo([5:1:0] == [10:1:0]);
        echo([0:n:1/0] == [5:1:0]);
        echo([0:1:5] == [0:1:5]);
        echo([0:1:5] == [0:1:6]);
        echo([5:1:0] == [0:1:5]);
        echo(n == n);
        echo([n] == [n]);
    ''')
    assert got == ["true", "true", "true", "false", "false", "false", "false"], got


def test_each_expands_strings_and_ranges(tmp_path):
    """`for` already expanded both; `each` handed them back whole. BOSL2's
    str_strip() tests `in_list(s[i], [each c])`, which never matched."""
    got = _echoes(tmp_path, '''
        echo([each "12"]);
        echo([each [0:2]]);
        echo([each [1,[2,3]]]);
        echo([each 5]);
        echo([each true]);
        echo([each undef]);
    ''')
    assert got == ['["1", "2"]', "[0, 1, 2]", "[1, [2, 3]]", "[5]", "[true]", "[]"], got


def test_strings_are_characters_not_bytes(tmp_path):
    """A string is a sequence of characters to a script. "aé—z" is 8 bytes
    and 4 characters, and the reference reports 4 everywhere -- len, index,
    for, each and search. Indexing used to hand back half of a multi-byte
    character, which then failed to decode: an error, not a wrong answer.
    """
    got = _echoes(tmp_path, '''
        s = "aé—z";
        echo(len(s));
        echo([s[0], s[1], s[2], s[3]]);
        echo([for (c = s) c]);
        echo([each s]);
        echo(ord(s[1]));
        echo(search("é", s));
        echo(s[4]);
        echo(len("plain ascii"));
    ''')
    assert got == [
        "4",
        '["a", "é", "—", "z"]',
        '["a", "é", "—", "z"]',
        '["a", "é", "—", "z"]',
        "233",
        "[1]",
        "undef",
        "11",
    ], got


def test_chr_of_a_multibyte_codepoint_has_length_one(tmp_path):
    """chr(8199) is a figure space -- three bytes, one character. BOSL2's
    echo_matrix pads with it and asserts len(char) == 1."""
    got = _echoes(tmp_path, '''
        echo(len(chr(8199)));
        echo(len(chr(0x2014)));
        echo(len(chr(65)));
    ''')
    assert got == ["1", "1", "1"], got


def test_degenerate_ranges_do_not_overflow_the_element_count(tmp_path):
    """A NaN range yields nothing, silently. An unbounded one is rejected with
    the reference's own count.

    Both used to go through a cast of an out-of-range double to size_t, which
    is undefined and which the platforms disagreed about: x86_64 wrapped to
    2^63 and warned "too many elements (9223372036854775809)", while arm64
    produced a small enough number that an infinite range iterated forever.
    BOSL2 reaches the NaN case through list_rotate() on an empty list, where
    ((n % 0) + 0) % 0 is NaN -- so this failed on Linux CI while passing on a
    developer's Mac.
    """
    from openscad_cpp_evaluator import Evaluator
    src = tmp_path / "r.scad"
    src.write_text('''
        n = 0/0;
        echo([for (i = [n : 1 : -1]) i]);
        echo([for (i = [0 : 1 : n]) i]);
        echo([for (i = [0 : 1 : 1/0]) 1]);
        echo([for (i = [0 : 1 : 1e300]) 1]);
        echo([for (i = [0 : 1 : 3]) i]);
    ''')
    msgs = []
    Evaluator(echo_fn=msgs.append).evaluate(str(src), {})
    echoes = [m[len("ECHO: "):] for m in msgs if m.startswith("ECHO: ")]
    warnings = [m for m in msgs if m.startswith("WARNING:")]

    assert echoes == ["[]", "[]", "[]", "[]", "[0, 1, 2, 3]"], echoes
    # Silent for NaN; the unbounded pair warn, with the reference's number.
    assert len(warnings) == 2, warnings
    assert all("too many elements (4294967295)" in w for w in warnings), warnings


def test_export_model_facade_accepts_split_components(tmp_path):
    """The Python facade's signature must track the nanobind binding's.

    They are two hand-written parameter lists for one call, and adding
    `split_components` to the binding alone left the facade dropping it --
    a TypeError the C++ suite cannot see, because it never goes through
    Python. Caught in practice, hence this.
    """
    from openscad_cpp_evaluator import Evaluator, export_model

    src = tmp_path / "pieces.scad"
    src.write_text("cube(10); translate([20,0,0]) cube(10); translate([40,0,0]) cube(10);")
    ev = Evaluator()
    ev.evaluate(str(src), {})

    def objects_in(path, **kwargs):
        export_model(str(path), ev.geometry, **kwargs)
        import zipfile
        with zipfile.ZipFile(path) as z:
            return z.read("3D/3dmodel.model").decode().count("<object ")

    # Default matches OpenSCAD: one object however many pieces.
    assert objects_in(tmp_path / "joined.3mf") == 1
    assert objects_in(tmp_path / "split.3mf", split_components=True) == 3


def test_strict_commas_reaches_the_parser(tmp_path):
    """Evaluator.evaluate(strict_commas=True) must survive both hand-written
    parameter lists -- the facade's and the nanobind binding's. The last
    argument added this way was dropped by the facade and no C++ test could
    see it (see test_export_model_facade_accepts_split_components)."""
    import pytest
    from openscad_cpp_evaluator import Evaluator, EvalError

    rejected = tmp_path / "call.scad"
    rejected.write_text("cube(1,);\n")
    accepted = tmp_path / "list.scad"
    accepted.write_text("a = [2, 4,];\ncube(a[0]);\n")

    # Off by default: both parse, as they do in current OpenSCAD.
    Evaluator(echo_fn=lambda _m: None).evaluate(str(rejected), {})
    Evaluator(echo_fn=lambda _m: None).evaluate(str(accepted), {})

    # On: a call's trailing comma is a syntax error, a list literal's is not
    # -- 2021.01 accepted the list.
    with pytest.raises(EvalError):
        Evaluator(echo_fn=lambda _m: None).evaluate(str(rejected), {}, strict_commas=True)
    Evaluator(echo_fn=lambda _m: None).evaluate(str(accepted), {}, strict_commas=True)


def test_strict_commas_does_not_leak_between_evaluations(tmp_path):
    """The parser mode is a scope object; a strict parse must not leave it on
    for the next caller."""
    import pytest
    from openscad_cpp_evaluator import Evaluator, EvalError

    src = tmp_path / "call.scad"
    src.write_text("cube(1,);\n")
    with pytest.raises(EvalError):
        Evaluator(echo_fn=lambda _m: None).evaluate(str(src), {}, strict_commas=True)
    Evaluator(echo_fn=lambda _m: None).evaluate(str(src), {})   # must not raise
