"""Python facade for the C++ OpenSCAD evaluator.

Mirrors the surface of BelfrySCAD's vendored `belfryscad.engine.evaluator`
so it can be swapped in with minimal call-site changes: `Evaluator`,
`ColoredBody`, `EvalError`, `to_renderable_bodies`.

The mesh boundary is arrays-only: each body carries a `_BodyShim` that quacks
like a manifold3d body (`.is_empty()` / `.to_mesh()`), so the renderer reads
`cb.body.to_mesh().vert_properties/tri_verts/run_original_id/run_index`
exactly as before -- no cross-module Manifold objects are ever exchanged.
"""
from dataclasses import dataclass, field
from typing import Optional

from . import _openscad_cpp_evaluator as _ext
from ._openscad_cpp_evaluator import FastContinueSignal, ManifoldCache

__all__ = [
    "Evaluator", "ColoredBody", "EvalError", "ParseError", "OscObject", "parse", "to_renderable_bodies",
    "ManifoldCache", "CallSiteProfile", "ProfileResult", "format_csg_tree", "bodies_from_dicts",
    "FastContinueSignal", "parse_ast", "parse_ast_string",
]


class EvalError(Exception):
    """Raised for an OpenSCAD evaluation error (assert, type error, etc.)."""


class ParseError(Exception):
    """Raised for a syntax error. `str()` is the caret-pointing diagnostic
    ("Syntax error in {origin} at line L, column C:\\n..."), matching the
    format the editor's error-marking already parses."""


class _Position:
    """Duck-types the parser's node.position for completion / go-to-def."""
    __slots__ = ("line", "column", "origin", "start_offset", "end_offset")

    def __init__(self, line, column, origin, start_offset, end_offset):
        self.line = line
        self.column = column
        self.origin = origin
        self.start_offset = start_offset
        self.end_offset = end_offset


class _DeclNode:
    __slots__ = ("name", "position")

    def __init__(self, name, position):
        self.name = name
        self.position = position


class OscObject:
    """OpenSCAD ``object()`` value: an ordered string-keyed map. Interface-
    compatible with BelfrySCAD's own OscObject (`.data` / `.items()` / `.get`
    / iteration), so the debugger's value formatters and viewers accept it."""
    __slots__ = ("data",)

    def __init__(self, data):
        self.data = dict(data)

    def __iter__(self):
        return iter(self.data)  # keys, insertion order

    def __len__(self):
        return len(self.data)

    def get(self, key):
        return self.data.get(key)

    def items(self):
        return self.data.items()

    def keys(self):
        return self.data.keys()

    def __repr__(self):
        return f"OscObject({self.data!r})"


class _ScadValue:
    """Display wrapper for values with no direct Python analog (ranges,
    function literals): carries the OpenSCAD source text so the debugger's
    ``_fmt`` shows it cleanly via ``str()``."""
    __slots__ = ("text",)

    def __init__(self, text):
        self.text = text

    def __str__(self):
        return self.text

    def __repr__(self):
        return self.text


class RootScope:
    """Mirrors the parser's root Scope surface the editor uses: `.variables`/
    `.functions`/`.modules` name dicts (for completion) and `lookup_*`
    (for go-to-definition, returning a node with `.position`)."""

    def __init__(self, decls):
        self.variables = {}
        self.functions = {}
        self.modules = {}
        tables = {"variable": self.variables, "function": self.functions, "module": self.modules}
        for ns, name, start, end, line, column, origin in decls:
            tables[ns][name] = _DeclNode(name, _Position(line, column, origin, start, end))

    def lookup_variable(self, name):
        return self.variables.get(name)

    def lookup_function(self, name):
        return self.functions.get(name)

    def lookup_module(self, name):
        return self.modules.get(name)


class _CSGNode:
    """One resolved (and, after evaluate(), generated) node in the CSG tree --
    mirrors the reference's CSGNode dataclass. `bodies` is a list of already-
    converted ColoredBody dicts (see bodyToDict on the C++ side); `params` is
    plain Python data (numbers/strings/bools/lists/dicts/OscObject), never an
    AST pointer -- eagerly converted, no lifetime tie to the Evaluator that
    produced it. Used by format_csg_tree(); not otherwise part of the public
    API surface (no generate_tree()-on-a-subset support yet -- see the
    reference's own "Phase 3" partial-render note, not implemented here)."""
    __slots__ = ("kind", "params", "bodies", "is_builtin", "children")

    def __init__(self, kind, params, bodies, is_builtin, children):
        self.kind = kind
        self.params = params
        self.bodies = bodies_from_dicts(bodies)
        self.is_builtin = is_builtin
        self.children = children


@dataclass
class CallSiteProfile:
    """Aggregated profiling data for one call site. Mirrors the reference's
    CallSiteProfile -- see ProfileResult's own docstring."""
    kind: str            # "module" | "function"
    name: str
    caller_name: str
    call_origin: str
    call_line: int
    decl_origin: str
    decl_line: int
    call_count: int = 0
    self_time: float = 0.0
    cumulative_time: float = 0.0


@dataclass
class ProfileResult:
    """Whole-evaluate() profiling summary, populated when Evaluator(profile=True).
    `resolve_time` always equals sum(s.self_time for s in call_sites) +
    `unattributed_time`, so a UI's percentages honestly sum to 100%. Mirrors
    the reference's ProfileResult."""
    call_sites: list
    resolve_time: float
    generate_time: float
    total_time: float
    unattributed_time: float
    # Calling-context tree: a flat list of dicts, paths[0] the <toplevel>
    # root, linked by `parent`/`children` INDICES into this same list.
    #
    # Where call_sites aggregates a site over every path that reached it,
    # each node here is one call site on ONE path -- so `cuboid` called
    # from `bracket` and from `rail` are separate nodes with their own
    # times, and a report can say what a particular path cost rather than
    # only what a name cost in total.
    #
    # Keys: parent, children, kind, name, call_origin, call_line,
    # decl_origin, decl_line, call_count, self_time, cumulative_time.
    #
    # Recursion is folded: re-entering a site already on the path reuses
    # that node (call_count rises) instead of unrolling a node per level.
    # Defaults to empty so a ProfileResult built by older code still
    # constructs.
    paths: list = field(default_factory=list)


def _summarize_param(value, max_items: int = 6, max_len: int = 40) -> str:
    """Compact one-line repr for a _CSGNode.params value, used by
    format_csg_tree -- collapses long lists/dicts (e.g. polyhedron points,
    imported STL verts, surface() height grids) to "<... of N>" instead of
    dumping them in full. Mirrors the reference's _summarize_param."""
    import numpy as np
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        if len(value) > max_items:
            kind = "tuple" if isinstance(value, tuple) else "list"
            return f"<{kind} of {len(value)}>"
        return "[" + ", ".join(_summarize_param(v) for v in value) + "]"
    if isinstance(value, dict):
        if len(value) > max_items:
            return f"<dict of {len(value)}>"
        return "{" + ", ".join(f"{k!r}: {_summarize_param(v)}" for k, v in value.items()) + "}"
    text = repr(value)
    return text if len(text) <= max_len else text[:max_len - 1] + "…"


# Params keys never shown in format_csg_tree's per-node summary, regardless
# of kind -- pure internal bookkeeping or already represented structurally
# elsewhere in the tree. Mirrors the reference's _DUMP_HIDDEN_PARAM_KEYS.
_DUMP_HIDDEN_PARAM_KEYS = frozenset({"color", "op", "name", "group_sizes"})

# Auto-generated tessellation data (not user-authored) for every kind except
# polyhedron, where the equivalent data *is* the user's own points/faces.
# Mirrors the reference's _DUMP_TESSELLATION_KEYS.
_DUMP_TESSELLATION_KEYS = frozenset({"verts", "tris", "tri_arr"})

# Display-only key renames. Mirrors the reference's _DUMP_KEY_RENAMES.
_DUMP_KEY_RENAMES = {"segs": "$fn"}


def _format_call_args(args: dict) -> str:
    """Render a resolved-args-shaped dict ({0: v0, 1: v1, 'name': v, ...} --
    positional args keyed by index, named args keyed by name) as OpenSCAD
    call-argument syntax. Mirrors the reference's _format_call_args."""
    parts = []
    for k, v in args.items():
        if isinstance(k, int):
            parts.append(_summarize_param(v))
        else:
            parts.append(f"{k}={_summarize_param(v)}")
    return ", ".join(parts)


def format_csg_tree(tree: list, indent: int = 0) -> str:
    """Human-readable recursive dump of a resolved CSG tree (list of
    _CSGNode) -- kind and a compact params summary. Used by a "Dump CSG Tree
    to Console" command. Mirrors the reference's format_csg_tree, including
    its indent-offset convention (+1 unit for every non-root line) and its
    deliberate omission of a generated-body count (a ManifoldCache hit skips
    recursing into that ancestor's children entirely -- see
    _DUMP_HIDDEN_PARAM_KEYS's neighboring reference doc comment for the full
    rationale)."""
    lines = []
    pad = "  " * (indent + 1) if indent > 0 else ""
    for node in tree:
        shown = {
            k: v for k, v in node.params.items()
            if k not in _DUMP_HIDDEN_PARAM_KEYS
            and not (k in _DUMP_TESSELLATION_KEYS and node.kind != "polyhedron")
        }
        parts = [
            _format_call_args(v) if k == "args" and isinstance(v, dict)
            else f"{_DUMP_KEY_RENAMES.get(k, k)}={_summarize_param(v)}"
            for k, v in shown.items()
        ]
        params_str = ", ".join(parts)
        lines.append(f"{pad}{node.kind}({params_str})")
        if node.children:
            lines.append(format_csg_tree(node.children, indent + 1))
    return "\n".join(lines)


def parse(path: str) -> RootScope:
    """Parse a .scad file with the C++ parser; return its root scope (top-level
    declarations) for the editor. Raises ParseError on a syntax error."""
    try:
        decls = _ext.parse_decls(path)
    except Exception as e:
        raise ParseError(str(e)) from e
    return RootScope(decls)


def parse_ast(path: str, include_comments: bool = False) -> list:
    """Parse a .scad file; return its AST as a list of plain nested dicts.

    Each node is `{"kind": str, "position": {...}, **fields}`, where fields
    are that kind's own (e.g. NumberLiteral has "val"; ModularCall has
    "name"/"arguments"/"children"). Child nodes nest as dicts, child lists
    as lists, and an absent optional child is None.

    `position` carries `start_offset`/`end_offset`, which is the point of
    this API: they let a caller slice the ORIGINAL source text of any
    subexpression and reuse it verbatim, instead of re-serialising a parsed
    value and losing how it was written (`1.500` -> `1.5`, `1e3` -> `1000`).

    This is a SNAPSHOT, not a view of live parser memory: it is safe to
    keep, mutate, share across threads, and outlive the parse. That costs
    one walk per call, which is why it is a separate entry point rather
    than something evaluate() always pays for.

    Raises ParseError on a syntax error.
    """
    try:
        return _ext.parse_ast(path, include_comments)
    except Exception as e:
        raise ParseError(str(e)) from e


def parse_ast_string(code: str, include_comments: bool = False) -> list:
    """As `parse_ast`, but parses a source string. `position.origin` is
    "<string>" for every node. Raises ParseError on a syntax error."""
    try:
        return _ext.parse_ast_string(code, include_comments)
    except Exception as e:
        raise ParseError(str(e)) from e


class _MeshShim:
    """Duck-types a manifold3d mesh: the four arrays the renderer reads."""
    __slots__ = ("vert_properties", "tri_verts", "run_original_id", "run_index", "num_prop")

    def __init__(self, d):
        self.vert_properties = d["vert_properties"]
        self.tri_verts = d["tri_verts"]
        self.run_original_id = d["run_original_id"]
        self.run_index = d["run_index"]
        self.num_prop = d["num_prop"]


class _BodyShim:
    """Duck-types a manifold3d Manifold for the renderer's read path."""
    __slots__ = ("_mesh",)

    def __init__(self, d):
        self._mesh = _MeshShim(d)

    def is_empty(self) -> bool:
        return self._mesh.tri_verts.shape[0] == 0

    def to_mesh(self) -> _MeshShim:
        return self._mesh


@dataclass
class ColoredBody:
    body: object                       # _BodyShim (3D mesh); C++ already extrudes 2D sections
    color: Optional[tuple]             # RGBA 0-1, or None to follow the theme
    section: object = None             # kept for API parity (always None here)
    flat_preview: bool = False
    tri_colors: Optional[object] = None  # (numTri, 4) float32 ndarray, real multi-color CSG merges only
    role: str = "normal"               # "normal" | "highlight" | "background" | "show_only"


def bodies_from_dicts(body_dicts) -> list:
    """Convert raw body-dicts (the shape both evaluate()/debug_evaluate() and
    a debug hook's `generate_partial()` callable return) to ColoredBody
    instances. Shared by Evaluator.evaluate() and a debug session's own live
    partial-render (see debug_evaluate()'s `generate_partial` hook kwarg,
    module.cpp) so both paths convert identically."""
    return [ColoredBody(_BodyShim(d), d["color"], None, d["flat_preview"], d.get("tri_colors"),
                         d.get("role", "normal")) for d in body_dicts]


class Evaluator:
    """Drop-in for belfryscad.engine.evaluator.Evaluator, backed by C++.

    Unlike the Python evaluator (which took a pre-parsed AST), `evaluate`
    takes a source file PATH -- the C++ backend parses internally. echo/
    warning output is batched and replayed through `echo_fn` after the run.

    `manifold_cache`: opt-in ManifoldCache (see manifold_cache.hpp), shared
    across evaluate() calls on possibly-different Evaluator instances --
    construct one and pass the same instance into every render/debug
    Evaluator() a host creates.
    `profile`: opt-in per-call-site timing; after evaluate() returns,
    `self.profile_result` is a ProfileResult, or None if profile=False.
    `return_hook`: fires after a user function/function-literal call
    computes its result, before returning -- (name, value, depth). Only
    meaningful with debug_hook set (the plain evaluate() path never calls it).
    `fast_continue_signal`: an optional FastContinueSignal the caller keeps
    across the whole debug session and calls `.request()` on (e.g. from
    DebugSession.pause()/set_breakpoints(), BelfrySCAD's own debugger.py)
    any time a checkDebug() checkpoint skipped via hook-skippable fast-
    continue mode (see debug_hook's own `set_fast_continue(breakpoints,
    hook_skippable)` kwarg) needs to stop skipping and consult Python
    again -- there is no other way to reach a running debug_evaluate() call
    from outside a hook invocation, since it runs as one single blocking
    call with the GIL released for its duration. Only meaningful with
    debug_hook set; ignored otherwise.

    After evaluate() returns, `self.csg_tree` (list of _CSGNode, see
    format_csg_tree), `self.dyn` (dict of every currently-visible
    $-prefixed variable), and `self.dyn_explicit` (the subset the script
    itself assigned, vs. merely seeded via viewport_params) are also set.
    Unlike the reference (whose root EvalContext is constructed inside its
    own evaluate(), needing a separate `_root_ctx` escape hatch), this
    port's `dyn`/`dyn_explicit` are exposed directly -- no such indirection.
    """

    def __init__(self, echo_fn=None, debug_hook=None, error_break_fn=None, return_hook=None,
                 manifold_cache=None, profile=False, fast_continue_signal=None):
        self._echo_fn = echo_fn
        self._debug_hook = debug_hook
        self._error_break_fn = error_break_fn
        self._return_hook = return_hook
        self._manifold_cache = manifold_cache
        self._profile = profile
        self._fast_continue_signal = fast_continue_signal
        self.csg_tree = []
        self.profile_result = None
        self.dyn = {}
        self.dyn_explicit = set()

    def evaluate(self, source_path: str, viewport_params: Optional[dict] = None):
        vp = viewport_params or {}
        try:
            if self._debug_hook is not None:
                # Debugger path: callbacks fire live under the GIL; echo is
                # delivered through echo_fn, not batched.
                body_dicts, echoes, id_spans, dyn, dyn_explicit = _ext.debug_evaluate(
                    source_path, vp, self._debug_hook,
                    self._error_break_fn or (lambda *a, **k: None),
                    self._echo_fn or (lambda _m: None),
                    self._manifold_cache, self._return_hook, self._fast_continue_signal)
                self.csg_tree = []
                self.profile_result = None
            else:
                body_dicts, echoes, id_spans, csg_tree, profile_result, dyn, dyn_explicit = _ext.evaluate(
                    source_path, vp, self._manifold_cache, self._profile)
                if self._echo_fn:
                    for line in echoes:
                        self._echo_fn(line)
                self.csg_tree = csg_tree
                self.profile_result = profile_result
            self.dyn = dyn
            self.dyn_explicit = dyn_explicit
        except EvalError:
            raise
        except Exception as e:  # ParseError/EvalError from C++ arrive as RuntimeError
            raise EvalError(str(e)) from e
        bodies = bodies_from_dicts(body_dicts)
        # originalID -> a node with `.position` (start/end offsets) for WYSIWYG
        # picking and gizmo write-back.
        id_to_node = {
            oid: _DeclNode(None, _Position(line, column, origin, start, end))
            for oid, (start, end, line, column, origin) in id_spans.items()
        }
        return bodies, id_to_node


def to_renderable_bodies(bodies):
    """Identity: the C++ backend already extrudes 2D sections to thin 3D."""
    return bodies
