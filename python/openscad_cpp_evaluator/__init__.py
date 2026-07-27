"""Python facade for the C++ OpenSCAD evaluator.

Mirrors the surface of BelfrySCAD's vendored `belfryscad.engine.evaluator`
so it can be swapped in with minimal call-site changes: `Evaluator`,
`ColoredBody`, `EvalError`, `to_renderable_bodies`.

The mesh boundary is arrays-only: each body carries a `_BodyShim` that quacks
like a manifold3d body (`.is_empty()` / `.to_mesh()`), so the renderer reads
`cb.body.to_mesh().vert_properties/tri_verts/run_original_id/run_index`
exactly as before -- no cross-module Manifold objects are ever exchanged.
"""
from dataclasses import dataclass
from typing import Optional

from . import _openscad_cpp_evaluator as _ext

__all__ = ["Evaluator", "ColoredBody", "EvalError", "ParseError", "OscObject", "parse", "to_renderable_bodies"]


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


def parse(path: str) -> RootScope:
    """Parse a .scad file with the C++ parser; return its root scope (top-level
    declarations) for the editor. Raises ParseError on a syntax error."""
    try:
        decls = _ext.parse_decls(path)
    except Exception as e:
        raise ParseError(str(e)) from e
    return RootScope(decls)


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


class Evaluator:
    """Drop-in for belfryscad.engine.evaluator.Evaluator, backed by C++.

    Unlike the Python evaluator (which took a pre-parsed AST), `evaluate`
    takes a source file PATH -- the C++ backend parses internally. echo/
    warning output is batched and replayed through `echo_fn` after the run.
    debug_hook/error_break_fn are accepted for API parity and wired in the
    debugger phase.
    """

    def __init__(self, echo_fn=None, debug_hook=None, error_break_fn=None):
        self._echo_fn = echo_fn
        self._debug_hook = debug_hook
        self._error_break_fn = error_break_fn

    def evaluate(self, source_path: str, viewport_params: Optional[dict] = None):
        vp = viewport_params or {}
        try:
            if self._debug_hook is not None:
                # Debugger path: callbacks fire live under the GIL; echo is
                # delivered through echo_fn, not batched.
                body_dicts, echoes, id_spans = _ext.debug_evaluate(
                    source_path, vp, self._debug_hook,
                    self._error_break_fn or (lambda *a, **k: None),
                    self._echo_fn or (lambda _m: None))
            else:
                body_dicts, echoes, id_spans = _ext.evaluate(source_path, vp)
                if self._echo_fn:
                    for line in echoes:
                        self._echo_fn(line)
        except EvalError:
            raise
        except Exception as e:  # ParseError/EvalError from C++ arrive as RuntimeError
            raise EvalError(str(e)) from e
        bodies = [ColoredBody(_BodyShim(d), d["color"], None, d["flat_preview"]) for d in body_dicts]
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
