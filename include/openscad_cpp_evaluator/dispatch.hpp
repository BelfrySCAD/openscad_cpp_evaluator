#pragma once

#include "openscad_cpp_evaluator/csg_node.hpp"
#include "openscad_cpp_evaluator/eval_context.hpp"

#include "openscad_cpp_parser/ast/module_instantiation.hpp"

#include <string_view>
#include <unordered_map>
#include <vector>

namespace oscadeval {

class Evaluator;

// Resolve step: parses a ModularCall's arguments (and, for a builtin with
// children -- transforms, booleans -- recursively resolves them via
// Evaluator::evalChildren, purely for the tree-building side effect: the
// return value here is unused, since real bodies don't exist until
// generate) into a plain-data CSGParams. Mirrors the reference's
// _resolve_X(node, ctx) -> dict.
using ResolveFn = CSGParams (*)(Evaluator&, const oscad::ModularCall&, EvalContext&);

// Generate step: does the actual Manifold/CrossSection work, reading
// pre-resolved params and this node's own already-generated children (via
// flattenCsgTree). No EvalContext access -- deliberately dynamic-scope-free,
// matching the reference (this is what makes a later phase's content-hash
// cache, ManifoldCache, sound: a generate call's output depends only on its
// arguments/children, not on ambient evaluation state). Mirrors
// _generate_X(params, children, node) -> list[ColoredBody].
//
// The `Evaluator&` here is narrower than that: only primitive-construction
// generate functions (cube, sphere, ...) use it, and only to write into
// Evaluator::idToNode/idToColor via tagGenerated() -- the provenance tables
// that make WYSIWYG picking possible, populated exactly once per freshly
// constructed Manifold (a ReserveIDs()-backed originalID). Transform/
// boolean generate functions ignore it entirely: a transform preserves its
// input body's existing IDs, and a boolean's result mesh inherits its
// operands' IDs through Manifold's own provenance tracking -- neither
// mints anything new. This doesn't reintroduce a cacheability problem: the
// Evaluator reference is used for a side-effect table write, never to
// *read* anything that would make the same (params, children) produce a
// different returned body.
using GenerateFn = std::vector<ColoredBody> (*)(Evaluator&, const CSGParams&, const std::vector<std::unique_ptr<CSGNode>>&,
                                                 const oscad::ASTNode&);

// Keyed by the *runtime call name string* ("cube", "union", ...), not
// NodeKind -- every builtin/user-module ModularCall shares one NodeKind
// regardless of which name it calls, so a string-keyed table is the correct
// match for what's actually being dispatched on here (mirrors the
// reference's _RESOLVE_DISPATCH/_GENERATE_DISPATCH exactly). Built once at
// static init in src/builtins/registry.cpp; plain function pointers, no
// std::function -- every builtin is a free function, no captures needed.
const std::unordered_map<std::string_view, ResolveFn>& resolveDispatch();
const std::unordered_map<std::string_view, GenerateFn>& generateDispatch();

// Every parameter name a builtin accepts, for the unexpected-argument
// warning (see eval_error.hpp). Returns nullptr for a name with no entry,
// which suppresses the check rather than warning about everything.
//
// Each list is the UNION of real OpenSCAD's own Parameters::parse
// declaration for that builtin (2022.08.22 -- the authority on what warns
// upstream) and any extra name this port itself reads via getArg, so this
// port never warns about an argument it goes on to honour.
const std::vector<std::string>* builtinParamNames(const std::string& name);

// builtinParamNames + warnUnexpectedArgs for one builtin module call. Must
// be invoked from EVERY path a builtin call can take: evalModularCall
// (interpreter, and Op::NativeStatement) plus the bytecode VM's own
// Op::PushBuiltinWrap/Op::PushCsgWrap handlers, which bypass
// evalModularCall entirely for the transform/color/hull/extrude/CSG family.
// Silently does nothing for a node that isn't a ModularCall (a `#`/`%`/`!`
// modifier shares BuiltinWrapSite but carries no arguments).
void warnUnexpectedBuiltinArgs(Evaluator& ev, const oscad::ASTNode& callNode);

// Concatenates every top-level node's already-generated `.bodies` (does NOT
// recurse into `.children` -- a parent's `.bodies` is already the fully
// combined result of its children). Mirrors the reference's
// flatten_csg_tree().
std::vector<ColoredBody> flattenCsgTree(const std::vector<std::unique_ptr<CSGNode>>& tree);

// Same, over just tree[begin, begin+count) -- lets a boolean-op generate
// function (union/difference/intersection) flatten one top-level
// statement's worth of children at a time, per its resolve step's
// group_sizes bookkeeping (see Evaluator::currentTreeFrameSize()),
// without needing ownership or a copy of the whole `tree` vector.
std::vector<ColoredBody> flattenCsgTree(const std::vector<std::unique_ptr<CSGNode>>& tree, size_t begin, size_t count);

} // namespace oscadeval
