#pragma once

#include "openscad_cpp_evaluator/eval_context.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include "openscad_cpp_parser/ast/expression.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace oscadeval {

class Evaluator;

// A ModularCall/PrimaryCall's evaluated arguments, split into positional
// (indexed by source position, 0-based) and named. Mirrors the Python
// reference's single _resolve_args() dict, which relies on Python dicts
// allowing mixed int/str keys in the one table; C++ needs two containers.
struct CallArgs {
    std::unordered_map<int, Value> positional;
    std::unordered_map<std::string, Value> named;
};

// Evaluates every argument expression against `ctx`. Mirrors
// Evaluator._resolve_args.
CallArgs resolveArgs(Evaluator& ev, const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx);

struct ResolvedCallArgs {
    CallArgs args;
    EvalContext ctx; // == the input ctx, unless a $-prefixed named arg overrode it (see below)
};

// resolveArgs() plus $-prefixed named-argument dynamic-context overrides
// (e.g. `sphere(r=2, $fn=64)`, `circle(r=2, $fn=64)`): any named argument
// starting with '$' is merged into a *new* EvalContext's `dyn` (via
// EvalContext::childCtx's newDyn parameter) rather than left sitting inert
// in the returned CallArgs, so a builtin that reads $fn/$fa/$fs from ctx
// (fnSegmentsFromCtx) sees the override. Mirrors
// Evaluator._resolve_call_args exactly. Every builtin resolve function
// should call this instead of resolveArgs() directly (matching the
// reference -- every migrated _resolve_X does) and use the returned .ctx,
// not the original `ctx`, for anything evaluated afterward (children,
// $fn lookups, ...).
ResolvedCallArgs resolveCallArgs(Evaluator& ev, const std::vector<std::unique_ptr<oscad::Argument>>& arguments,
                                  EvalContext& ctx);

// Named lookup takes precedence over a positional value at the same slot --
// mirrors Evaluator._get_arg exactly, including that apparent quirk (a call
// site could in principle supply both `cube(10, size=20)`; the named one
// wins). `pos = std::nullopt` for an argument with no positional slot at
// all (e.g. sphere's `d`, only ever named -- a later phase's need, plumbed
// through now since the signature shape matters more than early callers).
Value getArg(const CallArgs& args, std::optional<int> pos, const std::string& name, Value defaultValue = Value{});

// Encodes a CallArgs as a single Value (positional -> a list indexed 0..N,
// named -> an object) so a resolve function can carry a call's raw
// arguments across into CSGParams (which only holds Value) for a generate
// function to re-derive its own getArg() lookups from -- needed by any
// builtin whose 2D/3D dispatch (or other per-child-type behavior) can't be
// decided until generate time, when the actual body/section type is known
// (see builtins/transforms.cpp). Round-trips exactly for any CallArgs
// produced by resolveArgs(); a gap in positional indices (not possible
// from resolveArgs() itself, but not assumed away here either) fills with
// undef.
Value callArgsToValue(const CallArgs& args);
CallArgs valueToCallArgs(const Value& v);

// Both Argument subtypes (PositionalArgument/NamedArgument) carry an
// `.expr` member at the same conceptual slot; this reaches it without the
// caller needing to know/switch on which kind it is. Used by assert()'s
// expression form, which (unlike its statement form) indexes raw
// arguments positionally rather than through getArg()/CallArgs -- mirrors
// the reference's own `raw[0].expr` access exactly.
const oscad::Expression* argExpr(const oscad::Argument& arg);

// All positional arguments in call order (0..max index), padding any gap
// with undef -- variadic builtins (max/min/concat/str) need every
// positional argument, not a fixed named slot.
std::vector<Value> allPositional(const CallArgs& args);

} // namespace oscadeval
