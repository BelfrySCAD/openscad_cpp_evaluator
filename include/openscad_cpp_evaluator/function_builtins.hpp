#pragma once

#include "openscad_cpp_evaluator/call_args.hpp"

namespace oscad {
class ASTNode;
} // namespace oscad

namespace oscadeval {

class Evaluator;

// True for every math/string/list/type-check/version/parent_module/object/
// textmetrics/fontmetrics builtin *function* name -- a completely separate
// namespace from the builtin *modules* (cube, translate, ...) in
// dispatch.hpp's resolveDispatch/generateDispatch. A name in this set
// always wins over a same-named user function (see evalFunctionCall in
// user_calls.cpp), matching the reference's _eval_function_call precedence
// exactly (name in _BUILTIN_FN_NAMES is checked *before* the user-function
// lookup, not after). Mirrors Evaluator._BUILTIN_FN_NAMES.
bool isBuiltinFunctionName(const std::string& name);

// Dispatches one of the names above, except object() -- see builtinObject
// below, called directly from evalFunctionCall instead. `args` is the plain
// resolved argument bag (resolveArgs, not resolveCallArgs -- none of these
// accept $-prefixed named-argument overrides). Mirrors the reference's
// _math_fns dispatch table plus the textmetrics()/fontmetrics() special
// cases (implemented on top of FontProvider, text_metrics.hpp).
// One id per name evalBuiltinFunction handles (NOT the same list as
// isBuiltinFunctionName -- "object" is deliberately absent, since it is
// special-cased to builtinObject() before this function is reached). Public
// so a CALL SITE can resolve its callee once, at compile time
// (CompiledChunk::CallSite::builtinId), instead of hashing the name string
// on every single call.
enum class BuiltinFnId {
    None, // not a name evalBuiltinFunction handles
    TextMetrics, FontMetrics, Abs, Sign, Ceil, Floor, Round, Sqrt, Ln, Log, Exp, Sin, Cos, Tan,
    Asin, Acos, Atan, Atan2, Max, Min, Pow, Norm, Cross, Rands, Concat, Len, Str, Chr, Ord,
    IsUndef, IsNum, IsBool, IsString, IsList, IsFunction, IsObject, Search, Lookup, HasKey,
    Version, VersionNum, ParentModule, DxfDim, DxfCross, SupportedFeature, LinearSolve,
};

// `name` -> its id, or None. The one place that hashes the name; everything
// on a hot path should call this once at compile time and keep the answer.
BuiltinFnId builtinFnIdFor(const std::string& name);

Value evalBuiltinFunction(Evaluator& ev, const std::string& name, const CallArgs& args, const oscad::ASTNode& node);

// The same dispatch with both of its name lookups already done: `id` from
// builtinFnIdFor and `declaredParams` from builtinParamNames (dispatch.hpp),
// which for all but a couple of builtins is null. `name` survives only
// because diagnostics quote it. The string-taking overload above is just
// this one with the two lookups performed first, and remains the entry point
// for the AST interpreter, where there is no call site to cache them on.
Value evalBuiltinFunctionResolved(Evaluator& ev, BuiltinFnId id, const std::vector<std::string>* declaredParams,
                                   const std::string& name, const CallArgs& args, const oscad::ASTNode& node);

// object(a=1, b=2, ...) -- an ordered string-keyed map merging positional
// (an existing object's entries, or a list of [key,value] pairs) and named
// arguments, later arguments winning over earlier ones for the same key --
// in their exact call-site interleaved order, mirroring the reference's
// single _resolve_args dict (which mixes int/str keys in one table).
// Unlike every other builtin *function*, this one takes the raw argument
// list and evaluates it itself rather than going through the shared
// CallArgs (which splits positional/named into two containers and so can't
// represent their interleaving) -- called directly from evalFunctionCall,
// bypassing evalBuiltinFunction/resolveArgs entirely so arguments are
// evaluated exactly once (resolving into a CallArgs first and then
// re-evaluating from the raw list here would run every argument expression
// twice -- wrong for anything with a side effect, e.g. rands()).
Value builtinObject(Evaluator& ev, const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                     const oscad::ASTNode& node);

// The shared merge core builtinObject wraps: given ALREADY-EVALUATED
// (name-or-nullopt, Value) pairs in exact call-site order, merges them the
// same way (a named pair always setKey()s directly; a positional pair
// spreads an object's own entries or a list-of-[key,value]-pairs, same
// rules as above). Public so the bytecode VM's compiled object() call site
// (Op::CallFn's isBuiltin branch, bytecode_vm.cpp) can replay this exact
// logic against its own already-evaluated (argNames[i], args[i]) pairs
// (already in call-site order by construction -- see CallSite::argNames)
// without re-deriving the merge rules or touching raw AST nodes.
Value mergeObjectArgs(Evaluator& ev, const std::vector<std::pair<std::optional<std::string>, Value>>& evaluated,
                       const oscad::Position* pos);

} // namespace oscadeval
