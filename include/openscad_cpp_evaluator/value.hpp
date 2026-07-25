#pragma once

#include "openscad_cpp_evaluator/osc_range.hpp"

#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace oscad {
class FunctionLiteral;
} // namespace oscad

namespace oscadeval {

struct ValueList;
struct ValueObject;

using ListPtr = std::shared_ptr<const ValueList>;
using ObjectPtr = std::shared_ptr<const ValueObject>;

// The OpenSCAD dynamic value type. `bool` is a distinct alternative from
// `double` throughout the evaluator -- never implicitly coerced, unlike
// Python's own bool-is-int -- see every free function below.
//
// A function-literal *value* (`g = function(x) x*2;`) is the AST node
// pointer itself, mirroring the Python reference's _expr_function_literal
// (`return node`) -- no separate closure-capture wrapper. Non-owning; valid
// as long as the parsed AST it points into is alive (see evaluator.hpp's
// ownership contract, added in a later phase).
using Value = std::variant<std::monostate, // undef
                            bool,
                            double, // OpenSCAD has one numeric type
                            std::string,
                            ListPtr,
                            OscRange,
                            ObjectPtr,
                            const oscad::FunctionLiteral*>;

// Defined after Value so both can hold Value by value -- the standard
// recursive-variant pattern (indirection through a forward-declared,
// heap-allocated aggregate).
struct ValueList {
    std::vector<Value> items;
};

// Insertion-ordered key/value pairs, not a map: object()'s iteration order
// and `==` are order-sensitive (doc: openscad_evaluator/docs/evaluator.md,
// the object() entry).
struct ValueObject {
    std::vector<std::pair<std::string, Value>> items;
};

// "undefined" | "bool" | "number" | "string" | "vector" | "object" -- used
// in "undefined operation (...)" comparison warnings. OscRange and
// FunctionLiteral values both fall through to "undefined", matching the
// Python reference's _osc_type_name exactly.
std::string oscTypeName(const Value& v);

// `==`/`!=`: bool is a distinct type from number (`1 == true` is false).
// Lists/objects recurse element-wise; object equality is order-sensitive
// (same keys in the same order, not just the same set).
bool oscEqual(const Value& a, const Value& b);

// Lenient numeric coercion for contexts where the reference implementation
// calls Python's float() unconditionally (range bounds, primitive
// arguments like cube's `size`) and would raise on a genuinely non-numeric
// input -- this port degrades to 0.0 instead of crashing. `bool` coerces to
// 0.0/1.0 (matches Python's float(True) == 1.0); everything else (string,
// list, object, range, undef, function) is 0.0.
double toDoubleLenient(const Value& v);

// OpenSCAD's/Python's truthiness rule for `&&`/`||`/`!`/`if`/ternary
// conditions: undef/false/0.0(-0.0)/""/empty-list/empty-object are falsy;
// everything else -- including NaN and Inf, unlike a naive `v != 0` gut
// check would suggest for those two -- is truthy. OscRange and
// FunctionLiteral values have no falsy state (mirrors Python: neither
// OscRange nor FunctionLiteral define __bool__/__len__, so the default
// "every object is truthy" applies).
bool truthy(const Value& v);

// `<`/`>`/`<=`/`>=` gate: true only for same-type number/number, string/
// string, vector/vector, or bool/bool pairs -- anything else is an
// "undefined operation" warning at the caller. This function only answers
// "is comparing these two meaningful at all"; it does not itself order
// values (ordering is expr_eval's job, a later phase).
bool oscComparable(const Value& a, const Value& b);

// Vector/scalar arithmetic, recursing into nested lists. `scalar`/`divisor`
// are always plain numbers at every real call site (the caller -- e.g.
// `_expr_mul` -- has already confirmed the non-list operand is numeric
// before dispatching here), so these take `double` rather than `Value`.
// Division by zero follows IEEE 754 (nan for 0/0, signed inf otherwise),
// matching OpenSCAD's own `/` operator.
Value scale(double scalarValue, const Value& value);
Value divScale(const Value& value, double divisor);

// Elementwise `+`/`-` between two lists (zip semantics: truncates to the
// shorter length when lengths differ), or the scalar/mismatched-type
// fallback (numeric + numeric, everything else -> undef). `vecAdd` rejects
// string operands explicitly (OpenSCAD has no string `+`, unlike Python's
// str concatenation); `vecSub` doesn't need the same guard since there's no
// other type subtraction would otherwise silently succeed on.
Value vecAdd(const Value& a, const Value& b);
Value vecSub(const Value& a, const Value& b);

// OpenSCAD's `*` between two lists: vector.vector -> scalar dot product,
// matrix.vector / vector.matrix -> vector, matrix.matrix -> matrix (mirrors
// numpy.dot's shape rules from the Python reference). Any non-numeric
// element, ragged matrix, or dimension mismatch -> undef.
//
// ponytail: a `bool` element inside either operand is treated as
// non-numeric (-> undef) here, unlike Python's incidental bool-is-int
// duck typing in this one path (`_matmul`'s `a[i] * b[i]` happens to accept
// a Python bool silently). This divergence is undocumented/untested
// behavior in the reference implementation -- revisit only if a real
// script's `*` on a bool-containing vector is shown to need it.
Value matmul(const Value& a, const Value& b);

// OpenSCAD's echo()/str() number formatting: at most 6 significant digits,
// fixed-point for exponents in [-5, 5], scientific notation with no leading
// zero on the exponent otherwise (`1e+6` not `1e+06`), "-0.0" -> "0".
//
// ponytail: the reference implementation's internal mantissa rounding uses
// Python's round() (banker's/round-half-to-even); this port uses
// round-half-away-from-zero instead. The two differ only on an exact tie at
// the 6th significant digit, an astronomically rare case for arbitrary
// float input -- upgrade to a banker's-rounding implementation if a real
// script's output is shown to land on one.
std::string formatNumber(double v);

// echo()/str()/assert-message display format: "undef" | "true"/"false" |
// "[start : step : end]" (range) | formatNumber() (number) | "[e1, e2, ...]"
// (list, recursive) | "object(k1 = v1, ...)" ("object()" if empty) |
// `"the string"` (quoted) | "<function-literal>" (a function value has no
// meaningful textual form in the reference either -- falls to Python's
// str(), which isn't literally portable; this port picks a fixed
// placeholder instead of trying to reproduce Python's repr). Mirrors
// Evaluator._fmt_val.
std::string fmtValue(const Value& v);

// Converts a for()/intersection_for() loop assignment's evaluated RHS into
// the list of values to iterate: undef -> empty, range -> expanded
// (matching OscRange's own start/step/end iteration, half-open at the far
// end within a 1e-10 epsilon), object -> its keys as strings, string ->
// individual characters as 1-character strings, list -> its elements
// as-is, anything else (a bare scalar) -> a single-element list. Mirrors
// the shared expansion logic duplicated across the reference's
// _eval_for/_resolve_intersection_for.
std::vector<Value> expandIterable(const Value& v);

// `each <body>`'s own flatten-one-level rule, shared by the AST interpreter
// (evalListLiteral/evalListElement's ListCompEach handling, expr_eval.cpp)
// and the bytecode VM's ACCUM_APPEND_EACH opcode: a list value extends
// `out` with its own items, undef is dropped, anything else appends as a
// single item.
void appendEachInto(std::vector<Value>& out, const Value& v);

} // namespace oscadeval
