#pragma once

#include "openscad_cpp_evaluator/osc_range.hpp"

#include <functional>
#include <memory>
#include <optional>
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
struct Closure;

using ListPtr = std::shared_ptr<const ValueList>;
using ObjectPtr = std::shared_ptr<const ValueObject>;
using ClosurePtr = std::shared_ptr<const Closure>;

// The OpenSCAD dynamic value type. `bool` is a distinct alternative from
// `double` throughout the evaluator -- never implicitly coerced, unlike
// Python's own bool-is-int -- see every free function below.
//
// A function-literal *value* (`g = function(x) x*2;`) is a Closure: the AST
// node pointer plus a captured snapshot of its defining scope's own `let_`
// trail (see Closure's own doc comment below) -- unlike the Python
// reference's _expr_function_literal (a bare `return node`, no capture at
// all), needed for a closure that outlives the call that created it (see
// Closure's doc comment for the motivating BOSL2 example and why the
// simpler bare-pointer approach silently breaks that pattern).
using Value = std::variant<std::monostate, // undef
                            bool,
                            double, // OpenSCAD has one numeric type
                            std::string,
                            ListPtr,
                            OscRange,
                            ObjectPtr,
                            ClosurePtr>;

// A function-literal value: the AST node (params/body, non-owning -- valid
// as long as the parsed AST it points into is alive, same ownership
// contract as every other raw AST pointer this evaluator holds) plus
// `capturedLet`, a type-erased `shared_ptr<TrailView<Value>>` (see
// scope_trail.hpp) snapshotting the `let_` trail live at the moment this
// literal was evaluated into a value. Type-erased here (not
// `shared_ptr<TrailView<Value>>` directly) purely to avoid a value.hpp <->
// scope_trail.hpp circular include (scope_trail.hpp already includes
// value.hpp for Value itself); callers that need the typed pointer back
// (eval_context.cpp, user_calls.cpp, bytecode_vm.cpp -- anywhere
// scope_trail.hpp is already visible) use capturedLetTrail() below.
//
// WHY this exists: a plain `TrailView` level is popped (and its bindings
// destroyed) the instant the EvalContext holding it goes out of scope --
// fine for ordinary nested evaluation, wrong for a closure that ESCAPES
// its creating call, e.g. BOSL2's `hashmap()`:
//   function make_closure(captured) = function(k) captured + k;
//   c = make_closure(10);
//   echo(c(5));  // must be 15 -- `captured` has to survive make_closure()
//                // having already returned by the time c(5) runs.
// Holding this shared_ptr copy keeps that trail level's refcount above
// zero for as long as ANY closure value referencing it is still reachable,
// so `captured` stays resolvable via the ordinary ancestry-walk lookup
// long after the call that bound it has returned -- no snapshot/copy of
// the bindings themselves needed, just extending what already exists.
// `Closure::operator==` compares `node` only (matching the old bare-
// pointer equality exactly) -- two closures over the same AST node are
// still "the same function", regardless of what each captured.
struct Closure {
    const oscad::FunctionLiteral* node = nullptr;
    std::shared_ptr<void> capturedLet;

    friend bool operator==(const Closure& a, const Closure& b) { return a.node == b.node; }
};

template <typename T>
class TrailView;

// Typed access to Closure::capturedLet -- see its own doc comment for why
// the field itself is type-erased. Only ever called where scope_trail.hpp
// is already included.
inline std::shared_ptr<TrailView<Value>> capturedLetTrail(const Closure& c) {
    return std::static_pointer_cast<TrailView<Value>>(c.capturedLet);
}

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

// Read-only view over a for()/intersection_for() loop assignment's expanded
// iteration values (see expandIterable() below). When the source was
// already a list, this just holds the list's own shared_ptr (a refcount
// bump, no copy of `items`); object/string/scalar cases must synthesize a
// fresh vector regardless (there's no existing storage to view), which is
// held instead. A range is neither: it's kept as (start, step, end) and
// each element is produced on demand, in the SAME left-to-right
// accumulation order (`x += step`, repeated) the old eager expansion
// always used -- never a recomputed `start + i*step`, which can round
// differently than repeated addition over many steps, and this codebase's
// float fidelity is meant to be bit-for-bit, not "close enough". No
// vector is ever materialized for a range, however large.
//
// Every real caller reaches elements in strictly increasing order from 0
// (begin()/end()'s range-for, or the bytecode VM's own index that only
// ever increments by exactly 1 -- see bytecode_vm.cpp's Op::IterNext).
// operator[]'s one-slot cursor_ cache exploits exactly that pattern to
// stay O(1) per access; a genuinely out-of-order index (nothing here does
// this today) still returns the CORRECT value by restarting the
// accumulation from `start`, just slower -- never silently wrong.
class IterableValues {
public:
    IterableValues() = default;
    explicit IterableValues(ListPtr list) : list_(std::move(list)) {}
    explicit IterableValues(std::vector<Value> owned)
        : owned_(std::make_shared<std::vector<Value>>(std::move(owned))) {}
    IterableValues(double rangeStart, double rangeStep, double rangeEnd)
        : rangeStart_(rangeStart), rangeStep_(rangeStep), rangeEnd_(rangeEnd), isRange_(true) {}

    // O(1) for list/owned. For a range this walks the whole sequence once
    // (the same cost expandIterable's old eager version always paid to
    // find this out, just without allocating anything to store it in) --
    // kept for API completeness; no caller in this codebase actually
    // calls size() on a range today (they all reach elements via
    // begin()/end() or the sequential operator[] below, neither of which
    // needs a count up front).
    size_t size() const {
        if (!isRange_) return list_ ? list_->items.size() : (owned_ ? owned_->size() : 0);
        size_t n = 0;
        for (double x = rangeStart_; inRange(x); x += rangeStep_) ++n;
        return n;
    }

    const Value& operator[](size_t i) const {
        if (!isRange_) return list_ ? list_->items[i] : (*owned_)[i];
        if (!cursor_ || i < cursor_->index) cursor_ = Cursor{0, rangeStart_, Value{}};
        while (cursor_->index < i) {
            cursor_->value += rangeStep_;
            ++cursor_->index;
        }
        cursor_->cached = Value{cursor_->value};
        return cursor_->cached;
    }

    // A sentinel-style forward iterator: `end()`'s own position is never
    // read (operator!= only ever asks "is *this* exhausted", the usual
    // shape for a lazily-produced sequence with no predetermined length)
    // -- see IterableValues::exhausted()/advance()/dereference() below.
    class Iterator {
    public:
        const Value& operator*() const { return owner_->dereference(state_); }
        Iterator& operator++() {
            owner_->advance(state_);
            return *this;
        }
        bool operator!=(const Iterator&) const { return !owner_->exhausted(state_); }

    private:
        friend class IterableValues;
        struct State {
            size_t idx = 0;
            double x = 0;
            mutable Value cached;
        };
        Iterator(const IterableValues* owner, State state) : owner_(owner), state_(state) {}
        const IterableValues* owner_;
        State state_;
    };

    Iterator begin() const { return Iterator(this, Iterator::State{0, rangeStart_, Value{}}); }
    Iterator end() const { return Iterator(this, Iterator::State{}); }

private:
    bool inRange(double x) const {
        if (rangeStep_ > 0) return x <= rangeEnd_ + 1e-10;
        if (rangeStep_ < 0) return x >= rangeEnd_ - 1e-10;
        return false;
    }

    const Value& dereference(const Iterator::State& s) const {
        if (isRange_) {
            s.cached = Value{s.x};
            return s.cached;
        }
        return list_ ? list_->items[s.idx] : (*owned_)[s.idx];
    }

    void advance(Iterator::State& s) const {
        if (isRange_) {
            s.x += rangeStep_;
        } else {
            ++s.idx;
        }
    }

    bool exhausted(const Iterator::State& s) const {
        if (isRange_) return !inRange(s.x);
        const size_t n = list_ ? list_->items.size() : (owned_ ? owned_->size() : 0);
        return s.idx >= n;
    }

    ListPtr list_;
    std::shared_ptr<std::vector<Value>> owned_;
    double rangeStart_ = 0, rangeStep_ = 0, rangeEnd_ = 0;
    bool isRange_ = false;

    struct Cursor {
        size_t index;
        double value;
        Value cached;
    };
    mutable std::optional<Cursor> cursor_;
};

// Called (count = the range's own requested element count) when a range
// would be rejected for having too many elements -- see expandIterable's
// own doc comment. The caller's own warn(message, position) already knows
// how to format/emit a WARNING; this just hands back the number to put in
// it, keeping expandIterable() itself free of any Evaluator/echo coupling.
using RangeTooManyFn = std::function<void(size_t count)>;

// Converts a for()/intersection_for() loop assignment's evaluated RHS into
// the sequence of values to iterate: undef -> empty, range -> a LAZY
// sequence (matching OscRange's own start/step/end iteration, half-open
// at the far end within a 1e-10 epsilon -- see IterableValues's own doc
// comment; no vector is ever built for this case, however large the
// range), object -> its keys as strings, string -> individual characters
// as 1-character strings, list -> its elements as-is (no copy -- see
// IterableValues), anything else (a bare scalar) -> a single-element
// list. Mirrors the shared expansion logic duplicated across the
// reference's _eval_for/_resolve_intersection_for.
//
// A range whose own element count would be >= 1,000,000 is rejected
// entirely (onTooMany called, empty result returned) rather than iterated
// -- verified empirically against real OpenSCAD.app: a range producing
// exactly 999,999 elements iterates fine; exactly 1,000,000 emits
// "WARNING: Bad range parameter in for statement: too many elements (N)"
// and contributes zero iterations (not a truncation to 999,999, not an
// error like the C-style for's own _MAX_CFOR_ITERATIONS limit -- a
// different mechanism entirely, checked per-range, not against a
// cartesian-product total across a multi-variable for()). The Python
// reference (openscad_evaluator) has no equivalent check at all -- this
// was a real gap in both ports, closed here first since this port is the
// one with a live consumer (BelfrySCAD) that surfaced it. The count is
// computed in closed form (not IterableValues::size()'s O(n) walk) so a
// legitimate huge-but-under-the-limit range never pays an eager pass.
IterableValues expandIterable(const Value& v, const RangeTooManyFn& onTooMany = nullptr);

// `each <body>`'s own flatten-one-level rule, shared by the AST interpreter
// (evalListLiteral/evalListElement's ListCompEach handling, expr_eval.cpp)
// and the bytecode VM's ACCUM_APPEND_EACH opcode: a list value extends
// `out` with its own items, undef is dropped, anything else appends as a
// single item.
void appendEachInto(std::vector<Value>& out, const Value& v);

} // namespace oscadeval
