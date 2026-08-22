#include "openscad_cpp_evaluator/value.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

namespace oscadeval {

namespace {

std::optional<std::vector<double>> asFlatNumericVector(const ValueList& list) {
    std::vector<double> out;
    out.reserve(list.items.size());
    for (const Value& v : list.items) {
        const double* d = std::get_if<double>(&v);
        if (!d) return std::nullopt;
        out.push_back(*d);
    }
    return out;
}

// nullopt unless `list` is a non-ragged list of flat-numeric rows.
std::optional<std::vector<std::vector<double>>> asNumericMatrix(const ValueList& list) {
    std::vector<std::vector<double>> rows;
    rows.reserve(list.items.size());
    for (const Value& rowValue : list.items) {
        const ListPtr* rowList = std::get_if<ListPtr>(&rowValue);
        if (!rowList || !*rowList) return std::nullopt;
        auto row = asFlatNumericVector(**rowList);
        if (!row) return std::nullopt;
        rows.push_back(std::move(*row));
    }
    if (rows.empty()) return rows;
    const size_t width = rows.front().size();
    for (const auto& row : rows) {
        if (row.size() != width) return std::nullopt;
    }
    return rows;
}

Value makeList(std::vector<Value> items) {
    return Value{std::make_shared<const ValueList>(ValueList{std::move(items)})};
}

} // namespace

std::string oscTypeName(const Value& v) {
    if (std::holds_alternative<std::monostate>(v)) return "undefined";
    if (std::holds_alternative<bool>(v)) return "bool";
    if (std::holds_alternative<double>(v)) return "number";
    if (std::holds_alternative<std::string>(v)) return "string";
    if (std::holds_alternative<ListPtr>(v)) return "vector";
    if (std::holds_alternative<ObjectPtr>(v)) return "object";
    // The reference names these too (Value.cc's getTypeName), and its
    // "undefined operation (T op T)" diagnostics quote them verbatim --
    // calling either one "undefined" made `[1] + [0:2]` report a phantom
    // undef operand.
    if (std::holds_alternative<OscRange>(v)) return "range";
    if (std::holds_alternative<ClosurePtr>(v)) return "function";
    return "undefined";
}

bool oscEqual(const Value& a, const Value& b) {
    const bool aIsBool = std::holds_alternative<bool>(a);
    const bool bIsBool = std::holds_alternative<bool>(b);
    if (aIsBool != bIsBool) return false;

    const ListPtr* la = std::get_if<ListPtr>(&a);
    const ListPtr* lb = std::get_if<ListPtr>(&b);
    if (la || lb) {
        if (!la || !lb) return false;
        const auto& ia = (*la)->items;
        const auto& ib = (*lb)->items;
        if (ia.size() != ib.size()) return false;
        for (size_t i = 0; i < ia.size(); ++i) {
            if (!oscEqual(ia[i], ib[i])) return false;
        }
        return true;
    }

    const ObjectPtr* oa = std::get_if<ObjectPtr>(&a);
    const ObjectPtr* ob = std::get_if<ObjectPtr>(&b);
    if (oa || ob) {
        if (!oa || !ob) return false;
        const auto& pa = (*oa)->items;
        const auto& pb = (*ob)->items;
        if (pa.size() != pb.size()) return false;
        for (size_t i = 0; i < pa.size(); ++i) {
            if (pa[i].first != pb[i].first) return false;
            if (!oscEqual(pa[i].second, pb[i].second)) return false;
        }
        return true;
    }

    // ClosurePtr is a shared_ptr -- its own operator== compares the pointee
    // ADDRESS, not Closure::operator==(), so two independently-created
    // closures over the identical AST node (the semantics this mirrors --
    // see Closure's own doc comment) would wrongly compare unequal via the
    // variant fallthrough below. Dereference and compare explicitly.
    const ClosurePtr* ca = std::get_if<ClosurePtr>(&a);
    const ClosurePtr* cb = std::get_if<ClosurePtr>(&b);
    if (ca || cb) {
        if (!ca || !cb || !*ca || !*cb) return false;
        return **ca == **cb;
    }

    // Neither operand is a list, object, or closure here, so this only ever
    // compares monostate/bool/double/string/OscRange against its own kind
    // (variant::operator== checks the active index first).
    return a == b;
}

double toDoubleLenient(const Value& v) {
    if (const double* d = std::get_if<double>(&v)) return *d;
    if (const bool* b = std::get_if<bool>(&v)) return *b ? 1.0 : 0.0;
    return 0.0;
}

bool truthy(const Value& v) {
    if (const bool* b = std::get_if<bool>(&v)) return *b;
    if (const double* d = std::get_if<double>(&v)) return *d != 0.0; // NaN != 0.0 is true
    if (const std::string* s = std::get_if<std::string>(&v)) return !s->empty();
    if (const ListPtr* l = std::get_if<ListPtr>(&v)) return *l && !(*l)->items.empty();
    if (const ObjectPtr* o = std::get_if<ObjectPtr>(&v)) return *o && !(*o)->items.empty();
    if (std::holds_alternative<std::monostate>(v)) return false;
    return true; // OscRange, ClosurePtr
}

bool oscComparable(const Value& a, const Value& b) {
    const bool aBool = std::holds_alternative<bool>(a);
    const bool bBool = std::holds_alternative<bool>(b);
    if (aBool || bBool) return aBool && bBool;

    if (std::holds_alternative<double>(a) && std::holds_alternative<double>(b)) return true;
    if (std::holds_alternative<std::string>(a) && std::holds_alternative<std::string>(b)) return true;
    if (std::holds_alternative<ListPtr>(a) && std::holds_alternative<ListPtr>(b)) return true;
    return false;
}

Value scale(double scalarValue, const Value& value) {
    if (const ListPtr* list = std::get_if<ListPtr>(&value)) {
        if (!*list) return Value{};
        std::vector<Value> out;
        out.reserve((*list)->items.size());
        for (const Value& v : (*list)->items) out.push_back(scale(scalarValue, v));
        return makeList(std::move(out));
    }
    if (std::holds_alternative<bool>(value)) return Value{};
    if (const double* d = std::get_if<double>(&value)) return Value{scalarValue * *d};
    return Value{};
}

Value divScale(const Value& value, double divisor) {
    if (const ListPtr* list = std::get_if<ListPtr>(&value)) {
        if (!*list) return Value{};
        std::vector<Value> out;
        out.reserve((*list)->items.size());
        for (const Value& v : (*list)->items) out.push_back(divScale(v, divisor));
        return makeList(std::move(out));
    }
    if (std::holds_alternative<bool>(value)) return Value{};
    if (const double* d = std::get_if<double>(&value)) {
        if (divisor == 0.0) {
            return Value{*d == 0.0 ? std::numeric_limits<double>::quiet_NaN() : std::copysign(std::numeric_limits<double>::infinity(), *d)};
        }
        return Value{*d / divisor};
    }
    return Value{};
}

Value divInto(double numerator, const Value& value) {
    if (const ListPtr* list = std::get_if<ListPtr>(&value)) {
        if (!*list) return Value{};
        std::vector<Value> out;
        out.reserve((*list)->items.size());
        for (const Value& v : (*list)->items) out.push_back(divInto(numerator, v));
        return makeList(std::move(out));
    }
    if (std::holds_alternative<bool>(value)) return Value{};
    if (const double* d = std::get_if<double>(&value)) {
        if (*d == 0.0) {
            return Value{numerator == 0.0 ? std::numeric_limits<double>::quiet_NaN()
                                          : std::copysign(std::numeric_limits<double>::infinity(), numerator)};
        }
        return Value{numerator / *d};
    }
    return Value{};
}

namespace {

// Shared body for vecAdd/vecSub: list/list recursion (zip -- truncates to
// the shorter length) plus the numeric-fallback op, everything else undef.
template <typename NumericOp>
Value vecCombine(const Value& a, const Value& b, NumericOp numericOp) {
    const ListPtr* la = std::get_if<ListPtr>(&a);
    const ListPtr* lb = std::get_if<ListPtr>(&b);
    if (la && lb) {
        if (!*la || !*lb) return Value{};
        const auto& ia = (*la)->items;
        const auto& ib = (*lb)->items;
        const size_t n = std::min(ia.size(), ib.size());
        std::vector<Value> out;
        out.reserve(n);
        for (size_t i = 0; i < n; ++i) out.push_back(vecCombine(ia[i], ib[i], numericOp));
        return makeList(std::move(out));
    }
    if (std::holds_alternative<bool>(a) || std::holds_alternative<bool>(b)) return Value{};
    const double* da = std::get_if<double>(&a);
    const double* db = std::get_if<double>(&b);
    if (da && db) return Value{numericOp(*da, *db)};
    return Value{};
}

} // namespace

Value vecAdd(const Value& a, const Value& b) {
    if (std::holds_alternative<std::string>(a) || std::holds_alternative<std::string>(b)) return Value{};
    return vecCombine(a, b, [](double x, double y) { return x + y; });
}

Value vecSub(const Value& a, const Value& b) {
    return vecCombine(a, b, [](double x, double y) { return x - y; });
}

namespace {

// The four element-level products the reference splits `*` into, ported
// shape-for-shape from Value.cc's multvecvec/multmatvec/multvecmat plus the
// matrix*matrix branch of multiply_visitor -- including which check fires
// first, since that decides which diagnostic a malformed operand produces.
// `fail` records the message and returns undef.
const std::vector<Value>& itemsOf(const Value& v) {
    static const std::vector<Value> kEmpty;
    const ListPtr* l = std::get_if<ListPtr>(&v);
    return (l && *l) ? (*l)->items : kEmpty;
}
bool isNum(const Value& v) { return std::holds_alternative<double>(v); }
bool isVec(const Value& v) { return std::holds_alternative<ListPtr>(v); }

Value fail(std::string* error, std::string message) {
    if (error) *error = std::move(message);
    return Value{};
}

// Vector dot product. Sizes are equal by the caller's own check.
Value multVecVec(const std::vector<Value>& v1, const std::vector<Value>& v2, std::string* error) {
    double r = 0.0;
    for (size_t i = 0; i < v1.size(); ++i) {
        if (!isNum(v1[i]) || !isNum(v2[i])) {
            return fail(error, "undefined operation (" + oscTypeName(v1[i]) + " * " + oscTypeName(v2[i]) + ")");
        }
        r += std::get<double>(v1[i]) * std::get<double>(v2[i]);
    }
    return Value{r};
}

Value multMatVec(const std::vector<Value>& mat, const std::vector<Value>& vec, std::string* error) {
    std::vector<Value> out;
    out.reserve(mat.size());
    for (size_t i = 0; i < mat.size(); ++i) {
        const std::vector<Value>& row = itemsOf(mat[i]);
        if (!isVec(mat[i]) || row.size() != vec.size()) {
            return fail(error, "Matrix must be rectangular. Problem at row " + std::to_string(i));
        }
        double re = 0.0;
        for (size_t j = 0; j < row.size(); ++j) {
            if (!isNum(row[j])) {
                return fail(error, "Matrix must contain only numbers. Problem at row " + std::to_string(i) +
                                        ", col " + std::to_string(j));
            }
            if (!isNum(vec[j])) {
                return fail(error, "Vector must contain only numbers. Problem at index " + std::to_string(j));
            }
            re += std::get<double>(row[j]) * std::get<double>(vec[j]);
        }
        out.push_back(Value{re});
    }
    return makeList(std::move(out));
}

Value multVecMat(const std::vector<Value>& vec, const std::vector<Value>& mat, std::string* error) {
    const size_t firstRowSize = itemsOf(mat[0]).size();
    std::vector<Value> out;
    out.reserve(firstRowSize);
    for (size_t i = 0; i < firstRowSize; ++i) {
        double re = 0.0;
        for (size_t j = 0; j < vec.size(); ++j) {
            const std::vector<Value>& row = itemsOf(mat[j]);
            if (!isVec(mat[j]) || row.size() != firstRowSize) {
                return fail(error, "Matrix must be rectangular. Problem at row " + std::to_string(j));
            }
            if (!isNum(vec[j])) {
                return fail(error, "Vector must contain only numbers. Problem at index " + std::to_string(j));
            }
            if (!isNum(row[i])) {
                return fail(error, "Matrix must contain only numbers. Problem at row " + std::to_string(j) +
                                        ", col " + std::to_string(i));
            }
            re += std::get<double>(vec[j]) * std::get<double>(row[i]);
        }
        out.push_back(Value{re});
    }
    return makeList(std::move(out));
}

} // namespace

Value matmul(const Value& a, const Value& b, std::string* error) {
    const std::vector<Value>& al = itemsOf(a);
    const std::vector<Value>& bl = itemsOf(b);
    // The reference checks emptiness before anything else, so `[] * [1,2]`
    // is this message rather than a length mismatch.
    if (al.empty() || bl.empty()) return fail(error, "Multiplication is undefined on empty vectors");

    const Value& e1 = al.front();
    const Value& e2 = bl.front();

    // Which of the four shapes this is comes from the FIRST element's type
    // on each side, exactly as multiply_visitor decides it -- not from
    // whether the whole operand happens to be a well-formed matrix. A
    // ragged or non-numeric operand still enters the branch its first
    // element chose, and fails inside it with that branch's own message.
    if (isNum(e1)) {
        if (isNum(e2)) {
            if (al.size() == bl.size()) return multVecVec(al, bl, error);
            return fail(error, "vector*vector requires matching lengths (" + std::to_string(al.size()) +
                                    " != " + std::to_string(bl.size()) + ")");
        }
        if (isVec(e2)) {
            if (al.size() == bl.size()) return multVecMat(al, bl, error);
            return fail(error, "vector*matrix requires vector length to match matrix row count (" +
                                    std::to_string(al.size()) + " != " + std::to_string(bl.size()) + ")");
        }
    } else if (isVec(e1)) {
        const size_t cols = itemsOf(e1).size();
        if (isNum(e2)) {
            if (cols == bl.size()) return multMatVec(al, bl, error);
            return fail(error, "matrix*vector requires matrix column count to match vector length (" +
                                    std::to_string(cols) + " != " + std::to_string(bl.size()) + ")");
        }
        if (isVec(e2)) {
            if (cols != bl.size()) {
                return fail(error,
                            "matrix*matrix requires left operand column count to match right operand row count (" +
                                std::to_string(cols) + " != " + std::to_string(bl.size()) + ")");
            }
            std::vector<Value> rows;
            rows.reserve(al.size());
            for (size_t i = 0; i < al.size(); ++i) {
                const std::vector<Value>& srcRow = itemsOf(al[i]);
                if (srcRow.size() != bl.size()) {
                    return fail(error,
                                "matrix*matrix left operand row length does not match right operand row count (" +
                                    std::to_string(srcRow.size()) + " != " + std::to_string(bl.size()) +
                                    ") at row " + std::to_string(i));
                }
                std::string rowError;
                Value row = multVecMat(srcRow, bl, &rowError);
                if (!rowError.empty()) {
                    return fail(error, rowError + ": while processing left operand at row " + std::to_string(i));
                }
                rows.push_back(std::move(row));
            }
            return makeList(std::move(rows));
        }
    }
    return fail(error, "undefined vector*vector multiplication where first elements are types " + oscTypeName(e1) +
                            " and " + oscTypeName(e2));
}

std::string unescapeStringLiteral(const std::string& raw) {
    // Most strings carry no escape at all, and this runs on every
    // evaluation of a literal on the tree-walking path -- so don't build a
    // second copy of the string unless there is something to change.
    const size_t first = raw.find('\\');
    if (first == std::string::npos) return raw;

    std::string out;
    out.reserve(raw.size());
    out.append(raw, 0, first);
    for (size_t i = first; i < raw.size(); ++i) {
        if (raw[i] != '\\' || i + 1 >= raw.size()) {
            out.push_back(raw[i]);  // a trailing lone backslash stands for itself
            continue;
        }
        const char next = raw[i + 1];
        switch (next) {
            case 'n': out.push_back('\n'); ++i; break;
            case 't': out.push_back('\t'); ++i; break;
            case 'r': out.push_back('\r'); ++i; break;
            case '\n': ++i; break;  // line continuation: contributes nothing
            case '\r':
                // Only a CRLF pair is a line continuation; a lone CR is an
                // ordinary escaped character like any other.
                if (i + 2 < raw.size() && raw[i + 2] == '\n') { i += 2; break; }
                out.push_back('\r');
                ++i;
                break;
            default: out.push_back(next); ++i; break;  // \\ and \" land here too
        }
    }
    return out;
}

std::string formatNumber(double v) {
    if (std::isnan(v)) return "nan";
    if (std::isinf(v)) return v > 0 ? "inf" : "-inf";
    if (v == 0.0) return "0"; // also covers -0.0 (-0.0 == 0.0 is true)

    const bool neg = v < 0;
    const double av = std::abs(v);
    int exp = static_cast<int>(std::floor(std::log10(av)));
    double mantissa = std::round(av / std::pow(10.0, exp) * 1e5) / 1e5;
    if (mantissa >= 10.0) {
        mantissa /= 10.0;
        ++exp;
    }

    auto trimTrailing = [](std::string s) {
        if (s.find('.') == std::string::npos) return s;
        size_t last = s.find_last_not_of('0');
        s.erase(last + 1);
        if (!s.empty() && s.back() == '.') s.pop_back();
        return s;
    };

    std::string s;
    if (exp >= -5 && exp <= 5) {
        const int decimals = std::max(0, 5 - exp);
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(decimals);
        oss << av;
        s = trimTrailing(oss.str());
    } else {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(5);
        oss << mantissa;
        const std::string m = trimTrailing(oss.str());
        s = m + "e" + (exp >= 0 ? "+" : "-") + std::to_string(std::abs(exp));
    }
    return neg ? "-" + s : s;
}

std::string fmtValue(const Value& v) {
    if (std::holds_alternative<std::monostate>(v)) return "undef";
    if (const bool* b = std::get_if<bool>(&v)) return *b ? "true" : "false";
    if (const OscRange* r = std::get_if<OscRange>(&v)) {
        return "[" + formatNumber(r->start) + " : " + formatNumber(r->step) + " : " + formatNumber(r->end) + "]";
    }
    if (const double* d = std::get_if<double>(&v)) return formatNumber(*d);
    if (const ListPtr* l = std::get_if<ListPtr>(&v)) {
        if (!*l) return "[]";
        std::string s = "[";
        const auto& items = (*l)->items;
        for (size_t i = 0; i < items.size(); ++i) {
            if (i) s += ", ";
            s += fmtValue(items[i]);
        }
        return s + "]";
    }
    if (const ObjectPtr* o = std::get_if<ObjectPtr>(&v)) {
        // `{ a = 1; b = "x"; }`, not a constructor-call spelling: this is
        // how the reference echoes an object (ObjectType's own stream
        // operator), and it nests and appears inside lists/str() the same
        // way. We used to print `object(a = 1)`, which nothing else agrees
        // with.
        if (!*o || (*o)->items.empty()) return "{ }";
        std::string s = "{ ";
        for (const auto& [key, val] : (*o)->items) s += key + " = " + fmtValue(val) + "; ";
        return s + "}";
    }
    if (const std::string* s = std::get_if<std::string>(&v)) return "\"" + *s + "\"";
    return "<function-literal>"; // OscRange handled above; ClosurePtr has no meaningful textual form either
}

// (Range::numValues()'s own count, closed-form) -- 1 + floor((end-start)/step),
// the same epsilon IterableValues::inRange uses so this agrees exactly with
// how many elements a lazy walk of the same range would actually produce.
// nullopt for a step of 0 or a direction that disagrees with start/end (a
// naturally-empty range -- 0 elements, never "too many").
std::optional<size_t> rangeElementCount(const OscRange& r) {
    if (r.step == 0.0) return std::nullopt;
    const double n = (r.end - r.start) / r.step;
    if (n < -1e-10) return std::nullopt; // wrong direction -- naturally empty
    return static_cast<size_t>(std::floor(n + 1e-10)) + 1;
}

std::string rangeDirectionWarning(bool stepPositive) {
    return stepPositive ? "begin is greater than the end, but step is positive"
                        : "begin is smaller than the end, but step is negative";
}

IterableValues expandIterable(const Value& v, const RangeTooManyFn& onTooMany,
                               const RangeDirectionFn& onWrongDirection) {
    if (std::holds_alternative<std::monostate>(v)) return IterableValues{};
    if (const OscRange* r = std::get_if<OscRange>(&v)) {
        if (const std::optional<size_t> count = rangeElementCount(*r); count && *count >= 1'000'000) {
            if (onTooMany) onTooMany(*count);
            return IterableValues{};
        }
        // A step pointing away from the end yields nothing. That is almost
        // always a typo -- [1:0] where [1:-1:0] was meant -- so the
        // reference says so rather than silently running zero times.
        //
        // A zero step is deliberately NOT reported here: it is a different
        // failure (the reference calls it "too many elements") and shares
        // rangeElementCount's nullopt with this case only by coincidence.
        if (onWrongDirection && r->step != 0.0) {
            const double n = (r->end - r->start) / r->step;
            if (n < -1e-10) onWrongDirection(r->step > 0.0);
        }
        // Lazy -- IterableValues itself reproduces this exact
        // termination condition (a zero step is naturally empty: neither
        // `x <= end` nor `x >= end` branch ever fires for it) without
        // building a vector here.
        return IterableValues{r->start, r->step, r->end};
    }
    if (const ObjectPtr* o = std::get_if<ObjectPtr>(&v)) {
        std::vector<Value> out;
        if (*o) {
            for (const auto& [key, val] : (*o)->items) out.push_back(Value{key});
        }
        return IterableValues{std::move(out)};
    }
    if (const std::string* s = std::get_if<std::string>(&v)) {
        std::vector<Value> out;
        for (char c : *s) out.push_back(Value{std::string(1, c)});
        return IterableValues{std::move(out)};
    }
    if (const ListPtr* l = std::get_if<ListPtr>(&v)) {
        return *l ? IterableValues{*l} : IterableValues{};
    }
    return IterableValues{std::vector<Value>{v}}; // bare scalar -> single-element list
}

void appendEachInto(std::vector<Value>& out, const Value& v) {
    if (const ListPtr* l = std::get_if<ListPtr>(&v); l && *l) {
        for (const Value& x : (*l)->items) out.push_back(x);
    } else if (!std::holds_alternative<std::monostate>(v)) {
        out.push_back(v);
    }
}

} // namespace oscadeval
