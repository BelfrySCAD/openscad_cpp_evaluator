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
    return "undefined"; // OscRange, FunctionLiteral*
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

    // Neither operand is a list or object here, so this only ever compares
    // monostate/bool/double/string/OscRange/FunctionLiteral* against its own
    // kind (variant::operator== checks the active index first).
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
    return true; // OscRange, FunctionLiteral*
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

Value matmul(const Value& a, const Value& b) {
    const ListPtr* la = std::get_if<ListPtr>(&a);
    const ListPtr* lb = std::get_if<ListPtr>(&b);
    if (!la || !lb || !*la || !*lb) return Value{};
    const ValueList& al = **la;
    const ValueList& bl = **lb;

    const bool aIsMat = !al.items.empty() && std::holds_alternative<ListPtr>(al.items.front());
    const bool bIsMat = !bl.items.empty() && std::holds_alternative<ListPtr>(bl.items.front());

    if (!aIsMat && !bIsMat) {
        auto va = asFlatNumericVector(al);
        auto vb = asFlatNumericVector(bl);
        if (!va || !vb || va->size() != vb->size()) return Value{};
        double sum = 0.0;
        for (size_t i = 0; i < va->size(); ++i) sum += (*va)[i] * (*vb)[i];
        return Value{sum};
    }

    if (aIsMat && bIsMat) {
        auto ma = asNumericMatrix(al);
        auto mb = asNumericMatrix(bl);
        if (!ma || !mb || ma->empty() || mb->empty() || mb->size() != (*ma).front().size()) return Value{};
        const size_t m = ma->size(), n = mb->size(), p = mb->front().size();
        std::vector<Value> rows;
        rows.reserve(m);
        for (size_t i = 0; i < m; ++i) {
            std::vector<Value> row;
            row.reserve(p);
            for (size_t j = 0; j < p; ++j) {
                double sum = 0.0;
                for (size_t k = 0; k < n; ++k) sum += (*ma)[i][k] * (*mb)[k][j];
                row.push_back(Value{sum});
            }
            rows.push_back(makeList(std::move(row)));
        }
        return makeList(std::move(rows));
    }

    if (aIsMat) { // matrix (m x n) . vector (n) -> vector (m)
        auto ma = asNumericMatrix(al);
        auto vb = asFlatNumericVector(bl);
        if (!ma || !vb || ma->empty() || ma->front().size() != vb->size()) return Value{};
        std::vector<Value> out;
        out.reserve(ma->size());
        for (const auto& row : *ma) {
            double sum = 0.0;
            for (size_t k = 0; k < row.size(); ++k) sum += row[k] * (*vb)[k];
            out.push_back(Value{sum});
        }
        return makeList(std::move(out));
    }

    // vector (m) . matrix (m x n) -> vector (n)
    auto va = asFlatNumericVector(al);
    auto mb = asNumericMatrix(bl);
    if (!va || !mb || mb->empty() || mb->size() != va->size()) return Value{};
    const size_t n = mb->front().size();
    std::vector<Value> out;
    out.reserve(n);
    for (size_t j = 0; j < n; ++j) {
        double sum = 0.0;
        for (size_t k = 0; k < mb->size(); ++k) sum += (*va)[k] * (*mb)[k][j];
        out.push_back(Value{sum});
    }
    return makeList(std::move(out));
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
        if (!*o || (*o)->items.empty()) return "object()";
        std::string s = "object(";
        const auto& items = (*o)->items;
        for (size_t i = 0; i < items.size(); ++i) {
            if (i) s += ", ";
            s += items[i].first + " = " + fmtValue(items[i].second);
        }
        return s + ")";
    }
    if (const std::string* s = std::get_if<std::string>(&v)) return "\"" + *s + "\"";
    return "<function-literal>"; // const FunctionLiteral* -- no meaningful textual form in the reference either
}

IterableValues expandIterable(const Value& v) {
    if (std::holds_alternative<std::monostate>(v)) return IterableValues{};
    if (const OscRange* r = std::get_if<OscRange>(&v)) {
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
