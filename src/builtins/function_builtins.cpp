#include "openscad_cpp_evaluator/function_builtins.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"
#include "openscad_cpp_evaluator/text_metrics.hpp"

#include "openscad_cpp_parser/ast/ast_node.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>
#include <unordered_set>

namespace oscadeval {

namespace {

// -- shared helpers -----------------------------------------------------

bool isBoolOrListContainsBool(const Value& v) {
    if (std::holds_alternative<bool>(v)) return true;
    if (const ListPtr* l = std::get_if<ListPtr>(&v); l && *l) {
        for (const Value& item : (*l)->items) {
            if (std::holds_alternative<bool>(item)) return true;
        }
    }
    return false;
}

std::optional<std::vector<double>> allNumericList(const Value& v) {
    const ListPtr* l = std::get_if<ListPtr>(&v);
    if (!l || !*l) return std::nullopt;
    std::vector<double> out;
    out.reserve((*l)->items.size());
    for (const Value& item : (*l)->items) {
        const double* d = std::get_if<double>(&item);
        if (!d) return std::nullopt;
        out.push_back(*d);
    }
    return out;
}

Value listOf(std::vector<Value> items) { return Value{std::make_shared<const ValueList>(ValueList{std::move(items)})}; }
Value numList(const std::vector<double>& xs) {
    std::vector<Value> items;
    items.reserve(xs.size());
    for (double x : xs) items.push_back(Value{x});
    return listOf(std::move(items));
}

double degrees(double rad) { return rad * 180.0 / std::numbers::pi; }
double radians(double deg) { return deg * std::numbers::pi / 180.0; }

// At exact multiples of 90 degrees, sin/cos/tan use exact table values
// instead of sin/cos/tan(radians(x)), which accumulate floating-point
// noise (cos(90) -> 6.12e-17 etc) -- matches real OpenSCAD's degree-based
// trig. Mirrors _deg_trig.
constexpr std::array<double, 4> kSin90 = {0.0, 1.0, 0.0, -1.0};
constexpr std::array<double, 4> kCos90 = {1.0, 0.0, -1.0, 0.0};
const std::array<double, 4> kTan90 = {0.0, std::numeric_limits<double>::infinity(), 0.0,
                                       -std::numeric_limits<double>::infinity()};

double degTrig(double x, const std::array<double, 4>& table, double (*fallback)(double)) {
    if (std::isnan(x) || std::isinf(x)) return std::numeric_limits<double>::quiet_NaN();
    const double n = x / 90.0;
    const double rn = std::round(n);
    if (rn == n) {
        int idx = static_cast<int>(rn) % 4;
        if (idx < 0) idx += 4;
        return table[static_cast<size_t>(idx)];
    }
    return fallback(radians(x));
}

// UTF-8 encode/decode for chr()/ord() -- OpenSCAD strings are unicode text,
// not byte arrays, so a codepoint above U+007F needs real multi-byte
// handling, not a raw byte cast.
std::string utf8Encode(uint32_t cp) {
    std::string out;
    if (cp <= 0x7F) {
        out.push_back(static_cast<char>(cp));
    } else if (cp <= 0x7FF) {
        out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp <= 0xFFFF) {
        out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
        out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
    return out;
}

uint32_t utf8DecodeFirst(const std::string& s) {
    if (s.empty()) return 0;
    const auto b = [&](size_t i) { return static_cast<unsigned char>(s[i]); };
    const unsigned char c0 = b(0);
    if (c0 < 0x80) return c0;
    if ((c0 & 0xE0) == 0xC0 && s.size() >= 2) return static_cast<uint32_t>(((c0 & 0x1F) << 6) | (b(1) & 0x3F));
    if ((c0 & 0xF0) == 0xE0 && s.size() >= 3) {
        return static_cast<uint32_t>(((c0 & 0x0F) << 12) | ((b(1) & 0x3F) << 6) | (b(2) & 0x3F));
    }
    if ((c0 & 0xF8) == 0xF0 && s.size() >= 4) {
        return static_cast<uint32_t>(((c0 & 0x07) << 18) | ((b(1) & 0x3F) << 12) | ((b(2) & 0x3F) << 6) | (b(3) & 0x3F));
    }
    return c0; // malformed lead byte -- fall back to the raw byte value
}

// -- min/max/pow ----------------------------------------------------------

Value builtinMinMax(const CallArgs& args, bool wantMax) {
    const std::vector<Value> positional = allPositional(args);
    if (positional.size() == 1) {
        if (isBoolOrListContainsBool(positional[0])) return Value{};
        if (const auto nums = allNumericList(positional[0])) {
            if (nums->empty()) return Value{};
            return Value{wantMax ? *std::max_element(nums->begin(), nums->end())
                                  : *std::min_element(nums->begin(), nums->end())};
        }
        const double* d = std::get_if<double>(&positional[0]);
        return d ? Value{*d} : Value{};
    }
    if (positional.empty()) return Value{};
    std::vector<double> nums;
    nums.reserve(positional.size());
    for (const Value& v : positional) {
        if (isBoolOrListContainsBool(v)) return Value{};
        const double* d = std::get_if<double>(&v);
        if (!d) return Value{}; // any list among multiple scalar args -> undef
        nums.push_back(*d);
    }
    return Value{wantMax ? *std::max_element(nums.begin(), nums.end()) : *std::min_element(nums.begin(), nums.end())};
}

Value builtinPow(double a, double b) {
    if (a < 0 && std::floor(b) != b) return Value{std::numeric_limits<double>::quiet_NaN()};
    if (a == 0 && b < 0) return Value{std::numeric_limits<double>::infinity()};
    return Value{std::pow(a, b)};
}

// -- cross/rands/search/lookup --------------------------------------------

Value builtinCross(const Value& aArg, const Value& bArg) {
    const auto a = allNumericList(aArg);
    const auto b = allNumericList(bArg);
    if (!a || !b) return Value{};
    for (double x : *a) {
        if (!std::isfinite(x)) return Value{};
    }
    for (double x : *b) {
        if (!std::isfinite(x)) return Value{};
    }
    if (a->size() == 2 && b->size() == 2) return Value{(*a)[0] * (*b)[1] - (*a)[1] * (*b)[0]};
    if (a->size() == 3 && b->size() == 3) {
        return numList({(*a)[1] * (*b)[2] - (*a)[2] * (*b)[1], (*a)[2] * (*b)[0] - (*a)[0] * (*b)[2],
                         (*a)[0] * (*b)[1] - (*a)[1] * (*b)[0]});
    }
    return Value{};
}

Value builtinRands(double minv, double maxv, double nArg, const Value& seedArg) {
    // ponytail: doesn't reproduce Python's Mersenne-Twister bit-for-bit --
    // no script should depend on cross-language RNG equality, only on
    // "seeded => deterministic, same seed => same sequence" within one
    // process. A process-lifetime engine (reseeded only on an explicit
    // seed argument) mirrors random.seed()'s "reseed the global stream"
    // semantics closely enough.
    static std::mt19937 engine(std::random_device{}());
    if (!std::holds_alternative<std::monostate>(seedArg)) {
        engine.seed(static_cast<unsigned>(static_cast<long long>(toDoubleLenient(seedArg))));
    }
    const int n = static_cast<int>(nArg);
    std::vector<double> out;
    if (n <= 0) return numList(out);
    out.reserve(static_cast<size_t>(n));
    std::uniform_real_distribution<double> dist(minv, maxv);
    for (int i = 0; i < n; ++i) out.push_back(dist(engine));
    return numList(out);
}

Value builtinSearch(const CallArgs& args) {
    const Value matchArg = getArg(args, 0, "match", Value{});
    const Value vectorArg = getArg(args, 1, "vector", Value{});
    const ListPtr* vecPtr = std::get_if<ListPtr>(&vectorArg);
    if (!vecPtr || !*vecPtr) return Value{};
    const auto& vec = (*vecPtr)->items;
    const int numReturns = static_cast<int>(toDoubleLenient(getArg(args, 2, "num_returns", Value{1.0})));
    const int col = static_cast<int>(toDoubleLenient(getArg(args, 3, "index_col", Value{0.0})));

    const auto findAll = [&](const Value& val) {
        const bool valIsList = std::holds_alternative<ListPtr>(val);
        std::vector<int> results;
        for (size_t i = 0; i < vec.size(); ++i) {
            Value target;
            if (valIsList) {
                target = vec[i];
            } else if (const ListPtr* itemList = std::get_if<ListPtr>(&vec[i]); itemList && *itemList) {
                // ponytail: an out-of-range index_col silently matches
                // nothing for that row rather than aborting the whole
                // search() call (the reference raises and the outer
                // try/except turns the whole call into undef) -- a narrow
                // divergence on a malformed-argument edge case.
                target = (col >= 0 && static_cast<size_t>(col) < (*itemList)->items.size()) ? (*itemList)->items[static_cast<size_t>(col)]
                                                                                              : Value{};
            } else {
                target = vec[i];
            }
            if (oscEqual(target, val)) results.push_back(static_cast<int>(i));
        }
        return results;
    };

    const auto capped = [](std::vector<int> matches, int limit) {
        if (limit >= 0 && static_cast<size_t>(limit) < matches.size()) matches.resize(static_cast<size_t>(limit));
        return matches;
    };
    const auto toIdxList = [](const std::vector<int>& idxs) {
        std::vector<Value> items;
        items.reserve(idxs.size());
        for (int i : idxs) items.push_back(Value{static_cast<double>(i)});
        return listOf(std::move(items));
    };

    // Per-element result for the string-char and list-element match forms:
    // num_returns==1 returns a *bare number* on a match, or [] if none --
    // a genuinely mixed-type result, matching the reference's
    // _result_for exactly (not the always-a-list shape the scalar top-
    // level match form uses below).
    const auto resultForElement = [&](const Value& val) -> Value {
        const std::vector<int> matches = findAll(val);
        if (numReturns == 1) return matches.empty() ? toIdxList({}) : Value{static_cast<double>(matches[0])};
        if (numReturns == 0) return toIdxList(matches);
        return toIdxList(capped(matches, numReturns));
    };

    if (const std::string* str = std::get_if<std::string>(&matchArg)) {
        std::vector<Value> results;
        for (char c : *str) {
            const Value r = resultForElement(Value{std::string(1, c)});
            const bool rIsEmptyList = std::holds_alternative<ListPtr>(r) && std::get<ListPtr>(r) && std::get<ListPtr>(r)->items.empty();
            if (numReturns != 1 || !rIsEmptyList) results.push_back(r);
        }
        return listOf(std::move(results));
    }
    if (const ListPtr* matchList = std::get_if<ListPtr>(&matchArg); matchList && *matchList) {
        std::vector<Value> results;
        results.reserve((*matchList)->items.size());
        for (const Value& m : (*matchList)->items) results.push_back(resultForElement(m));
        return listOf(std::move(results));
    }
    const std::vector<int> matches = findAll(matchArg);
    if (numReturns == 1) return toIdxList(capped(matches, 1));
    if (numReturns == 0) return toIdxList(matches);
    return toIdxList(capped(matches, numReturns));
}

Value builtinLookup(const CallArgs& args) {
    const double key = toDoubleLenient(getArg(args, 0, "key", Value{}));
    const Value tableArg = getArg(args, 1, "table", Value{});
    const ListPtr* tablePtr = std::get_if<ListPtr>(&tableArg);
    if (!tablePtr || !*tablePtr) return Value{};

    std::vector<std::pair<double, double>> pairs;
    for (const Value& row : (*tablePtr)->items) {
        const ListPtr* rowPtr = std::get_if<ListPtr>(&row);
        if (!rowPtr || !*rowPtr || (*rowPtr)->items.size() < 2) continue;
        pairs.emplace_back(toDoubleLenient((*rowPtr)->items[0]), toDoubleLenient((*rowPtr)->items[1]));
    }
    if (pairs.empty()) return Value{};
    std::stable_sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    if (key <= pairs.front().first) return Value{pairs.front().second};
    if (key >= pairs.back().first) return Value{pairs.back().second};
    for (size_t i = 0; i + 1 < pairs.size(); ++i) {
        const double k0 = pairs[i].first, k1 = pairs[i + 1].first;
        if (k0 <= key && key <= k1) {
            const double t = (k1 == k0) ? 0.0 : (key - k0) / (k1 - k0);
            return Value{pairs[i].second + t * (pairs[i + 1].second - pairs[i].second)};
        }
    }
    return Value{0.0};
}

std::string asStringOr(const Value& v, const std::string& fallback) {
    const std::string* s = std::get_if<std::string>(&v);
    return s ? *s : fallback;
}

Value objectOf(std::vector<std::pair<std::string, Value>> items) {
    return Value{std::make_shared<const ValueObject>(ValueObject{std::move(items)})};
}

// textmetrics(text=, size=10, halign=, valign=, spacing=, font=) -- measures
// `text` against the FontProvider-resolved font (same resolution text()
// uses) and returns an OscObject with position/size/ascent/descent/offset/
// advance, matching real OpenSCAD's key order. Mirrors _builtin_textmetrics.
Value builtinTextmetrics(Evaluator& ev, const CallArgs& args) {
    const std::string text = asStringOr(getArg(args, 0, "text", Value{std::string("")}), "");
    const double size = toDoubleLenient(getArg(args, 1, "size", Value{10.0}));
    const std::string halign = asStringOr(getArg(args, std::nullopt, "halign", Value{std::string("left")}), "left");
    const std::string valign = asStringOr(getArg(args, std::nullopt, "valign", Value{std::string("baseline")}), "baseline");
    const double spacing = toDoubleLenient(getArg(args, std::nullopt, "spacing", Value{1.0}));
    const std::string fontSpec = asStringOr(getArg(args, std::nullopt, "font", Value{std::string("")}), "");

    FontProvider& fp = ev.fontProvider();
    const FontHandle handle = fp.resolveFont(fontSpec);
    const TextMeasurement m = measureText(fp, handle, text, size, spacing);
    const auto [offsetX, offsetY] = textAlignOffset(halign, valign, m);

    return objectOf({
        {"position", numList({offsetX + m.inkMinX, offsetY + m.descent})},
        {"size", numList({m.inkMaxX - m.inkMinX, m.ascent - m.descent})},
        {"ascent", Value{m.ascent}},
        {"descent", Value{m.descent}},
        {"offset", numList({offsetX, offsetY})},
        {"advance", numList({m.advanceX, 0.0})},
    });
}

// fontmetrics(size=10, font=) -- global metrics of the FontProvider-
// resolved font, scaled for `size`. Mirrors _builtin_fontmetrics.
Value builtinFontmetrics(Evaluator& ev, const CallArgs& args) {
    const double size = toDoubleLenient(getArg(args, 0, "size", Value{10.0}));
    const std::string fontSpec = asStringOr(getArg(args, std::nullopt, "font", Value{std::string("")}), "");

    FontProvider& fp = ev.fontProvider();
    const FontHandle handle = fp.resolveFont(fontSpec);
    const FontMetrics fm = fp.metrics(handle);
    const double scale = size * (100.0 / 72.0) / fm.unitsPerEm;

    return objectOf({
        {"nominal", objectOf({{"ascent", Value{fm.ascent * scale}}, {"descent", Value{fm.descent * scale}}})},
        {"max", objectOf({{"ascent", Value{fm.yMax * scale}}, {"descent", Value{fm.yMin * scale}}})},
        {"interline", Value{(fm.ascent - fm.descent + fm.lineGap) * scale}},
        {"font", objectOf({{"family", Value{fm.family}}, {"style", Value{fm.style}}})},
    });
}

const std::unordered_set<std::string>& numericOnlyNames() {
    static const std::unordered_set<std::string> names = {
        "abs", "sign", "ceil", "floor", "round", "sqrt", "ln", "log", "exp", "sin", "cos", "tan",
        "asin", "acos", "atan", "atan2", "pow", "max", "min", "norm", "cross",
    };
    return names;
}

} // namespace

bool isBuiltinFunctionName(const std::string& name) {
    static const std::unordered_set<std::string> names = {
        "abs", "sign", "ceil", "floor", "round", "sqrt", "ln", "log", "exp", "sin", "cos", "tan", "asin",
        "acos", "atan", "atan2", "max", "min", "pow", "norm", "cross", "rands", "concat", "len", "str",
        "chr", "ord", "is_undef", "is_num", "is_bool", "is_string", "is_list", "is_function", "is_object",
        "search", "lookup", "has_key", "version", "version_num", "parent_module",
        "object", "textmetrics", "fontmetrics",
    };
    return names.count(name) > 0;
}

Value builtinObject(Evaluator& ev, const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx) {
    std::vector<std::pair<std::string, Value>> result;
    const auto setKey = [&](const std::string& k, const Value& v) {
        for (auto& [ek, ev2] : result) {
            if (ek == k) {
                ev2 = v;
                return;
            }
        }
        result.emplace_back(k, v);
    };
    for (const auto& argPtr : arguments) {
        const Value v = ev.evalExpr(*argExpr(*argPtr), ctx);
        if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            setKey(static_cast<const oscad::NamedArgument&>(*argPtr).name->name, v);
            continue;
        }
        if (const ObjectPtr* o = std::get_if<ObjectPtr>(&v); o && *o) {
            for (const auto& [k, kv] : (*o)->items) setKey(k, kv);
        } else if (const ListPtr* l = std::get_if<ListPtr>(&v); l && *l) {
            for (const Value& entry : (*l)->items) {
                const ListPtr* pair = std::get_if<ListPtr>(&entry);
                if (pair && *pair && (*pair)->items.size() == 2 && std::holds_alternative<std::string>((*pair)->items[0])) {
                    setKey(std::get<std::string>((*pair)->items[0]), (*pair)->items[1]);
                } else {
                    return Value{};
                }
            }
        } else if (!std::holds_alternative<std::monostate>(v)) {
            return Value{};
        }
    }
    return Value{std::make_shared<const ValueObject>(ValueObject{std::move(result)})};
}

Value evalBuiltinFunction(Evaluator& ev, const std::string& name, const CallArgs& args, const oscad::ASTNode& node) {
    (void)node;

    if (name == "textmetrics") return builtinTextmetrics(ev, args);
    if (name == "fontmetrics") return builtinFontmetrics(ev, args);

    if (numericOnlyNames().count(name)) {
        for (const Value& v : allPositional(args)) {
            if (isBoolOrListContainsBool(v)) return Value{};
        }
    }

    if (name == "abs") return Value{std::fabs(toDoubleLenient(getArg(args, 0, "x", Value{})))};
    if (name == "sign") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        return Value{x > 0 ? 1.0 : x < 0 ? -1.0 : 0.0};
    }
    if (name == "ceil") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        return Value{(std::isnan(x) || std::isinf(x)) ? x : std::ceil(x)};
    }
    if (name == "floor") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        return Value{(std::isnan(x) || std::isinf(x)) ? x : std::floor(x)};
    }
    if (name == "round") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        if (std::isnan(x) || std::isinf(x)) return Value{x};
        return Value{x >= 0 ? std::floor(x + 0.5) : std::ceil(x - 0.5)};
    }
    if (name == "sqrt") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        return Value{x < 0 ? std::numeric_limits<double>::quiet_NaN() : std::sqrt(x)};
    }
    if (name == "ln") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        if (x == 0) return Value{-std::numeric_limits<double>::infinity()};
        return Value{x < 0 ? std::numeric_limits<double>::quiet_NaN() : std::log(x)};
    }
    if (name == "log") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        if (x == 0) return Value{-std::numeric_limits<double>::infinity()};
        return Value{x < 0 ? std::numeric_limits<double>::quiet_NaN() : std::log10(x)};
    }
    if (name == "exp") return Value{std::exp(toDoubleLenient(getArg(args, 0, "x", Value{})))};
    if (name == "sin") return Value{degTrig(toDoubleLenient(getArg(args, 0, "x", Value{})), kSin90, &std::sin)};
    if (name == "cos") return Value{degTrig(toDoubleLenient(getArg(args, 0, "x", Value{})), kCos90, &std::cos)};
    if (name == "tan") return Value{degTrig(toDoubleLenient(getArg(args, 0, "x", Value{})), kTan90, &std::tan)};
    if (name == "asin") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        return Value{std::fabs(x) > 1 ? std::numeric_limits<double>::quiet_NaN() : degrees(std::asin(x))};
    }
    if (name == "acos") {
        const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
        return Value{std::fabs(x) > 1 ? std::numeric_limits<double>::quiet_NaN() : degrees(std::acos(x))};
    }
    if (name == "atan") return Value{degrees(std::atan(toDoubleLenient(getArg(args, 0, "x", Value{}))))};
    if (name == "atan2") {
        return Value{degrees(std::atan2(toDoubleLenient(getArg(args, 0, "y", Value{})), toDoubleLenient(getArg(args, 1, "x", Value{}))))};
    }
    if (name == "max") return builtinMinMax(args, true);
    if (name == "min") return builtinMinMax(args, false);
    if (name == "pow") return builtinPow(toDoubleLenient(getArg(args, 0, "x", Value{})), toDoubleLenient(getArg(args, 1, "y", Value{})));
    if (name == "norm") {
        const auto v = allNumericList(getArg(args, 0, "v", Value{}));
        if (!v) return Value{};
        double sum = 0;
        for (double x : *v) sum += x * x;
        return Value{std::sqrt(sum)};
    }
    if (name == "cross") return builtinCross(getArg(args, 0, "a", Value{}), getArg(args, 1, "b", Value{}));
    if (name == "rands") {
        ev.noteRandsCall();
        return builtinRands(toDoubleLenient(getArg(args, 0, "min_value", Value{})), toDoubleLenient(getArg(args, 1, "max_value", Value{})),
                             toDoubleLenient(getArg(args, 2, "value_count", Value{})), getArg(args, 3, "seed", Value{}));
    }
    if (name == "concat") {
        std::vector<Value> out;
        for (const Value& a : allPositional(args)) {
            if (const ListPtr* l = std::get_if<ListPtr>(&a); l && *l) {
                out.insert(out.end(), (*l)->items.begin(), (*l)->items.end());
            } else {
                out.push_back(a);
            }
        }
        return listOf(std::move(out));
    }
    if (name == "len") {
        const Value x = getArg(args, 0, "x", Value{});
        if (const ListPtr* l = std::get_if<ListPtr>(&x); l && *l) return Value{static_cast<double>((*l)->items.size())};
        if (const std::string* s = std::get_if<std::string>(&x)) return Value{static_cast<double>(s->size())};
        if (const ObjectPtr* o = std::get_if<ObjectPtr>(&x); o && *o) return Value{static_cast<double>((*o)->items.size())};
        return Value{};
    }
    if (name == "str") {
        std::string out;
        for (const Value& a : allPositional(args)) {
            if (const std::string* s = std::get_if<std::string>(&a)) {
                out += *s;
            } else {
                out += fmtValue(a);
            }
        }
        return Value{out};
    }
    if (name == "chr") {
        const Value x = getArg(args, 0, "x", Value{});
        const auto valid = [](const Value& c) {
            const double* d = std::get_if<double>(&c);
            return d != nullptr && std::isfinite(*d);
        };
        if (const ListPtr* l = std::get_if<ListPtr>(&x); l && *l) {
            std::string out;
            for (const Value& c : (*l)->items) {
                if (valid(c)) out += utf8Encode(static_cast<uint32_t>(std::get<double>(c)));
            }
            return Value{out};
        }
        return Value{valid(x) ? utf8Encode(static_cast<uint32_t>(std::get<double>(x))) : std::string{}};
    }
    if (name == "ord") {
        const Value s = getArg(args, 0, "s", Value{});
        const std::string* str = std::get_if<std::string>(&s);
        if (!str || str->empty()) return Value{};
        return Value{static_cast<double>(utf8DecodeFirst(*str))};
    }
    if (name == "is_undef") return Value{std::holds_alternative<std::monostate>(getArg(args, 0, "x", Value{}))};
    if (name == "is_num") {
        const Value x = getArg(args, 0, "x", Value{});
        const double* d = std::get_if<double>(&x);
        return Value{d != nullptr && !std::isnan(*d)};
    }
    if (name == "is_bool") return Value{std::holds_alternative<bool>(getArg(args, 0, "x", Value{}))};
    if (name == "is_string") return Value{std::holds_alternative<std::string>(getArg(args, 0, "x", Value{}))};
    if (name == "is_list") {
        const Value x = getArg(args, 0, "x", Value{});
        return Value{std::holds_alternative<ListPtr>(x) && std::get<ListPtr>(x) != nullptr};
    }
    if (name == "is_function") {
        const Value x = getArg(args, 0, "x", Value{});
        return Value{std::holds_alternative<const oscad::FunctionLiteral*>(x) && std::get<const oscad::FunctionLiteral*>(x) != nullptr};
    }
    if (name == "is_object") {
        const Value x = getArg(args, 0, "x", Value{});
        return Value{std::holds_alternative<ObjectPtr>(x) && std::get<ObjectPtr>(x) != nullptr};
    }
    if (name == "search") return builtinSearch(args);
    if (name == "lookup") return builtinLookup(args);
    if (name == "has_key") {
        const Value objArg = getArg(args, 0, "object", Value{});
        const ObjectPtr* obj = std::get_if<ObjectPtr>(&objArg);
        if (!obj || !*obj) return Value{};
        const Value keyArg = getArg(args, 1, "key", Value{});
        const std::string* key = std::get_if<std::string>(&keyArg);
        if (!key) return Value{false};
        for (const auto& [k, v] : (*obj)->items) {
            if (k == *key) return Value{true};
        }
        return Value{false};
    }
    if (name == "version") return numList({2025.0, 1.0, 1.0});
    if (name == "version_num") return Value{20250101.0};
    if (name == "parent_module") return ev.parentModuleName(static_cast<int>(toDoubleLenient(getArg(args, 0, "index", Value{0.0}))));

    return Value{};
}

} // namespace oscadeval
