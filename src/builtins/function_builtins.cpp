#include "openscad_cpp_evaluator/function_builtins.hpp"

#include "builtins.hpp"   // builtinDxfDim/builtinDxfCross

#include "openscad_cpp_evaluator/dispatch.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"
#include "openscad_cpp_evaluator/text_metrics.hpp"

#include "openscad_cpp_parser/ast/ast_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <random>
#include <unordered_map>
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

// -- linear_solve(A, b) --------------------------------------------------
//
// One pivoted LU answers three questions at once -- the solution, the
// determinant, and whether A is singular -- because all three fall out of
// the same factorisation.
//
// The motivation is BOSL2's determinant() (linalg.scad), a cofactor
// expansion: O(n!) time AND O(n!) intermediate list allocation. Measured
// through this evaluator: 0.04s at n=8, 0.38s at n=9, 2.7s at n=10, and
// projecting to minutes at n=12. This is O(n^3). BOSL2's linear_solve, by
// contrast, was already fast (128x128 in ~0.18s after its Apr 2026
// Householder rewrite), so speed is not the argument for the solve half --
// a correct singularity test is. See below.
//
// Square systems only. BOSL2's linear_solve also handles overdetermined
// (least-squares) and underdetermined (minimum-norm) systems via QR; LU
// cannot, and those paths are quick enough in script. A non-square matrix
// warns rather than silently doing something else.

// Defined further down, next to the other object() helpers.
Value objectOf(std::vector<std::pair<std::string, Value>> items);

// A rectangular numeric matrix as (row-major values, column count).
// nullopt for anything that is not a non-empty list of equal-length,
// non-empty numeric lists.
std::optional<std::pair<std::vector<double>, size_t>> numericMatrix(const Value& v) {
    const ListPtr* rows = std::get_if<ListPtr>(&v);
    if (!rows || !*rows || (*rows)->items.empty()) return std::nullopt;
    std::vector<double> flat;
    size_t cols = 0;
    for (size_t r = 0; r < (*rows)->items.size(); ++r) {
        const std::optional<std::vector<double>> row = allNumericList((*rows)->items[r]);
        if (!row || row->empty()) return std::nullopt;
        if (r == 0) {
            cols = row->size();
        } else if (row->size() != cols) {
            return std::nullopt;
        }
        flat.insert(flat.end(), row->begin(), row->end());
    }
    return std::make_pair(std::move(flat), cols);
}

// Householder QR of a tall matrix `a` (m rows, n cols, m >= n), in place.
//
// On return the upper triangle of `a` is R. Below the diagonal, column j
// holds the reflector vector v_j from element 1 onwards -- v_j[0] is
// normalised to 1 and therefore not stored -- and `taus[j]` is its scale,
// so H_j = I - tau_j v_j v_j^T. Q is never formed: everything that needs
// it applies the reflectors instead, which is the whole reason this is
// smaller and quicker than materialising an m x m matrix.
//
// The sign of alpha is chosen away from x[0] so that v[0] can never cancel
// to zero -- the textbook guard against catastrophic cancellation here.
//
// Returns false if a diagonal falls at or below `tol`, i.e. rank-deficient
// to the tolerance given. Without column pivoting this cannot tell
// rank-deficient from merely very ill-conditioned; see the caller's note.
bool householderQr(std::vector<double>& a, size_t m, size_t n, std::vector<double>& taus, double tol) {
    taus.assign(n, 0.0);
    for (size_t j = 0; j < n; ++j) {
        double normx = 0.0;
        for (size_t r = j; r < m; ++r) normx += a[r * n + j] * a[r * n + j];
        normx = std::sqrt(normx);
        if (normx <= tol) return false;

        const double x0 = a[j * n + j];
        const double alpha = (x0 >= 0.0) ? -normx : normx;
        const double v0 = x0 - alpha;
        // Store v scaled so v[0] == 1; the below-diagonal entries are the tail.
        double vv = 1.0;
        for (size_t r = j + 1; r < m; ++r) {
            a[r * n + j] /= v0;
            vv += a[r * n + j] * a[r * n + j];
        }
        taus[j] = 2.0 / vv;
        a[j * n + j] = alpha;

        // Apply H_j to the trailing columns.
        for (size_t c = j + 1; c < n; ++c) {
            double dot = a[j * n + c];
            for (size_t r = j + 1; r < m; ++r) dot += a[r * n + j] * a[r * n + c];
            const double f = taus[j] * dot;
            a[j * n + c] -= f;
            for (size_t r = j + 1; r < m; ++r) a[r * n + c] -= f * a[r * n + j];
        }
    }
    return true;
}

// b <- Q^T b, for b with `k` columns. Reflectors applied in forward order.
void applyQtranspose(const std::vector<double>& a, size_t m, size_t n, const std::vector<double>& taus,
                      std::vector<double>& b, size_t k) {
    for (size_t j = 0; j < n; ++j) {
        for (size_t c = 0; c < k; ++c) {
            double dot = b[j * k + c];
            for (size_t r = j + 1; r < m; ++r) dot += a[r * n + j] * b[r * k + c];
            const double f = taus[j] * dot;
            b[j * k + c] -= f;
            for (size_t r = j + 1; r < m; ++r) b[r * k + c] -= f * a[r * n + j];
        }
    }
}

// z <- Q z. Same reflectors, reverse order -- that is the only difference
// between applying Q and applying Q^T.
void applyQ(const std::vector<double>& a, size_t m, size_t n, const std::vector<double>& taus,
             std::vector<double>& z, size_t k) {
    for (size_t jj = n; jj-- > 0;) {
        for (size_t c = 0; c < k; ++c) {
            double dot = z[jj * k + c];
            for (size_t r = jj + 1; r < m; ++r) dot += a[r * n + jj] * z[r * k + c];
            const double f = taus[jj] * dot;
            z[jj * k + c] -= f;
            for (size_t r = jj + 1; r < m; ++r) z[r * k + c] -= f * a[r * n + jj];
        }
    }
}

Value builtinLinearSolve(Evaluator& ev, const Value& aArg, const Value& bArg, bool haveB,
                          const oscad::ASTNode& node) {
    const auto parsed = numericMatrix(aArg);
    if (!parsed) {
        ev.warn("linear_solve() requires a matrix of numbers", &node.position());
        return Value{};
    }
    std::vector<double> lu = parsed->first;
    const size_t n = parsed->second;          // columns == unknowns
    const size_t m = lu.size() / n;           // rows == equations
    double maxAbs = 0.0;
    for (double x : lu) {
        if (!std::isfinite(x)) {
            ev.warn("linear_solve() matrix contains a non-finite value", &node.position());
            return Value{};
        }
        maxAbs = std::max(maxAbs, std::abs(x));
    }

    // Right-hand side: a vector of n is one column; a matrix of n rows is
    // one column per column. Kept row-major alongside the factorisation.
    size_t k = 0;
    bool bWasVector = false;
    std::vector<double> rhs;
    if (haveB) {
        if (const std::optional<std::vector<double>> vec = allNumericList(bArg); vec && vec->size() == m) {
            bWasVector = true;
            k = 1;
            rhs = *vec;
        } else if (const auto bm = numericMatrix(bArg); bm && bm->first.size() / bm->second == m) {
            k = bm->second;
            rhs = bm->first;
        } else {
            ev.warn("linear_solve() right-hand side must be a vector of " + std::to_string(m) +
                        " numbers, or a matrix with that many rows",
                    &node.position());
            return Value{};
        }
        for (double x : rhs) {
            if (!std::isfinite(x)) {
                ev.warn("linear_solve() right-hand side contains a non-finite value", &node.position());
                return Value{};
            }
        }
    }

    // Relative singularity threshold. BOSL2 compares R's diagonal against a
    // fixed ABSOLUTE 1e-9 (its _EPSILON) with no scaling by the size of the
    // matrix, so a perfectly well-conditioned system scaled down by 1e-10
    // is declared singular there. Scaling by maxAbs is what makes
    // linear_solve(A*1e-10) still solvable.
    const double tol =
        std::numeric_limits<double>::epsilon() * static_cast<double>(std::max(m, n)) * std::max(maxAbs, 1.0);

    // Non-square: QR, and no determinant to report.
    //
    // m > n  overdetermined -> least squares. Factor A, apply Q^T to b, then
    //        back-substitute the leading n x n block of R.
    // m < n  underdetermined -> minimum norm. Factor A^T instead, forward-solve
    //        R^T y = b, and return Q [y; 0]. Padding with zeros is what makes
    //        it the SMALLEST solution rather than just any solution.
    if (m != n) {
        std::vector<std::pair<std::string, Value>> outNs;
        std::vector<double> taus;
        const auto answer = [&](Value x, bool sing) {
            outNs.emplace_back("x", std::move(x));
            outNs.emplace_back("det", Value{});      // undefined for a non-square matrix
            outNs.emplace_back("singular", Value{sing});
            return objectOf(std::move(outNs));
        };
        const auto shape = [&](const std::vector<double>& v, size_t rows, size_t cols) {
            if (bWasVector) return numList(v);
            std::vector<Value> rowsOut;
            rowsOut.reserve(rows);
            for (size_t r = 0; r < rows; ++r) {
                rowsOut.push_back(numList(std::vector<double>(v.begin() + static_cast<long>(r * cols),
                                                               v.begin() + static_cast<long>((r + 1) * cols))));
            }
            return listOf(std::move(rowsOut));
        };

        if (m > n) {
            if (!householderQr(lu, m, n, taus, tol)) return answer(Value{}, true);
            if (!haveB) return answer(Value{}, false);
            applyQtranspose(lu, m, n, taus, rhs, k);
            for (size_t row = n; row-- > 0;) {
                for (size_t c = 0; c < k; ++c) {
                    double acc = rhs[row * k + c];
                    for (size_t j = row + 1; j < n; ++j) acc -= lu[row * n + j] * rhs[j * k + c];
                    rhs[row * k + c] = acc / lu[row * n + row];
                }
            }
            rhs.resize(n * k);
            return answer(shape(rhs, n, k), false);
        }

        // m < n: factor the transpose, which is the tall one.
        std::vector<double> at(n * m);
        for (size_t r = 0; r < m; ++r)
            for (size_t c = 0; c < n; ++c) at[c * m + r] = lu[r * n + c];
        if (!householderQr(at, n, m, taus, tol)) return answer(Value{}, true);
        if (!haveB) return answer(Value{}, false);
        // Forward-solve R^T y = b (R is m x m upper, so R^T is lower).
        for (size_t row = 0; row < m; ++row) {
            for (size_t c = 0; c < k; ++c) {
                double acc = rhs[row * k + c];
                for (size_t j = 0; j < row; ++j) acc -= at[j * m + row] * rhs[j * k + c];
                rhs[row * k + c] = acc / at[row * m + row];
            }
        }
        std::vector<double> z(n * k, 0.0);
        for (size_t r = 0; r < m; ++r)
            for (size_t c = 0; c < k; ++c) z[r * k + c] = rhs[r * k + c];
        applyQ(at, n, m, taus, z, k);
        return answer(shape(z, n, k), false);
    }

    double det = 1.0;
    bool singular = false;
    for (size_t col = 0; col < n && !singular; ++col) {
        size_t pivot = col;
        for (size_t r = col + 1; r < n; ++r) {
            if (std::abs(lu[r * n + col]) > std::abs(lu[pivot * n + col])) pivot = r;
        }
        if (std::abs(lu[pivot * n + col]) <= tol) {
            singular = true;
            break;
        }
        if (pivot != col) {
            for (size_t c = 0; c < n; ++c) std::swap(lu[col * n + c], lu[pivot * n + c]);
            for (size_t c = 0; c < k; ++c) std::swap(rhs[col * k + c], rhs[pivot * k + c]);
            det = -det;
        }
        const double p = lu[col * n + col];
        det *= p;
        for (size_t r = col + 1; r < n; ++r) {
            const double f = lu[r * n + col] / p;
            if (f == 0.0) continue;
            lu[r * n + col] = 0.0;
            for (size_t c = col + 1; c < n; ++c) lu[r * n + c] -= f * lu[col * n + c];
            for (size_t c = 0; c < k; ++c) rhs[r * k + c] -= f * rhs[col * k + c];
        }
    }

    std::vector<std::pair<std::string, Value>> out;
    if (singular) {
        // Not a misuse -- "is this matrix singular?" is a legitimate
        // question to ask linear_solve, so it answers rather than warning.
        out.emplace_back("x", Value{});
        out.emplace_back("det", Value{0.0});
        out.emplace_back("singular", Value{true});
        return objectOf(std::move(out));
    }

    if (haveB) {
        for (size_t col = n; col-- > 0;) {
            for (size_t c = 0; c < k; ++c) {
                double acc = rhs[col * k + c];
                for (size_t j = col + 1; j < n; ++j) acc -= lu[col * n + j] * rhs[j * k + c];
                rhs[col * k + c] = acc / lu[col * n + col];
            }
        }
        if (bWasVector) {
            out.emplace_back("x", numList(rhs));
        } else {
            std::vector<Value> rowsOut;
            rowsOut.reserve(n);
            for (size_t r = 0; r < n; ++r) {
                rowsOut.push_back(numList(std::vector<double>(rhs.begin() + static_cast<long>(r * k),
                                                               rhs.begin() + static_cast<long>((r + 1) * k))));
            }
            out.emplace_back("x", listOf(std::move(rowsOut)));
        }
    } else {
        out.emplace_back("x", Value{});
    }
    out.emplace_back("det", Value{det});
    out.emplace_back("singular", Value{false});
    return objectOf(std::move(out));
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

// Ported branch-for-branch from the reference's builtin_search plus its
// three static search() overloads (builtin_functions.cc). The dispatch is on
// what is being searched FOR -- number, string, or vector; every other type
// (undef, bool, range, function, object) falls off the end into a bare
// undef. What is being searched IN is never type-checked at all: the
// reference just calls .toVector() on it, which yields an EMPTY vector for
// any non-vector, so searching in undef/a number/a bool finds nothing rather
// than failing. Only the string-in-string form treats a string haystack as
// a sequence of characters -- a `search(["a"], "abc")` vector needle sees
// that same string as an empty table, not as characters.
Value builtinSearch(const CallArgs& args, Evaluator& ev, const oscad::Position* pos) {
    const Value matchArg = getArg(args, 0, "match", Value{});
    const Value tableArg = getArg(args, 1, "vector", Value{});
    const double nrRaw = toDoubleLenient(getArg(args, 2, "num_returns", Value{1.0}));
    const double icRaw = toDoubleLenient(getArg(args, 3, "index_col", Value{0.0}));
    // Both are `unsigned int` in the reference, so a negative argument wraps
    // to a huge positive rather than meaning "unlimited"/"column 0".
    const unsigned numReturns = static_cast<unsigned>(static_cast<long long>(nrRaw));
    const unsigned indexCol = static_cast<unsigned>(static_cast<long long>(icRaw));

    static const std::vector<Value> kNoItems;
    const ListPtr* tablePtr = std::get_if<ListPtr>(&tableArg);
    const std::vector<Value>& table = (tablePtr && *tablePtr) ? (*tablePtr)->items : kNoItems;

    const auto itemsOf = [](const Value& v) -> const std::vector<Value>& {
        const ListPtr* l = std::get_if<ListPtr>(&v);
        return (l && *l) ? (*l)->items : kNoItems;
    };
    // The reference's two-clause hit test, verbatim: the whole entry counts
    // as a match only at index_col 0, and the indexed sub-element counts
    // only when the entry really is a long enough vector.
    const auto hits = [&](const Value& needle, const Value& entry) {
        if (indexCol == 0 && oscEqual(needle, entry)) return true;
        const std::vector<Value>& ev2 = itemsOf(entry);
        return indexCol < ev2.size() && oscEqual(needle, ev2[indexCol]);
    };
    const auto num = [](size_t j) { return Value{static_cast<double>(j)}; };

    if (std::holds_alternative<double>(matchArg)) {
        std::vector<Value> out;
        unsigned matchCount = 0;
        for (size_t j = 0; j < table.size(); ++j) {
            if (!hits(matchArg, table[j])) continue;
            out.push_back(num(j));
            if (numReturns != 0 && ++matchCount >= numReturns) break;
        }
        return listOf(std::move(out));
    }

    if (const std::string* needle = std::get_if<std::string>(&matchArg)) {
        std::vector<Value> out;
        if (const std::string* hay = std::get_if<std::string>(&tableArg)) {
            for (size_t i = 0; i < needle->size(); ++i) {
                unsigned matchCount = 0;
                std::vector<Value> resultvec;
                for (size_t j = 0; j < hay->size(); ++j) {
                    if ((*needle)[i] != (*hay)[j]) continue;
                    ++matchCount;
                    if (numReturns == 1) {
                        out.push_back(num(j));
                        break;
                    }
                    resultvec.push_back(num(j));
                    if (numReturns > 1 && matchCount >= numReturns) break;
                }
                if (numReturns == 0 || numReturns > 1) out.push_back(listOf(std::move(resultvec)));
            }
            return listOf(std::move(out));
        }
        // String needle, vector table: every entry must itself be a vector
        // with more than index_col elements. A single bad entry aborts the
        // WHOLE call with an empty result, not just that row -- which is
        // why `search("a", ["a","b"])` is [] and not [0].
        for (size_t i = 0; i < needle->size(); ++i) {
            unsigned matchCount = 0;
            std::vector<Value> resultvec;
            for (size_t j = 0; j < table.size(); ++j) {
                const std::vector<Value>& entryVec = itemsOf(table[j]);
                if (entryVec.size() <= indexCol) {
                    ev.warn("Invalid entry in search vector at index " + std::to_string(j) +
                                ", required number of values in the entry: " + std::to_string(indexCol + 1) +
                                ". Invalid entry: " + fmtValue(table[j]),
                            pos);
                    return listOf({});
                }
                const std::string* entry = std::get_if<std::string>(&entryVec[indexCol]);
                // A type mismatch just doesn't match, exactly as `==` would.
                if (!entry || entry->empty() || (*entry)[0] != (*needle)[i]) continue;
                ++matchCount;
                if (numReturns == 1) {
                    out.push_back(num(j));
                    break;
                }
                resultvec.push_back(num(j));
                if (numReturns > 1 && matchCount >= numReturns) break;
            }
            if (numReturns == 0 || numReturns > 1) out.push_back(listOf(std::move(resultvec)));
        }
        return listOf(std::move(out));
    }

    if (std::holds_alternative<ListPtr>(matchArg)) {
        std::vector<Value> out;
        for (const Value& needle : itemsOf(matchArg)) {
            unsigned matchCount = 0;
            std::vector<Value> resultvec;
            for (size_t j = 0; j < table.size(); ++j) {
                if (!hits(needle, table[j])) continue;
                ++matchCount;
                if (numReturns == 1) {
                    out.push_back(num(j));
                    break;
                }
                resultvec.push_back(num(j));
                if (numReturns > 1 && matchCount >= numReturns) break;
            }
            if ((numReturns == 1 && matchCount == 0) || numReturns == 0 || numReturns > 1) {
                out.push_back(listOf(std::move(resultvec)));
            }
        }
        return listOf(std::move(out));
    }

    return Value{};
}

Value builtinLookup(const CallArgs& args) {
    // Guarded here rather than via scalarNumericArity() -- the second
    // argument is a table, so the all-arguments-are-numbers rule can't
    // apply. Without this, toDoubleLenient() turns a non-numeric key into
    // 0.0 and the lookup silently returns the table's first value.
    const Value keyArg = getArg(args, 0, "key", Value{});
    if (!std::holds_alternative<double>(keyArg)) return Value{};
    const double key = std::get<double>(keyArg);
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

// textmetrics(text=, size=10, font=, direction=, language=, script=,
// halign=, valign=, spacing=) -- measures `text` against the
// FontProvider-resolved font, laid out by exactly the same shaping call
// text() makes, and returns an OscObject with position/size/ascent/
// descent/offset/advance in real OpenSCAD's key order. Going through the
// same measureText() the geometry goes through is the point: the numbers
// a script positions against cannot disagree with what it draws.
Value builtinTextmetrics(Evaluator& ev, const CallArgs& args) {
    const std::string text = asStringOr(getArg(args, 0, "text", Value{std::string("")}), "");
    const double size = toDoubleLenient(getArg(args, 1, "size", Value{10.0}));
    const std::string halign = asStringOr(getArg(args, std::nullopt, "halign", Value{std::string("left")}), "left");
    const std::string valign = asStringOr(getArg(args, std::nullopt, "valign", Value{std::string("baseline")}), "baseline");
    const double spacing = toDoubleLenient(getArg(args, std::nullopt, "spacing", Value{1.0}));
    const std::string fontSpec = asStringOr(getArg(args, std::nullopt, "font", Value{std::string("")}), "");
    ShapeOptions shape;
    shape.direction = asStringOr(getArg(args, std::nullopt, "direction", Value{std::string("")}), "");
    shape.language = asStringOr(getArg(args, std::nullopt, "language", Value{std::string("")}), "");
    shape.script = asStringOr(getArg(args, std::nullopt, "script", Value{std::string("")}), "");

    FontProvider& fp = ev.fontProvider();
    const FontHandle handle = fp.resolveFont(fontSpec);
    const TextMeasurement m = measureText(fp, handle, text, size, spacing, shape);
    const auto [offsetX, offsetY] = textAlignOffset(halign, valign, m);

    return objectOf({
        {"position", numList({offsetX + m.inkMinX, offsetY + m.descent})},
        {"size", numList({m.inkMaxX - m.inkMinX, m.ascent - m.descent})},
        {"ascent", Value{m.ascent}},
        {"descent", Value{m.descent}},
        {"offset", numList({offsetX, offsetY})},
        {"advance", numList({m.advanceX, m.advanceY})},
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

// Take only scalar numbers, mapped to how many leading positional
// arguments must each actually BE a number. Anything else there (undef, a
// string, a list, a bool, an object) is a type error yielding undef, and
// so is supplying fewer arguments than the arity -- matching the
// reference, which rejects the call outright rather than coercing.
//
// This has to be checked up front rather than left to each case below:
// toDoubleLenient() silently turns undef/string/list into 0.0, which
// would make `cos(misspelled_var)` quietly 1 instead of undef, and a
// typo'd variable then produces plausible-looking geometry instead of an
// obvious failure. Arity is deliberately a floor, not an exact match --
// the reference also rejects EXTRA arguments, but tightening that too
// risks breaking working library code for no correctness gain here.
const std::unordered_map<std::string, size_t>& scalarNumericArity() {
    static const std::unordered_map<std::string, size_t> arity = {
        {"abs", 1}, {"sign", 1}, {"ceil", 1}, {"floor", 1}, {"round", 1}, {"sqrt", 1},
        {"ln", 1}, {"log", 1}, {"exp", 1}, {"sin", 1}, {"cos", 1}, {"tan", 1},
        {"asin", 1}, {"acos", 1}, {"atan", 1}, {"atan2", 2}, {"pow", 2}, {"rands", 3},
    };
    return arity;
}

// Numeric functions that legitimately take a LIST argument, so the
// all-arguments-must-be-numbers rule above can't apply -- each validates
// its own operands, and only needs the bool case caught here (Value's
// bool and double are distinct alternatives, but the reference rejects
// `max(true, 1)` as a type error rather than treating true as 1).
const std::unordered_set<std::string>& numericOnlyNames() {
    static const std::unordered_set<std::string> names = {"max", "min", "norm", "cross"};
    return names;
}

} // namespace

bool isBuiltinFunctionName(const std::string& name) {
    static const std::unordered_set<std::string> names = {
        "abs", "sign", "ceil", "floor", "round", "sqrt", "ln", "log", "exp", "sin", "cos", "tan", "asin",
        "acos", "atan", "atan2", "max", "min", "pow", "norm", "cross", "rands", "concat", "len", "str",
        "chr", "ord", "is_undef", "is_num", "is_bool", "is_string", "is_list", "is_function", "is_object",
        "search", "lookup", "has_key", "version", "version_num", "parent_module",
        "object", "textmetrics", "fontmetrics", "dxf_dim", "dxf_cross", "supported_feature",
        "linear_solve",
    };
    return names.count(name) > 0;
}

// object(...) argument merging, matching the reference's own semantics and
// diagnostics (Builtins.cc's builtin_object).
//
// An unnamed argument is either another object (its keys are merged in) or
// a LIST of entries, where each entry is:
//   [key, value]  -- set (or overwrite) that key
//   [key]         -- DELETE that key
//
// The single-element delete form is the part that is easy to miss. Deleting
// removes the key outright rather than blanking it, so a later re-set
// appends at the end: object(a, [["b"], ["b", 99]]) puts b last, while
// object(a, [["b", 99], ["b"]]) has no b at all. That ordering is
// observable -- ValueObject is insertion-ordered and oscEqual is
// order-sensitive.
//
// Deleting a key that is not there is a silent no-op, as it is upstream.
// Every malformed entry warns and abandons the whole call (returning undef),
// stopping at the first one. The warning text is quoted verbatim from the
// reference, including its own inconsistent spacing -- the "not a list"
// case really does put spaces inside the parens where the others do not,
// and the "unnamed argument" case really does end with a trailing space.
Value mergeObjectArgs(Evaluator& ev, const std::vector<std::pair<std::optional<std::string>, Value>>& evaluated,
                       const oscad::Position* pos) {
    static const char* kEntryRules =
        " In an unnamed list, entries must be [key,value] to set or [key] to delete."
        " The key must be <string>.";

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
    const auto deleteKey = [&](const std::string& k) {
        for (auto it = result.begin(); it != result.end(); ++it) {
            if (it->first == k) {
                result.erase(it);
                return;
            }
        }
        // Deleting an absent key is deliberately silent.
    };

    for (size_t argIdx = 0; argIdx < evaluated.size(); ++argIdx) {
        const auto& [name, v] = evaluated[argIdx];
        if (name) {
            setKey(*name, v);
            continue;
        }
        const std::string argPrefix = "object(Argument " + std::to_string(argIdx) + " ";
        if (const ObjectPtr* o = std::get_if<ObjectPtr>(&v); o && *o) {
            for (const auto& [k, kv] : (*o)->items) setKey(k, kv);
            continue;
        }
        const ListPtr* l = std::get_if<ListPtr>(&v);
        if (!l || !*l) {
            // undef is accepted and contributes nothing, as upstream.
            if (std::holds_alternative<std::monostate>(v)) continue;
            ev.warn(argPrefix + "<" + oscTypeName(v) + ">) An unnamed argument must be either <object> or"
                                 " <list>, it is <" + oscTypeName(v) + ">. ",
                    pos);
            return Value{};
        }
        for (size_t elemIdx = 0; elemIdx < (*l)->items.size(); ++elemIdx) {
            const Value& entry = (*l)->items[elemIdx];
            const std::string where = "[Element " + std::to_string(elemIdx) + " ";
            const ListPtr* pair = std::get_if<ListPtr>(&entry);
            if (!pair || !*pair) {
                // Note the spaces inside the parens: upstream's own quirk.
                ev.warn("object( Argument " + std::to_string(argIdx) + " " + where + "<" + oscTypeName(entry) +
                            ">] ) Entry type is not a list, it is <" + oscTypeName(entry) + ">." + kEntryRules,
                        pos);
                return Value{};
            }
            const size_t n = (*pair)->items.size();
            if (n == 0) {
                ev.warn(argPrefix + where + "[]]) Entry is empty." + kEntryRules, pos);
                return Value{};
            }
            if (n > 2) {
                ev.warn(argPrefix + where + "[...]]) Entry length is " + std::to_string(n) +
                            ", must be 1 [key] or 2 [key,value]." + kEntryRules,
                        pos);
                return Value{};
            }
            const Value& key = (*pair)->items[0];
            if (!std::holds_alternative<std::string>(key)) {
                const std::string shape = n == 2 ? "[<" + oscTypeName(key) + ">,value]"
                                                  : "[<" + oscTypeName(key) + ">]";
                ev.warn(argPrefix + where + shape + "]) The key of the entry is not <string> but <" +
                            oscTypeName(key) + ">." + kEntryRules,
                        pos);
                return Value{};
            }
            if (n == 2) {
                setKey(std::get<std::string>(key), (*pair)->items[1]);
            } else {
                deleteKey(std::get<std::string>(key));
            }
        }
    }
    return Value{std::make_shared<const ValueObject>(ValueObject{std::move(result)})};
}

Value builtinObject(Evaluator& ev, const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx,
                     const oscad::ASTNode& node) {
    std::vector<std::pair<std::optional<std::string>, Value>> evaluated;
    evaluated.reserve(arguments.size());
    for (const auto& argPtr : arguments) {
        Value v = ev.evalExpr(*argExpr(*argPtr), ctx);
        std::optional<std::string> name;
        if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            name = static_cast<const oscad::NamedArgument&>(*argPtr).name->name;
        }
        evaluated.emplace_back(std::move(name), std::move(v));
    }
    return mergeObjectArgs(ev, evaluated, &node.position());
}

namespace {

// One id per name evalBuiltinFunction actually handles (NOT the same list
// as isBuiltinFunctionName -- "object" is deliberately absent here, since
// evalFunctionCall special-cases it to builtinObject() before this
// function is ever reached, see function_builtins.hpp). A single hash
// lookup into this table replaces what used to be an up-to-~40-way
// `if (name == "...")` string-compare chain; the switch below then
// compiles to a jump table instead of sequential comparisons. No branch's
// logic changed, only how it's reached.
enum class BuiltinFnId {
    TextMetrics, FontMetrics, Abs, Sign, Ceil, Floor, Round, Sqrt, Ln, Log, Exp, Sin, Cos, Tan,
    Asin, Acos, Atan, Atan2, Max, Min, Pow, Norm, Cross, Rands, Concat, Len, Str, Chr, Ord,
    IsUndef, IsNum, IsBool, IsString, IsList, IsFunction, IsObject, Search, Lookup, HasKey,
    Version, VersionNum, ParentModule, DxfDim, DxfCross, SupportedFeature, LinearSolve,
};

// supported_feature("name") -> the level at which this build implements that
// feature, or 0 for one it does not implement (and for a name it has never
// heard of, which is deliberate: probing for a feature from a future release
// is supposed to be safe).
//
// A LEVEL rather than a boolean so a feature whose semantics change later
// can be told apart from its earlier self. Everything here is 1 today;
// nothing has changed since this function existed, and a script cannot
// observe what a build without supported_feature() did anyway.
//
// This is the answer to a real hazard: OpenSCAD does not reject arguments or
// names it doesn't know -- children(separate=true) is silently ignored there
// -- so a script using an extension runs and quietly renders something else.
// Guarding on supported_feature() is how a script says so out loud.
const std::unordered_map<std::string, double>& featureLevels() {
    static const std::unordered_map<std::string, double> levels = {
        {"render-expr", 1.0},        // render() in expression position
        {"linear-solve", 1.0},       // linear_solve(A, b) -> {x, det, singular}
        {"warp-op", 1.0},            // warp(f) -- per-vertex displacement
        {"polyhedron-vnf", 1.0},     // polyhedron(vnf) / polyhedron(object)
        {"separate-children", 1.0},  // children(..., separate=true)
        {"minkowski-diff", 1.0},     // minkowski_difference()
        {"sphere-styles", 1.0},      // sphere(style=)
        {"export-name", 1.0},        // $export_name
        {"simplify-op", 1.0},        // simplify()
        {"expr-import", 1.0},        // import() in expression position
        {"object-function", 1.0},    // object(), unconditional here
        {"roof-op", 1.0},            // roof(), method="voronoi" only
    };
    return levels;
}

const std::unordered_map<std::string, BuiltinFnId>& builtinFnIds() {
    static const std::unordered_map<std::string, BuiltinFnId> ids = {
        {"textmetrics", BuiltinFnId::TextMetrics}, {"fontmetrics", BuiltinFnId::FontMetrics},
        {"abs", BuiltinFnId::Abs}, {"sign", BuiltinFnId::Sign}, {"ceil", BuiltinFnId::Ceil},
        {"floor", BuiltinFnId::Floor}, {"round", BuiltinFnId::Round}, {"sqrt", BuiltinFnId::Sqrt},
        {"ln", BuiltinFnId::Ln}, {"log", BuiltinFnId::Log}, {"exp", BuiltinFnId::Exp},
        {"sin", BuiltinFnId::Sin}, {"cos", BuiltinFnId::Cos}, {"tan", BuiltinFnId::Tan},
        {"asin", BuiltinFnId::Asin}, {"acos", BuiltinFnId::Acos}, {"atan", BuiltinFnId::Atan},
        {"atan2", BuiltinFnId::Atan2}, {"max", BuiltinFnId::Max}, {"min", BuiltinFnId::Min},
        {"pow", BuiltinFnId::Pow}, {"norm", BuiltinFnId::Norm}, {"cross", BuiltinFnId::Cross},
        {"rands", BuiltinFnId::Rands}, {"concat", BuiltinFnId::Concat}, {"len", BuiltinFnId::Len},
        {"str", BuiltinFnId::Str}, {"chr", BuiltinFnId::Chr}, {"ord", BuiltinFnId::Ord},
        {"is_undef", BuiltinFnId::IsUndef}, {"is_num", BuiltinFnId::IsNum},
        {"is_bool", BuiltinFnId::IsBool}, {"is_string", BuiltinFnId::IsString},
        {"is_list", BuiltinFnId::IsList}, {"is_function", BuiltinFnId::IsFunction},
        {"is_object", BuiltinFnId::IsObject}, {"search", BuiltinFnId::Search},
        {"lookup", BuiltinFnId::Lookup}, {"has_key", BuiltinFnId::HasKey},
        {"version", BuiltinFnId::Version}, {"version_num", BuiltinFnId::VersionNum},
        {"parent_module", BuiltinFnId::ParentModule},
        {"dxf_dim", BuiltinFnId::DxfDim}, {"dxf_cross", BuiltinFnId::DxfCross},
        {"supported_feature", BuiltinFnId::SupportedFeature},
        {"linear_solve", BuiltinFnId::LinearSolve},
    };
    return ids;
}

// -- reference-parity argument diagnostics -------------------------------
//
// Real OpenSCAD checks a builtin's arguments before running it and warns in
// two fixed shapes (Parameters.cc / builtin_functions.cc):
//
//   NAME() number of parameters does not match: expected N, found M
//   NAME() parameter could not be converted: WHERE: expected T, found T (v)
//
// and returns undef either way. We used to do neither -- the value came out
// undef, silently, so `ord(undef)` and `abs("a")` alike said nothing at all.
// Every spec below (arity text included, since it is free-form per builtin)
// was read off OpenSCAD 2026.02.01 directly rather than guessed.
enum : unsigned {
    TUndef = 1u << 0,
    TBool = 1u << 1,
    TNum = 1u << 2,
    TStr = 1u << 3,
    TVec = 1u << 4,
    TRange = 1u << 5,
    TFunc = 1u << 6,
    TObj = 1u << 7,
};

unsigned typeBit(const Value& v) {
    if (std::holds_alternative<bool>(v)) return TBool;
    if (std::holds_alternative<double>(v)) return TNum;
    if (std::holds_alternative<std::string>(v)) return TStr;
    if (std::holds_alternative<ListPtr>(v)) return TVec;
    if (std::holds_alternative<OscRange>(v)) return TRange;
    if (std::holds_alternative<ClosurePtr>(v)) return TFunc;
    if (std::holds_alternative<ObjectPtr>(v)) return TObj;
    return TUndef;
}

struct ArgReq {
    unsigned allowed;
    const char* expected; // the word the reference prints, which is not
                          // always the full allowed set -- len() accepts a
                          // vector or an object but still says "string".
};

struct BuiltinCheck {
    int minArgs;            // -1 disables the arity check entirely
    int maxArgs;            // -1 = unbounded
    const char* arityText;  // free-form, e.g. "1", "3 or 4", "between 2 and 4"
    std::vector<ArgReq> types;
};

const ArgReq kNum{TNum, "number"};
const ArgReq kVec{TVec, "vector"};

const std::unordered_map<int, BuiltinCheck>& builtinChecks() {
    static const auto build = [] {
        std::unordered_map<int, BuiltinCheck> m;
        const auto add = [&m](BuiltinFnId id, BuiltinCheck c) { m.emplace(static_cast<int>(id), std::move(c)); };
        for (BuiltinFnId id : {BuiltinFnId::Abs, BuiltinFnId::Sign, BuiltinFnId::Ceil, BuiltinFnId::Floor,
                                BuiltinFnId::Round, BuiltinFnId::Sqrt, BuiltinFnId::Ln, BuiltinFnId::Log,
                                BuiltinFnId::Exp, BuiltinFnId::Sin, BuiltinFnId::Cos, BuiltinFnId::Tan,
                                BuiltinFnId::Asin, BuiltinFnId::Acos, BuiltinFnId::Atan}) {
            add(id, {1, 1, "1", {kNum}});
        }
        add(BuiltinFnId::Atan2, {2, 2, "2", {kNum, kNum}});
        add(BuiltinFnId::Pow, {2, 2, "2", {kNum, kNum}});
        add(BuiltinFnId::Cross, {2, 2, "2", {kVec, kVec}});
        add(BuiltinFnId::Lookup, {2, 2, "2", {kNum, kVec}});
        add(BuiltinFnId::Norm, {1, 1, "1", {kVec}});
        // len() takes a vector or an object happily, and still calls the
        // expected type "string" when handed anything else.
        add(BuiltinFnId::Len, {1, 1, "1", {{TStr | TVec | TObj, "string"}}});
        add(BuiltinFnId::Ord, {1, 1, "1", {{TStr, "string"}}});
        add(BuiltinFnId::Rands, {3, 4, "3 or 4", {kNum, kNum, kNum, kNum}});
        add(BuiltinFnId::Search, {2, 4, "between 2 and 4", {}});
        // parent_module() with no argument defaults to 1 rather than
        // warning, so only the too-many case is an arity error.
        add(BuiltinFnId::ParentModule, {0, 1, "1", {kNum}});
        add(BuiltinFnId::HasKey, {-1, -1, nullptr, {{TObj, "object"}, {TStr, "string"}}});
        add(BuiltinFnId::SupportedFeature, {1, 1, "1", {}});
        add(BuiltinFnId::LinearSolve, {1, 2, "1 or 2", {kVec}});
        for (BuiltinFnId id : {BuiltinFnId::IsUndef, BuiltinFnId::IsNum, BuiltinFnId::IsBool,
                                BuiltinFnId::IsString, BuiltinFnId::IsList, BuiltinFnId::IsFunction,
                                BuiltinFnId::IsObject}) {
            add(id, {1, 1, "1", {}});
        }
        return m;
    };
    static const std::unordered_map<int, BuiltinCheck> checks = build();
    return checks;
}

void warnArity(Evaluator& ev, const std::string& name, const char* expected, size_t found,
                const oscad::Position* pos) {
    ev.warn(name + "() number of parameters does not match: expected " + expected + ", found " +
                std::to_string(found),
            pos);
}

void warnConversion(Evaluator& ev, const std::string& name, const std::string& where, const char* expected,
                     const Value& found, const oscad::Position* pos) {
    ev.warn(name + "() parameter could not be converted: " + where + ": expected " + expected + ", found " +
                oscTypeName(found) + " (" + fmtValue(found) + ")",
            pos);
}

// max()/min() are their own shape: either one vector of numbers, or N bare
// numbers, with a distinct "at least 1 vector element" arity text for the
// empty-vector case.
bool checkMinMax(Evaluator& ev, const std::string& name, const CallArgs& args, const oscad::Position* pos) {
    const std::vector<Value> positional = allPositional(args);
    const size_t count = positional.size() + args.named.size();
    if (count < 1) {
        warnArity(ev, name, "at least 1", count, pos);
        return false;
    }
    if (positional.size() == 1 && std::holds_alternative<ListPtr>(positional[0])) {
        static const std::vector<Value> kEmptyItems;
        const ListPtr& l = std::get<ListPtr>(positional[0]);
        const std::vector<Value>& items = l ? l->items : kEmptyItems;
        if (items.empty()) {
            warnArity(ev, name, "at least 1 vector element", 0, pos);
            return false;
        }
        for (size_t i = 0; i < items.size(); ++i) {
            if (!std::holds_alternative<double>(items[i])) {
                warnConversion(ev, name, "vector element " + std::to_string(i), "number", items[i], pos);
                return false;
            }
        }
        return true;
    }
    for (size_t i = 0; i < positional.size(); ++i) {
        if (!std::holds_alternative<double>(positional[i])) {
            warnConversion(ev, name, "argument " + std::to_string(i), "number", positional[i], pos);
            return false;
        }
    }
    return true;
}

// Returns false when a diagnostic was emitted and the call must answer undef.
bool checkBuiltinArgs(Evaluator& ev, const std::string& name, BuiltinFnId id, const CallArgs& args,
                       const oscad::Position* pos) {
    if (id == BuiltinFnId::Max || id == BuiltinFnId::Min) return checkMinMax(ev, name, args, pos);
    const auto& checks = builtinChecks();
    const auto it = checks.find(static_cast<int>(id));
    if (it == checks.end()) return true;
    const BuiltinCheck& c = it->second;

    const std::vector<Value> positional = allPositional(args);
    const size_t count = positional.size() + args.named.size();
    if (c.arityText && (static_cast<int>(count) < c.minArgs || (c.maxArgs >= 0 && static_cast<int>(count) > c.maxArgs))) {
        warnArity(ev, name, c.arityText, count, pos);
        return false;
    }
    // Positional only: a named argument's position in the reference's own
    // flat argument list can't be reconstructed from our split
    // positional/named form, and warning on the wrong index would be worse
    // than staying quiet on a spelling nobody uses for these builtins.
    for (size_t i = 0; i < c.types.size() && i < positional.size(); ++i) {
        if (!(typeBit(positional[i]) & c.types[i].allowed)) {
            warnConversion(ev, name, "argument " + std::to_string(i), c.types[i].expected, positional[i], pos);
            return false;
        }
    }
    return true;
}


} // namespace

Value evalBuiltinFunction(Evaluator& ev, const std::string& name, const CallArgs& args, const oscad::ASTNode& node) {

    const auto& ids = builtinFnIds();
    const auto idIt = ids.find(name);
    // Not one of the names this function handles (e.g. "object", routed
    // elsewhere before reaching here) -- mirrors the old chain's fallthrough.
    if (idIt == ids.end()) return Value{};

    // Only textmetrics/fontmetrics have an entry -- every other builtin
    // function reads its arguments positionally upstream and warns about
    // nothing. See builtinParamNames (registry.cpp).
    if (const std::vector<std::string>* declared = builtinParamNames(name)) {
        for (const auto& [argName, _] : args.named) {
            if (!isConfigVariable(argName) &&
                std::find(declared->begin(), declared->end(), argName) == declared->end()) {
                warnUnexpectedNamedArg(ev, argName, &node.position());
            }
        }
    }

    // Arity and argument types, with the reference's own two diagnostics.
    // This replaced a silent version of the same gate (scalarNumericArity /
    // numericOnlyNames), which returned undef without ever saying why.
    if (!checkBuiltinArgs(ev, name, idIt->second, args, &node.position())) return Value{};

    switch (idIt->second) {
        case BuiltinFnId::TextMetrics: return builtinTextmetrics(ev, args);
        case BuiltinFnId::FontMetrics: return builtinFontmetrics(ev, args);
        case BuiltinFnId::Abs: return Value{std::fabs(toDoubleLenient(getArg(args, 0, "x", Value{})))};
        case BuiltinFnId::Sign: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            return Value{x > 0 ? 1.0 : x < 0 ? -1.0 : 0.0};
        }
        case BuiltinFnId::Ceil: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            return Value{(std::isnan(x) || std::isinf(x)) ? x : std::ceil(x)};
        }
        case BuiltinFnId::Floor: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            return Value{(std::isnan(x) || std::isinf(x)) ? x : std::floor(x)};
        }
        case BuiltinFnId::Round: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            if (std::isnan(x) || std::isinf(x)) return Value{x};
            return Value{x >= 0 ? std::floor(x + 0.5) : std::ceil(x - 0.5)};
        }
        case BuiltinFnId::Sqrt: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            return Value{x < 0 ? std::numeric_limits<double>::quiet_NaN() : std::sqrt(x)};
        }
        case BuiltinFnId::Ln: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            if (x == 0) return Value{-std::numeric_limits<double>::infinity()};
            return Value{x < 0 ? std::numeric_limits<double>::quiet_NaN() : std::log(x)};
        }
        case BuiltinFnId::Log: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            if (x == 0) return Value{-std::numeric_limits<double>::infinity()};
            return Value{x < 0 ? std::numeric_limits<double>::quiet_NaN() : std::log10(x)};
        }
        case BuiltinFnId::Exp: return Value{std::exp(toDoubleLenient(getArg(args, 0, "x", Value{})))};
        case BuiltinFnId::Sin: return Value{degTrig(toDoubleLenient(getArg(args, 0, "x", Value{})), kSin90, &std::sin)};
        case BuiltinFnId::Cos: return Value{degTrig(toDoubleLenient(getArg(args, 0, "x", Value{})), kCos90, &std::cos)};
        case BuiltinFnId::Tan: return Value{degTrig(toDoubleLenient(getArg(args, 0, "x", Value{})), kTan90, &std::tan)};
        case BuiltinFnId::Asin: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            return Value{std::fabs(x) > 1 ? std::numeric_limits<double>::quiet_NaN() : degrees(std::asin(x))};
        }
        case BuiltinFnId::Acos: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            return Value{std::fabs(x) > 1 ? std::numeric_limits<double>::quiet_NaN() : degrees(std::acos(x))};
        }
        case BuiltinFnId::Atan: return Value{degrees(std::atan(toDoubleLenient(getArg(args, 0, "x", Value{}))))};
        case BuiltinFnId::Atan2:
            return Value{degrees(std::atan2(toDoubleLenient(getArg(args, 0, "y", Value{})), toDoubleLenient(getArg(args, 1, "x", Value{}))))};
        case BuiltinFnId::Max: return builtinMinMax(args, true);
        case BuiltinFnId::Min: return builtinMinMax(args, false);
        case BuiltinFnId::Pow: return builtinPow(toDoubleLenient(getArg(args, 0, "x", Value{})), toDoubleLenient(getArg(args, 1, "y", Value{})));
        case BuiltinFnId::Norm: {
            // The vector-ness of the argument is the table's job; a
            // non-numeric ELEMENT gets this separate, terser message
            // instead of the usual conversion one.
            const auto v = allNumericList(getArg(args, 0, "v", Value{}));
            if (!v) {
                ev.warn("Incorrect arguments to norm()", &node.position());
                return Value{};
            }
            double sum = 0;
            for (double x : *v) sum += x * x;
            return Value{std::sqrt(sum)};
        }
        case BuiltinFnId::Cross: {
            const Value a = getArg(args, 0, "a", Value{});
            const Value b = getArg(args, 1, "b", Value{});
            // Two distinct messages, and the size check runs first: a
            // 4-element operand is a size complaint even when it also holds
            // a string.
            const auto sizeOf = [](const Value& v) {
                const ListPtr* l = std::get_if<ListPtr>(&v);
                return (l && *l) ? (*l)->items.size() : size_t{0};
            };
            const size_t na = sizeOf(a), nb = sizeOf(b);
            if (na != nb || (na != 2 && na != 3)) {
                ev.warn("Invalid vector size of parameter for cross()", &node.position());
                return Value{};
            }
            if (!allNumericList(a) || !allNumericList(b)) {
                ev.warn("Invalid value in parameter vector for cross()", &node.position());
                return Value{};
            }
            return builtinCross(a, b);
        }
        case BuiltinFnId::Rands: {
            ev.noteRandsCall();
            return builtinRands(toDoubleLenient(getArg(args, 0, "min_value", Value{})), toDoubleLenient(getArg(args, 1, "max_value", Value{})),
                                 toDoubleLenient(getArg(args, 2, "value_count", Value{})), getArg(args, 3, "seed", Value{}));
        }
        case BuiltinFnId::Concat: {
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
        case BuiltinFnId::Len: {
            const Value x = getArg(args, 0, "x", Value{});
            if (const ListPtr* l = std::get_if<ListPtr>(&x); l && *l) return Value{static_cast<double>((*l)->items.size())};
            if (const std::string* s = std::get_if<std::string>(&x)) return Value{static_cast<double>(s->size())};
            if (const ObjectPtr* o = std::get_if<ObjectPtr>(&x); o && *o) return Value{static_cast<double>((*o)->items.size())};
            return Value{};
        }
        case BuiltinFnId::Str: {
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
        case BuiltinFnId::Chr: {
            // Variadic, and every argument contributes: chr(65, 66) is "AB".
            // A number only encodes when it is a codepoint g_unichar_validate
            // would accept -- strictly positive, below 0x110000, and not a
            // surrogate. Anything else contributes nothing at all. Without
            // that range check chr(-1)/chr(1e9) emitted raw invalid UTF-8,
            // which propagated out and broke the caller's own decoding.
            const std::function<std::string(const Value&)> encode = [&](const Value& c) -> std::string {
                if (const double* d = std::get_if<double>(&c)) {
                    if (!std::isfinite(*d) || *d <= 0) return {};
                    const auto cp = static_cast<std::uint32_t>(*d);
                    if (cp == 0 || cp > 0x10FFFFu || (cp >= 0xD800u && cp <= 0xDFFFu)) return {};
                    return utf8Encode(cp);
                }
                if (const ListPtr* l = std::get_if<ListPtr>(&c); l && *l) {
                    std::string out;
                    for (const Value& item : (*l)->items) out += encode(item);
                    return out;
                }
                if (const OscRange* r = std::get_if<OscRange>(&c)) {
                    std::string out;
                    const IterableValues seq = expandIterable(Value{*r});
                    for (const Value& item : seq) out += encode(item);
                    return out;
                }
                return {};
            };
            std::string out;
            for (const Value& a : allPositional(args)) out += encode(a);
            return Value{out};
        }
        case BuiltinFnId::Ord: {
            const Value s = getArg(args, 0, "s", Value{});
            const std::string* str = std::get_if<std::string>(&s);
            if (!str || str->empty()) return Value{};
            return Value{static_cast<double>(utf8DecodeFirst(*str))};
        }
        case BuiltinFnId::IsUndef: return Value{std::holds_alternative<std::monostate>(getArg(args, 0, "x", Value{}))};
        case BuiltinFnId::IsNum: {
            const Value x = getArg(args, 0, "x", Value{});
            const double* d = std::get_if<double>(&x);
            return Value{d != nullptr && !std::isnan(*d)};
        }
        case BuiltinFnId::IsBool: return Value{std::holds_alternative<bool>(getArg(args, 0, "x", Value{}))};
        case BuiltinFnId::IsString: return Value{std::holds_alternative<std::string>(getArg(args, 0, "x", Value{}))};
        case BuiltinFnId::IsList: {
            const Value x = getArg(args, 0, "x", Value{});
            return Value{std::holds_alternative<ListPtr>(x) && std::get<ListPtr>(x) != nullptr};
        }
        case BuiltinFnId::IsFunction: {
            const Value x = getArg(args, 0, "x", Value{});
            return Value{std::holds_alternative<ClosurePtr>(x) && std::get<ClosurePtr>(x) != nullptr};
        }
        case BuiltinFnId::IsObject: {
            const Value x = getArg(args, 0, "x", Value{});
            return Value{std::holds_alternative<ObjectPtr>(x) && std::get<ObjectPtr>(x) != nullptr};
        }
        case BuiltinFnId::Search: return builtinSearch(args, ev, &node.position());
        case BuiltinFnId::Lookup: return builtinLookup(args);
        case BuiltinFnId::HasKey: {
            const Value objArg = getArg(args, 0, "object", Value{});
            const ObjectPtr* obj = std::get_if<ObjectPtr>(&objArg);
            if (!obj || !*obj) return Value{};
            const Value keyArg = getArg(args, 1, "key", Value{});
            // A non-string key is an argument-conversion failure in the
            // reference (expected string), so it is undef -- not the "no,
            // that key isn't present" false this used to answer.
            const std::string* key = std::get_if<std::string>(&keyArg);
            if (!key) return Value{};
            for (const auto& [k, v] : (*obj)->items) {
                if (k == *key) return Value{true};
            }
            return Value{false};
        }
        // The OpenSCAD release we track. version_num() is that same
        // year/month/day folded as y * 10000 + m * 100 + d, exactly like the
        // reference's own builtin_version_num (builtin_functions.cc).
        case BuiltinFnId::LinearSolve:
            return builtinLinearSolve(ev, getArg(args, 0, "A", Value{}), getArg(args, 1, "b", Value{}),
                                       args.findPositional(1) != nullptr || args.findNamed("b") != nullptr, node);
        case BuiltinFnId::SupportedFeature: {
            const Value name = getArg(args, 0, "feature", Value{});
            const std::string* s = std::get_if<std::string>(&name);
            // A non-string is 0 rather than an error, same as an unknown
            // name: the whole point of this function is that asking is
            // always safe.
            if (!s) return Value{0.0};
            const auto& levels = featureLevels();
            const auto it = levels.find(*s);
            return Value{it == levels.end() ? 0.0 : it->second};
        }
        case BuiltinFnId::Version: return numList({2026.0, 1.0, 1.0});
        case BuiltinFnId::VersionNum: {
            // The optional vector argument the reference also accepts: with
            // no argument this is our own version() folded; with one, it is
            // whatever [y, m] / [y, m, d] the caller passed, so
            // version_num([2019, 5, 0]) == 20190500 regardless of what
            // release we report. A 2-element vector defaults the day to 0
            // (getVec3's own defaultval), and anything else -- a non-list, a
            // wrong length, a non-numeric element -- is undef.
            //
            // One deliberate divergence: the reference's size-2 path ignores
            // getVec2's own failure and folds uninitialized doubles for e.g.
            // version_num(["a", "b"]). That is undef here rather than
            // whatever happened to be on the stack.
            const Value* arg = args.findPositional(0);
            if (!arg) return Value{20260101.0};
            const auto v = allNumericList(*arg);
            if (!v || (v->size() != 2 && v->size() != 3)) return Value{};
            return Value{(*v)[0] * 10000.0 + (*v)[1] * 100.0 + (v->size() == 3 ? (*v)[2] : 0.0)};
        }
        case BuiltinFnId::DxfDim: return builtinDxfDim(ev, args, node);
        case BuiltinFnId::DxfCross: return builtinDxfCross(ev, args, node);
        case BuiltinFnId::ParentModule: {
            // Defaults to 1, not 0, when called with no argument at all --
            // and an index past the end of the stack is a warning, not a
            // silent undef.
            const Value* given = args.findPositional(0);
            const int index = given ? static_cast<int>(toDoubleLenient(*given)) : 1;
            Value r = ev.parentModuleName(index);
            if (std::holds_alternative<std::monostate>(r)) {
                ev.warn("Parent module index (" + std::to_string(index) +
                            ") greater than the number of modules on the stack",
                        &node.position());
            }
            return r;
        }
    }
    return Value{}; // unreachable: every BuiltinFnId has a case above
}

} // namespace oscadeval
