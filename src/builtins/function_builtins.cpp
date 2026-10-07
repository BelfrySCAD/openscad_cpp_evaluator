#include "openscad_cpp_evaluator/function_builtins.hpp"

#include "builtins.hpp"   // builtinDxfDim/builtinDxfCross

#include "openscad_cpp_evaluator/dispatch.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/segments.hpp"
#include "openscad_cpp_evaluator/text_metrics.hpp"
#include "openscad_cpp_evaluator/utf8.hpp"

#include "openscad_cpp_parser/ast/ast_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
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

Value listOf(std::vector<Value> items) { return Value{makeList(std::move(items))}; }
Value numList(const std::vector<double>& xs) {
    std::vector<Value> items;
    items.reserve(xs.size());
    for (double x : xs) items.push_back(Value{x});
    return listOf(std::move(items));
}

// Degree trigonometry, a direct port of real OpenSCAD's
// src/utils/degree_trig.cc. Not `std::sin(x * pi / 180)`: that is off by an
// ULP from OpenSCAD for most angles, and OpenSCAD's own extra structure --
// exact values at 30/45/60, and computing the >45 range through the
// COMPLEMENT function -- is what makes identities like sin(45) == cos(45)
// and cos(30) == sqrt(3)/2 hold EXACTLY there.
//
// That exactness is load-bearing for real libraries, not cosmetic. BOSL2's
// rect(rounding=...) builds a corner arc and then filters intersections
// with `isect != seg[0]`, an exact float comparison. One ULP of drift makes
// the shared vertex of two adjacent segments compare unequal, the filter
// keeps two corner points instead of one, and rect() dies on its own
// `assert(len(cornerpt)==1, "Cannot find corner point to anchor")`.
//
// The constants are OpenSCAD's own literals, deliberately -- M_DEG2RAD as a
// single pre-rounded double is not bitwise identical to `pi / 180.0`.
constexpr double kSqrt3 = 1.73205080756887719318;    // sqrt(3)
constexpr double kSqrt3_4 = 0.86602540378443859659;  // sqrt(3/4) == sqrt(3)/2
constexpr double kSqrt1_3 = 0.57735026918962573106;  // sqrt(1/3) == sqrt(3)/3
constexpr double kSqrt1_2 = 0.70710678118654752440;  // sqrt(1/2)
constexpr double kRad2Deg = 57.2957795130823208767;  // 180/PI
constexpr double kDeg2Rad = 0.017453292519943295769; // PI/180

// Assumes a 26+26=52-bit mantissa; beyond it, reduction loses all accuracy
// and OpenSCAD returns NaN rather than a meaningless answer.
constexpr double kTrigHugeVal = (1L << 26) * 360.0 * (1L << 26);

double degrees(double rad) { return rad * kRad2Deg; }
double radians(double deg) { return deg * kDeg2Rad; }

double sinDegrees(double x) {
    // Positive tests, so Inf/NaN fall through to the domain check.
    if (x < 360.0 && x >= 0.0) {
        // already reduced
    } else if (x < kTrigHugeVal && x > -kTrigHugeVal) {
        x -= 360.0 * std::floor(x / 360.0);
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
    bool oppose = x >= 180.0;
    if (oppose) x -= 180.0;
    if (x > 90.0) x = 180.0 - x;
    if (x < 45.0) {
        x = (x == 30.0) ? 0.5 : std::sin(radians(x));
    } else if (x == 45.0) {
        x = kSqrt1_2;
    } else if (x == 60.0) {
        x = kSqrt3_4;
    } else { // Inf/NaN would fall here
        x = std::cos(radians(90.0 - x));
    }
    return oppose ? -x : x;
}

double cosDegrees(double x) {
    if (x < 360.0 && x >= 0.0) {
        // already reduced
    } else if (x < kTrigHugeVal && x > -kTrigHugeVal) {
        x -= 360.0 * std::floor(x / 360.0);
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
    bool oppose = x >= 180.0;
    if (oppose) x -= 180.0;
    if (x > 90.0) {
        x = 180.0 - x;
        oppose = !oppose;
    }
    if (x > 45.0) {
        x = (x == 60.0) ? 0.5 : std::sin(radians(90.0 - x));
    } else if (x == 45.0) {
        x = kSqrt1_2;
    } else if (x == 30.0) {
        x = kSqrt3_4;
    } else { // Inf/NaN would fall here
        x = std::cos(radians(x));
    }
    return oppose ? -x : x;
}

double tanDegrees(double x) {
    const double cycles = std::floor(x / 180.0);
    if (x < 180.0 && x >= 0.0) {
        // already reduced
    } else if (x < kTrigHugeVal && x > -kTrigHugeVal) {
        x -= 180.0 * cycles;
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const bool evenCycle = std::fmod(cycles, 2.0) == 0.0;
    bool oppose = x > 90.0;
    if (oppose) x = 180.0 - x;
    if (x == 0.0) {
        x = evenCycle ? 0.0 : -0.0;
    } else if (x == 30.0) {
        x = kSqrt1_3;
    } else if (x == 45.0) {
        x = 1.0;
    } else if (x == 60.0) {
        x = kSqrt3;
    } else if (x == 90.0) {
        x = evenCycle ? std::numeric_limits<double>::infinity()
                      : -std::numeric_limits<double>::infinity();
    } else {
        x = std::tan(radians(x));
    }
    return oppose ? -x : x;
}

// The inverses snap to a whole number of degrees whenever the forward
// function reproduces the input exactly, so asin(sin(30)) is 30 and not
// 29.999999999999996.
double asinDegrees(double x) {
    const double degs = degrees(std::asin(x));
    const double whole = std::round(degs);
    return sinDegrees(whole) == x ? whole : degs;
}

double acosDegrees(double x) {
    const double degs = degrees(std::acos(x));
    const double whole = std::round(degs);
    return cosDegrees(whole) == x ? whole : degs;
}

double atanDegrees(double x) {
    const double degs = degrees(std::atan(x));
    const double whole = std::round(degs);
    return tanDegrees(whole) == x ? whole : degs;
}

// atan2 snaps on a tolerance rather than a round-trip: OpenSCAD's own rule.
double atan2Degrees(double y, double x) {
    const double degs = degrees(std::atan2(y, x));
    const double whole = std::round(degs);
    return std::fabs(degs - whole) < 3.0E-14 ? whole : degs;
}

// utf8Encode lives in utf8.cpp -- the string-literal escape decoder
// (\uXXXX) needs it too.
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

// Python's float hash, as upstream ports it (linalg.cc hash_floating_point):
// what a rands() seed actually seeds with. Truncating to an integer, as this
// used to, sent seeds 1.5, 1e20, 2^32 and 2^32+1 to 1, -1, 0 and 1.
int32_t hashFloatingPoint(double v) {
    constexpr int kBits = 31;
    constexpr uint32_t kModulus = (uint32_t{1} << kBits) - 1;
    if (!std::isfinite(v)) return std::isinf(v) ? (v > 0 ? 314159 : -314159) : 0;
    int e;
    double m = std::frexp(v, &e);
    int sign = 1;
    if (m < 0) {
        sign = -1;
        m = -m;
    }
    uint32_t x = 0;
    while (m) {
        x = ((x << 28) & kModulus) | x >> (kBits - 28);
        m *= 268435456.0;  // 2**28
        e -= 28;
        const uint32_t y = static_cast<uint32_t>(m);
        m -= y;
        x += y;
        if (x >= kModulus) x -= kModulus;
    }
    e = e >= 0 ? e % kBits : kBits - 1 - ((-1 - e) % kBits);
    x = ((x << e) & kModulus) | x >> (kBits - e);
    x = x * static_cast<uint32_t>(sign);
    return static_cast<int32_t>(x);
}

// Upstream builtin_rands, case for case: non-finite bounds are reset with
// its two warnings, reversed bounds are swapped, the count is |count|
// truncated, and a non-finite count becomes 1 -- it used to saturate to
// INT_MAX and hang building two billion doubles.
Value builtinRands(Evaluator& ev, double minv, double maxv, double nArg, const Value& seedArg,
                   const oscad::Position* pos) {
    static std::mt19937 engine(std::random_device{}());
    const auto resetBound = [&](double& bound, const char* which, double to) {
        ev.warn(std::string("rands() range ") + which + " cannot be infinite", pos);
        bound = to;
        char buf[400];
        std::snprintf(buf, sizeof buf, "%f", to);
        ev.warn(std::string("resetting to ") + buf, nullptr);
    };
    if (!std::isfinite(minv)) resetBound(minv, "min", -std::numeric_limits<double>::max() / 2);
    if (!std::isfinite(maxv)) resetBound(maxv, "max", std::numeric_limits<double>::max() / 2);
    if (maxv < minv) std::swap(minv, maxv);
    double count = std::fabs(nArg);
    if (!std::isfinite(count)) {
        ev.warn("rands() cannot create an infinite number of results", pos);
        ev.warn("resetting number of results to 1", nullptr);
        count = 1;
    }
    if (!std::holds_alternative<std::monostate>(seedArg)) {
        engine.seed(static_cast<uint32_t>(hashFloatingPoint(toDoubleLenient(seedArg))));
    }
    const size_t n = static_cast<size_t>(count);
    std::vector<double> out;
    out.reserve(n);
    if (minv >= maxv) {  // uniform_real_distribution needs min < max
        out.assign(n, minv);
    } else {
        std::uniform_real_distribution<double> dist(minv, maxv);
        for (size_t i = 0; i < n; ++i) out.push_back(dist(engine));
    }
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
    // The reference's own documented parameter names. These used to be
    // "match"/"vector"/"num_returns"/"index_col", which are not names
    // OpenSCAD ever documented, so a script written against the manual --
    // BOSL2's in_list(), which passes num_returns_per_match and
    // index_col_num -- bound neither and silently got the defaults.
    //
    // Worth knowing, though deliberately NOT copied: the reference ignores
    // these names altogether and binds search() purely by position.
    // `search(zzz=m, qqq=t, www=1, eee=1)` returns the right answer there,
    // while the correct names in the wrong order return the wrong one.
    // CallArgs keeps positional and named arguments apart and does not
    // record where a named one was written, so matching that would mean
    // changing how every call collects its arguments -- for a quirk no
    // script relies on deliberately.
    const Value matchArg = getArg(args, 0, "match_value", Value{});
    const Value tableArg = getArg(args, 1, "string_or_vector", Value{});
    const double nrRaw = toDoubleLenient(getArg(args, 2, "num_returns_per_match", Value{1.0}));
    const double icRaw = toDoubleLenient(getArg(args, 3, "index_col_num", Value{0.0}));
    // Both are `unsigned int` in the reference, so a negative argument wraps
    // to a huge positive rather than meaning "unlimited"/"column 0".
    const unsigned numReturns = static_cast<unsigned>(static_cast<long long>(nrRaw));
    const unsigned indexCol = static_cast<unsigned>(static_cast<long long>(icRaw));

    static const ListItems kNoItems;
    const ListPtr* tablePtr = std::get_if<ListPtr>(&tableArg);
    const ListItems& table = (tablePtr && *tablePtr) ? (*tablePtr)->items : kNoItems;

    const auto itemsOf = [](const Value& v) -> const ListItems& {
        const ListPtr* l = std::get_if<ListPtr>(&v);
        return (l && *l) ? (*l)->items : kNoItems;
    };
    // The reference's two-clause hit test, verbatim: the whole entry counts
    // as a match only at index_col 0, and the indexed sub-element counts
    // only when the entry really is a long enough vector.
    const auto hits = [&](const Value& needle, const Value& entry) {
        if (indexCol == 0 && oscEqual(needle, entry)) return true;
        const ListItems& ev2 = itemsOf(entry);
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
            // Both sides walked as CHARACTERS, and the indices reported are
            // character indices. Comparing bytes made a multi-byte needle
            // search once per byte -- search("é", "aé—z") answered [1, 2]
            // where the reference answers [1].
            const std::vector<std::string> needleChars = utf8Chars(*needle);
            const std::vector<std::string> hayChars = utf8Chars(*hay);
            for (size_t i = 0; i < needleChars.size(); ++i) {
                unsigned matchCount = 0;
                std::vector<Value> resultvec;
                for (size_t j = 0; j < hayChars.size(); ++j) {
                    if (needleChars[i] != hayChars[j]) continue;
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
        // Character-wise here too: a needle character is what gets looked
        // up in each row, so a multi-byte one must not be split.
        const std::vector<std::string> needleChars = utf8Chars(*needle);
        for (size_t i = 0; i < needleChars.size(); ++i) {
            unsigned matchCount = 0;
            std::vector<Value> resultvec;
            for (size_t j = 0; j < table.size(); ++j) {
                const ListItems& entryVec = itemsOf(table[j]);
                if (entryVec.size() <= indexCol) {
                    ev.warn("Invalid entry in search vector at index " + std::to_string(j) +
                                ", required number of values in the entry: " + std::to_string(indexCol + 1) +
                                ". Invalid entry: " + fmtValue(table[j]),
                            pos);
                    return listOf({});
                }
                const std::string* entry = std::get_if<std::string>(&entryVec[indexCol]);
                // A type mismatch just doesn't match, exactly as `==` would.
                // First CHARACTER of the entry against the needle character:
                // comparing first bytes made "é" and any other character
                // sharing a lead byte look equal.
                if (!entry || entry->empty() || utf8CharAt(*entry, 0) != needleChars[i]) continue;
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

// upstream builtin_lookup, line for line: a non-finite key warns; the
// first row must be exactly two numbers or the answer is undef; later rows
// that aren't are skipped; the low/high candidates are picked in table
// order (the first of equal keys wins), then interpolated. This used to
// sort the table, coerce malformed rows, and accept inf/nan keys silently.
Value builtinLookup(Evaluator& ev, const CallArgs& args, const oscad::Position* pos) {
    const Value keyArg = getArg(args, 0, "key", Value{});
    const double p = std::get<double>(keyArg);  // checkBuiltinArgs: a number
    if (!std::isfinite(p)) {
        ev.warn("lookup(" + fmtValue(keyArg) + ", ...) first argument is not a number", pos);
        return Value{};
    }
    const auto& vec = std::get<ListPtr>(getArg(args, 1, "table", Value{}))->items;  // checkBuiltinArgs: a vector
    const auto vec2 = [](const Value& v, double& a, double& b) {
        const ListPtr* l = std::get_if<ListPtr>(&v);
        if (!l || !*l || (*l)->items.size() != 2) return false;
        const double* x = std::get_if<double>(&(*l)->items[0]);
        const double* y = std::get_if<double>(&(*l)->items[1]);
        if (!x || !y) return false;
        a = *x;
        b = *y;
        return true;
    };
    double lowP, lowV, highP, highV;
    if (vec.empty() || !vec2(vec[0], lowP, lowV)) return Value{};
    highP = lowP;
    highV = lowV;
    for (size_t k = 1; k < vec.size(); ++k) {
        double thisP, thisV;
        if (!vec2(vec[k], thisP, thisV)) continue;
        if (thisP <= p && (thisP > lowP || lowP > p)) {
            lowP = thisP;
            lowV = thisV;
        }
        if (thisP >= p && (thisP < highP || highP < p)) {
            highP = thisP;
            highV = thisV;
        }
    }
    if (p <= lowP) return Value{highV};
    if (p >= highP) return Value{lowV};
    const double f = (p - lowP) / (highP - lowP);
    return Value{highV * f + lowV * (1 - f)};
}

std::string asStringOr(const Value& v, const std::string& fallback) {
    const std::string* s = std::get_if<std::string>(&v);
    return s ? *s : fallback;
}

Value objectOf(std::vector<std::pair<std::string, Value>> items) {
    return Value{std::make_shared<const ValueObject>(ValueObject{std::move(items)})};
}

// textmetrics(text=, size=10, font=, direction=, language=, script=,
// halign=, valign=, spacing=, em=) -- measures `text` against the
// FontProvider-resolved font, laid out by exactly the same shaping call
// text() makes, and returns an OscObject with position/size/ascent/
// descent/offset/advance in real OpenSCAD's key order. Going through the
// same measureText() the geometry goes through is the point: the numbers
// a script positions against cannot disagree with what it draws.
Value builtinTextmetrics(Evaluator& ev, const CallArgs& args, const oscad::Position* pos) {
    const std::string text = asStringOr(getArg(args, 0, "text", Value{std::string("")}), "");
    const double size = textSizeArg(ev, args, 1, 9, "textmetrics", pos);
    // Positional indices, all of them: OpenSCAD's own signature is
    // textmetrics(text, size, font, direction, language, script, halign,
    // valign, spacing, em) and it honours every one of those positionally --
    // verified against the 2026.02.01 binary, not read off a doc page.
    // `font` being name-only here is what made
    // `textmetrics("Hi", 10, "Liberation Sans:style=Bold")` silently
    // measure the default face (BelfrySCAD #381).
    const std::string halign = asStringOr(getArg(args, 6, "halign", Value{std::string("default")}), "default");
    const std::string valign = asStringOr(getArg(args, 7, "valign", Value{std::string("default")}), "default");
    const double spacing = toDoubleLenient(getArg(args, 8, "spacing", Value{1.0}));
    const std::string fontSpec = asStringOr(getArg(args, 2, "font", Value{std::string("")}), "");
    ShapeOptions shape;
    shape.direction = asStringOr(getArg(args, 3, "direction", Value{std::string("")}), "");
    shape.language = asStringOr(getArg(args, 4, "language", Value{std::string("")}), "");
    shape.script = asStringOr(getArg(args, 5, "script", Value{std::string("")}), "");

    FontProvider& fp = ev.fontProvider();
    const FontHandle handle = fp.resolveFont(fontSpec);
    const TextMeasurement m = measureText(fp, handle, text, size, spacing, shape);
    const auto [offsetX, offsetY] = textAlignOffset(
        halign, valign, m, [&](const std::string& w) { ev.warn(w, pos); });

    return objectOf({
        {"position", numList({offsetX + m.left, offsetY + m.bottom})},
        {"size", numList({m.right - m.left, m.top - m.bottom})},
        {"ascent", Value{m.ascent}},
        {"descent", Value{m.descent}},
        {"offset", numList({offsetX, offsetY})},
        {"advance", numList({m.advanceX, m.advanceY})},
    });
}

// fontmetrics(size=10, font=, em=) -- global metrics of the FontProvider-
// resolved font, scaled for `size`. Mirrors _builtin_fontmetrics.
Value builtinFontmetrics(Evaluator& ev, const CallArgs& args, const oscad::Position* pos) {
    const double size = textSizeArg(ev, args, 0, 2, "fontmetrics", pos);
    // Positional too: fontmetrics(10, "Liberation Sans:style=Bold") is
    // how the bug report was written, and how the real binary reads it.
    const std::string fontSpec = asStringOr(getArg(args, 1, "font", Value{std::string("")}), "");

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
// BuiltinFnId itself now lives in function_builtins.hpp -- see builtinFnIdFor.

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
        {"levelset", 1.0},           // levelset(field, bounds, isovalue)
        {"polyhedron-vnf", 1.0},     // polyhedron(vnf) / polyhedron(object)
        {"separate-children", 1.0},  // children(..., separate=true)
        {"minkowski-diff", 1.0},     // minkowski_difference()
        {"sphere-styles", 1.0},      // sphere(style=)
        {"export-name", 1.0},        // $export_name
        {"simplify-op", 1.0},        // simplify()
        {"mesh-repair", 1.0},        // mesh_repair(), and import(tolerance=)
        {"expr-import", 1.0},        // import() in expression position
        {"object-function", 1.0},    // object(), unconditional here
        {"roof-op", 1.0},            // roof(), method="voronoi" only
        {"svg-class", 1.0},          // import(svg, class=); id= is upstream's
        {"discretization-by-error", 1.0}, // $fe, always on (upstream: behind --enable)
        {"profile-time", 1.0},       // profile_time(label) { ... } and profile_time(label) expr
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
                                BuiltinFnId::Round, BuiltinFnId::Sqrt, BuiltinFnId::Ln,
                                BuiltinFnId::Exp, BuiltinFnId::Sin, BuiltinFnId::Cos, BuiltinFnId::Tan,
                                BuiltinFnId::Asin, BuiltinFnId::Acos, BuiltinFnId::Atan}) {
            add(id, {1, 1, "1", {kNum}});
        }
        add(BuiltinFnId::Atan2, {2, 2, "2", {kNum, kNum}});
        // log(x) or log(base, x); upstream's count warning names 2.
        add(BuiltinFnId::Log, {1, 2, "2", {kNum, kNum}});
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
    const size_t nPos = positionalCount(args);
    const size_t count = nPos + args.named.size();
    if (count < 1) {
        warnArity(ev, name, "at least 1", count, pos);
        return false;
    }
    if (nPos == 1 && std::holds_alternative<ListPtr>(positionalAt(args, 0))) {
        static const ListItems kEmptyItems;
        const ListPtr& l = std::get<ListPtr>(positionalAt(args, 0));
        const ListItems& items = l ? l->items : kEmptyItems;
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
    for (size_t i = 0; i < nPos; ++i) {
        if (!std::holds_alternative<double>(positionalAt(args, i))) {
            warnConversion(ev, name, "argument " + std::to_string(i), "number", positionalAt(args, i), pos);
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

    const size_t nPos = positionalCount(args);
    const size_t count = nPos + args.named.size();
    if (c.arityText && (static_cast<int>(count) < c.minArgs || (c.maxArgs >= 0 && static_cast<int>(count) > c.maxArgs))) {
        warnArity(ev, name, c.arityText, count, pos);
        return false;
    }
    // Positional only: a named argument's position in the reference's own
    // flat argument list can't be reconstructed from our split
    // positional/named form, and warning on the wrong index would be worse
    // than staying quiet on a spelling nobody uses for these builtins.
    for (size_t i = 0; i < c.types.size() && i < nPos; ++i) {
        const Value& v = positionalAt(args, i);
        if (!(typeBit(v) & c.types[i].allowed)) {
            warnConversion(ev, name, "argument " + std::to_string(i), c.types[i].expected, v, pos);
            return false;
        }
    }
    return true;
}


} // namespace

BuiltinFnId builtinFnIdFor(const std::string& name) {
    const auto& ids = builtinFnIds();
    const auto it = ids.find(name);
    return it == ids.end() ? BuiltinFnId::None : it->second;
}

Value evalBuiltinFunctionInOrder(Evaluator& ev, BuiltinFnId id, const std::string& name, const CallArgs& args,
                                 const oscad::ASTNode& node);

Value evalBuiltinFunction(Evaluator& ev, const std::string& name, const CallArgs& args, const oscad::ASTNode& node) {
    return evalBuiltinFunctionResolved(ev, builtinFnIdFor(name), builtinParamNames(name), name, args, node);
}

Value evalBuiltinFunctionResolved(Evaluator& ev, BuiltinFnId id, const std::vector<std::string>* declaredParams,
                                   const std::string& name, const CallArgs& args, const oscad::ASTNode& node) {
    // Not one of the names this function handles (e.g. "object", routed
    // elsewhere before reaching here) -- mirrors the old chain's fallthrough.
    if (id == BuiltinFnId::None) return Value{};

    // Only textmetrics/fontmetrics have an entry -- every other builtin
    // function reads its arguments positionally upstream and warns about
    // nothing. See builtinParamNames (registry.cpp).
    if (const std::vector<std::string>* declared = declaredParams) {
        for (const auto& [argName, _] : args.named) {
            if (!isConfigVariable(argName) &&
                std::find(declared->begin(), declared->end(), argName) == declared->end()) {
                warnUnexpectedNamedArg(ev, argName, &node.position());
            }
        }
    }

    // Every other builtin function reads its arguments IN ORDER and ignores
    // their names, exactly as upstream's builtin_* functions index
    // `arguments[i]`: sin(a=90) is sin(90), pow(y=3, x=2) is 3^2, and
    // rands(0, 1, 2, seed_value=5) is seeded. Names used to bind by this
    // port's own spelling and drop the rest silently -- sin(a=90) was 0,
    // concat(a=[1], b=[2]) was [], rands(seed_value=) went unseeded.
    // (textmetrics/fontmetrics -- with declaredParams -- and dxf_dim/
    // dxf_cross parse names upstream, through Parameters::parse.)
    const bool byName = declaredParams || id == BuiltinFnId::DxfDim || id == BuiltinFnId::DxfCross;
    CallArgs ordered;
    if (!byName && node.kind() == oscad::NodeKind::PrimaryCall && !args.named.empty()) {
        int k = 0, out = 0;
        for (const auto& a : static_cast<const oscad::PrimaryCall&>(node).arguments) {
            const Value* v = nullptr;
            if (a->kind() == oscad::NodeKind::PositionalArgument) {
                v = args.findPositional(k++);
            } else {
                const std::string& n = static_cast<const oscad::NamedArgument&>(*a).name->name;
                if (isConfigVariable(n)) continue;  // a $-override, not an argument
                v = args.findNamed(n);
            }
            ordered.setPositional(out++, v ? *v : Value{});
        }
    }
    const CallArgs& argsInOrder = (!byName && !args.named.empty()) ? ordered : args;
    return evalBuiltinFunctionInOrder(ev, id, name, argsInOrder, node);
}

Value evalBuiltinFunctionInOrder(Evaluator& ev, BuiltinFnId id, const std::string& name, const CallArgs& args,
                                 const oscad::ASTNode& node) {
    // Arity and argument types, with the reference's own two diagnostics.
    // This replaced a silent version of the same gate (scalarNumericArity /
    // numericOnlyNames), which returned undef without ever saying why.
    if (!checkBuiltinArgs(ev, name, id, args, &node.position())) return Value{};

    switch (id) {
        case BuiltinFnId::None: return Value{}; // unreachable: handled above
        case BuiltinFnId::TextMetrics: return builtinTextmetrics(ev, args, &node.position());
        case BuiltinFnId::FontMetrics: return builtinFontmetrics(ev, args, &node.position());
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
            // std::round, as upstream: halves away from zero, exactly. The
            // floor(x + 0.5) this was rounded 0.49999999999999994 up to 1
            // (the sum rounds to 1.0) and broke integers above 2^52.
            return Value{std::round(x)};
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
            // upstream builtin_log: log(y) / log(base), base 10 by default --
            // that formula, not log10(), so results agree to the last bit.
            // The two-argument form was missing (log(2, 8) was undef).
            const bool twoArgs = positionalCount(args) == 2;
            const double base = twoArgs ? toDoubleLenient(getArg(args, 0, "base", Value{})) : 10.0;
            const double y = toDoubleLenient(getArg(args, twoArgs ? 1 : 0, "x", Value{}));
            return Value{std::log(y) / std::log(base)};
        }
        case BuiltinFnId::Exp: return Value{std::exp(toDoubleLenient(getArg(args, 0, "x", Value{})))};
        case BuiltinFnId::Sin: return Value{sinDegrees(toDoubleLenient(getArg(args, 0, "x", Value{})))};
        case BuiltinFnId::Cos: return Value{cosDegrees(toDoubleLenient(getArg(args, 0, "x", Value{})))};
        case BuiltinFnId::Tan: return Value{tanDegrees(toDoubleLenient(getArg(args, 0, "x", Value{})))};
        case BuiltinFnId::Asin: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            return Value{std::fabs(x) > 1 ? std::numeric_limits<double>::quiet_NaN() : asinDegrees(x)};
        }
        case BuiltinFnId::Acos: {
            const double x = toDoubleLenient(getArg(args, 0, "x", Value{}));
            return Value{std::fabs(x) > 1 ? std::numeric_limits<double>::quiet_NaN() : acosDegrees(x)};
        }
        case BuiltinFnId::Atan: return Value{atanDegrees(toDoubleLenient(getArg(args, 0, "x", Value{})))};
        case BuiltinFnId::Atan2:
            return Value{atan2Degrees(toDoubleLenient(getArg(args, 0, "y", Value{})),
                                       toDoubleLenient(getArg(args, 1, "x", Value{})))};
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
            // upstream builtin_cross, check for check: two 2-vectors are
            // multiplied as they are (a non-number counts as 0, inf and nan
            // propagate -- no checks at all); otherwise both must be
            // 3-vectors, and each element pair is checked in order for a
            // non-number, then NaN, then infinity, each with its own warning.
            const Value a = getArg(args, 0, "a", Value{});
            const Value b = getArg(args, 1, "b", Value{});
            const auto& va = std::get<ListPtr>(a)->items;
            const auto& vb = std::get<ListPtr>(b)->items;
            const auto num = [](const Value& v) {
                const double* d = std::get_if<double>(&v);
                return d ? *d : 0.0;
            };
            if (va.size() == 2 && vb.size() == 2) return Value{num(va[0]) * num(vb[1]) - num(va[1]) * num(vb[0])};
            if (va.size() != 3 || vb.size() != 3) {
                ev.warn("Invalid vector size of parameter for cross()", &node.position());
                return Value{};
            }
            for (size_t k = 0; k < 3; ++k) {
                if (!std::holds_alternative<double>(va[k]) || !std::holds_alternative<double>(vb[k])) {
                    ev.warn("Invalid value in parameter vector for cross()", &node.position());
                    return Value{};
                }
                const double d0 = std::get<double>(va[k]), d1 = std::get<double>(vb[k]);
                if (std::isnan(d0) || std::isnan(d1)) {
                    ev.warn("Invalid value (NaN) in parameter vector for cross()", &node.position());
                    return Value{};
                }
                if (std::isinf(d0) || std::isinf(d1)) {
                    ev.warn("Invalid value (INF) in parameter vector for cross()", &node.position());
                    return Value{};
                }
            }
            return numList({num(va[1]) * num(vb[2]) - num(va[2]) * num(vb[1]),
                            num(va[2]) * num(vb[0]) - num(va[0]) * num(vb[2]),
                            num(va[0]) * num(vb[1]) - num(va[1]) * num(vb[0])});
        }
        case BuiltinFnId::Rands: {
            ev.noteRandsCall();
            return builtinRands(ev, toDoubleLenient(getArg(args, 0, "min_value", Value{})),
                                toDoubleLenient(getArg(args, 1, "max_value", Value{})),
                                toDoubleLenient(getArg(args, 2, "value_count", Value{})), getArg(args, 3, "seed", Value{}),
                                &node.position());
        }
        case BuiltinFnId::Concat: {
            // Everything after the first argument, appended to the first
            // when it is a list: listAppend extends its buffer in place when
            // it can, so an accumulator loop is linear, not quadratic.
            const size_t count = positionalCount(args);
            const ListPtr* first = count ? std::get_if<ListPtr>(&positionalAt(args, 0)) : nullptr;
            std::vector<Value> out;
            for (size_t i = (first && *first) ? 1 : 0; i < count; ++i) {
                const Value& a = positionalAt(args, i);
                if (const ListPtr* l = std::get_if<ListPtr>(&a); l && *l) {
                    out.insert(out.end(), (*l)->items.begin(), (*l)->items.end());
                } else {
                    out.push_back(a);
                }
            }
            if (first && *first) return Value{listAppend(*first, std::move(out))};
            return listOf(std::move(out));
        }
        case BuiltinFnId::Len: {
            const Value x = getArg(args, 0, "x", Value{});
            if (const ListPtr* l = std::get_if<ListPtr>(&x); l && *l) return Value{static_cast<double>((*l)->items.size())};
            // Characters, not bytes: the reference reports len("aé—z") as 4.
            if (const std::string* s = std::get_if<std::string>(&x)) return Value{static_cast<double>(utf8Length(*s))};
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
        case BuiltinFnId::Lookup: return builtinLookup(ev, args, &node.position());
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
        case BuiltinFnId::LinearSolve: {
            // An explicitly-undef b counts as ABSENT, not as a bad argument.
            // A wrapper with a fixed signature -- BOSL2's builtins.scad
            // pattern, `function _linear_solve(A, b) = linear_solve(A, b);`
            // -- always forwards b, so determinant()-style callers that pass
            // no right-hand side would otherwise warn on every call.
            // Passing undef is idiomatically the same as not passing in
            // OpenSCAD, which is how optional arguments are threaded through
            // wrappers throughout BOSL2.
            const Value bArg = getArg(args, 1, "b", Value{});
            const bool haveB = !std::holds_alternative<std::monostate>(bArg);
            return builtinLinearSolve(ev, getArg(args, 0, "A", Value{}), bArg, haveB, node);
        }
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

// OpenSCAD's sin_degrees/cos_degrees (exact at multiples of 30 and 45
// degrees), for the transforms. See builtins.hpp.
double sinDeg(double x) { return sinDegrees(x); }
double cosDeg(double x) { return cosDegrees(x); }

} // namespace oscadeval
