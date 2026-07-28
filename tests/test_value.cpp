#include "openscad_cpp_evaluator/value.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace oscadeval;

namespace {

Value list(std::vector<Value> items) {
    return Value{std::make_shared<const ValueList>(ValueList{std::move(items)})};
}

Value num(double d) { return Value{d}; }

double asNum(const Value& v) {
    return std::get<double>(v);
}

} // namespace

// -- oscTypeName --------------------------------------------------------

TEST(OscTypeName, CoversEveryDistinguishedAlternative) {
    EXPECT_EQ(oscTypeName(Value{}), "undefined");
    EXPECT_EQ(oscTypeName(Value{true}), "bool");
    EXPECT_EQ(oscTypeName(Value{1.0}), "number");
    EXPECT_EQ(oscTypeName(Value{std::string("x")}), "string");
    EXPECT_EQ(oscTypeName(list({})), "vector");
    EXPECT_EQ(oscTypeName(Value{std::make_shared<const ValueObject>()}), "object");
    EXPECT_EQ(oscTypeName(Value{OscRange{0, 1, 5}}), "undefined");
}

// -- oscEqual -------------------------------------------------------------

TEST(OscEqual, RangeComparesComponentwise) {
    // OscRange::operator== -- oscEqual falls through to the variant's own
    // operator== when neither side is a list/object, which for two
    // OscRange values invokes this.
    EXPECT_TRUE(oscEqual(Value{OscRange{1, 2, 5}}, Value{OscRange{1, 2, 5}}));
    EXPECT_FALSE(oscEqual(Value{OscRange{1, 2, 5}}, Value{OscRange{1, 2, 6}}));
}

TEST(OscEqual, BoolIsDistinctFromNumber) {
    EXPECT_FALSE(oscEqual(Value{true}, Value{1.0}));
    EXPECT_FALSE(oscEqual(Value{false}, Value{0.0}));
    EXPECT_TRUE(oscEqual(Value{true}, Value{true}));
}

TEST(OscEqual, NumbersAndStrings) {
    EXPECT_TRUE(oscEqual(num(1.0), num(1.0)));
    EXPECT_FALSE(oscEqual(num(1.0), num(2.0)));
    EXPECT_TRUE(oscEqual(Value{std::string("a")}, Value{std::string("a")}));
    EXPECT_FALSE(oscEqual(Value{std::string("a")}, Value{std::string("b")}));
}

TEST(OscEqual, UndefEqualsUndef) {
    EXPECT_TRUE(oscEqual(Value{}, Value{}));
}

TEST(OscEqual, ListsRecurseElementwiseAndRejectMismatchedType) {
    EXPECT_TRUE(oscEqual(list({num(1), num(2)}), list({num(1), num(2)})));
    EXPECT_FALSE(oscEqual(list({num(1), num(2)}), list({num(1), num(3)})));
    EXPECT_FALSE(oscEqual(list({num(1), Value{true}}), list({num(1), num(1)})));
    EXPECT_FALSE(oscEqual(list({num(1)}), list({num(1), num(2)}))); // mismatched length
    EXPECT_FALSE(oscEqual(list({}), num(0))); // one list, one not
}

TEST(OscEqual, ObjectEqualityIsOrderSensitive) {
    auto a = std::make_shared<const ValueObject>(ValueObject{{{"a", num(1)}, {"b", num(2)}}});
    auto sameOrder = std::make_shared<const ValueObject>(ValueObject{{{"a", num(1)}, {"b", num(2)}}});
    auto differentOrder = std::make_shared<const ValueObject>(ValueObject{{{"b", num(2)}, {"a", num(1)}}});
    EXPECT_TRUE(oscEqual(Value{a}, Value{sameOrder}));
    EXPECT_FALSE(oscEqual(Value{a}, Value{differentOrder}));
}

// -- oscComparable ----------------------------------------------------------

TEST(OscComparable, SameTypePairsOnly) {
    EXPECT_TRUE(oscComparable(num(1), num(2)));
    EXPECT_TRUE(oscComparable(Value{std::string("a")}, Value{std::string("b")}));
    EXPECT_TRUE(oscComparable(list({}), list({})));
    EXPECT_TRUE(oscComparable(Value{true}, Value{false}));
}

TEST(OscComparable, MismatchedTypesAreNotComparable) {
    EXPECT_FALSE(oscComparable(Value{true}, num(0)));
    EXPECT_FALSE(oscComparable(Value{std::string("a")}, num(1)));
    EXPECT_FALSE(oscComparable(list({}), num(1)));
    EXPECT_FALSE(oscComparable(Value{}, num(1)));
}

// -- scale / divScale -------------------------------------------------------

TEST(Scale, MultipliesFlatAndNestedLists) {
    Value v = scale(2.0, list({num(1), num(2), num(3)}));
    auto items = std::get<ListPtr>(v)->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 2.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 6.0);

    Value nested = scale(2.0, list({list({num(1), num(1)}), num(5)}));
    auto outer = std::get<ListPtr>(nested)->items;
    auto inner = std::get<ListPtr>(outer[0])->items;
    EXPECT_DOUBLE_EQ(asNum(inner[0]), 2.0);
    EXPECT_DOUBLE_EQ(asNum(outer[1]), 10.0);
}

TEST(Scale, BoolElementIsUndef) {
    Value v = scale(2.0, Value{true});
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
}

TEST(DivScale, DivisionByZeroFollowsIeee754) {
    Value posInf = divScale(num(1.0), 0.0);
    EXPECT_TRUE(std::isinf(asNum(posInf)) && asNum(posInf) > 0);

    Value negInf = divScale(num(-1.0), 0.0);
    EXPECT_TRUE(std::isinf(asNum(negInf)) && asNum(negInf) < 0);

    Value nan = divScale(num(0.0), 0.0);
    EXPECT_TRUE(std::isnan(asNum(nan)));
}

TEST(DivScale, OrdinaryDivision) {
    Value v = divScale(list({num(2), num(4), num(6)}), 2.0);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 3.0);
}

// -- vecAdd / vecSub ----------------------------------------------------

TEST(VecAdd, ElementwiseListAdditionZipsToShorterLength) {
    Value v = vecAdd(list({num(1), num(2), num(3)}), list({num(10), num(20)}));
    auto items = std::get<ListPtr>(v)->items;
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 11.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 22.0);
}

TEST(VecAdd, NestedListsRecurse) {
    Value v = vecAdd(list({list({num(0), num(0)})}), list({list({num(1), num(2)})}));
    auto outer = std::get<ListPtr>(v)->items;
    auto inner = std::get<ListPtr>(outer[0])->items;
    EXPECT_DOUBLE_EQ(asNum(inner[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(inner[1]), 2.0);
}

TEST(VecAdd, RejectsStringsUnlikePythonConcatenation) {
    Value v = vecAdd(Value{std::string("ab")}, Value{std::string("cd")});
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
}

TEST(VecAdd, RejectsBoolOperands) {
    EXPECT_TRUE(std::holds_alternative<std::monostate>(vecAdd(Value{true}, num(1))));
}

TEST(VecAdd, PlainNumberFallback) {
    Value v = vecAdd(num(1), num(2));
    EXPECT_DOUBLE_EQ(asNum(v), 3.0);
}

TEST(VecSub, PlainNumberFallback) {
    Value v = vecSub(num(5), num(2));
    EXPECT_DOUBLE_EQ(asNum(v), 3.0);
}

TEST(VecSub, MismatchedTypesAreUndef) {
    EXPECT_TRUE(std::holds_alternative<std::monostate>(vecSub(list({num(1)}), num(1))));
}

// -- matmul -------------------------------------------------------------

TEST(Matmul, VectorDotVectorIsScalar) {
    Value v = matmul(list({num(1), num(2), num(3)}), list({num(4), num(5), num(6)}));
    EXPECT_DOUBLE_EQ(asNum(v), 32.0); // 1*4 + 2*5 + 3*6
}

TEST(Matmul, MatrixTimesVector) {
    // [[1,0],[0,1]] * [3,4] = [3,4]
    Value m = list({list({num(1), num(0)}), list({num(0), num(1)})});
    Value v = matmul(m, list({num(3), num(4)}));
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), 3.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 4.0);
}

TEST(Matmul, VectorTimesMatrix) {
    Value m = list({list({num(1), num(2)}), list({num(3), num(4)})});
    Value v = matmul(list({num(1), num(1)}), m);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), 4.0); // 1*1 + 1*3
    EXPECT_DOUBLE_EQ(asNum(items[1]), 6.0); // 1*2 + 1*4
}

TEST(Matmul, MatrixTimesMatrix) {
    Value a = list({list({num(1), num(2)}), list({num(3), num(4)})});
    Value b = list({list({num(5), num(6)}), list({num(7), num(8)})});
    Value r = matmul(a, b);
    auto rows = std::get<ListPtr>(r)->items;
    auto row0 = std::get<ListPtr>(rows[0])->items;
    auto row1 = std::get<ListPtr>(rows[1])->items;
    EXPECT_DOUBLE_EQ(asNum(row0[0]), 19.0); // 1*5+2*7
    EXPECT_DOUBLE_EQ(asNum(row0[1]), 22.0); // 1*6+2*8
    EXPECT_DOUBLE_EQ(asNum(row1[0]), 43.0); // 3*5+4*7
    EXPECT_DOUBLE_EQ(asNum(row1[1]), 50.0); // 3*6+4*8
}

TEST(Matmul, DimensionMismatchIsUndef) {
    Value v = matmul(list({num(1), num(2)}), list({num(1), num(2), num(3)}));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
}

// -- formatNumber ---------------------------------------------------------

TEST(FormatNumber, SpecialValues) {
    EXPECT_EQ(formatNumber(std::numeric_limits<double>::quiet_NaN()), "nan");
    EXPECT_EQ(formatNumber(std::numeric_limits<double>::infinity()), "inf");
    EXPECT_EQ(formatNumber(-std::numeric_limits<double>::infinity()), "-inf");
    EXPECT_EQ(formatNumber(0.0), "0");
    EXPECT_EQ(formatNumber(-0.0), "0");
}

TEST(FormatNumber, FixedPointWithinExponentRange) {
    EXPECT_EQ(formatNumber(1.0), "1");
    EXPECT_EQ(formatNumber(-1.0), "-1");
    EXPECT_EQ(formatNumber(0.00001), "0.00001"); // exp == -5, doc-pinned
    EXPECT_EQ(formatNumber(3.14159), "3.14159");
}

TEST(FormatNumber, ScientificNotationDropsLeadingExponentZero) {
    EXPECT_EQ(formatNumber(1000000.0), "1e+6"); // doc-pinned, not "1e+06"
    EXPECT_EQ(formatNumber(1.23456789e-7), "1.23457e-7"); // doc-pinned
}

TEST(FormatNumber, MantissaRoundingCarryBumpsExponent) {
    // 9.99999999e13's mantissa rounds to exactly 10.00000 at 5-decimal
    // precision -- the carry-fixup branch (mantissa /= 10, ++exp) corrects
    // this to "1e+14" instead of the malformed "10.00000e+13".
    EXPECT_EQ(formatNumber(9.99999999e13), "1e+14");
}

// -- fmtValue ---------------------------------------------------------------

TEST(FmtValue, Scalars) {
    EXPECT_EQ(fmtValue(Value{}), "undef");
    EXPECT_EQ(fmtValue(Value{true}), "true");
    EXPECT_EQ(fmtValue(Value{false}), "false");
    EXPECT_EQ(fmtValue(Value{42.0}), "42");
    EXPECT_EQ(fmtValue(Value{std::string("hi")}), "\"hi\"");
}

TEST(FmtValue, Range) {
    EXPECT_EQ(fmtValue(Value{OscRange{2, 1, 10}}), "[2 : 1 : 10]");
}

TEST(FmtValue, List) {
    EXPECT_EQ(fmtValue(list({num(1), num(2), num(3)})), "[1, 2, 3]");
    EXPECT_EQ(fmtValue(Value{ListPtr{}}), "[]"); // null list pointer
}

TEST(FmtValue, Object) {
    auto obj = std::make_shared<const ValueObject>(ValueObject{{{"a", num(1)}, {"b", num(2)}}});
    EXPECT_EQ(fmtValue(Value{obj}), "object(a = 1, b = 2)");
    EXPECT_EQ(fmtValue(Value{std::make_shared<const ValueObject>()}), "object()");
}

// -- toDoubleLenient --------------------------------------------------------

TEST(ToDoubleLenient, BoolAndFallback) {
    EXPECT_DOUBLE_EQ(toDoubleLenient(Value{true}), 1.0);
    EXPECT_DOUBLE_EQ(toDoubleLenient(Value{false}), 0.0);
    EXPECT_DOUBLE_EQ(toDoubleLenient(Value{std::string("x")}), 0.0); // non-numeric fallback
}

// -- truthy -------------------------------------------------------------

TEST(Truthy, RangeAndFunctionLiteralAreAlwaysTrue) {
    EXPECT_TRUE(truthy(Value{OscRange{0, 1, 5}}));
}

// -- scale / divScale: non-numeric list-element fallbacks --------------------

TEST(Scale, NonNumericListElementIsUndef) {
    Value v = scale(2.0, list({num(1), Value{std::string("x")}, num(3)}));
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), 2.0);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(items[1]));
}

TEST(DivScale, NonNumericListElementIsUndef) {
    Value v = divScale(list({num(4), Value{std::string("x")}}), 2.0);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), 2.0);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(items[1]));
}

// -- expandIterable -----------------------------------------------------

TEST(ExpandIterable, ObjectExpandsToItsKeys) {
    auto obj = std::make_shared<const ValueObject>(ValueObject{{{"a", num(1)}, {"b", num(2)}}});
    auto items = expandIterable(Value{obj});
    ASSERT_EQ(items.size(), 2u);
    EXPECT_EQ(std::get<std::string>(items[0]), "a");
    EXPECT_EQ(std::get<std::string>(items[1]), "b");
}

TEST(ExpandIterable, StringExpandsToItsCharacters) {
    auto items = expandIterable(Value{std::string("ab")});
    ASSERT_EQ(items.size(), 2u);
    EXPECT_EQ(std::get<std::string>(items[0]), "a");
    EXPECT_EQ(std::get<std::string>(items[1]), "b");
}

TEST(ExpandIterable, BareScalarWrapsToSingleElementList) {
    auto items = expandIterable(num(5));
    ASSERT_EQ(items.size(), 1u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 5.0);
}

TEST(ExpandIterable, DescendingRangeStepsDown) {
    auto items = expandIterable(Value{OscRange{5, -1, 2}});
    ASSERT_EQ(items.size(), 4u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 5.0);
    EXPECT_DOUBLE_EQ(asNum(items[3]), 2.0);
}

TEST(ExpandIterable, ZeroStepRangeIsEmpty) {
    auto items = expandIterable(Value{OscRange{0, 0, 10}});
    EXPECT_EQ(items.size(), 0u);
    EXPECT_FALSE(items.begin() != items.end());
}

TEST(ExpandIterable, FractionalStepRangeViaRangeForMatchesAccumulation) {
    // Values must match repeated `x += step` accumulation exactly (not a
    // recomputed start + i*step, which can round differently) -- ranges
    // are lazy now (see IterableValues), so this exercises that the
    // on-demand path reproduces the same sequence the old eager
    // expansion always produced.
    auto items = expandIterable(Value{OscRange{0.0, 0.1, 0.5}});
    double expected = 0.0;
    size_t count = 0;
    for (const Value& v : items) {
        EXPECT_DOUBLE_EQ(asNum(v), expected);
        expected += 0.1;
        ++count;
    }
    EXPECT_EQ(count, 6u); // 0.0, 0.1, 0.2, 0.3, 0.4, 0.5
}

TEST(ExpandIterable, OutOfOrderIndexAccessStillReturnsCorrectValue) {
    // Nothing in this codebase reads a range's expansion out of
    // sequence (see IterableValues's own doc comment for why the
    // sequential path is what's optimized), but an out-of-order
    // operator[] call must still be CORRECT, just slower -- confirms the
    // cursor's "restart from start" fallback rather than silently
    // returning a stale/wrong value.
    auto items = expandIterable(Value{OscRange{0, 1, 9}});
    EXPECT_DOUBLE_EQ(asNum(items[7]), 7.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 2.0); // rewinds behind the cursor's last position
    EXPECT_DOUBLE_EQ(asNum(items[5]), 5.0);
}

TEST(ExpandIterable, LargeRangePartialIterationStaysLazy) {
    // A huge (but under the 1,000,000-element "too many" limit -- see the
    // TooManyElements tests below) range must not eagerly materialize a
    // vector: constructing expandIterable() and reading only the first few
    // elements has to stay instant regardless of the range's nominal size
    // (999,999 elements here, the largest value that doesn't trigger
    // rejection). If this regresses to eager expansion, this test times
    // out/OOMs instead of merely failing an assertion.
    auto items = expandIterable(Value{OscRange{0, 1, 999'998.0}});
    auto it = items.begin();
    EXPECT_DOUBLE_EQ(asNum(*it), 0.0);
    ++it;
    EXPECT_DOUBLE_EQ(asNum(*it), 1.0);
    ++it;
    EXPECT_DOUBLE_EQ(asNum(*it), 2.0);
}

TEST(ExpandIterable, RangeUnderOneMillionElementsIteratesNormally) {
    // 999,999 elements (indices 0..999998) -- the largest range that must
    // NOT be rejected. Verified against real OpenSCAD.app: this exact size
    // iterates fine, one more element does not (see the next two tests).
    bool warned = false;
    auto items = expandIterable(Value{OscRange{0, 1, 999'998.0}}, [&](size_t) { warned = true; });
    EXPECT_FALSE(warned);
    EXPECT_EQ(items.size(), 999'999u);
}

TEST(ExpandIterable, RangeOfExactlyOneMillionElementsIsRejected) {
    // 1,000,000 elements (indices 0..999999) -- real OpenSCAD.app rejects
    // the whole range at exactly this size (WARNING + zero iterations, not
    // a truncation to 999,999) -- a different mechanism from the C-style
    // for loop's own _MAX_CFOR_ITERATIONS, which instead ALLOWS exactly
    // 1,000,000 iterations and only errors past it. Verified empirically:
    // these are genuinely different thresholds in real OpenSCAD, not a
    // copy-paste of the same constant.
    size_t warnedCount = 0;
    auto items = expandIterable(Value{OscRange{0, 1, 999'999.0}}, [&](size_t count) { warnedCount = count; });
    EXPECT_EQ(warnedCount, 1'000'000u);
    EXPECT_EQ(items.size(), 0u);
}

TEST(ExpandIterable, HugeRangeIsRejectedInConstantTime) {
    // The rejection check itself must be O(1) (closed-form), not a lazy
    // walk to find out the count -- a billion-element range must reject
    // instantly, not time out. This is exactly the case
    // LargeRangePartialIterationStaysLazy used to (wrongly) exercise as
    // "still iterable"; real OpenSCAD.app rejects it outright.
    size_t warnedCount = 0;
    auto items = expandIterable(Value{OscRange{0, 1, 1'000'000'000.0}}, [&](size_t count) { warnedCount = count; });
    EXPECT_EQ(warnedCount, 1'000'000'001u);
    EXPECT_EQ(items.size(), 0u);
}

TEST(ExpandIterable, NegativeStepRangeTooManyElementsIsRejectedToo) {
    // Verified against real OpenSCAD.app: the too-many-elements check
    // applies symmetrically to a descending range, with the correct count.
    size_t warnedCount = 0;
    auto items = expandIterable(Value{OscRange{1'099'999.0, -1, 0}}, [&](size_t count) { warnedCount = count; });
    EXPECT_EQ(warnedCount, 1'100'000u);
    EXPECT_EQ(items.size(), 0u);
}

TEST(ExpandIterable, ZeroStepRangeIsNaturallyEmptyNotTooMany) {
    // A zero step never terminates by walking, but must not be
    // misclassified as "too many" either -- it's naturally empty (0
    // elements), no warning.
    bool warned = false;
    auto items = expandIterable(Value{OscRange{0, 0, 10}}, [&](size_t) { warned = true; });
    EXPECT_FALSE(warned);
    EXPECT_EQ(items.size(), 0u);
}
