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
