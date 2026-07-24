#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

// Evaluates `code` as a bare expression (`x = {code};`) against a fresh
// root EvalContext built from the wrapped AST's own scope.
Value evalSrc(const std::string& code, Evaluator& ev) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    const oscad::Expression* expr = exprSrc(code, ast);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    return ev.evalExpr(*expr, ctx);
}

double asNum(const Value& v) { return std::get<double>(v); }
bool asBool(const Value& v) { return std::get<bool>(v); }

} // namespace

// -- Literals -------------------------------------------------------------

TEST(ExprEvalLiterals, NumberBoolStringUndef) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("42", ev)), 42.0);
    EXPECT_TRUE(asBool(evalSrc("true", ev)));
    EXPECT_FALSE(asBool(evalSrc("false", ev)));
    EXPECT_EQ(std::get<std::string>(evalSrc("\"hi\"", ev)), "hi");
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("undef", ev)));
}

TEST(ExprEvalLiterals, PiConstant) {
    Evaluator ev;
    EXPECT_NEAR(asNum(evalSrc("PI", ev)), 3.14159265358979, 1e-12);
}

// -- Identifiers ------------------------------------------------------------

TEST(ExprEvalIdentifiers, UnknownVariableWarnsAndIsUndef) {
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    Value v = evalSrc("nope", ev);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
    EXPECT_NE(lastWarning.find("Ignoring unknown variable 'nope'"), std::string::npos);
}

TEST(ExprEvalIdentifiers, ResolvesViaScopeFallbackWhenNotEagerlyBound) {
    // "a" is a real top-level declaration but evalChildren() was never run,
    // so ctx.let_ has no eager binding for it -- this exercises
    // scope->lookupVariable()'s lazy-evaluation fallback path, distinct
    // from the eager ctx.let_ path exercised by test_scoping.cpp.
    Evaluator ev;
    auto ast = parseSrc("a = 5;\nb = a + 1;");
    auto scope = oscad::buildScopes(ast);
    auto* assignB = dynamic_cast<oscad::Assignment*>(ast[1].get());
    ASSERT_NE(assignB, nullptr);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    Value v = ev.evalExpr(*assignB->expr, ctx);
    EXPECT_DOUBLE_EQ(asNum(v), 6.0);
}

// -- Arithmetic -------------------------------------------------------------

TEST(ExprEvalArithmetic, OperatorPrecedence) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("1 + 2 * 3", ev)), 7.0);
}

TEST(ExprEvalArithmetic, DivisionByZeroIsIeee754) {
    Evaluator ev;
    EXPECT_TRUE(std::isnan(asNum(evalSrc("0 / 0", ev))));
    Value posInf = evalSrc("1 / 0", ev);
    EXPECT_TRUE(std::isinf(asNum(posInf)) && asNum(posInf) > 0);
    Value negInf = evalSrc("-1 / 0", ev);
    EXPECT_TRUE(std::isinf(asNum(negInf)) && asNum(negInf) < 0);
}

TEST(ExprEvalArithmetic, ModuloTakesDivisorSign) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("-1 % 3", ev)), 2.0); // Python-style, not C++ fmod's -1
}

TEST(ExprEvalArithmetic, ExponentZeroToNegativeIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("0 ^ -1", ev)));
}

TEST(ExprEvalArithmetic, BoolOperandsAreUndef) {
    Evaluator ev;
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("true + 1", ev)));
}

TEST(ExprEvalArithmetic, VectorPlusVector) {
    Evaluator ev;
    Value v = evalSrc("[1,2,3] + [10,20,30]", ev);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), 11.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 33.0);
}

TEST(ExprEvalArithmetic, StringPlusStringIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("\"ab\" + \"cd\"", ev)));
}

TEST(ExprEvalArithmetic, ScalarTimesVector) {
    Evaluator ev;
    Value v = evalSrc("2 * [1,2,3]", ev);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[1]), 4.0);
}

TEST(ExprEvalArithmetic, VectorDotVectorIsScalar) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("[1,2,3] * [4,5,6]", ev)), 32.0);
}

TEST(ExprEvalArithmetic, UnaryMinusOnListNegatesElementwise) {
    Evaluator ev;
    Value v = evalSrc("-[1,-2,3]", ev);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), -1.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 2.0);
}

// -- Logical operators (eager, not short-circuit) ----------------------------

TEST(ExprEvalLogical, AndOrNot) {
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("true && true", ev)));
    EXPECT_FALSE(asBool(evalSrc("true && false", ev)));
    EXPECT_TRUE(asBool(evalSrc("false || true", ev)));
    EXPECT_TRUE(asBool(evalSrc("!false", ev)));
}

TEST(ExprEvalLogical, BothSidesAlwaysEvaluateEvenWhenShortCircuitable) {
    // The reference implementation's && / || evaluate both operands
    // unconditionally (no short-circuit) -- verified via an unknown-
    // variable warning firing on the right side of `false && nope`, which
    // a short-circuiting implementation would never touch.
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    evalSrc("false && nope", ev);
    EXPECT_NE(lastWarning.find("nope"), std::string::npos);
}

// -- Comparisons ------------------------------------------------------------

TEST(ExprEvalComparisons, NumbersAndStrings) {
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("1 == 1", ev)));
    EXPECT_TRUE(asBool(evalSrc("1 != 2", ev)));
    EXPECT_TRUE(asBool(evalSrc("2 > 1", ev)));
    EXPECT_TRUE(asBool(evalSrc("1 <= 1", ev)));
    EXPECT_TRUE(asBool(evalSrc("\"a\" < \"b\"", ev)));
}

TEST(ExprEvalComparisons, BoolIsNotEqualToNumber) {
    Evaluator ev;
    EXPECT_FALSE(asBool(evalSrc("1 == true", ev)));
}

TEST(ExprEvalComparisons, MismatchedTypesWarnAndUndef) {
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    Value v = evalSrc("true > 0", ev);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
    EXPECT_NE(lastWarning.find("undefined operation (bool > number)"), std::string::npos);
}

TEST(ExprEvalComparisons, VectorLexicographic) {
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("[1,2] < [1,3]", ev)));
    EXPECT_TRUE(asBool(evalSrc("[1] < [1,2]", ev))); // shorter prefix-equal list is less
}

// -- Ternary ------------------------------------------------------------

TEST(ExprEvalTernary, PicksCorrectBranch) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("true ? 1 : 2", ev)), 1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("false ? 1 : 2", ev)), 2.0);
}

TEST(ExprEvalTernary, OnlyChosenBranchEvaluates) {
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    evalSrc("true ? 1 : nope", ev);
    EXPECT_TRUE(lastWarning.empty());
}

// -- Ranges -------------------------------------------------------------

TEST(ExprEvalRanges, StartStepEnd) {
    Evaluator ev;
    Value v = evalSrc("[2:3:11]", ev);
    OscRange r = std::get<OscRange>(v);
    EXPECT_DOUBLE_EQ(r.start, 2.0);
    EXPECT_DOUBLE_EQ(r.step, 3.0);
    EXPECT_DOUBLE_EQ(r.end, 11.0);
}

TEST(ExprEvalRanges, DefaultStepIsOne) {
    Evaluator ev;
    Value v = evalSrc("[1:5]", ev);
    OscRange r = std::get<OscRange>(v);
    EXPECT_DOUBLE_EQ(r.step, 1.0);
}

// -- Vector literals ----------------------------------------------------

TEST(ExprEvalVectorLiterals, PlainElements) {
    Evaluator ev;
    Value v = evalSrc("[1, 2, 3]", ev);
    auto items = std::get<ListPtr>(v)->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 3.0);
}

TEST(ExprEvalVectorLiterals, NestedAndMixedTypes) {
    Evaluator ev;
    Value v = evalSrc("[1, \"a\", [2, 3]]", ev);
    auto items = std::get<ListPtr>(v)->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_EQ(std::get<std::string>(items[1]), "a");
    auto nested = std::get<ListPtr>(items[2])->items;
    EXPECT_DOUBLE_EQ(asNum(nested[0]), 2.0);
}

// -- Math builtin dispatch (Phase 5) -----------------------------------

TEST(ExprEvalMathBuiltins, SinAtExactMultipleOfNinety) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("sin(0)", ev)), 0.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("cos(90)", ev)), 0.0); // not 6.12e-17
}

// -- Bitwise operators ----------------------------------------------------

// Real OpenSCAD added |, &, ~, <<, >> in PR #4833 (merged 2025-03-14, "Fixes
// #3345"). Semantics/test cases here are ported directly from that PR's own
// tests/data/scad/functions/bitwise-operators.scad: truncate-to-int64
// operands, int64 two's-complement arithmetic, cast back to double.

TEST(ExprEvalBitwiseOps, BasicOperations) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("12 & 5", ev)), 4.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("12 | 5", ev)), 13.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("2 << 3", ev)), 16.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("16 >> 2", ev)), 4.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("~0", ev)), -1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("~-1", ev)), 0.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("~5", ev)), -6.0);
}

TEST(ExprEvalBitwiseOps, Precedence) {
    // Shift binds tighter than binary-and/or but looser than addition;
    // binary-and binds tighter than binary-or but looser than shift;
    // binary-or binds tighter than comparison -- real OpenSCAD's grammar
    // deliberately differs from C here specifically to avoid `x & 1 == 0`
    // meaning `x & (1 == 0)`. Each pair below is the same assertion style
    // the real PR's own regression test uses: the explicitly-parenthesized
    // form must equal the bare form, and the *other* grouping must not.
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("1 << 3 + 1", ev)), asNum(evalSrc("1 << (3 + 1)", ev)));
    EXPECT_DOUBLE_EQ(asNum(evalSrc("3 << 1 & 14", ev)), asNum(evalSrc("(3 << 1) & 14", ev)));
    EXPECT_DOUBLE_EQ(asNum(evalSrc("1 & 2 | 3", ev)), asNum(evalSrc("(1 & 2) | 3", ev)));
    EXPECT_TRUE(asBool(evalSrc("(1 | 2 > 3) == ((1 | 2) > 3)", ev)));
}

TEST(ExprEvalBitwiseOps, NegativeNumbersAndFractions) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("-1 | 0", ev)), -1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("1.4 | 0", ev)), 1.0);  // trunc, not round
    EXPECT_DOUBLE_EQ(asNum(evalSrc("1.6 | 0", ev)), 1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("-1.4 & 3", ev)), 3.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("-1.6 & 3", ev)), 3.0);
}

TEST(ExprEvalBitwiseOps, ShiftOverflowWrapsLike64BitTwosComplement) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("1 << 32 << 32", ev)), 0.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("1 >> 1", ev)), 0.0);
}

TEST(ExprEvalBitwiseOps, OutOfRangeShiftIsUndefAndWarns) {
    std::vector<std::string> echoed;
    Evaluator ev([&](const std::string& m) { echoed.push_back(m); });
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("1 << 64", ev)));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("1 << -1", ev)));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("1 >> 64", ev)));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("1 >> -1", ev)));
    ASSERT_EQ(echoed.size(), 4u);
    EXPECT_NE(echoed[0].find("WARNING: shift too large"), std::string::npos);
    EXPECT_NE(echoed[1].find("WARNING: negative shift"), std::string::npos);
    EXPECT_NE(echoed[2].find("WARNING: shift too large"), std::string::npos);
    EXPECT_NE(echoed[3].find("WARNING: negative shift"), std::string::npos);
}

TEST(ExprEvalBitwiseOps, NonNumericOperandIsUndefAndWarns) {
    std::vector<std::string> echoed;
    Evaluator ev([&](const std::string& m) { echoed.push_back(m); });
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("1 | \"hello\"", ev)));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("1 & true", ev)));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("1 << \"hello\"", ev)));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("1 >> \"hello\"", ev)));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("~\"hello\"", ev)));
    ASSERT_EQ(echoed.size(), 5u);
    EXPECT_NE(echoed[0].find("WARNING: undefined operation (number | string)"), std::string::npos);
    EXPECT_NE(echoed[1].find("WARNING: undefined operation (number & bool)"), std::string::npos);
    EXPECT_NE(echoed[4].find("WARNING: undefined operation (~string)"), std::string::npos);
}
