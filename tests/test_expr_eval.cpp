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

TEST(ExprEvalIdentifiers, UndefinedDollarVariableIsUndefAndWarns) {
    // A never-assigned $-name reads as undef and warns, exactly like a
    // never-declared bare identifier.
    //
    // This test used to assert the opposite, on the strength of BOSL2's
    // constants.scad -- `function get_slop() = is_undef($slop) ? 0 : $slop;`
    // with $slop never assigned -- being silent in real OpenSCAD. It is,
    // but not because $-reads are exempt: is_undef() does not warn about the
    // name it probes, and the else branch never runs while is_undef() is
    // true. Measured against the reference, a PLAIN read warns in every
    // context tried -- echo, assignment, function body, module body,
    // comparison -- so the old blanket exemption was over-generalised from
    // one guarded expression.
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    Value v = evalSrc("$slop", ev);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
    EXPECT_NE(lastWarning.find("Ignoring unknown variable '$slop'"), std::string::npos) << lastWarning;
}

TEST(ExprEvalIdentifiers, TheBoslGetSlopIdiomStaysSilent) {
    // The case the old exemption existed for, kept as a test in its own
    // right: it must remain silent through the is_undef() probe alone.
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    Value v = evalSrc("is_undef($slop) ? 0 : $slop", ev);
    EXPECT_TRUE(lastWarning.empty()) << lastWarning;
    EXPECT_EQ(std::get<double>(v), 0.0);
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

TEST(ExprEvalArithmetic, ModuloTakesDividendSign) {
    // C's fmod, so the result carries the LEFT operand's sign -- NOT a
    // floored/Python modulo, which this used to (wrongly) assert. Checked
    // against OpenSCAD 2026.02.01: -7 % 3 is -1 there, 7 % -3 is 1.
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("-1 % 3", ev)), -1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("-7 % 3", ev)), -1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("7 % -3", ev)), 1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("7.5 % 2", ev)), 1.5);
}

TEST(ExprEvalArithmetic, ExponentZeroToNegativeIsInf) {
    // pow() straight through, no zero special-case -- the reference has no
    // ZeroDivisionError to translate here (verified: OpenSCAD 2026.02.01
    // echoes inf).
    Evaluator ev;
    EXPECT_TRUE(std::isinf(asNum(evalSrc("0 ^ -1", ev))));
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

TEST(ExprEvalArithmetic, ScalarTimesMatrixAndMatrixTimesScalar) {
    // End-to-end expression-evaluator dispatch for both multiplication
    // directions -- matmul()/scale()'s own nested-list recursion is
    // unit-tested directly in test_value.cpp, but neither direction was
    // exercised via the real `*` operator dispatch (isNumber/isList
    // branch selection in expr_eval.cpp's MultiplicationOp case).
    Evaluator ev;
    Value a = evalSrc("2 * [[1,2],[3,4]]", ev);
    Value b = evalSrc("[[1,2],[3,4]] * 2", ev);
    auto rowsA = std::get<ListPtr>(a)->items;
    auto rowsB = std::get<ListPtr>(b)->items;
    EXPECT_DOUBLE_EQ(asNum(std::get<ListPtr>(rowsA[0])->items[0]), 2.0);
    EXPECT_DOUBLE_EQ(asNum(std::get<ListPtr>(rowsA[1])->items[1]), 8.0);
    EXPECT_DOUBLE_EQ(asNum(std::get<ListPtr>(rowsB[0])->items[0]), 2.0);
    EXPECT_DOUBLE_EQ(asNum(std::get<ListPtr>(rowsB[1])->items[1]), 8.0);
}

TEST(ExprEvalArithmetic, MatrixTimesVectorAndVectorTimesMatrix) {
    Evaluator ev;
    // [[1,2],[3,4]] * [1,1] -> row-wise dot products: [3, 7]
    Value mv = evalSrc("[[1,2],[3,4]] * [1,1]", ev);
    auto mvItems = std::get<ListPtr>(mv)->items;
    ASSERT_EQ(mvItems.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(mvItems[0]), 3.0);
    EXPECT_DOUBLE_EQ(asNum(mvItems[1]), 7.0);
    // [1,1] * [[1,2],[3,4]] -> column-wise dot products: [4, 6]
    Value vm = evalSrc("[1,1] * [[1,2],[3,4]]", ev);
    auto vmItems = std::get<ListPtr>(vm)->items;
    ASSERT_EQ(vmItems.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(vmItems[0]), 4.0);
    EXPECT_DOUBLE_EQ(asNum(vmItems[1]), 6.0);
}

TEST(ExprEvalArithmetic, MatrixTimesMatrix) {
    Evaluator ev;
    Value v = evalSrc("[[1,2],[3,4]] * [[5,6],[7,8]]", ev);
    auto rows = std::get<ListPtr>(v)->items;
    // Standard matrix product: [[1*5+2*7, 1*6+2*8], [3*5+4*7, 3*6+4*8]] = [[19,22],[43,50]]
    EXPECT_DOUBLE_EQ(asNum(std::get<ListPtr>(rows[0])->items[0]), 19.0);
    EXPECT_DOUBLE_EQ(asNum(std::get<ListPtr>(rows[0])->items[1]), 22.0);
    EXPECT_DOUBLE_EQ(asNum(std::get<ListPtr>(rows[1])->items[0]), 43.0);
    EXPECT_DOUBLE_EQ(asNum(std::get<ListPtr>(rows[1])->items[1]), 50.0);
}

TEST(ExprEvalArithmetic, UnaryMinusOnListNegatesElementwise) {
    Evaluator ev;
    Value v = evalSrc("-[1,-2,3]", ev);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), -1.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 2.0);
}

TEST(ExprEvalArithmetic, UnaryMinusOnStringIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("-\"abc\"", ev)));
}

TEST(ExprEvalArithmetic, SubtractionNumericAndVector) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("5 - 3", ev)), 2.0);
    Value v = evalSrc("[10,20,30] - [1,2,3]", ev);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), 9.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 27.0);
}

TEST(ExprEvalArithmetic, MultiplicationBoolOperandIsUndef) {
    // Neither operand is number/number, list*list, list*number, or
    // number*list -- the multiplication case's final catch-all undef.
    Evaluator ev;
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("true * 2", ev)));
}

TEST(ExprEvalArithmetic, DivisionNumeric) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("10 / 4", ev)), 2.5);
}

TEST(ExprEvalArithmetic, DivisionListByNumberScalesElementwise) {
    Evaluator ev;
    Value v = evalSrc("[10,20] / 2", ev);
    auto items = std::get<ListPtr>(v)->items;
    EXPECT_DOUBLE_EQ(asNum(items[0]), 5.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 10.0);
}

TEST(ExprEvalArithmetic, DivisionBoolOperandIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("true / 1", ev)));
}

TEST(ExprEvalArithmetic, DivisionListByListIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("[1,2] / [3,4]", ev)));
}

TEST(ExprEvalArithmetic, ModuloByZeroIsNan) {
    // fmod(x, 0) is nan, and the reference passes it through unchanged --
    // it is NOT undef (verified against OpenSCAD 2026.02.01).
    Evaluator ev;
    EXPECT_TRUE(std::isnan(asNum(evalSrc("5 % 0", ev))));
}

TEST(ExprEvalArithmetic, ExponentNumeric) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("2 ^ 10", ev)), 1024.0);
}

TEST(ExprEvalArithmetic, ExponentNonNumericIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("\"a\" ^ 2", ev)));
}

// -- Logical operators (short-circuit) --------------------------------------

TEST(ExprEvalLogical, AndOrNot) {
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("true && true", ev)));
    EXPECT_FALSE(asBool(evalSrc("true && false", ev)));
    EXPECT_TRUE(asBool(evalSrc("false || true", ev)));
    EXPECT_TRUE(asBool(evalSrc("!false", ev)));
}

TEST(ExprEvalLogical, RightSideNotEvaluatedWhenShortCircuited) {
    // `&&`/`||` must short-circuit: the reference's
    // `bool(eval(left)) and bool(eval(right))` relies on Python's own
    // `and`/`or` not evaluating the right operand once the left one already
    // decides the result -- previously mis-ported as "always evaluate both
    // sides," which broke the extremely common BOSL2 idiom
    // `is_undef(x) || (assert(is_num(x)) ...)` (the assert would always run).
    // Verified via an unknown-variable warning that would fire on the right
    // side of `false && nope`/`true || nope` if it were ever evaluated.
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    EXPECT_FALSE(asBool(evalSrc("false && nope", ev)));
    EXPECT_EQ(lastWarning.find("nope"), std::string::npos);
    EXPECT_TRUE(asBool(evalSrc("true || nope", ev)));
    EXPECT_EQ(lastWarning.find("nope"), std::string::npos);
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

TEST(ExprEvalComparisons, GreaterThanOrEqual) {
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("2 >= 1", ev)));
    EXPECT_TRUE(asBool(evalSrc("1 >= 1", ev))); // equal branch
    EXPECT_FALSE(asBool(evalSrc("0 >= 1", ev)));
}

TEST(ExprEvalComparisons, MismatchedTypesWarnAndUndefForGreaterEqual) {
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    Value v = evalSrc("true >= 0", ev);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
    EXPECT_NE(lastWarning.find("undefined operation (bool >= number)"), std::string::npos);
}

TEST(ExprEvalComparisons, MismatchedTypesWarnAndUndefForLessThan) {
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    Value v = evalSrc("true < 0", ev);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
    EXPECT_NE(lastWarning.find("undefined operation (bool < number)"), std::string::npos);
}

TEST(ExprEvalComparisons, MismatchedTypesWarnAndUndefForLessThanOrEqual) {
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    Value v = evalSrc("true <= 0", ev);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(v));
    EXPECT_NE(lastWarning.find("undefined operation (bool <= number)"), std::string::npos);
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

// -- Indexing / member access ----------------------------------------------

TEST(ExprEvalIndexing, StringIndexingValidAndOutOfRange) {
    Evaluator ev;
    EXPECT_EQ(std::get<std::string>(evalSrc("\"abc\"[1]", ev)), "b");
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("\"abc\"[10]", ev)));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("\"abc\"[-1]", ev)));
}

TEST(ExprEvalIndexing, RangeIndexingByComponent) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("[2:3:11][0]", ev)), 2.0); // start
    EXPECT_DOUBLE_EQ(asNum(evalSrc("[2:3:11][1]", ev)), 3.0); // step
    EXPECT_DOUBLE_EQ(asNum(evalSrc("[2:3:11][2]", ev)), 11.0); // end
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("[2:3:11][3]", ev)));
}

TEST(ExprEvalIndexing, ObjectIndexingByKey) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("object(a=1, b=2)[\"b\"]", ev)), 2.0);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("object(a=1)[\"nope\"]", ev)));
}

// -- object() entry lists -------------------------------------------------
//
// An unnamed list argument holds entries: [key, value] SETS, and the
// single-element [key] DELETES. Every value and every warning below was
// diffed character-for-character against OpenSCAD 2026.02.01 running with
// --enable=object-function.

namespace {

// Renders an object as its key list, "a,c,d" -- enough to pin both which
// keys survived and their ORDER, which is observable (ValueObject is
// insertion-ordered and oscEqual is order-sensitive).
std::string keysOf(const Value& v) {
    const ObjectPtr* o = std::get_if<ObjectPtr>(&v);
    if (!o || !*o) return "<undef>";
    std::string out;
    for (const auto& [k, _] : (*o)->items) {
        if (!out.empty()) out += ",";
        out += k;
    }
    return out;
}

} // namespace

TEST(ExprEvalObject, SingleElementEntryDeletesTheKey) {
    Evaluator ev;
    EXPECT_EQ(keysOf(evalSrc("object(object(a=42,b=53,c=8), [[\"d\",18],[\"b\"]])", ev)), "a,c,d");
    EXPECT_EQ(keysOf(evalSrc("object(object(a=42,b=53,c=8), [[\"b\"]])", ev)), "a,c");
}

TEST(ExprEvalObject, TwoElementEntryStillSets) {
    Evaluator ev;
    EXPECT_EQ(keysOf(evalSrc("object(object(a=42,b=53,c=8), [[\"d\",18]])", ev)), "a,b,c,d");
    EXPECT_DOUBLE_EQ(asNum(evalSrc("object(object(a=1), [[\"a\",9]]).a", ev)), 9.0);
}

TEST(ExprEvalObject, DeleteRemovesRatherThanBlanks) {
    // Observable through ORDER: a deleted key re-set afterwards lands at the
    // end, where an overwrite would have kept its original position.
    Evaluator ev;
    EXPECT_EQ(keysOf(evalSrc("object(object(a=42,b=53,c=8), [[\"b\"],[\"b\",99]])", ev)), "a,c,b");
    EXPECT_EQ(keysOf(evalSrc("object(object(a=42,b=53,c=8), [[\"b\",99],[\"b\"]])", ev)), "a,c");
    EXPECT_DOUBLE_EQ(asNum(evalSrc("object(object(a=42,b=53,c=8), [[\"b\"],[\"b\",99]]).b", ev)), 99.0);
}

TEST(ExprEvalObject, DeletingAnAbsentKeyIsASilentNoOp) {
    std::vector<std::string> warnings;
    Evaluator ev([&](const std::string& m) { warnings.push_back(m); });
    EXPECT_EQ(keysOf(evalSrc("object(object(a=42,b=53,c=8), [[\"zz\"]])", ev)), "a,b,c");
    EXPECT_TRUE(warnings.empty()) << "unexpected: " << (warnings.empty() ? "" : warnings[0]);
}

TEST(ExprEvalObject, DeletingTwiceIsHarmless) {
    Evaluator ev;
    EXPECT_EQ(keysOf(evalSrc("object(object(a=42,b=53,c=8), [[\"b\"],[\"b\"]])", ev)), "a,c");
}

TEST(ExprEvalObject, MalformedEntriesWarnAndYieldUndef) {
    // Each abandons the whole call at the first bad entry.
    const char* cases[] = {
        "object(object(a=1), [[]])",              // empty entry
        "object(object(a=1), [[\"a\",1,2]])",       // too long
        "object(object(a=1), [[5,1]])",           // non-string key, 2 elements
        "object(object(a=1), [[5]])",             // non-string key, 1 element
        "object(object(a=1), [\"b\"])",             // entry is not a list
        "object(object(a=1), 42)",                // argument is neither object nor list
    };
    for (const char* src : cases) {
        std::vector<std::string> warnings;
        Evaluator ev([&](const std::string& m) { warnings.push_back(m); });
        EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc(src, ev))) << src;
        EXPECT_EQ(warnings.size(), 1u) << src;
    }
}

TEST(ExprEvalObject, WarningTextMatchesTheReference) {
    // Quoted verbatim from OpenSCAD 2026.02.01, including its own
    // inconsistent spacing: the "not a list" case puts spaces inside the
    // parens where the others do not, and the "unnamed argument" case ends
    // with a trailing space before the position suffix.
    struct Case { const char* src; const char* want; };
    const Case cases[] = {
        {"object(object(a=1), [[]])",
         "object(Argument 1 [Element 0 []]) Entry is empty."},
        {"object(object(a=1), [[\"a\",1,2]])",
         "object(Argument 1 [Element 0 [...]]) Entry length is 3, must be 1 [key] or 2 [key,value]."},
        {"object(object(a=1), [[5,1]])",
         "object(Argument 1 [Element 0 [<number>,value]]) The key of the entry is not <string> but <number>."},
        {"object(object(a=1), [[5]])",
         "object(Argument 1 [Element 0 [<number>]]) The key of the entry is not <string> but <number>."},
        {"object(object(a=1), [\"b\"])",
         "object( Argument 1 [Element 0 <string>] ) Entry type is not a list, it is <string>."},
        {"object(object(a=1), 42)",
         "object(Argument 1 <number>) An unnamed argument must be either <object> or <list>, it is <number>. "},
    };
    for (const auto& c : cases) {
        std::vector<std::string> warnings;
        Evaluator ev([&](const std::string& m) { warnings.push_back(m); });
        evalSrc(c.src, ev);
        ASSERT_EQ(warnings.size(), 1u) << c.src;
        EXPECT_NE(warnings[0].find(c.want), std::string::npos)
            << "for: " << c.src << "\n  got: " << warnings[0] << "\n want: " << c.want;
    }
}

TEST(ExprEvalIndexing, ObjectMemberAccessByName) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("object(a=1, b=2).b", ev)), 2.0);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(evalSrc("object(a=1).nope", ev)));
}

TEST(ExprEvalIndexing, ListSwizzleWComponent) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("[1,2,3,4].w", ev)), 4.0);
}

// -- CommentedExpr unwrapping -----------------------------------------------

TEST(ExprEvalCommentedExpr, LeadingCommentDoesNotAffectValue) {
    // Only reachable when the AST was parsed with includeComments=true
    // (getASTFromString's default is false, which every other test in this
    // file relies on implicitly via exprSrc/parseSrc) -- evalExpr's own
    // CommentedExpr case just unwraps and re-evaluates the inner
    // expression, otherwise untested anywhere in this suite.
    auto ast = oscad::getASTFromString("x = /* c */ 1;\n", /*includeComments=*/true);
    auto scope = oscad::buildScopes(ast);
    Evaluator ev;
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    auto* a = dynamic_cast<oscad::Assignment*>(ast[0].get());
    ASSERT_NE(a, nullptr);
    ASSERT_EQ(a->expr->kind(), oscad::NodeKind::CommentedExpr);
    Value v = ev.evalExpr(*a->expr, ctx);
    EXPECT_DOUBLE_EQ(asNum(v), 1.0);
}

// -- assert() expression form: message formatting ---------------------------

TEST(ExprEvalAssertExpression, StringMessageIsQuoted) {
    Evaluator ev;
    try {
        evalSrc("assert(false, \"boom\") 1", ev);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        EXPECT_NE(std::string(e.what()).find("failed: \"boom\""), std::string::npos);
    }
}

TEST(ExprEvalAssertExpression, NonStringMessageUsesFmtValue) {
    // Note: the surrounding `": \"" + ... + "\""` wrapping is unconditional
    // (not just for actual strings), so a non-string message's fmtValue()
    // rendering still ends up inside literal quotes in the final message --
    // this is a pre-existing quirk of evalAssertExpr, not something this
    // test is asserting is "correct," just documenting the real output.
    Evaluator ev;
    try {
        evalSrc("assert(false, 42) 1", ev);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        EXPECT_NE(std::string(e.what()).find("failed: \"42\""), std::string::npos);
    }
}

// -- Arithmetic diagnostics (reference parity) ---------------------------
//
// Every expectation below was read off OpenSCAD 2026.02.01, not inferred:
// the rule is "warn exactly when the top-level result is undef, never
// otherwise", checked over all 392 ordered type pairs for + - * / % ^ plus
// unary minus. Before this the arithmetic operators warned about nothing at
// all, for any bad type -- `ord(undef)`'s missing warning was one symptom of
// the same hole.

namespace {

// Warnings carry a " in file <string>, line N" suffix; the messages are what
// these tests are about, so strip it.
std::vector<std::string> warningsFrom(const std::string& code) {
    std::vector<std::string> out;
    Evaluator ev([&](const std::string& msg) {
        const size_t at = msg.find(" in file ");
        out.push_back(at == std::string::npos ? msg : msg.substr(0, at));
    });
    evalSrc(code, ev);
    return out;
}

} // namespace

TEST(ExprEvalArithmeticWarnings, UndefinedOperationNamesBothOperandTypes) {
    EXPECT_EQ(warningsFrom("undef + 1"),
              (std::vector<std::string>{"WARNING: undefined operation (undefined + number)"}));
    EXPECT_EQ(warningsFrom("\"a\" - [1]"),
              (std::vector<std::string>{"WARNING: undefined operation (string - vector)"}));
    EXPECT_EQ(warningsFrom("true % 2"),
              (std::vector<std::string>{"WARNING: undefined operation (bool % number)"}));
    EXPECT_EQ(warningsFrom("[1] ^ 2"),
              (std::vector<std::string>{"WARNING: undefined operation (vector ^ number)"}));
    EXPECT_EQ(warningsFrom("-\"a\""),
              (std::vector<std::string>{"WARNING: undefined operation (-string)"}));
    // Range and function are named too -- oscTypeName used to call both
    // "undefined", inventing a phantom undef operand in the message.
    EXPECT_EQ(warningsFrom("[1] + [0:2]"),
              (std::vector<std::string>{"WARNING: undefined operation (vector + range)"}));
    EXPECT_EQ(warningsFrom("1 * (function(x) x)"),
              (std::vector<std::string>{"WARNING: undefined operation (number * function)"}));
}

TEST(ExprEvalArithmeticWarnings, SuccessfulOperationsStaySilent) {
    EXPECT_TRUE(warningsFrom("1 + 2").empty());
    EXPECT_TRUE(warningsFrom("[1,2] + [3,4,5]").empty()); // zips to the shorter length
    EXPECT_TRUE(warningsFrom("5 / 0").empty());           // inf, not an error
    EXPECT_TRUE(warningsFrom("5 % 0").empty());           // nan, not an error
    // An undef left INSIDE a list is not a top-level undef, so no warning --
    // matching the reference, which only reports on the returned value.
    EXPECT_TRUE(warningsFrom("[1,\"a\"] * 2").empty());
}

TEST(ExprEvalArithmeticWarnings, MultiplicationHasItsOwnVectorDiagnostics) {
    EXPECT_EQ(warningsFrom("[] * []"),
              (std::vector<std::string>{"WARNING: Multiplication is undefined on empty vectors"}));
    EXPECT_EQ(warningsFrom("[1,2] * [1,2,3]"),
              (std::vector<std::string>{"WARNING: vector*vector requires matching lengths (2 != 3)"}));
    EXPECT_EQ(warningsFrom("[1,2,3] * [[1,2],[3,4]]"),
              (std::vector<std::string>{
                  "WARNING: vector*matrix requires vector length to match matrix row count (3 != 2)"}));
    EXPECT_EQ(warningsFrom("[[1,2],[3,4]] * [1,2,3]"),
              (std::vector<std::string>{
                  "WARNING: matrix*vector requires matrix column count to match vector length (2 != 3)"}));
    EXPECT_EQ(warningsFrom("[[1,2,3]] * [[1,2],[3,4]]"),
              (std::vector<std::string>{"WARNING: matrix*matrix requires left operand column count to "
                                        "match right operand row count (3 != 2)"}));
    EXPECT_EQ(warningsFrom("[[1,2],[3,\"a\"]] * [1,2]"),
              (std::vector<std::string>{
                  "WARNING: Matrix must contain only numbers. Problem at row 1, col 1"}));
    // The dot-product element mismatch reports ELEMENT types, not operand types.
    EXPECT_EQ(warningsFrom("[1,\"a\"] * [1,2]"),
              (std::vector<std::string>{"WARNING: undefined operation (string * number)"}));
}

TEST(ExprEvalArithmetic, NumberDividedByVectorIsElementwise) {
    // The reference's NUMBER/VECTOR branch, which we simply did not have --
    // 5 / [1,2] was undef here and [5, 2.5] there.
    Evaluator ev;
    Value v = evalSrc("5 / [1,2]", ev);
    const auto& items = std::get<ListPtr>(v)->items;
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(std::get<double>(items[0]), 5.0);
    EXPECT_DOUBLE_EQ(std::get<double>(items[1]), 2.5);
    // Division by a zero element follows IEEE, same as the scalar form.
    Value inf = evalSrc("5 / [1,0]", ev);
    EXPECT_TRUE(std::isinf(std::get<double>(std::get<ListPtr>(inf)->items[1])));
}
