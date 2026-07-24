#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

Value evalSrc(const std::string& code, Evaluator& ev) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    const oscad::Expression* expr = exprSrc(code, ast);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    return ev.evalExpr(*expr, ctx);
}

double asNum(const Value& v) { return std::get<double>(v); }
bool asBool(const Value& v) { return std::get<bool>(v); }
bool isUndef(const Value& v) { return std::holds_alternative<std::monostate>(v); }
std::string asStr(const Value& v) { return std::get<std::string>(v); }
const std::vector<Value>& asList(const Value& v) { return std::get<ListPtr>(v)->items; }

} // namespace

// -- Math: nan/inf passthrough, exact table trig -------------------------

TEST(MathBuiltins, AbsSignCeilFloorRound) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("abs(-3)", ev)), 3.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("sign(-5)", ev)), -1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("sign(0)", ev)), 0.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("ceil(1.2)", ev)), 2.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("floor(1.8)", ev)), 1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("round(1.5)", ev)), 2.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("round(-1.5)", ev)), -2.0);
    EXPECT_TRUE(std::isinf(asNum(evalSrc("ceil(1/0)", ev))));
}

TEST(MathBuiltins, SqrtLnLogSpecialCases) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("sqrt(4)", ev)), 2.0);
    EXPECT_TRUE(std::isnan(asNum(evalSrc("sqrt(-1)", ev))));
    EXPECT_TRUE(std::isinf(asNum(evalSrc("ln(0)", ev))) && asNum(evalSrc("ln(0)", ev)) < 0);
    EXPECT_TRUE(std::isnan(asNum(evalSrc("ln(-1)", ev))));
    EXPECT_TRUE(std::isinf(asNum(evalSrc("log(0)", ev))) && asNum(evalSrc("log(0)", ev)) < 0);
    EXPECT_NEAR(asNum(evalSrc("log(100)", ev)), 2.0, 1e-12);
}

TEST(MathBuiltins, TrigExactAtMultiplesOfNinety) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("sin(90)", ev)), 1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("cos(90)", ev)), 0.0);
    EXPECT_TRUE(std::isinf(asNum(evalSrc("tan(90)", ev))));
    EXPECT_NEAR(asNum(evalSrc("sin(30)", ev)), 0.5, 1e-9);
    EXPECT_TRUE(std::isnan(asNum(evalSrc("asin(2)", ev))));
    EXPECT_TRUE(std::isnan(asNum(evalSrc("acos(-2)", ev))));
    EXPECT_NEAR(asNum(evalSrc("atan2(1,1)", ev)), 45.0, 1e-9);
}

TEST(MathBuiltins, PowSpecialCases) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("pow(2,3)", ev)), 8.0);
    EXPECT_TRUE(std::isinf(asNum(evalSrc("pow(0,-1)", ev))) && asNum(evalSrc("pow(0,-1)", ev)) > 0);
    EXPECT_TRUE(std::isnan(asNum(evalSrc("pow(-1,0.5)", ev))));
}

TEST(MathBuiltins, MaxMinVectorAndScalarForms) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("max([3,1,4,1,5])", ev)), 5.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("min(3,1,4,1,5)", ev)), 1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("max(3)", ev)), 3.0);
    EXPECT_TRUE(isUndef(evalSrc("max(3, [1,2])", ev))); // mixing scalar+vector -> undef
}

TEST(MathBuiltins, NormAndCross) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("norm([3,4])", ev)), 5.0);
    Value c3 = evalSrc("cross([1,0,0],[0,1,0])", ev);
    ASSERT_EQ(asList(c3).size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(asList(c3)[2]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("cross([1,0],[0,1])", ev)), 1.0);
    EXPECT_TRUE(isUndef(evalSrc("cross([1/0,0,0],[0,1,0])", ev))); // inf component rejected
}

TEST(MathBuiltins, BoolArgumentIsUndefForNumericOnlyFns) {
    // Confirmed against real OpenSCAD 2022.08.22 -- bool is a distinct type,
    // not silently treated as 0/1, unlike Python's bool-is-int.
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("abs(true)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("max(true, 1)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("norm([true, 0])", ev)));
}

TEST(MathBuiltins, Rands) {
    Evaluator ev;
    Value v = evalSrc("rands(1, 2, 5, 42)", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 5u);
    for (const Value& x : items) {
        EXPECT_GE(asNum(x), 1.0);
        EXPECT_LE(asNum(x), 2.0);
    }
}

// -- String/list functions ------------------------------------------------

TEST(StringListBuiltins, Concat) {
    Evaluator ev;
    Value v = evalSrc("concat([1,2],3,[4])", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 4u);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 3.0);
}

TEST(StringListBuiltins, Len) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("len([1,2,3])", ev)), 3.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("len(\"abcd\")", ev)), 4.0);
    EXPECT_TRUE(isUndef(evalSrc("len(5)", ev)));
}

TEST(StringListBuiltins, StrFormatsNonStringArgs) {
    Evaluator ev;
    EXPECT_EQ(asStr(evalSrc("str(\"x=\", 1.5, true)", ev)), "x=1.5true");
}

TEST(StringListBuiltins, ChrOrdRoundTrip) {
    Evaluator ev;
    EXPECT_EQ(asStr(evalSrc("chr(65)", ev)), "A");
    EXPECT_DOUBLE_EQ(asNum(evalSrc("ord(\"A\")", ev)), 65.0);
    EXPECT_EQ(asStr(evalSrc("chr([65, 66])", ev)), "AB");
    EXPECT_EQ(asStr(evalSrc("chr(1/0)", ev)), ""); // non-finite -> "" not undef
}

TEST(StringListBuiltins, TypeChecks) {
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("is_undef(undef)", ev)));
    EXPECT_TRUE(asBool(evalSrc("is_num(3.5)", ev)));
    EXPECT_FALSE(asBool(evalSrc("is_num(true)", ev))); // bool is not num
    EXPECT_TRUE(asBool(evalSrc("is_bool(false)", ev)));
    EXPECT_TRUE(asBool(evalSrc("is_string(\"x\")", ev)));
    EXPECT_TRUE(asBool(evalSrc("is_list([1,2])", ev)));
}

TEST(StringListBuiltins, SearchScalarMatch) {
    Evaluator ev;
    Value v = evalSrc("search(3, [1,2,3,4,3])", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 1u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 2.0);
}

TEST(StringListBuiltins, SearchAllMatchesWithNumReturnsZero) {
    Evaluator ev;
    Value v = evalSrc("search(3, [1,2,3,4,3], 0)", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 4.0);
}

TEST(StringListBuiltins, LookupInterpolatesAndClamps) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("lookup(5, [[0,0],[10,100]])", ev)), 50.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("lookup(-5, [[0,0],[10,100]])", ev)), 0.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("lookup(50, [[0,0],[10,100]])", ev)), 100.0);
}

TEST(StringListBuiltins, HasKey) {
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("has_key(object(a=1,b=2), \"a\")", ev)));
    EXPECT_FALSE(asBool(evalSrc("has_key(object(a=1), \"z\")", ev)));
}

TEST(StringListBuiltins, VersionBuiltins) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("version_num()", ev)), 20250101.0);
}

// -- object(): call-site interleaved positional/named merge order --------

TEST(ObjectBuiltin, PositionalAfterNamedOverridesInCallSiteOrder) {
    // "a" is set by the named argument first, then overridden by the
    // positional [key,value]-list merge that comes after it at the call
    // site -- only correct if positional/named merges interleave in true
    // call-site order rather than (incorrectly) applying every positional
    // merge before any named override.
    Evaluator ev;
    Value v = evalSrc("object(a=1, [[\"a\", 2]])", ev);
    auto obj = std::get<ObjectPtr>(v);
    ASSERT_TRUE(obj);
    ASSERT_EQ(obj->items.size(), 1u);
    EXPECT_EQ(obj->items[0].first, "a");
    EXPECT_DOUBLE_EQ(std::get<double>(obj->items[0].second), 2.0);
}

TEST(ObjectBuiltin, NamedAfterPositionalOverridesInCallSiteOrder) {
    Evaluator ev;
    Value v = evalSrc("object([[\"a\", 1]], a=2)", ev);
    auto obj = std::get<ObjectPtr>(v);
    ASSERT_TRUE(obj);
    ASSERT_EQ(obj->items.size(), 1u);
    EXPECT_DOUBLE_EQ(std::get<double>(obj->items[0].second), 2.0);
}

TEST(ObjectBuiltin, ArgumentsEvaluatedExactlyOnce) {
    // builtinObject() evaluates the raw argument list itself (for
    // call-site-order merging) instead of going through the normal
    // resolveArgs()-then-dispatch path; this guards against evaluating
    // each argument expression twice, which would double any side effect
    // (echo, rands()) an argument expression has.
    std::vector<std::string> echoed;
    Evaluator ev([&](const std::string& m) { echoed.push_back(m); });
    Value v = evalSrc("object(a=1, echo(\"fired\") [[\"a\", 2]])", ev);
    auto obj = std::get<ObjectPtr>(v);
    ASSERT_TRUE(obj);
    ASSERT_EQ(obj->items.size(), 1u);
    EXPECT_DOUBLE_EQ(std::get<double>(obj->items[0].second), 2.0);
    EXPECT_EQ(echoed.size(), 1u);
}

TEST(ObjectBuiltin, PreservesInsertionOrderAcrossInterleavedMerges) {
    Evaluator ev;
    Value v = evalSrc("object(a=1, [[\"b\", 2]], c=3)", ev);
    auto obj = std::get<ObjectPtr>(v);
    ASSERT_TRUE(obj);
    ASSERT_EQ(obj->items.size(), 3u);
    EXPECT_EQ(obj->items[0].first, "a");
    EXPECT_EQ(obj->items[1].first, "b");
    EXPECT_EQ(obj->items[2].first, "c");
}

// -- Precedence: a builtin function name always wins over a same-named --
// user function, unlike modules (where a user module CAN shadow a builtin).

TEST(FunctionBuiltinPrecedence, BuiltinWinsOverSameNamedUserFunction) {
    Evaluator ev;
    auto ast = parseSrc("function abs(x) = 999;\nresult = abs(-3);");
    auto scope = oscad::buildScopes(ast);
    auto* assign = dynamic_cast<oscad::Assignment*>(ast[1].get());
    ASSERT_NE(assign, nullptr);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    Value v = ev.evalExpr(*assign->expr, ctx);
    EXPECT_DOUBLE_EQ(asNum(v), 3.0); // builtin abs(), not the user's abs()
}
