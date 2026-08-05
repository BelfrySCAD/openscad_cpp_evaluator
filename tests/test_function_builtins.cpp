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

// Every scalar-numeric builtin rejects a non-number argument outright
// rather than coercing it through toDoubleLenient()'s 0.0. Each expected
// value below was confirmed against real OpenSCAD 2022.08.22, which warns
// ("parameter could not be converted") and yields undef. Before this,
// `cos(undef)` was 1, `ln(undef)` was -inf and `acos([1,2])` was 90 --
// a misspelled variable produced plausible geometry instead of failing.
TEST(MathBuiltins, NonNumericArgumentIsUndefForScalarNumericFns) {
    Evaluator ev;
    for (const char* fn : {"abs", "sign", "ceil", "floor", "round", "sqrt", "ln", "log", "exp",
                           "sin", "cos", "tan", "asin", "acos", "atan"}) {
        const std::string f(fn);
        EXPECT_TRUE(isUndef(evalSrc(f + "(undef)", ev))) << f << "(undef)";
        EXPECT_TRUE(isUndef(evalSrc(f + "(\"hi\")", ev))) << f << "(\"hi\")";
        EXPECT_TRUE(isUndef(evalSrc(f + "([1,2])", ev))) << f << "([1,2])";
        EXPECT_TRUE(isUndef(evalSrc(f + "(true)", ev))) << f << "(true)";
    }
    EXPECT_TRUE(isUndef(evalSrc("atan2(1, undef)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("atan2(undef, 1)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("pow(2, undef)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("pow(undef, 2)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("rands(undef, 1, 3, 42)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("rands(0, 1, undef, 42)", ev)));
    // lookup() can't use the same rule (its second argument is a table),
    // so it guards its key separately -- without that, a non-numeric key
    // became 0.0 and silently returned the table's first value.
    EXPECT_TRUE(isUndef(evalSrc("lookup(undef, [[0,1],[10,5]])", ev)));
    EXPECT_TRUE(isUndef(evalSrc("lookup(5, undef)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("lookup(5, \"hi\")", ev)));
}

// Too few arguments is a type error too, not a call against an implicit 0.
TEST(MathBuiltins, MissingArgumentIsUndefForScalarNumericFns) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("abs()", ev)));
    EXPECT_TRUE(isUndef(evalSrc("pow(2)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("atan2(1)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("rands(0, 1)", ev)));
}

// The guard must not disturb the functions that legitimately take lists.
TEST(MathBuiltins, ListAcceptingNumericFnsStillWork) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("max([4,9,2])", ev)), 9.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("min([4,9,2])", ev)), 2.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("norm([3,4])", ev)), 5.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("lookup(5, [[0,5],[10,8],[20,2]])", ev)), 6.5);
    EXPECT_TRUE(isUndef(evalSrc("max(undef, 1)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("norm(undef)", ev)));
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

TEST(StringListBuiltins, ChrOrdMultiByteUtf8CodepointsRoundTrip) {
    // utf8Encode/utf8DecodeFirst's own 2-byte/3-byte/4-byte branches --
    // every other chr()/ord() test in this file uses codepoints <= 0x7F.
    Evaluator ev;
    EXPECT_EQ(asStr(evalSrc("chr(233)", ev)), "\xC3\xA9"); // U+00E9 'é', 2-byte
    EXPECT_EQ(asStr(evalSrc("chr(8364)", ev)), "\xE2\x82\xAC"); // U+20AC '€', 3-byte
    EXPECT_EQ(asStr(evalSrc("chr(128512)", ev)), "\xF0\x9F\x98\x80"); // U+1F600, 4-byte
    EXPECT_DOUBLE_EQ(asNum(evalSrc("ord(\"\xC3\xA9\")", ev)), 233.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("ord(\"\xE2\x82\xAC\")", ev)), 8364.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("ord(\"\xF0\x9F\x98\x80\")", ev)), 128512.0);
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

TEST(StringListBuiltins, SearchIndexColLooksUpWithinEachRow) {
    // findAll's own "vec[i] is itself a list, matchArg is a scalar" branch
    // (index_col) -- every other search() test in this file matches a flat
    // number list, never a list-of-rows.
    Evaluator ev;
    Value v = evalSrc("search(3, [[1,10],[2,20],[3,30]])", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 1u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 2.0); // row index 2 has col0==3
}

TEST(StringListBuiltins, SearchOutOfRangeIndexColMatchesNothing) {
    Evaluator ev;
    Value v = evalSrc("search(3, [[1,10],[2,20],[3,30]], 0, 5)", ev);
    EXPECT_TRUE(asList(v).empty());
}

TEST(StringListBuiltins, SearchStringMatchSearchesEachCharacter) {
    // findAll's "matchArg is itself a string" branch.
    Evaluator ev;
    Value v = evalSrc("search(\"ac\", [\"a\", \"b\", \"c\", \"a\"])", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 0.0); // first 'a'
    EXPECT_DOUBLE_EQ(asNum(items[1]), 2.0); // 'c'
}

TEST(StringListBuiltins, SearchListMatchSearchesEachElement) {
    // findAll's "matchArg is itself a list of scalars" branch.
    Evaluator ev;
    Value v = evalSrc("search([2,4], [1,2,3,4,5])", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 3.0);
}

TEST(StringListBuiltins, SearchStringHaystackSingleCharMatch) {
    // Regression test: builtinSearch used to reject a string "vector"
    // argument outright (returning undef unconditionally), unlike the
    // reference, where a Python string is natively iterable/indexable by
    // character -- `search("b", "abc")` is a real, common OpenSCAD idiom
    // (finding a character's position in a string) that silently produced
    // undef before this fix.
    Evaluator ev;
    Value found = evalSrc("search(\"b\", \"abc\")", ev);
    const auto& items = asList(found);
    ASSERT_EQ(items.size(), 1u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 1.0);
    EXPECT_TRUE(asList(evalSrc("search(\"z\", \"abc\")", ev)).empty());
}

TEST(StringListBuiltins, SearchMultiCharMatchAgainstStringHaystack) {
    // Each character of `match` is searched independently against the
    // string haystack, same as against a list haystack.
    Evaluator ev;
    Value v = evalSrc("search(\"ba\", \"abcd\")", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 1.0); // 'b' at index 1
    EXPECT_DOUBLE_EQ(asNum(items[1]), 0.0); // 'a' at index 0
}

TEST(StringListBuiltins, SearchStringHaystackWithNumReturnsZero) {
    Evaluator ev;
    Value v = evalSrc("search(\"a\", \"abcdabcd\", 0)", ev);
    const auto& outer = asList(v);
    ASSERT_EQ(outer.size(), 1u);
    const auto& inner = asList(outer[0]);
    ASSERT_EQ(inner.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(inner[0]), 0.0);
    EXPECT_DOUBLE_EQ(asNum(inner[1]), 4.0);
}

TEST(StringListBuiltins, SearchListOfListsMatchesWholeRows) {
    // findAll's own "val (an element of matchArg) is itself a list" branch
    // -- target=vec[i] directly (whole-row equality), rather than an
    // index_col lookup within each row.
    Evaluator ev;
    Value v = evalSrc("search([[1,2],[3,4]], [[1,2],[5,6],[3,4]])", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 0.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 2.0);
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

TEST(StringListBuiltins, HasKeyOnNonObjectIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("has_key(5, \"a\")", ev)));
    EXPECT_TRUE(isUndef(evalSrc("has_key([1,2], \"a\")", ev)));
    EXPECT_TRUE(isUndef(evalSrc("has_key(\"str\", \"a\")", ev)));
    EXPECT_TRUE(isUndef(evalSrc("has_key(undef, \"a\")", ev)));
}

TEST(StringListBuiltins, HasKeyOnEmptyObjectIsFalse) {
    Evaluator ev;
    EXPECT_FALSE(asBool(evalSrc("has_key(object(), \"a\")", ev)));
}

TEST(StringListBuiltins, LenOfObjectCountsKeys) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("len(object(a=1,b=2,c=3))", ev)), 3.0);
}

TEST(StringListBuiltins, ObjectPlusObjectIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("object(a=1) + object(b=2)", ev)));
}

TEST(StringListBuiltins, StrFormatsNestedObject) {
    Evaluator ev;
    EXPECT_EQ(asStr(evalSrc("str(object(a=1, nested=object(x=10,y=20)))", ev)), "object(a = 1, nested = object(x = 10, y = 20))");
}

TEST(StringListBuiltins, ForLoopOverObjectIteratesKeysInInsertionOrder) {
    std::vector<std::string> echoed;
    oscadeval::test::evalSrc("for (k = object(z=1,a=2,m=3)) echo(k);",
                              [&](const std::string& msg) { echoed.push_back(msg); });
    ASSERT_EQ(echoed.size(), 3u);
    EXPECT_EQ(echoed[0], "ECHO: \"z\"");
    EXPECT_EQ(echoed[1], "ECHO: \"a\"");
    EXPECT_EQ(echoed[2], "ECHO: \"m\"");
}

TEST(StringListBuiltins, FunctionValuedObjectMemberIsCallable) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("object(fn=function(x) x*2).fn(5)", ev)), 10.0);
}

TEST(StringListBuiltins, VersionBuiltins) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("version_num()", ev)), 20250101.0);
}

TEST(StringListBuiltins, VersionReturnsThreeElementList) {
    Evaluator ev;
    Value v = evalSrc("version()", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 3u);
}

TEST(StringListBuiltins, IsFunctionAndIsObject) {
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("is_function(function(x) x)", ev)));
    EXPECT_FALSE(asBool(evalSrc("is_function(5)", ev)));
    EXPECT_TRUE(asBool(evalSrc("is_object(object(a=1))", ev)));
    EXPECT_FALSE(asBool(evalSrc("is_object(5)", ev)));
}

TEST(StringListBuiltins, CrossDimensionMismatchIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("cross([1,2,3], [1,2])", ev)));
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

TEST(ObjectBuiltin, PositionalListWithNonPairEntryIsUndef) {
    // A positional list argument whose entries aren't all valid
    // [string-key, value] pairs -- the whole object() call is undef, not
    // just that one entry.
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("object([1, 2, 3])", ev)));
}

TEST(ObjectBuiltin, PositionalNonObjectNonListArgumentIsUndef) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("object(5)", ev)));
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
