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
    // A string needle searched in a VECTOR requires every entry to itself be
    // a vector longer than index_col -- a bare string entry is "invalid" and
    // aborts the whole call with an empty result plus one warning. This test
    // used to assert [0, 2] for the bare-string table, which real OpenSCAD
    // has never returned (checked against 2026.02.01).
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    Value bare = evalSrc("search(\"ac\", [\"a\", \"b\", \"c\", \"a\"])", ev);
    EXPECT_TRUE(asList(bare).empty());
    EXPECT_NE(lastWarning.find("Invalid entry in search vector at index 0"), std::string::npos);
    EXPECT_NE(lastWarning.find("required number of values in the entry: 1"), std::string::npos);

    // Wrapped one level, the same search does match, per character.
    Value wrapped = evalSrc("search(\"ac\", [[\"a\"], [\"b\"], [\"c\"], [\"a\"]])", ev);
    const auto& items = asList(wrapped);
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 0.0); // first 'a'
    EXPECT_DOUBLE_EQ(asNum(items[1]), 2.0); // 'c'
}

TEST(StringListBuiltins, SearchDispatchesOnWhatIsSearchedFor) {
    // Only number/string/vector needles are handled; everything else is
    // undef. And the haystack is never type-checked -- a non-vector is an
    // empty table, so searching in undef finds nothing rather than failing.
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("search(undef, \"abc\")", ev)));
    EXPECT_TRUE(isUndef(evalSrc("search(true, \"abc\")", ev)));
    EXPECT_TRUE(asList(evalSrc("search(\"a\", undef)", ev)).empty());
    EXPECT_TRUE(asList(evalSrc("search(5, undef)", ev)).empty());
    EXPECT_TRUE(asList(evalSrc("search(5, 42)", ev)).empty());
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
    EXPECT_EQ(asStr(evalSrc("str(object(a=1, nested=object(x=10,y=20)))", ev)), "{ a = 1; nested = { x = 10; y = 20; }; }");
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
    EXPECT_DOUBLE_EQ(asNum(evalSrc("version_num()", ev)), 20260101.0);
}

// version_num() also takes the optional vector the reference's own
// builtin_version_num does -- it folds whatever y/m/d it is handed, not just
// this build's release.
TEST(StringListBuiltins, VersionNumFoldsAGivenVersionVector) {
    Evaluator ev;
    EXPECT_DOUBLE_EQ(asNum(evalSrc("version_num([2019, 5, 0])", ev)), 20190500.0);
    EXPECT_DOUBLE_EQ(asNum(evalSrc("version_num([2021, 1, 0])", ev)), 20210100.0);
    // A 2-element vector defaults the day to 0 (getVec3's own defaultval).
    EXPECT_DOUBLE_EQ(asNum(evalSrc("version_num([2019, 5])", ev)), 20190500.0);
}

TEST(StringListBuiltins, VersionNumRejectsAnythingButA2Or3NumberVector) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("version_num(5)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("version_num(undef)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("version_num([2019])", ev)));
    EXPECT_TRUE(isUndef(evalSrc("version_num([2019, 5, 0, 7])", ev)));
    EXPECT_TRUE(isUndef(evalSrc("version_num([\"a\", \"b\"])", ev)));
    EXPECT_TRUE(isUndef(evalSrc("version_num([\"a\", \"b\", \"c\"])", ev)));
}

TEST(StringListBuiltins, VersionReturnsThreeElementList) {
    Evaluator ev;
    Value v = evalSrc("version()", ev);
    const auto& items = asList(v);
    ASSERT_EQ(items.size(), 3u);
    // Pinned to the same release version_num() reports above -- the reference
    // derives one from the other (y * 10000 + m * 100 + d, see
    // builtin_version_num), so the two must not drift apart here either.
    EXPECT_DOUBLE_EQ(asNum(items[0]), 2026.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 1.0);
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

// -- Precedence: a user function SHADOWS a same-named builtin ------------
//
// Same rule as modules, which always got this right. It used to be
// backwards for functions: the builtin won, because the port copied the old
// Python evaluator's _eval_function_call order rather than OpenSCAD's.
//
// Read off OpenSCAD 2026.02.01:
//   function abs(x) = 999;  echo(abs(-3));            -> 999
//   function is_undef(x) = "mine"; echo(is_undef(q)); -> "mine", and it
//       WARNS about the unknown variable q, because a user function
//       evaluates its arguments normally -- only the builtin probe is
//       silent.
//
// Every case runs under both engines: the VM decides builtin-vs-user at
// COMPILE time (bytecode_compiler.cpp), so it needs its own fix and its
// own coverage.

namespace {

class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

std::vector<std::string> echoesOf(const std::string& src) {
    std::vector<std::string> out;
    oscadeval::test::evalSrc(src, [&](const std::string& m) { out.push_back(m); });
    return out;
}

} // namespace

TEST(FunctionBuiltinPrecedence, UserFunctionShadowsSameNamedBuiltin) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e = echoesOf("function abs(x) = 999;\necho(abs(-3));");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm;
        EXPECT_NE(e[0].find("999"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}

TEST(FunctionBuiltinPrecedence, TheBuiltinStillWinsWhenNobodyRedefinesIt) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e = echoesOf("echo(abs(-3));");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm;
        EXPECT_NE(e[0].find("3"), std::string::npos) << "vm=" << vm << ", got " << e[0];
        EXPECT_EQ(e[0].find("999"), std::string::npos) << "vm=" << vm;
    }
}

TEST(FunctionBuiltinPrecedence, ShadowingHoldsInsideAUserFunctionBody) {
    // The call is inside another function, so on the VM path it is reached
    // through a COMPILED chunk rather than the interpreter -- this is the
    // case a compiler-side fix is needed for.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e =
            echoesOf("function abs(x) = 999;\nfunction outer() = abs(-3);\necho(outer());");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm;
        EXPECT_NE(e[0].find("999"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}

TEST(FunctionBuiltinPrecedence, AUserDefinedIsUndefShadowsTheProbe) {
    // is_undef() is the one builtin with argument handling of its own (it
    // must not warn about the name it is probing). A user's own is_undef
    // takes over completely, arguments evaluated the ordinary way -- so the
    // unknown-variable warning the probe suppresses DOES appear, exactly as
    // upstream.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        std::vector<std::string> msgs;
        oscadeval::test::evalSrc("function is_undef(x) = \"mine\";\necho(is_undef(nothing));",
                                  [&](const std::string& m) { msgs.push_back(m); });
        bool sawMine = false, sawWarning = false;
        for (const std::string& m : msgs) {
            if (m.find("mine") != std::string::npos) sawMine = true;
            if (m.find("nothing") != std::string::npos) sawWarning = true;
        }
        EXPECT_TRUE(sawMine) << "vm=" << vm;
        EXPECT_TRUE(sawWarning) << "vm=" << vm << ": a user function evaluates its argument normally";
    }
}

TEST(FunctionBuiltinPrecedence, TheIsUndefProbeIsStillSilentWhenNotRedefined) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e = echoesOf("echo(is_undef(nothing));");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm << ": probing must not warn";
        EXPECT_NE(e[0].find("true"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}

// -- Builtin argument diagnostics (reference parity) ---------------------
//
// Real OpenSCAD warns in two fixed shapes before returning undef, and we
// used to emit neither -- the undef came out silently, so a misspelled
// variable reaching abs()/ord()/len() said nothing at all. Every message
// below was read off OpenSCAD 2026.02.01.

namespace {

std::vector<std::string> builtinWarnings(const std::string& code) {
    std::vector<std::string> out;
    Evaluator ev([&](const std::string& msg) {
        const size_t at = msg.find(" in file ");
        out.push_back(at == std::string::npos ? msg : msg.substr(0, at));
    });
    evalSrc(code, ev);
    return out;
}

} // namespace

TEST(BuiltinArgDiagnostics, ConversionWarningNamesArgumentExpectedAndFound) {
    EXPECT_EQ(builtinWarnings("ord(undef)"),
              (std::vector<std::string>{
                  "WARNING: ord() parameter could not be converted: argument 0: expected string, found "
                  "undefined (undef)"}));
    EXPECT_EQ(builtinWarnings("abs(\"a\")"),
              (std::vector<std::string>{
                  "WARNING: abs() parameter could not be converted: argument 0: expected number, found "
                  "string (\"a\")"}));
    EXPECT_EQ(builtinWarnings("atan2(1, true)"),
              (std::vector<std::string>{
                  "WARNING: atan2() parameter could not be converted: argument 1: expected number, found "
                  "bool (true)"}));
    // len() takes a vector happily but still calls the expected type "string".
    EXPECT_EQ(builtinWarnings("len(5)"),
              (std::vector<std::string>{
                  "WARNING: len() parameter could not be converted: argument 0: expected string, found "
                  "number (5)"}));
    EXPECT_EQ(builtinWarnings("norm(undef)"),
              (std::vector<std::string>{
                  "WARNING: norm() parameter could not be converted: argument 0: expected vector, found "
                  "undefined (undef)"}));
    EXPECT_TRUE(builtinWarnings("abs(-1)").empty());
    EXPECT_TRUE(builtinWarnings("len([1,2])").empty());
}

TEST(BuiltinArgDiagnostics, ArityWarningUsesEachBuiltinsOwnWording) {
    EXPECT_EQ(builtinWarnings("abs()"),
              (std::vector<std::string>{
                  "WARNING: abs() number of parameters does not match: expected 1, found 0"}));
    EXPECT_EQ(builtinWarnings("abs(1, 2)"),
              (std::vector<std::string>{
                  "WARNING: abs() number of parameters does not match: expected 1, found 2"}));
    EXPECT_EQ(builtinWarnings("rands(0, 1)"),
              (std::vector<std::string>{
                  "WARNING: rands() number of parameters does not match: expected 3 or 4, found 2"}));
    EXPECT_EQ(builtinWarnings("search(1)"),
              (std::vector<std::string>{"WARNING: search() number of parameters does not match: expected "
                                        "between 2 and 4, found 1"}));
    EXPECT_EQ(builtinWarnings("is_num(1, 2)"),
              (std::vector<std::string>{
                  "WARNING: is_num() number of parameters does not match: expected 1, found 2"}));
    // ...and the value is undef, not the answer for the first argument.
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("abs(1, 2)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("is_num(1, 2)", ev)));
}

TEST(BuiltinArgDiagnostics, MaxMinDistinguishVectorElementFromArgument) {
    EXPECT_EQ(builtinWarnings("max([1, undef])"),
              (std::vector<std::string>{"WARNING: max() parameter could not be converted: vector element "
                                        "1: expected number, found undefined (undef)"}));
    EXPECT_EQ(builtinWarnings("min(1, \"a\")"),
              (std::vector<std::string>{"WARNING: min() parameter could not be converted: argument 1: "
                                        "expected number, found string (\"a\")"}));
    EXPECT_EQ(builtinWarnings("max()"),
              (std::vector<std::string>{
                  "WARNING: max() number of parameters does not match: expected at least 1, found 0"}));
    // An empty vector gets its own arity wording rather than a conversion error.
    EXPECT_EQ(builtinWarnings("max([])"),
              (std::vector<std::string>{"WARNING: max() number of parameters does not match: expected at "
                                        "least 1 vector element, found 0"}));
}

TEST(BuiltinArgDiagnostics, NormAndCrossHaveTheirOwnMessages) {
    // A non-numeric ELEMENT is terser than the usual conversion complaint.
    EXPECT_EQ(builtinWarnings("norm([1, undef])"),
              (std::vector<std::string>{"WARNING: Incorrect arguments to norm()"}));
    EXPECT_EQ(builtinWarnings("cross([1,2],[1,2,3])"),
              (std::vector<std::string>{"WARNING: Invalid vector size of parameter for cross()"}));
    EXPECT_EQ(builtinWarnings("cross([1,2,3,4],[1,2,3,4])"),
              (std::vector<std::string>{"WARNING: Invalid vector size of parameter for cross()"}));
    // Size is checked before element values.
    EXPECT_EQ(builtinWarnings("cross([1,undef,3],[1,2,3])"),
              (std::vector<std::string>{"WARNING: Invalid value in parameter vector for cross()"}));
    EXPECT_TRUE(builtinWarnings("cross([1,2],[3,4])").empty());
}

TEST(BuiltinArgDiagnostics, HasKeyRejectsANonStringKeyAsUndef) {
    // This answered `false` before, which reads as "the key isn't there"
    // rather than "that isn't a key".
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("has_key(object(a=1), undef)", ev)));
    EXPECT_TRUE(isUndef(evalSrc("has_key(undef, \"a\")", ev)));
    EXPECT_TRUE(asBool(evalSrc("has_key(object(a=1), \"a\")", ev)));
    EXPECT_FALSE(asBool(evalSrc("has_key(object(a=1), \"b\")", ev)));
}

TEST(StringListBuiltins, ChrIsVariadicAndRangeChecked) {
    Evaluator ev;
    EXPECT_EQ(asStr(evalSrc("chr(65, 66)", ev)), "AB"); // every argument contributes
    EXPECT_EQ(asStr(evalSrc("chr([65, 66])", ev)), "AB");
    EXPECT_EQ(asStr(evalSrc("chr([65:66])", ev)), "AB");
    // Out-of-range codepoints contribute nothing rather than emitting raw
    // invalid UTF-8, which used to escape and break the caller's decoding.
    EXPECT_EQ(asStr(evalSrc("chr(-1)", ev)), "");
    EXPECT_EQ(asStr(evalSrc("chr(0)", ev)), "");
    EXPECT_EQ(asStr(evalSrc("chr(1e9)", ev)), "");
    EXPECT_EQ(asStr(evalSrc("chr(55296)", ev)), ""); // 0xD800, a surrogate
    EXPECT_EQ(asStr(evalSrc("chr(1114112)", ev)), ""); // 0x110000, one past the top
}

// -- linear_solve(A, b) --------------------------------------------------
//
// One pivoted LU, three answers: x, det, singular. Oracle is BOSL2 plus
// arithmetic -- OpenSCAD has no equivalent, so there is no reference
// binary to compare against.
//
// Measured against BOSL2 through the CLI, same matrices, identical
// determinants: n=10  3.17s -> 0.33s, n=11  31.49s -> 0.33s. BOSL2's
// determinant() is a cofactor expansion, growing x10 per step; this is
// O(n^3).

TEST(LinearSolve, SolvesASquareSystem) {
    Evaluator ev;
    // asList() returns a reference INTO v, so v has to outlive it -- taking
    // asList(evalSrc(...)) directly dangles the moment the temporary dies.
    const Value v = evalSrc("linear_solve([[2,1],[1,3]], [5,10]).x", ev);
    const std::vector<Value>& x = asList(v);
    ASSERT_EQ(x.size(), 2u);
    EXPECT_NEAR(asNum(x[0]), 1.0, 1e-12);
    EXPECT_NEAR(asNum(x[1]), 3.0, 1e-12);
}

TEST(LinearSolve, DeterminantComesFreeFromTheSameFactorisation) {
    Evaluator ev;
    EXPECT_NEAR(asNum(evalSrc("linear_solve([[2,1],[1,3]]).det", ev)), 5.0, 1e-12);
    EXPECT_NEAR(asNum(evalSrc("linear_solve([[1,0,0],[0,1,0],[0,0,1]]).det", ev)), 1.0, 1e-12);
    // Sign follows the row swaps, not just the magnitude.
    EXPECT_NEAR(asNum(evalSrc("linear_solve([[0,1],[1,0]]).det", ev)), -1.0, 1e-12);
}

TEST(LinearSolve, WithNoRightHandSideThereIsNoSolution) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("linear_solve([[2,1],[1,3]]).x", ev)));
}

TEST(LinearSolve, ASingularMatrixAnswersRatherThanWarning) {
    // "Is this matrix singular?" is a fair question to ask, so it is
    // reported, not warned about: det 0, singular true, no solution.
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("linear_solve([[1,2],[2,4]], [1,2]).singular", ev)));
    EXPECT_NEAR(asNum(evalSrc("linear_solve([[1,2],[2,4]], [1,2]).det", ev)), 0.0, 1e-12);
    EXPECT_TRUE(isUndef(evalSrc("linear_solve([[1,2],[2,4]], [1,2]).x", ev)));
}

TEST(LinearSolve, SingularityIsRelativeToTheMatrixNotAbsolute) {
    // 2*I scaled by 1e-10 is perfectly conditioned, and BOSL2's
    // linear_solve calls it singular -- its test is a fixed absolute 1e-9
    // on R's diagonal with no scaling by the size of the matrix. Verified
    // through the CLI: BOSL2 says SINGULAR at 1e-10 and 1e-12 where this
    // solves correctly.
    Evaluator ev;
    EXPECT_FALSE(asBool(evalSrc("linear_solve([[2e-10,0],[0,2e-10]], [1e-10,1e-10]).singular", ev)));
    const Value v = evalSrc("linear_solve([[2e-10,0],[0,2e-10]], [1e-10,1e-10]).x", ev);
    const std::vector<Value>& x = asList(v);
    ASSERT_EQ(x.size(), 2u);
    EXPECT_NEAR(asNum(x[0]), 0.5, 1e-9);
}

TEST(LinearSolve, AMatrixRightHandSideSolvesEveryColumn) {
    Evaluator ev;
    // Solving against the identity is the inverse; [[2,1],[1,3]] has
    // det 5, so the inverse is [[0.6,-0.2],[-0.2,0.4]].
    const Value v = evalSrc("linear_solve([[2,1],[1,3]], [[1,0],[0,1]]).x", ev);
    const std::vector<Value>& rows = asList(v);
    ASSERT_EQ(rows.size(), 2u);
    const std::vector<Value>& r0 = asList(rows[0]);
    ASSERT_EQ(r0.size(), 2u);
    EXPECT_NEAR(asNum(r0[0]), 0.6, 1e-12);
    EXPECT_NEAR(asNum(r0[1]), -0.2, 1e-12);
}

TEST(LinearSolve, RejectsWhatItCannotFactor) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("linear_solve([[1,2],[3,\"x\"]], [1,2])", ev)));   // not numeric
    EXPECT_TRUE(isUndef(evalSrc("linear_solve([[1,2],[3,4]], [1,2,3])", ev)));     // wrong rhs length
    EXPECT_TRUE(isUndef(evalSrc("linear_solve([[1,2],[3,4,5]], [1,2])", ev)));     // ragged
}

// -- non-square: QR ------------------------------------------------------
//
// m > n is least squares, m < n is minimum norm -- the same two answers
// BOSL2's linear_solve gives, and cross-checked against it.

TEST(LinearSolve, OverdeterminedGivesTheLeastSquaresFit) {
    // Fit y = a + b*x to (0,1) (1,3) (2,5) (3,7): exactly on the line
    // y = 1 + 2x, so the residual is zero and a=1, b=2.
    Evaluator ev;
    const Value v = evalSrc("linear_solve([[1,0],[1,1],[1,2],[1,3]], [1,3,5,7]).x", ev);
    const std::vector<Value>& x = asList(v);
    ASSERT_EQ(x.size(), 2u);
    EXPECT_NEAR(asNum(x[0]), 1.0, 1e-10);
    EXPECT_NEAR(asNum(x[1]), 2.0, 1e-10);
}

TEST(LinearSolve, OverdeterminedWithNoExactSolutionMinimisesTheResidual) {
    // (0,0) (1,1) (2,2) (3,10): no line fits. The normal equations put the
    // least-squares line at y = -1.4 + 3.1x -- computed independently, not
    // read off this implementation.
    Evaluator ev;
    const Value v = evalSrc("linear_solve([[1,0],[1,1],[1,2],[1,3]], [0,1,2,10]).x", ev);
    const std::vector<Value>& x = asList(v);
    ASSERT_EQ(x.size(), 2u);
    EXPECT_NEAR(asNum(x[0]), -1.4, 1e-9);
    EXPECT_NEAR(asNum(x[1]), 3.1, 1e-9);
}

TEST(LinearSolve, UnderdeterminedGivesTheMinimUmNormSolution) {
    // x + y = 2 has infinitely many solutions; the smallest is [1,1].
    // Anything satisfying the equation passes a residual check, so this
    // asserts the NORM -- that is the whole claim of a minimum-norm solve.
    Evaluator ev;
    const Value v = evalSrc("linear_solve([[1,1]], [2]).x", ev);
    const std::vector<Value>& x = asList(v);
    ASSERT_EQ(x.size(), 2u);
    EXPECT_NEAR(asNum(x[0]), 1.0, 1e-12);
    EXPECT_NEAR(asNum(x[1]), 1.0, 1e-12);
}

TEST(LinearSolve, UnderdeterminedSatisfiesEveryEquation) {
    // 2 x 3: the plane_intersection shape BOSL2 relies on.
    Evaluator ev;
    const Value v = evalSrc("linear_solve([[1,0,1],[0,1,1]], [3,5]).x", ev);
    const std::vector<Value>& x = asList(v);
    ASSERT_EQ(x.size(), 3u);
    const double a = asNum(x[0]), b = asNum(x[1]), c = asNum(x[2]);
    EXPECT_NEAR(a + c, 3.0, 1e-12);
    EXPECT_NEAR(b + c, 5.0, 1e-12);
    // minimum norm: the component along the null direction [1,1,-1] is zero
    EXPECT_NEAR(a + b - c, 0.0, 1e-9);
}

TEST(LinearSolve, NonSquareHasNoDeterminant) {
    Evaluator ev;
    EXPECT_TRUE(isUndef(evalSrc("linear_solve([[1,0],[1,1],[1,2]], [1,2,3]).det", ev)));
    EXPECT_FALSE(asBool(evalSrc("linear_solve([[1,0],[1,1],[1,2]], [1,2,3]).singular", ev)));
}

TEST(LinearSolve, RankDeficientNonSquareIsReportedNotWrong) {
    // Two identical columns: rank 1, not 2.
    Evaluator ev;
    EXPECT_TRUE(asBool(evalSrc("linear_solve([[1,1],[2,2],[3,3]], [1,2,3]).singular", ev)));
    EXPECT_TRUE(isUndef(evalSrc("linear_solve([[1,1],[2,2],[3,3]], [1,2,3]).x", ev)));
}


TEST(LinearSolve, WorksUnderBothEngines) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e =
            echoesOf("r = linear_solve([[2,1],[1,3]], [5,10]);\necho(r.x, r.det, r.singular);");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm;
        EXPECT_NE(e[0].find("[1, 3]"), std::string::npos) << "vm=" << vm << ", got " << e[0];
        EXPECT_NE(e[0].find("5"), std::string::npos) << "vm=" << vm << ", got " << e[0];
        EXPECT_NE(e[0].find("false"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}


TEST(LinearSolve, AnExplicitlyUndefRightHandSideCountsAsAbsent) {
    // BOSL2's builtins.scad pattern is a wrapper with a fixed signature:
    //   function _linear_solve(A, b) = linear_solve(A, b);
    // so a determinant()-style caller that passes no right-hand side still
    // forwards b as undef. Treating that as a BAD argument would warn on
    // every such call. Passing undef is idiomatically the same as not
    // passing in OpenSCAD -- BOSL2 threads optional arguments through
    // wrappers that way throughout.
    Evaluator ev;
    EXPECT_NEAR(asNum(evalSrc("linear_solve([[2,1],[1,3]], undef).det", ev)), 5.0, 1e-12);
    EXPECT_TRUE(isUndef(evalSrc("linear_solve([[2,1],[1,3]], undef).x", ev)));
    EXPECT_TRUE(builtinWarnings("linear_solve([[2,1],[1,3]], undef)").empty());
}


// -- degree trig is bit-exact, not merely close ---------------------------
//
// A direct port of real OpenSCAD's src/utils/degree_trig.cc, and these
// identities are the reason. `std::sin(x * pi / 180)` gets them wrong by an
// ULP, and that is enough to break real libraries: BOSL2's
// rect(rounding=...) filters corner intersections with an exact `!=`
// comparison, so one ULP of drift makes it find two corner points instead
// of one and die on its own "Cannot find corner point to anchor" assert.
//
// Every expectation below was read off OpenSCAD 2026.02.01.

TEST(DegreeTrig, SinAndCosAgreeExactlyAt45) {
    Evaluator ev;
    EXPECT_EQ(asNum(evalSrc("sin(45) - cos(45)", ev)), 0.0);
}

TEST(DegreeTrig, ExactAtThirtyAndSixty) {
    Evaluator ev;
    EXPECT_EQ(asNum(evalSrc("sin(30)", ev)), 0.5);
    EXPECT_EQ(asNum(evalSrc("cos(60)", ev)), 0.5);
    // cos(30) == sqrt(3)/2 exactly, and sin(60) likewise.
    EXPECT_EQ(asNum(evalSrc("cos(30) - sqrt(3)/2", ev)), 0.0);
    EXPECT_EQ(asNum(evalSrc("sin(60) - sqrt(3)/2", ev)), 0.0);
}

TEST(DegreeTrig, ScalingKeepsTheIdentity) {
    // The exact form the BOSL2 failure took: polar_to_xy(10, 45).
    Evaluator ev;
    EXPECT_EQ(asNum(evalSrc("10*cos(45) - 10*sin(45)", ev)), 0.0);
}

TEST(DegreeTrig, ExactAtMultiplesOfNinety) {
    Evaluator ev;
    EXPECT_EQ(asNum(evalSrc("sin(90)", ev)), 1.0);
    EXPECT_EQ(asNum(evalSrc("cos(90)", ev)), 0.0);
    EXPECT_EQ(asNum(evalSrc("sin(180)", ev)), 0.0);
    EXPECT_EQ(asNum(evalSrc("cos(180)", ev)), -1.0);
    EXPECT_EQ(asNum(evalSrc("sin(-90)", ev)), -1.0);
}

TEST(DegreeTrig, TanExactValues) {
    Evaluator ev;
    EXPECT_EQ(asNum(evalSrc("tan(45)", ev)), 1.0);
    EXPECT_EQ(asNum(evalSrc("tan(0)", ev)), 0.0);
    EXPECT_TRUE(std::isinf(asNum(evalSrc("tan(90)", ev))));
}

TEST(DegreeTrig, InversesSnapToWholeDegrees) {
    // asin/acos/atan return a whole number of degrees whenever the forward
    // function reproduces the input exactly -- so this is 30, not
    // 29.999999999999996.
    Evaluator ev;
    EXPECT_EQ(asNum(evalSrc("asin(sin(30))", ev)), 30.0);
    EXPECT_EQ(asNum(evalSrc("acos(cos(60))", ev)), 60.0);
    EXPECT_EQ(asNum(evalSrc("atan(tan(45))", ev)), 45.0);
    EXPECT_EQ(asNum(evalSrc("atan2(1, 1)", ev)), 45.0);
    EXPECT_EQ(asNum(evalSrc("atan2(1, 0)", ev)), 90.0);
}

TEST(DegreeTrig, HugeAnglesAreNaNNotMeaningless) {
    // Beyond a 52-bit mantissa's reduction range OpenSCAD returns NaN
    // rather than a meaningless answer; matched deliberately.
    Evaluator ev;
    EXPECT_TRUE(std::isnan(asNum(evalSrc("sin(1e30)", ev))));
    EXPECT_TRUE(std::isnan(asNum(evalSrc("cos(1e30)", ev))));
}
