#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {
double asNum(const Value& v) { return std::get<double>(v); }
} // namespace

// Phase 1's exit criterion, end to end through the real parser: assignment
// evaluation with correct sequential visibility between sibling statements.
TEST(Scoping, SequentialAssignmentVisibility) {
    Evaluator ev;
    auto ast = parseSrc("a = 1 + 2 * 3;\nb = a * 2;");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evalChildren(ast, ctx);
    EXPECT_DOUBLE_EQ(asNum(ctx.let_->at("a")), 7.0);
    EXPECT_DOUBLE_EQ(asNum(ctx.let_->at("b")), 14.0); // sees a's value, not stale
}

TEST(Scoping, DoubleAssignmentInSameScopeWarnsWithExactFormatAndLastWins) {
    std::string lastWarning;
    Evaluator ev([&](const std::string& msg) { lastWarning = msg; });
    auto ast = parseSrc("a = 1;\na = 2;");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evalChildren(ast, ctx);
    EXPECT_DOUBLE_EQ(asNum(ctx.let_->at("a")), 2.0);
    EXPECT_EQ(lastWarning, "WARNING: a was assigned on line 1 but was overwritten in file <string>, line 2");
}

TEST(Scoping, DollarVarAssignmentGoesToDynNotLet) {
    Evaluator ev;
    auto ast = parseSrc("$fn = 64;");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evalChildren(ast, ctx);
    EXPECT_DOUBLE_EQ(asNum(ctx.dyn->at("$fn")), 64.0);
    EXPECT_FALSE(ctx.let_->count("$fn"));
    EXPECT_TRUE(ctx.dynExplicit->count("$fn")); // the script itself assigned it, not just seeded
}

// NOTE: OpenSCAD's "all assignments in a scope run before any geometry
// statement" grouping (doc: "Assignment execution order") isn't
// independently testable yet -- every non-Assignment statement kind is
// still a no-op as of Phase 1 (see Evaluator::evalStatement), so there is
// no observable side effect to distinguish "ran in source order" from "ran
// in assignment-then-others order" until a later phase adds echo() or a
// geometry statement.
