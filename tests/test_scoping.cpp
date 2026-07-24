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

TEST(Scoping, SelfReferentialParameterDefaultDoesNotInfinitelyRecurse) {
    // `x = is_undef(x) ? 5 : x;` inside a module body assigns `x` in terms
    // of its own (not-yet-assigned) prior value -- must resolve via
    // sequential-visibility rules (the RHS's `x` reference sees whatever
    // `x` was bound to on entry, i.e. the parameter's own undef default
    // when the caller passes nothing), not recurse into evaluating its own
    // not-yet-complete assignment.
    std::string captured;
    Evaluator ev([&](const std::string& msg) { captured = msg; });
    auto ast = parseSrc("module m(x) { x = is_undef(x) ? 5 : x; echo(x); }\nm();");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    EXPECT_EQ(captured, "ECHO: 5");
}

TEST(Scoping, SelfReferentialParameterDefaultDoesNotWarnAboutOverwriting) {
    // The same shadow-reassignment pattern as above must NOT trigger the
    // "was assigned ... but was overwritten" warning (DoubleAssignmentIn-
    // SameScopeWarnsWithExactFormatAndLastWins above) -- `x` here is a
    // parameter being reassigned once, not a plain variable assigned twice
    // in the same scope.
    std::vector<std::string> warnings;
    Evaluator ev([&](const std::string& msg) { warnings.push_back(msg); });
    auto ast = parseSrc("module m(x) { x = is_undef(x) ? 5 : x; } m();");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    for (const auto& w : warnings) {
        EXPECT_EQ(w.find("was overwritten"), std::string::npos) << w;
    }
}

TEST(Scoping, FunctionForwardReferenceResolves) {
    std::string captured;
    Evaluator ev([&](const std::string& msg) { captured = msg; });
    auto ast = parseSrc("echo(double(5));\nfunction double(x) = x * 2;");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    EXPECT_EQ(captured, "ECHO: 10");
}

TEST(Scoping, ModuleForwardReferenceResolves) {
    auto ast = parseSrc("box(3);\nmodule box(s) { cube(s); }");
    auto scope = oscad::buildScopes(ast);
    Evaluator ev;
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    auto tree = ev.resolveTree(ast, ctx);
    auto bodies = ev.generateTree(tree);
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_NEAR(bodies[0].body->Volume(), 27.0, 1e-9);
}

TEST(Scoping, ModuleLocalVariableDoesNotLeakToOuterScope) {
    std::vector<std::string> echoed;
    Evaluator ev([&](const std::string& msg) { echoed.push_back(msg); });
    auto ast = parseSrc("x = 10;\nmodule m() { x = 20; echo(x); }\nm();\necho(x);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    ASSERT_EQ(echoed.size(), 2u);
    EXPECT_EQ(echoed[0], "ECHO: 20");
    EXPECT_EQ(echoed[1], "ECHO: 10");
}
