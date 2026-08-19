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

TEST(Scoping, DollarArgUndeclaredByCalleeFunctionDoesNotLeakToCallerDyn) {
    // f() declares no $-prefixed parameter, so hasDollarParam(f) is false
    // and the shareDyn fast path (callCtxFor -> childCtx/callCtx with
    // shareDyn=true) applies -- child_ctx.dyn aliases the CALLER's dyn map
    // instead of copying it. bindArgs still writes ANY named argument into
    // `bound`, including one that doesn't match any of f's declared
    // parameters (this is how `$fn=64`-style call-site dynamic-scope
    // overrides work generally) -- so the bound-argument loop in
    // evalUserFunction writes "$fn" into child_ctx.dyn even though f()
    // never declared it. If dyn were actually aliased (not just
    // conditionally shared), that write would mutate the CALLER's own dyn
    // map in place, leaking $fn=99 out to sibling statements after f()
    // returns. It doesn't leak here because bindArgs (unlike an ordinary
    // declared-$-param bind) is the one case shareDyn's "no declared
    // $-param" precondition doesn't fully cover -- verifying real,
    // observed behavior, not just re-asserting the optimization's own
    // stated invariant.
    Evaluator ev;
    auto ast = parseSrc("function f(x) = x;\na = f(x=1, $fn=99);\nb = $fn;");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evalChildren(ast, ctx);
    EXPECT_DOUBLE_EQ(asNum(ctx.let_->at("b")), 0.0); // untouched ambient default, not leaked 99
    EXPECT_DOUBLE_EQ(asNum(ctx.dyn->at("$fn")), 0.0);
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

TEST(Scoping, ChildrenForwardedThroughIsolatedCallWithSameNamedParamStillSeesAncestorBinding) {
    // A real bug found running a real BOSL2 script (attachable()/
    // trapezoid(), which deep-forwards children() through several more
    // calls while an intermediate ISOLATED call -- attachable() itself,
    // with its own unbound "path"/"h" parameters -- stays on the call
    // stack throughout). `wrapper(path)` below mirrors that shape: an
    // isolated (non-closure) call whose own declared parameter shares a
    // name with an ancestor's variable, forwarding children() straight
    // through past itself via `leaf()`. The deferred child block
    // (`echo(path)`, lexically inside `outer()`) must resolve `path`
    // against OUTER's own binding, not wrapper's own same-named
    // (unbound, undef) parameter -- even though wrapper's own scope is
    // still open (unpopped) the entire time the deferred block runs.
    //
    // This was broken by an earlier trail-storage design that judged
    // visibility purely by level-number ordering ("pushed no later than
    // my own level"): wrapper's own undef "path" binding is chronologically
    // *more recent* than outer's real one, so a numeric-only check picked
    // it up as if it were an ancestor, even though wrapper is not an
    // ancestor of the deferred child block at all -- just an unrelated,
    // still-open branch. Fixed by tracking true parent-chain ancestry per
    // level instead of comparing level numbers (see scope_trail.hpp).
    std::vector<std::string> echoed;
    Evaluator ev([&](const std::string& msg) { echoed.push_back(msg); });
    auto ast = parseSrc(
        "module leaf() { children(0); }\n"
        "module wrapper(path) { leaf() children(0); }\n"
        "module outer() {\n"
        "    path = [1, 2, 3];\n"
        "    wrapper() echo(path);\n"
        "}\n"
        "outer();\n");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    ASSERT_EQ(echoed.size(), 1u);
    EXPECT_EQ(echoed[0], "ECHO: [1, 2, 3]");
}

// $preview is always false: there is no preview mode here, every render is a
// full CSG render. It has to EXIST rather than be undef, though -- the
// `$preview ? cheap : real` idiom is common, and against undef it would take
// the cheap branch by accident and warn about an unknown variable doing it.
TEST(SpecialVariables, PreviewIsAlwaysFalse) {
    std::vector<std::string> msgs;
    Evaluated e = evalSrc("echo($preview, is_bool($preview), $preview ? \"p\" : \"r\");",
                           [&](const std::string& m) { msgs.push_back(m); });
    ASSERT_EQ(msgs.size(), 1u) << (msgs.empty() ? "" : msgs[0]);
    EXPECT_EQ(msgs[0], "ECHO: false, true, \"r\"");
}
