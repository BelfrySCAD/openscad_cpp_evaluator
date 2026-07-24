#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

double asNum(const Value& v) { return std::get<double>(v); }

// Runs `code` as a full top-level script (all statements, not just a bare
// expression) and returns the root EvalContext's final `let_` bindings --
// lets a test inspect an assignment's final value the way a script author
// would, without needing echo() output capture.
struct RunResult {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    std::unique_ptr<oscad::Scope> scope;
    Evaluator ev;
    EvalContext ctx;
};

RunResult runScript(const std::string& code, EchoFn echoFn = {}) {
    auto ast = parseSrc(code);
    auto scope = oscad::buildScopes(ast);
    Evaluator ev(std::move(echoFn));
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    // resolveTree() (not a bare evalChildren() call) -- it initializes the
    // tree-build stack that evalModularCall needs even when the script's
    // top-level statements are pure Assignments and no geometry is ever
    // inspected; a raw evalChildren() call would leave treeStack_ empty and
    // hit its back() undefined-behavior the moment any ModularCall (e.g. a
    // module invocation used only for its echo()/assignment side effects)
    // shows up in the script.
    ev.resolveTree(ast, ctx);
    return RunResult{std::move(ast), std::move(scope), std::move(ev), std::move(ctx)};
}

Value varValue(const RunResult& r, const std::string& name) { return r.ctx.let_->at(name); }

} // namespace

// -- for loops (statement) -------------------------------------------------

TEST(ForLoop, ProducesOneCubePerIteration) {
    Evaluated e = evalSrc("for (i = [0:2]) translate([i*3,0,0]) cube(1);");
    ASSERT_EQ(e.tree.size(), 3u); // transparent -- 3 sibling translate nodes, no wrapping "for" node
    ASSERT_EQ(e.bodies.size(), 3u);
    manifold::Box b0 = e.bodies[0].body->BoundingBox();
    manifold::Box b2 = e.bodies[2].body->BoundingBox();
    EXPECT_NEAR(b0.min.x, 0.0, 1e-9);
    EXPECT_NEAR(b2.min.x, 6.0, 1e-9);
}

TEST(ForLoop, MultipleVariablesProduceCartesianProduct) {
    Evaluated e = evalSrc("for (i = [0:1], j = [0:1]) translate([i,j,0]) cube(1);");
    EXPECT_EQ(e.bodies.size(), 4u); // 2x2
}

TEST(ForLoop, IteratesOverAPlainList) {
    Evaluated e = evalSrc("for (r = [1,2,3]) translate([r*10,0,0]) sphere(r=r, $fn=8);");
    EXPECT_EQ(e.bodies.size(), 3u);
}

// -- if / if-else -----------------------------------------------------------

TEST(IfStatement, TrueBranchProducesGeometry) {
    Evaluated e = evalSrc("if (true) cube(1);");
    EXPECT_EQ(e.bodies.size(), 1u);
}

TEST(IfStatement, FalseBranchProducesNothing) {
    Evaluated e = evalSrc("if (false) cube(1);");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(IfElseStatement, PicksCorrectBranch) {
    Evaluated e = evalSrc("if (1 > 2) cube(1); else sphere(1, $fn=8);");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "sphere");
}

// -- let (statement + expression forms) --------------------------------

TEST(LetStatement, BindsVisibleInsideBlockOnly) {
    Evaluated e = evalSrc("let(s=3) cube(s);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 3.0, 1e-9);
}

TEST(LetExpression, SequentialBindingsSeeEarlierOnes) {
    RunResult r = runScript("x = let(a=1, b=a+1, c=b+1) c;");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "x")), 3.0);
}

TEST(LetStatementForm, DoesNotSeeItsOwnEarlierBindingsUnlikeExpressionForm) {
    // Deliberate, documented port-fidelity divergence from the expression
    // form (see evalLetBlock's own comment): the statement form evaluates
    // every RHS against the *original* enclosing scope, not the growing
    // let-scope, so `b`'s `a+1` does NOT see this let's own `a`.
    RunResult r = runScript("a = 100;\nlet(a=1, b=a+1) { x = b; }");
    // b = (outer a=100) + 1 = 101, NOT 1+1=2.
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "a")), 100.0);
}

// -- echo / assert --------------------------------------------------------

TEST(Echo, FormatsArgumentsCommaSeparated) {
    std::string captured;
    runScript("echo(1, \"a\", true);", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: 1, \"a\", true");
}

TEST(AssertStatement, PassingAssertionIsANoOp) {
    EXPECT_NO_THROW(runScript("assert(1 == 1);"));
}

TEST(AssertStatement, FailingAssertionThrowsWithExactMessageFormat) {
    try {
        runScript("assert(1 == 2, \"nope\");");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        EXPECT_NE(std::string(e.what()).find("Assertion '1 == 2' failed: \"nope\""), std::string::npos);
    }
}

TEST(AssertExpression, FailingAssertionThrows) {
    EXPECT_THROW(runScript("x = assert(false) 1;"), EvalError);
}

// -- list comprehensions -------------------------------------------------

TEST(ListComprehension, ForClauseExpandsRange) {
    RunResult r = runScript("x = [for (i = [0:4]) i];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 5u);
    EXPECT_DOUBLE_EQ(asNum(items[4]), 4.0);
}

TEST(ListComprehension, ForIfFiltersElements) {
    RunResult r = runScript("x = [for (i = [0:4]) if (i % 2 == 0) i];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 0.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 2.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 4.0);
}

TEST(ListComprehension, ForIfElseMapsBothBranches) {
    RunResult r = runScript("x = [for (i = [0:3]) i % 2 == 0 ? \"even\" : \"odd\"];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 4u);
    EXPECT_EQ(std::get<std::string>(items[0]), "even");
    EXPECT_EQ(std::get<std::string>(items[1]), "odd");
}

TEST(ListComprehension, EachFlattensNestedLists) {
    RunResult r = runScript("x = [each [1,2], each [3,4]];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 4u);
    EXPECT_DOUBLE_EQ(asNum(items[3]), 4.0);
}

TEST(ListComprehension, LetClauseBindsIntoBody) {
    RunResult r = runScript("x = [for (i = [0:2]) let (sq = i*i) sq];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 4.0);
}

TEST(ListComprehension, CStyleForAccumulates) {
    RunResult r = runScript("x = [for (a = 0, i = 0; i < 5; a = a + i, i = i + 1) a];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 5u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 0.0);
    EXPECT_DOUBLE_EQ(asNum(items[4]), 6.0); // 0,0,1,3,6
}

// -- user functions: recursion, defaults, closures -----------------------

TEST(UserFunction, RecursiveFactorial) {
    RunResult r = runScript("function fact(n) = n <= 1 ? 1 : n * fact(n - 1);\nx = fact(5);");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "x")), 120.0);
}

TEST(UserFunction, DefaultParameterEvaluatedLexicallyNotAgainstCaller) {
    // Verified directly against real OpenSCAD (doc: docs/evaluator.md,
    // "Assignment execution order"): a default expression resolves against
    // the function's own declaration scope, ignoring a caller's let()
    // shadow of the same name -- `k` in `y=k`'s default is the global 100,
    // not the caller's let-bound 1.
    RunResult r = runScript("function f(x, y=k) = x + y;\n"
                             "k = 100;\n"
                             "function g() = let(k=1) f(1);\n"
                             "result = g();");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "result")), 101.0);
}

TEST(UserFunction, DollarVarLetOverridePropagatesIntoCalledFunction) {
    // The doc's own shipped-bug regression (evaluator.md line ~239):
    // let($fn=99) must remain visible to a function CALLED from inside the
    // let, not just the let's own body -- $-vars are dynamically scoped
    // down the call chain, not lexically scoped to the let block itself.
    RunResult r = runScript("function f() = $fn;\n"
                             "$fn = 10;\n"
                             "function g() = let($fn=99) f();\n"
                             "y = g();\n"
                             "z = f();"); // no leak-back: a bare f() after g() still sees the outer $fn=10
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "y")), 99.0);
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "z")), 10.0);
}

TEST(UserFunction, RecursiveSumOverAVectorWithoutLen) {
    // len() isn't ported until Phase 5, so this recurses on a fixed
    // 3-element vector with an explicit base case instead of the more
    // natural `i >= len(v)` -- still exercises real recursion + indexing
    // together, which is the point.
    RunResult r = runScript("function sum3(v, i=0) = i > 2 ? 0 : v[i] + sum3(v, i + 1);\n"
                             "x = sum3([10, 20, 30]);");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "x")), 60.0);
}

TEST(UserFunction, VectorIndexingWorks) {
    RunResult r = runScript("v = [10, 20, 30];\nx = v[1] + v[2];");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "x")), 50.0);
}

TEST(UserFunction, SwizzleMemberAccess) {
    RunResult r = runScript("v = [1, 2, 3];\nx = v.x;\ny = v.z;");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "x")), 1.0);
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "y")), 3.0);
}

TEST(FunctionLiteral, CanBeStoredAndCalledAsAValue) {
    RunResult r = runScript("g = function(x) x * 2;\nresult = g(21);");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "result")), 42.0);
}

// -- user modules: nested closures, children(), recursion ------------------

TEST(UserModule, NestedModuleSeesReassignedParameterViaClosure) {
    // BOSL2's cuboid()-shaped pattern: a module reassigns its own
    // parameter, then a module declared *inside its body* references that
    // name -- must see the reassignment (closure over the enclosing call's
    // locals), not recurse back into the parameter's own declaration via
    // plain scope lookup. Exercises callCtxFor's span-containment
    // closure detection directly.
    Evaluated e = evalSrc("module outer(edges) {"
                          "  edges = edges * 2;"
                          "  module inner() { cube(edges); }"
                          "  inner();"
                          "}"
                          "outer(3);");
    ASSERT_EQ(e.bodies.size(), 1u);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 6.0, 1e-9); // edges = 3*2 = 6, not the original 3
}

TEST(UserModule, ChildrenForwardsCallSiteGeometry) {
    Evaluated e = evalSrc("module wrapper() { color(\"red\") children(); }\n"
                          "wrapper() cube(2);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].color.has_value());
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[0], 1.0f); // red
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-9);
}

TEST(UserModule, ChildrenWithIndexSelectsOneStatement) {
    Evaluated e = evalSrc("module first_only() { children(0); }\n"
                          "first_only() { cube(1); translate([5,0,0]) cube(1); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 1.0, 1e-9); // the first cube, not the translated second one
}

TEST(UserModule, DollarChildrenCountsStatementsNotBodies) {
    // 2 child *statements* regardless of the 2nd producing 0 bodies (a
    // false if() forwards no geometry but still counts as one child).
    std::string captured;
    runScript("module counter() { echo($children); }\n"
              "counter() { cube(1); if (false) cube(1); }",
              [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: 2");
}

TEST(UserModule, RecursiveModuleCall) {
    Evaluated e = evalSrc("module stack(n) {"
                          "  if (n > 0) {"
                          "    cube(1);"
                          "    translate([0,0,1]) stack(n - 1);"
                          "  }"
                          "}"
                          "stack(3);");
    EXPECT_EQ(e.bodies.size(), 3u);
}

TEST(UserModule, DefaultParameterApplied) {
    Evaluated e = evalSrc("module box(size=5) { cube(size); }\nbox();");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 5.0, 1e-9);
}

TEST(UserModule, NamedAndPositionalArgumentsBind) {
    Evaluated e = evalSrc("module box(w, h, d) { cube([w,h,d]); }\nbox(1, d=3, h=2);");
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 1.0, 1e-9);
    EXPECT_NEAR(bbox.max.y, 2.0, 1e-9);
    EXPECT_NEAR(bbox.max.z, 3.0, 1e-9);
}

// -- intersection_for -----------------------------------------------------

TEST(IntersectionFor, IntersectsAllIterations) {
    // Each iteration cube(2) is centered at a different offset; the
    // intersection of all 3 should be a smaller sub-region than any one.
    Evaluated e = evalSrc("intersection_for (i = [0,1,2]) translate([i,0,0]) cube(3);");
    ASSERT_EQ(e.tree.size(), 1u);
    EXPECT_EQ(e.tree[0]->kind, "intersection_for");
    ASSERT_EQ(e.bodies.size(), 1u);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    // cubes span [0,3],[1,4],[2,5] on x -- intersection is [2,3].
    EXPECT_NEAR(bbox.min.x, 2.0, 1e-6);
    EXPECT_NEAR(bbox.max.x, 3.0, 1e-6);
}
