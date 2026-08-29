#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

// Mirrors test_bytecode_compiler.cpp's/test_tail_calls.cpp's own ScopedVm --
// the plain OSCAD_BYTECODE_VM env var is cached forever after its first
// read, so forcing it per-test needs this override instead (see
// Evaluator::setBytecodeVmEnabledForTesting's own doc comment).
class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

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

// A later `for`-clause dimension's own range CAN depend on an earlier
// dimension's current binding (a standard triangular-loop idiom) --
// verified directly against real OpenSCAD.app: `for (i=[0:2], j=[0:i])`
// produces (0,0)(1,0)(1,1)(2,0)(2,1)(2,2), NOT a flat 3x3 product and NOT
// an "unknown variable 'i'" warning. Regression test for a real bug: this
// used to evaluate every dimension's own range expression exactly once,
// upfront, against the ORIGINAL (pre-loop) ctx, so `j`'s own `[0:i]` never
// saw `i` at all. Pinned under both VM states explicitly (compileForLoop
// and evalFor had independent copies of the same bug).
TEST(ForLoop, LaterDimensionRangeCanDependOnEarlierBindingCompiled) {
    ScopedVm vm(true);
    std::vector<std::string> echoed;
    runScript("for (i = [0:2], j = [0:i]) echo(i, j);", [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: 0, 0", "ECHO: 1, 0", "ECHO: 1, 1", "ECHO: 2, 0", "ECHO: 2, 1",
                                                 "ECHO: 2, 2"}));
}

TEST(ForLoop, LaterDimensionRangeCanDependOnEarlierBindingInterpreted) {
    ScopedVm vm(false);
    std::vector<std::string> echoed;
    runScript("for (i = [0:2], j = [0:i]) echo(i, j);", [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: 0, 0", "ECHO: 1, 0", "ECHO: 1, 1", "ECHO: 2, 0", "ECHO: 2, 1",
                                                 "ECHO: 2, 2"}));
}

TEST(ForLoop, IteratesOverAPlainList) {
    Evaluated e = evalSrc("for (r = [1,2,3]) translate([r*10,0,0]) sphere(r=r, $fn=8);");
    EXPECT_EQ(e.bodies.size(), 3u);
}

TEST(ForLoop, ScalarIterableIsTreatedAsSingleElement) {
    // The statement-form `for` (distinct from a list-comprehension `for`
    // clause, which never reaches this since scalars there are handled by
    // expandIterable's own bare-scalar fallback tested in test_value.cpp)
    // -- a bare scalar iterable produces exactly one iteration.
    std::vector<std::string> echoed;
    runScript("for (x = 5) { echo(x); }", [&](const std::string& msg) { echoed.push_back(msg); });
    ASSERT_EQ(echoed.size(), 1u);
    EXPECT_EQ(echoed[0], "ECHO: 5");
}

TEST(ForLoop, UndefIterableProducesNoIterations) {
    std::vector<std::string> echoed;
    runScript("for (x = undef) { echo(x); }", [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_TRUE(echoed.empty());
}

TEST(ForLoop, BodyLocalVariableVisibleToSiblingStatementsInBody) {
    std::vector<std::string> echoed;
    runScript("for (a = [1:3]) { x = a * 2; echo(x); }", [&](const std::string& msg) { echoed.push_back(msg); });
    ASSERT_EQ(echoed.size(), 3u);
    EXPECT_EQ(echoed[0], "ECHO: 2");
    EXPECT_EQ(echoed[1], "ECHO: 4");
    EXPECT_EQ(echoed[2], "ECHO: 6");
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

// An if/else branch body is its own scope in OpenSCAD: assignments inside
// it are visible for the rest of the branch, but must NOT survive the
// closing brace. Regression tests for a real bug -- the branch used to be
// evaluated against the ENCLOSING context, so its writes escaped and
// silently changed later geometry. BOSL2's half_of() hit exactly this:
// it reassigns `v` inside an `else if`, which clobbered the outer `v` for
// everything after it, and also produced a bogus "assigned on line N but
// was overwritten" warning for what is really a shadow.
//
// Every case below was verified against real OpenSCAD 2026.02.01 first.
// Pinned under both VM states explicitly: evalStatement's ModularIf/
// ModularIfElse and compileOneStatement's own copies are independent.

namespace {

void checkBranchIsItsOwnScope(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> echoed;
    runScript("a = 1;\n"
              "if (true) { a = 2; if (true) { a = 3; echo(deep=a); } echo(mid=a); }\n"
              "echo(outer=a);\n",
              [&](const std::string& msg) { echoed.push_back(msg); });
    // Each nesting level shadows independently: the inner writes are gone
    // by the time control leaves each brace.
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: deep = 3", "ECHO: mid = 2", "ECHO: outer = 1"}));
}

void checkBranchDollarVarDoesNotEscape(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> echoed;
    runScript("$fn = 8;\n"
              "if (true) { $fn = 64; echo(inside=$fn); }\n"
              "echo(after=$fn);\n",
              [&](const std::string& msg) { echoed.push_back(msg); });
    // `$`-vars are scoped by the branch too, not just plain names.
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: inside = 64", "ECHO: after = 8"}));
}

void checkDoubleAssignmentWithinOneBranchStillWarns(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> messages;
    runScript("if (true) { z = 1; z = 2; echo(z=z); }\n",
              [&](const std::string& msg) { messages.push_back(msg); });
    // The branch gets a FRESH dynPositions level, so shadowing an outer
    // name is not reported -- but two assignments to the same name inside
    // the one branch share that level and must still warn, exactly as
    // OpenSCAD does. Guards against "fixing" the leak by disabling the
    // check outright.
    const bool warned = std::any_of(messages.begin(), messages.end(), [](const std::string& m) {
        return m.find("z was assigned on line 1 but was overwritten") != std::string::npos;
    });
    EXPECT_TRUE(warned) << "expected the same-scope reassignment warning";
    EXPECT_NE(std::find(messages.begin(), messages.end(), "ECHO: z = 2"), messages.end());
}

void checkShadowingAnOuterNameDoesNotWarn(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> messages;
    runScript("y = 1;\n"
              "if (true) { y = 99; }\n",
              [&](const std::string& msg) { messages.push_back(msg); });
    const bool warned = std::any_of(messages.begin(), messages.end(), [](const std::string& m) {
        return m.find("was overwritten") != std::string::npos;
    });
    EXPECT_FALSE(warned) << "shadowing an outer name in a branch is not an overwrite";
}

void checkElseIfBranchIsItsOwnScope(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> echoed;
    // The shape BOSL2's half_of() has: a module-body assignment, then a
    // reassignment of the same name inside an `else if` branch.
    runScript("module m(v=1) {\n"
              "    v = v + 1;\n"
              "    if (false) { echo(unused=v); } else if (true) { v = v * 10; echo(inner=v); }\n"
              "    echo(outer=v);\n"
              "}\n"
              "m(2);\n",
              [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: inner = 30", "ECHO: outer = 3"}));
}

} // namespace

TEST(IfBranchScope, BranchIsItsOwnScopeCompiled) { checkBranchIsItsOwnScope(true); }
TEST(IfBranchScope, BranchIsItsOwnScopeInterpreted) { checkBranchIsItsOwnScope(false); }

TEST(IfBranchScope, DollarVarDoesNotEscapeCompiled) { checkBranchDollarVarDoesNotEscape(true); }
TEST(IfBranchScope, DollarVarDoesNotEscapeInterpreted) { checkBranchDollarVarDoesNotEscape(false); }

TEST(IfBranchScope, DoubleAssignmentWithinOneBranchStillWarnsCompiled) {
    checkDoubleAssignmentWithinOneBranchStillWarns(true);
}
TEST(IfBranchScope, DoubleAssignmentWithinOneBranchStillWarnsInterpreted) {
    checkDoubleAssignmentWithinOneBranchStillWarns(false);
}

TEST(IfBranchScope, ShadowingAnOuterNameDoesNotWarnCompiled) { checkShadowingAnOuterNameDoesNotWarn(true); }
TEST(IfBranchScope, ShadowingAnOuterNameDoesNotWarnInterpreted) { checkShadowingAnOuterNameDoesNotWarn(false); }

TEST(IfBranchScope, ElseIfBranchIsItsOwnScopeCompiled) { checkElseIfBranchIsItsOwnScope(true); }
TEST(IfBranchScope, ElseIfBranchIsItsOwnScopeInterpreted) { checkElseIfBranchIsItsOwnScope(false); }

// A builtin operator's braced child block is its own scope too -- the same
// OpenSCAD rule as an if-branch, on a different code path (the interpreter
// gets it from resolveCallArgs' child ctx, the compiled path from an
// Op::OpenLetScope/Op::CloseExprScope bracket inside emitBuiltinWrap /
// emitCsgWrap). Regression tests for the same class of bug: these writes
// used to escape into the enclosing scope.
//
// User-module child blocks were always correct (their children are
// evaluated through childrenCallerCtx), and are pinned here so that stays
// true.

namespace {

void checkOperatorBlockIsItsOwnScope(bool useVm, const std::string& opCall) {
    ScopedVm vm(useVm);
    std::vector<std::string> echoed;
    runScript("a = 1;\n" + opCall + " { a = 99; cube(1); }\n" + "echo(after=a);\n",
              [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: after = 1"})) << "in: " << opCall;
}

void checkOperatorBlockDollarVarDoesNotEscape(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> echoed;
    runScript("$fn = 8;\n"
              "translate([0,0,0]) { $fn = 64; echo(inside=$fn); cube(1); }\n"
              "echo(after=$fn);\n",
              [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: inside = 64", "ECHO: after = 8"}));
}

void checkOperatorBlockAssignmentIsVisibleWithinTheBlock(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> echoed;
    // Scoping the block must not break the ordinary case: a name assigned
    // in the block is still visible to the rest of it (and, for a CSG
    // operator, across resolveCsg's assignment/geometry two-pass split).
    runScript("union() { s = 3; echo(seen=s); cube(s); }\n",
              [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: seen = 3"}));
}

void checkBlockAssignmentCannotFeedBackIntoOperatorArguments(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> messages;
    // The operator's own arguments are resolved against the OUTER ctx,
    // before the block scope opens -- so this translates by 1, not by 99.
    // (That is why the compiled bracket sits INSIDE Push/PopBuiltinWrap.)
    RunResult r = runScript("d = 1;\n"
                            "translate([d,0,0]) { d = 99; cube(1); }\n",
                            [&](const std::string& msg) { messages.push_back(msg); });
    (void)messages;
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "d")), 1.0);
}

void checkUserModuleChildBlockIsItsOwnScope(bool useVm) {
    ScopedVm vm(useVm);
    std::vector<std::string> echoed;
    runScript("c = 1;\n"
              "module wrap() { children(); }\n"
              "wrap() { c = 99; cube(1); }\n"
              "echo(after=c);\n",
              [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: after = 1"}));
}

} // namespace

TEST(OperatorBlockScope, TransformBlockIsItsOwnScopeCompiled) {
    checkOperatorBlockIsItsOwnScope(true, "translate([0,0,0])");
}
TEST(OperatorBlockScope, TransformBlockIsItsOwnScopeInterpreted) {
    checkOperatorBlockIsItsOwnScope(false, "translate([0,0,0])");
}

TEST(OperatorBlockScope, CsgBlockIsItsOwnScopeCompiled) { checkOperatorBlockIsItsOwnScope(true, "union()"); }
TEST(OperatorBlockScope, CsgBlockIsItsOwnScopeInterpreted) { checkOperatorBlockIsItsOwnScope(false, "union()"); }

TEST(OperatorBlockScope, ColorBlockIsItsOwnScopeCompiled) { checkOperatorBlockIsItsOwnScope(true, "color(\"red\")"); }
TEST(OperatorBlockScope, ColorBlockIsItsOwnScopeInterpreted) { checkOperatorBlockIsItsOwnScope(false, "color(\"red\")"); }

TEST(OperatorBlockScope, DollarVarDoesNotEscapeCompiled) { checkOperatorBlockDollarVarDoesNotEscape(true); }
TEST(OperatorBlockScope, DollarVarDoesNotEscapeInterpreted) { checkOperatorBlockDollarVarDoesNotEscape(false); }

TEST(OperatorBlockScope, AssignmentVisibleWithinTheBlockCompiled) {
    checkOperatorBlockAssignmentIsVisibleWithinTheBlock(true);
}
TEST(OperatorBlockScope, AssignmentVisibleWithinTheBlockInterpreted) {
    checkOperatorBlockAssignmentIsVisibleWithinTheBlock(false);
}

TEST(OperatorBlockScope, BlockAssignmentCannotFeedBackIntoArgumentsCompiled) {
    checkBlockAssignmentCannotFeedBackIntoOperatorArguments(true);
}
TEST(OperatorBlockScope, BlockAssignmentCannotFeedBackIntoArgumentsInterpreted) {
    checkBlockAssignmentCannotFeedBackIntoOperatorArguments(false);
}

TEST(OperatorBlockScope, UserModuleChildBlockIsItsOwnScopeCompiled) { checkUserModuleChildBlockIsItsOwnScope(true); }
TEST(OperatorBlockScope, UserModuleChildBlockIsItsOwnScopeInterpreted) { checkUserModuleChildBlockIsItsOwnScope(false); }

// A function literal declared in a MODULE body captures that body's
// bindings, including the module's own parameters, and keeps them when it
// escapes -- passed to another module and called there.
//
// The interpreter always did this (it captures ctx.let_ wholesale). The VM
// tries to snapshot only the names it can resolve, and inside a module body
// it can resolve none: module parameters are bound natively into ctx.let_
// rather than slot-addressed, and a module chunk compiles with no enclosing
// level. So the literal was frozen as a captureless constant and its free
// names resolved against whatever ctx CALLED it -- silently undef.
//
// Verified against real OpenSCAD 2026.02.01: every case below echoes 20.

namespace {

void checkEscapingClosureKeepsItsDefiningScope(bool useVm, const std::string& body) {
    ScopedVm vm(useVm);
    std::vector<std::string> echoed;
    runScript(body, [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: r = 20"})) << "in: " << body;
}

} // namespace

TEST(EscapingClosure, CapturesModuleParameterCompiled) {
    checkEscapingClosureKeepsItsDefiningScope(true,
        "module callee(fn) { echo(r = fn(2)); }\n"
        "module caller(s=100) { f = function (x) s / 8 * x; callee(f); }\n"
        "caller(80);\n");
}
TEST(EscapingClosure, CapturesModuleParameterInterpreted) {
    checkEscapingClosureKeepsItsDefiningScope(false,
        "module callee(fn) { echo(r = fn(2)); }\n"
        "module caller(s=100) { f = function (x) s / 8 * x; callee(f); }\n"
        "caller(80);\n");
}

// BOSL2's attachable(override=...) shape: the closure travels inside a list.
TEST(EscapingClosure, SurvivesInsideAListCompiled) {
    checkEscapingClosureKeepsItsDefiningScope(true,
        "module callee(spec) { echo(r = spec[0](2)); }\n"
        "module caller(s=100) { f = function (x) s / 8 * x; callee([f]); }\n"
        "caller(80);\n");
}
TEST(EscapingClosure, SurvivesInsideAListInterpreted) {
    checkEscapingClosureKeepsItsDefiningScope(false,
        "module callee(spec) { echo(r = spec[0](2)); }\n"
        "module caller(s=100) { f = function (x) s / 8 * x; callee([f]); }\n"
        "caller(80);\n");
}

// The callee has its OWN `s`. The closure must still see the one it was
// written next to -- this is what proves it captures rather than merely
// resolving late.
TEST(EscapingClosure, IsNotCapturedByTheCallersOwnBindingCompiled) {
    checkEscapingClosureKeepsItsDefiningScope(true,
        "module callee(fn, s=999) { echo(r = fn(2)); }\n"
        "module caller(s=100) { f = function (x) s / 8 * x; callee(f); }\n"
        "caller(80);\n");
}
TEST(EscapingClosure, IsNotCapturedByTheCallersOwnBindingInterpreted) {
    checkEscapingClosureKeepsItsDefiningScope(false,
        "module callee(fn, s=999) { echo(r = fn(2)); }\n"
        "module caller(s=100) { f = function (x) s / 8 * x; callee(f); }\n"
        "caller(80);\n");
}

// A literal closing over nothing keeps the cheap frozen-constant path.
TEST(EscapingClosure, CapturelessLiteralStillWorksCompiled) {
    ScopedVm vm(true);
    std::vector<std::string> echoed;
    runScript("module callee(fn) { echo(r = fn(4)); }\n"
              "module caller() { callee(function (x) x * 5); }\n"
              "caller();\n",
              [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: r = 20"}));
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

TEST(Echo, FormatsListValue) {
    // fmtValue's own ListPtr branch -- every other echo test in this file
    // uses only scalar arguments.
    std::string captured;
    runScript("echo([1, 2, 3]);", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: [1, 2, 3]");
}

TEST(Echo, FormatsRangeValue) {
    std::string captured;
    runScript("echo([2:1:10]);", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: [2 : 1 : 10]");
}

TEST(Echo, FormatsNonEmptyObjectValue) {
    std::string captured;
    runScript("echo(object(a=1, b=2));", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: { a = 1; b = 2; }");
}

TEST(Echo, FormatsEmptyObjectValue) {
    std::string captured;
    runScript("echo(object());", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: { }");
}

TEST(Echo, FormatsFunctionLiteralValue) {
    // fmtValue's own final fallback -- a FunctionLiteral* value has no
    // meaningful textual form (matches the reference exactly).
    std::string captured;
    runScript("echo(function(x) x);", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: <function-literal>");
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

// Same "later dimension depends on earlier one" fix as ForLoop's own
// (stmt_eval.cpp's evalFor) but for the list-comprehension sibling
// (evalListElement's ListCompFor case / compileListElement's own compiled
// form) -- this exact shape (`p` bound by the first clause, referenced by
// the second clause's own RHS) is a real, common BOSL/BOSL2 idiom (e.g.
// snappy-reprap's wiring.scad `fillet_path`). Verified against real
// OpenSCAD.app: `[for (p=[1:3], pt=p*10) pt]` == `[10,20,30]`.
TEST(ListComprehension, LaterForClauseCanDependOnEarlierBindingCompiled) {
    ScopedVm vm(true);
    RunResult r = runScript("x = [for (p = [1:3], pt = p*10) pt];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 10.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 20.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 30.0);
}

TEST(ListComprehension, LaterForClauseCanDependOnEarlierBindingInterpreted) {
    ScopedVm vm(false);
    RunResult r = runScript("x = [for (p = [1:3], pt = p*10) pt];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 10.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 20.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 30.0);
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

// The two tests above assert only the resulting LIST, which an eager
// implementation -- one that evaluated an `if` body whose condition failed,
// or both sides of an `if`/`else` -- would still get right. So they cannot
// fail if evalListElement's ListCompIf/ListCompIfElse cases (or their
// compiled counterparts in compileListElement) lose their laziness, the way
// `&&`/`||` demonstrably once did (see expr_eval.cpp's LogicalAndOp comment).
// These four watch for a marker echo() from the branch that must never run.
namespace {

// Every echo() the script emits, newline-joined -- the surrounding tests'
// own `captured = msg` lambdas keep only the last one, which is no use when
// the point of the test is exactly how MANY echoes happened.
std::string echoesFrom(const std::string& code) {
    std::string captured;
    runScript(code, [&](const std::string& msg) {
        if (!captured.empty()) captured += "\n";
        captured += msg;
    });
    return captured;
}

} // namespace

TEST(ListComprehension, ForIfSkipsBodyWhenConditionFailsInterpreted) {
    ScopedVm vm(false);
    EXPECT_EQ(echoesFrom("function mark(i) = echo(str(\"BODY\", i)) i;\n"
                         "x = [for (i = [0:2]) if (i == 1) mark(i)];"),
              "ECHO: \"BODY1\"");
}

TEST(ListComprehension, ForIfSkipsBodyWhenConditionFailsCompiled) {
    ScopedVm vm(true);
    EXPECT_EQ(echoesFrom("function mark(i) = echo(str(\"BODY\", i)) i;\n"
                         "function mk() = [for (i = [0:2]) if (i == 1) mark(i)];\n"
                         "x = mk();"),
              "ECHO: \"BODY1\"");
}

TEST(ListComprehension, ForIfElseEvaluatesOnlyTheChosenBranchInterpreted) {
    ScopedVm vm(false);
    // One echo per iteration, not two: i=0 takes `yes`, i=1 takes `no`.
    EXPECT_EQ(echoesFrom("function yes() = echo(\"TRUE-BRANCH\") 1;\n"
                         "function no() = echo(\"FALSE-BRANCH\") 2;\n"
                         "x = [for (i = [0:1]) if (i == 0) yes() else no()];"),
              "ECHO: \"TRUE-BRANCH\"\nECHO: \"FALSE-BRANCH\"");
}

TEST(ListComprehension, ForIfElseEvaluatesOnlyTheChosenBranchCompiled) {
    ScopedVm vm(true);
    EXPECT_EQ(echoesFrom("function yes() = echo(\"TRUE-BRANCH\") 1;\n"
                         "function no() = echo(\"FALSE-BRANCH\") 2;\n"
                         "function mk() = [for (i = [0:1]) if (i == 0) yes() else no()];\n"
                         "x = mk();"),
              "ECHO: \"TRUE-BRANCH\"\nECHO: \"FALSE-BRANCH\"");
}

TEST(ListComprehension, EachFlattensNestedLists) {
    RunResult r = runScript("x = [each [1,2], each [3,4]];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 4u);
    EXPECT_DOUBLE_EQ(asNum(items[3]), 4.0);
}

TEST(ListComprehension, LetClauseBindsIntoBody) {
    // Note: `for (...) let (...) sq` parses `let (...) sq` as a plain
    // expression-level LetOp (evaluated via evalExpr's own LetOp case, not
    // evalListElement's ListCompLet case) -- ListCompLet is a genuinely
    // different AST node that only arises when `let(...)` is immediately
    // followed by ANOTHER comprehension clause (see
    // ListCompLetPrecedingForBindsIntoLaterClause below for that shape).
    RunResult r = runScript("x = [for (i = [0:2]) let (sq = i*i) sq];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 4.0);
}

TEST(ListComprehension, ListCompLetPrecedingForBindsIntoLaterClause) {
    // The real ListCompLet AST node shape: `let(...)` directly followed by
    // another clause (here `for`), both inside the same `[...]` -- confirmed
    // via the parser's own JSON AST dump that only THIS shape produces
    // NodeKind::ListCompLet (LetClauseBindsIntoBody above's `for (...) let
    // (...) body` shape produces a plain LetOp instead).
    RunResult r = runScript("x = [let (a = 1) for (i = [0:2]) i + a];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 3.0);
}

TEST(ListComprehension, CStyleForAccumulates) {
    RunResult r = runScript("x = [for (a = 0, i = 0; i < 5; a = a + i, i = i + 1) a];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 5u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 0.0);
    EXPECT_DOUBLE_EQ(asNum(items[4]), 6.0); // 0,0,1,3,6
}

TEST(ListComprehension, IfElseClauseSyntax) {
    // The real `if (cond) a else b` clause shape (a distinct ListCompIfElse
    // AST node) -- ForIfElseMapsBothBranches above exercises the same
    // per-element branching via a plain ternary expression instead, which
    // is a different NodeKind and never reaches ListCompIfElse's own
    // evalListElement case.
    RunResult r = runScript("x = [for (i = [0:3]) if (i % 2 == 0) \"even\" else \"odd\"];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 4u);
    EXPECT_EQ(std::get<std::string>(items[0]), "even");
    EXPECT_EQ(std::get<std::string>(items[1]), "odd");
}

TEST(ListComprehension, EachWrappingAForClauseFlattensIt) {
    // isListCompClauseKind's own dispatch: `each` directly wrapping a
    // for/if/let CLAUSE (not a plain list literal, which
    // EachFlattensNestedLists above already covers) takes a different path
    // in evalListElement's own ListCompEach case.
    RunResult r = runScript("x = [each for (i = [0:2]) i];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 2.0);
}

TEST(ListComprehension, EachWrappingAnIfOnlyClauseFlattensIt) {
    RunResult r = runScript("x = [each if (true) 5];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 1u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 5.0);
}

TEST(ListComprehension, EachWrappingAnIfElseClauseFlattensIt) {
    RunResult r = runScript("x = [each if (false) 5 else 6];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 1u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 6.0);
}

TEST(ListComprehension, EachOnScalarAppendsSingleValue) {
    RunResult r = runScript("x = [each 5, each 6];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 5.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 6.0);
}

TEST(ListComprehension, EachOnUndefDropsIt) {
    RunResult r = runScript("x = [1, each undef, 2];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 2u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 2.0);
}

TEST(ListComprehension, ForClauseWithNestedListLiteralBody) {
    // The body of a `for` clause can itself be a plain vector literal
    // (also NodeKind::ListComprehension) rather than a scalar expression or
    // another comprehension clause -- a separate code path from the
    // ordinary evalListCompBody() call every other `for` test here takes.
    RunResult r = runScript("x = [for (i = [0:1]) [i, i * 2]];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 2u);
    auto first = std::get<ListPtr>(items[0])->items;
    EXPECT_DOUBLE_EQ(asNum(first[0]), 0.0);
    EXPECT_DOUBLE_EQ(asNum(first[1]), 0.0);
    auto second = std::get<ListPtr>(items[1])->items;
    EXPECT_DOUBLE_EQ(asNum(second[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(second[1]), 2.0);
}

TEST(ListComprehension, CStyleForWithNestedListLiteralBody) {
    RunResult r = runScript("x = [for (a = 0; a < 2; a = a + 1) [a, a]];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 2u);
    auto second = std::get<ListPtr>(items[1])->items;
    EXPECT_DOUBLE_EQ(asNum(second[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(second[1]), 1.0);
}

TEST(ListComprehension, CStyleForExceedingMaxIterationsThrows) {
    // A condition that never becomes false hits evalListElement's own
    // 1,000,000-iteration safety guard.
    EXPECT_THROW(runScript("x = [for (a = 0; true; a = a + 1) a];"), EvalError);
}

TEST(ListComprehension, ForClauseWithEachScalarBody) {
    // The opposite nesting from EachWrappingAForClauseFlattensIt above:
    // `for` is the OUTER clause here, and each iteration's own BODY is
    // `each i` (a scalar) -- appendEach's scalar-append branch, reached
    // via evalListElement's own isNestedLc==false ListCompFor path this
    // time, not evalListCompBody's generic dispatch.
    RunResult r = runScript("x = [for (i = [1:3]) each i];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 3u);
    EXPECT_DOUBLE_EQ(asNum(items[0]), 1.0);
    EXPECT_DOUBLE_EQ(asNum(items[2]), 3.0);
}

TEST(ListComprehension, ForUndefIterableProducesEmptyList) {
    RunResult r = runScript("x = [for (i = undef) i];");
    EXPECT_TRUE(std::get<ListPtr>(varValue(r, "x"))->items.empty());
}

TEST(ListComprehension, ForOverUndefBodyKeepsUndefAsAnElement) {
    // Unlike a for-BODY that IS undef being an iterable (dropped, above),
    // a for clause whose per-iteration BODY EXPRESSION evaluates to undef
    // must keep that undef as a real list element, not drop it -- distinct
    // from `each`'s own drop-undef rule (EachOnUndefDropsIt).
    RunResult r = runScript("x = [for (i = [1,2]) undef];");
    const auto& items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 2u);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(items[0]));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(items[1]));
}

TEST(ListComprehension, ZeroStepRangeIterationYieldsNothing) {
    RunResult r = runScript("x = [for (i = [1:0:5]) i];");
    EXPECT_TRUE(std::get<ListPtr>(varValue(r, "x"))->items.empty());
}

TEST(Echo, FormatsZeroStepRangeValue) {
    std::string captured;
    runScript("echo([1:0:5]);", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: [1 : 0 : 5]");
}

TEST(ExprEvalFunctionCall, CallingANonFunctionVariableWarnsAndIsUndef) {
    std::vector<std::string> warnings;
    RunResult r = runScript("x = [1,2,3];\ny = x();", [&](const std::string& msg) { warnings.push_back(msg); });
    EXPECT_TRUE(std::holds_alternative<std::monostate>(varValue(r, "y")));
    ASSERT_FALSE(warnings.empty());
    EXPECT_NE(warnings.back().find("Ignoring unknown function 'x'"), std::string::npos);
}

TEST(Intersection, EchoInDisabledFirstStatementStillFiresDespiteEmptyResult) {
    // A deliberate resolve/generate-split consequence: resolve can't yet
    // know the whole intersection() will end up geometrically empty (that
    // isn't known until generate time), so echo()'s side effect during
    // resolve still fires even though the final result has zero bodies.
    std::string captured;
    Evaluated e = evalSrc("intersection() { *cube(10); echo(\"fired\"); cube(2); }", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: \"fired\"");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(ListComprehension, IfClauseWithNestedListLiteralBody) {
    // evalListCompBody's own "body is itself a plain vector literal"
    // branch -- a separate code path from ListCompFor/ListCompCFor's own
    // direct evalListLiteral call (see ForClauseWithNestedListLiteralBody),
    // only reachable via a clause (ListCompIf here) that calls
    // evalListCompBody() on its own branch expression.
    RunResult r = runScript("x = [for (i = [0:2]) if (i > 0) [i, i]];");
    auto items = std::get<ListPtr>(varValue(r, "x"))->items;
    ASSERT_EQ(items.size(), 2u);
    auto first = std::get<ListPtr>(items[0])->items;
    EXPECT_DOUBLE_EQ(asNum(first[0]), 1.0);
    auto second = std::get<ListPtr>(items[1])->items;
    EXPECT_DOUBLE_EQ(asNum(second[0]), 2.0);
}

// -- Indexing / member access: remaining fallback branches -----------------

TEST(ExprIndexingFallbacks, NonNumericIndexOnListOrStringIsUndef) {
    RunResult r = runScript("a = [1,2,3][\"bad\"];\nb = \"abc\"[\"bad\"];");
    EXPECT_TRUE(std::holds_alternative<std::monostate>(varValue(r, "a")));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(varValue(r, "b")));
}

TEST(ExprIndexingFallbacks, IndexingANonIndexableTypeIsUndef) {
    RunResult r = runScript("a = (5)[0];");
    EXPECT_TRUE(std::holds_alternative<std::monostate>(varValue(r, "a")));
}

TEST(ExprIndexingFallbacks, UnrecognizedSwizzleLetterIsUndef) {
    RunResult r = runScript("a = [1,2,3].q;");
    EXPECT_TRUE(std::holds_alternative<std::monostate>(varValue(r, "a")));
}

TEST(ExprIndexingFallbacks, SwizzleLetterOutOfRangeForShortListIsUndef) {
    RunResult r = runScript("a = [1,2].z;"); // .z is index 2, list has only 2 elements
    EXPECT_TRUE(std::holds_alternative<std::monostate>(varValue(r, "a")));
}

TEST(ExprIndexingFallbacks, MemberAccessOnANonMemberTypeIsUndef) {
    RunResult r = runScript("a = (5).x;");
    EXPECT_TRUE(std::holds_alternative<std::monostate>(varValue(r, "a")));
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

TEST(LetStatementForm, DollarVarOverridePropagatesIntoCalledModuleAndFunction) {
    // Same shipped-bug regression as DollarVarLetOverridePropagatesInto-
    // CalledFunction above, but for the STATEMENT form of let (`let (...)
    // { ... }`) rather than the expression form -- a genuinely different
    // code path (evalLetBlock vs. evalLetExpr).
    std::string captured;
    runScript("function f() = $fn;\n"
              "module m() { echo($fn); }\n"
              "let ($fn=99) { m(); echo(f()); }",
              [&](const std::string& msg) { captured += msg + ";"; });
    EXPECT_EQ(captured, "ECHO: 99;ECHO: 99;");
}

TEST(UserFunction, DollarVarLetAsAssignmentRhsDoesNotLeak) {
    // The let-expression-as-assignment-RHS shape specifically (distinct
    // from DollarVarLetOverridePropagatesIntoCalledFunction's inline-call
    // shape): the override must still be scoped to evaluating that one
    // RHS, not leak into subsequent statements.
    RunResult r = runScript("function f() = $fn;\n"
                             "$fn = 10;\n"
                             "v1 = let($fn=55) f();\n"
                             "v2 = f();");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "v1")), 55.0);
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "v2")), 10.0);
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

TEST(UserFunction, UnboundParameterWithNoDefaultIsUndef) {
    // bindArgs/applyDefaults' own "no default value at all" branch --
    // DefaultParameterApplied/NamedAndPositionalArgumentsBind-shaped tests
    // elsewhere always either supply every argument or give every
    // unsupplied one a default; this leaves `b` genuinely unbound.
    RunResult r = runScript("function f(a, b) = is_undef(b);\nx = f(1);");
    EXPECT_TRUE(std::get<bool>(varValue(r, "x")));
}

TEST(UserFunction, DollarPrefixedParameterBindsIntoDynNotLet) {
    // Regression test: applyDefaults used to check ONLY childCtx.let_ for
    // "already bound", regardless of a param's $-prefix -- so a $-prefixed
    // param bound into ctx.dyn (by the loop just above applyDefaults'
    // call) still looked "unbound" to it, and its own no-default branch
    // clobbered the real value with a fresh undef written into ctx.let_.
    // Since evalIdentifier checks let_ before dyn, that stray undef then
    // shadowed the correct dyn value for any bare `$fn` reference in the
    // body -- even though geometry builtins reading ctx.dyn directly
    // (sphere's own $fn lookup) never noticed, since they never go through
    // evalIdentifier at all. Fixed by making applyDefaults check dyn (not
    // let_) for $-prefixed parameter names.
    RunResult r = runScript("function f($fn) = $fn;\nx = f($fn=16);");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "x")), 16.0);
}

TEST(UserFunction, DollarPrefixedParameterDefaultValueAppliesToDyn) {
    // Same bug, the OTHER trigger: a $-prefixed parameter's OWN declared
    // default (not overridden by the caller at all) used to be evaluated
    // and written into let_ instead of dyn, so it never reached
    // evalIdentifier's dyn fallback either.
    RunResult r = runScript("function f($fn=16) = $fn;\nx = f();");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "x")), 16.0);
}

TEST(UserFunction, ExceptionInsideBodyPropagatesThroughCallStackCleanup) {
    // evalUserFunction's own catch(...)/rethrow cleanup path (pops
    // callStack_/profiling before rethrowing) -- every other assert-failure
    // test in this suite fails at the TOP level, never from inside a user
    // function's own body, so this path was otherwise dead in the suite.
    EXPECT_THROW(runScript("function f() = assert(false, \"boom\") 1;\nx = f();"), EvalError);
}

TEST(FunctionLiteral, CanBeStoredAndCalledAsAValue) {
    RunResult r = runScript("g = function(x) x * 2;\nresult = g(21);");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "result")), 42.0);
}

TEST(FunctionLiteral, DollarPrefixedParameterBindsIntoDynNotLet) {
    // Same applyDefaults bug as UserFunction's own regression test above --
    // evalFunctionLiteral shares the identical bindArgs+applyDefaults shape.
    RunResult r = runScript("g = function($fn) $fn;\nx = g($fn=16);");
    EXPECT_DOUBLE_EQ(asNum(varValue(r, "x")), 16.0);
}

TEST(FunctionLiteral, ExceptionInsideBodyStillPopsCallStack) {
    EXPECT_THROW(runScript("g = function() assert(false, \"boom\") 1;\nx = g();"), EvalError);
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

TEST(UserModule, ChildrenWithNoChildrenPassedProducesNoGeometry) {
    Evaluated e = evalSrc("module m() { children(); }\nm();");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(UserModule, ColorWrappingChildrenInsideModuleForwardsGeometry) {
    // Regression shape: color()'s own child_ctx(color=rgba) call used to
    // reset childrenNodes/childrenCallerCtx to their defaults instead of
    // inheriting them, silently swallowing children()'s own forwarded
    // geometry the moment a module wrapped children() in color(c).
    Evaluated e = evalSrc("module m(c) { color(c) children(); }\nm(\"blue\") cube(5);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].color.has_value());
    EXPECT_FLOAT_EQ((*e.bodies[0].color)[2], 1.0f); // blue
    EXPECT_NEAR(e.bodies[0].body->Volume(), 125.0, 1e-6);
}

TEST(UserModule, DollarVarNamedArgOverrideWrappingChildrenForwardsGeometry) {
    // Same root cause as ColorWrappingChildrenInsideModuleForwardsGeometry
    // above, via transform()'s own $fn-named-arg dyn-override branch
    // instead of color()'s -- translate(..., $fn=8) still must forward
    // children() geometry, not swallow it.
    Evaluated e = evalSrc("module m() { translate([0,0,0], $fn=8) children(); }\nm() cube(5);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 125.0, 1e-6);
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

TEST(UserModule, GuardedIndexedChildrenForwardingThroughWrapperPreservesRealChildrenCount) {
    // Regression: prepareChildrenForward's dyn-copy loop (meant only to
    // forward $fn/$fa/$fs/$t-style overrides) used to also overwrite
    // $children/$parent_modules with the WRAPPER module's ("left" here) own
    // single-statement bookkeeping instead of preserving the real target's
    // ("outer()"'s actual call site) count already inherited via
    // callerCtx. A bare children() forwarding chain through ANY
    // intermediate wrapper silently corrupted $children, so a guard like
    // `if ($children > N) children(N)` (a standard BOSL/BOSL2 idiom, e.g.
    // GDMUtils.scad's left()/right()/up()) evaluated false and dropped
    // geometry with no warning at all. Found via a real user project.
    Evaluated e = evalSrc("module left(x=0) { translate([-x,0,0]) children(); }\n"
                          "module outer() {\n"
                          "    left(10) { if ($children > 0) children(0); }\n"
                          "    left(20) { if ($children > 1) children(1); }\n"
                          "}\n"
                          "outer() { cube(101); cube(102); }");
    ASSERT_EQ(e.bodies.size(), 2u);
    manifold::Box bbox0 = e.bodies[0].body->BoundingBox();
    manifold::Box bbox1 = e.bodies[1].body->BoundingBox();
    EXPECT_NEAR(bbox0.max.x - bbox0.min.x, 101.0, 1e-9);
    EXPECT_NEAR(bbox1.max.x - bbox1.min.x, 102.0, 1e-9);
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

TEST(UserModule, DeepNonTailRecursionHitsAControlledErrorInterpreted) {
    // Interpreted modules still have no trampoline and no compiled path
    // (see Op::CallModule/tryCompileModuleBody, Stage 2) -- every logical
    // recursive call here is still a real, unavoidable C++ recursive call,
    // so this guard still matters for this path specifically. The
    // COMPILED path's own version of this exact script now SUCCEEDS
    // instead -- see BytecodeCompiler.
    // RecursiveModuleWithIfSucceedsWellPastTheOldNativeLimitCompiled
    // (test_bytecode_compiler.cpp) -- these two together are the
    // differential proof Stage 2 actually changed something, not a
    // reversion.
    ScopedVm vm(false);
    try {
        evalSrc("module recur(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\nrecur(500);");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("Recursion too deep"), std::string::npos);
        EXPECT_NE(what.find("'recur'"), std::string::npos);
    }
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

TEST(UserModule, DollarPrefixedParameterBindsIntoDynNotLet) {
    // evalUserModule's own "$-prefixed bound arg goes to ctx.dyn" branch --
    // distinct from a caller-side let($fn=...) override (already tested
    // elsewhere): here the MODULE's own parameter declaration is itself
    // $-prefixed. If the binding landed in ctx.let_ instead, sphere()'s own
    // $fn lookup (which only ever reads ctx.dyn) wouldn't see it, and the
    // triangle count would match the default $fn instead of 6.
    Evaluated withModule = evalSrc("module s($fn) { sphere(r=2); }\ns($fn=6);");
    Evaluated direct = evalSrc("sphere(r=2, $fn=6);");
    ASSERT_EQ(withModule.bodies.size(), 1u);
    ASSERT_EQ(direct.bodies.size(), 1u);
    EXPECT_EQ(withModule.bodies[0].body->GetMeshGL().NumTri(), direct.bodies[0].body->GetMeshGL().NumTri());
}

TEST(UserModule, DollarPrefixedParameterBoundValueVisibleAsPlainIdentifier) {
    // Same applyDefaults regression as UserFunction's own test, but for a
    // module body that references the $-prefixed parameter by name (via
    // echo) rather than only through a builtin that reads ctx.dyn
    // directly -- exercises evalIdentifier's own let_-before-dyn lookup
    // order for a module call specifically.
    std::string captured;
    runScript("module m($fn) { echo($fn); }\nm($fn=16);", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: 16");
}

TEST(UserModule, DollarPrefixedParameterDefaultValueVisibleAsPlainIdentifier) {
    std::string captured;
    runScript("module m($fn=16) { echo($fn); }\nm();", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: 16");
}

TEST(UserModule, ExceptionInsideBodyPropagatesThroughCallStackCleanup) {
    EXPECT_THROW(evalSrc("module m() { assert(false, \"boom\"); }\nm();"), EvalError);
}

// -- Error call-chain message content (TRACE lines, not just "it throws") --

TEST(ModuleErrorCallChain, SingleModuleFrameNamedInTrace) {
    try {
        evalSrc("module bad() { assert(false, \"boom\"); }\nbad();");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        EXPECT_NE(std::string(e.what()).find("called by 'bad'"), std::string::npos);
    }
}

TEST(ModuleErrorCallChain, NestedModulesBothNamedInTrace) {
    try {
        evalSrc("module inner() { assert(false, \"boom\"); }\n"
                "module outer() { inner(); }\n"
                "outer();");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("called by 'inner'"), std::string::npos);
        EXPECT_NE(what.find("called by 'outer'"), std::string::npos);
    }
}

TEST(ModuleErrorCallChain, FunctionCalledFromModuleBothNamedInTrace) {
    try {
        evalSrc("function bad() = assert(false, \"boom\") 1;\n"
                "module m() { echo(bad()); }\n"
                "m();");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("called by 'bad'"), std::string::npos);
        EXPECT_NE(what.find("called by 'm'"), std::string::npos);
    }
}

TEST(FunctionBuiltins, ParentModuleReturnsInnermostEnclosingModuleName) {
    std::string captured;
    runScript("module m() { echo(parent_module(0)); }\nm();", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: \"m\"");
}

TEST(FunctionBuiltins, ParentModuleOutOfRangeIndexIsUndef) {
    std::string captured;
    runScript("module m() { echo(parent_module(5)); }\nm();", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: undef");
}

TEST(FunctionBuiltins, ParentModuleAtTopLevelIsUndef) {
    // No enclosing module call at all (an empty call stack) -- distinct
    // from the out-of-range-index case above, which still has one frame.
    std::string captured;
    runScript("echo(parent_module(0));", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: undef");
}

// -- intersection_for -----------------------------------------------------

// Same "later dimension depends on earlier one" fix as ForLoop's own, for
// intersection_for's own cartesian loop (resolveIntersectionFor/
// compileIntersectionForLoop). Verified against real OpenSCAD.app:
// `intersection_for(i=[0:1], j=[0:i]) { echo(i,j); cube(1); }` fires the
// body (and its echo) exactly 3 times, for (0,0),(1,0),(1,1) -- a flat
// 2x2 product (4 firings) or an "unknown variable 'i'" warning would both
// be wrong.
TEST(IntersectionFor, LaterDimensionRangeCanDependOnEarlierBindingCompiled) {
    ScopedVm vm(true);
    std::vector<std::string> echoed;
    runScript("intersection_for (i = [0:1], j = [0:i]) { echo(i, j); cube(1); }",
              [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: 0, 0", "ECHO: 1, 0", "ECHO: 1, 1"}));
}

TEST(IntersectionFor, LaterDimensionRangeCanDependOnEarlierBindingInterpreted) {
    ScopedVm vm(false);
    std::vector<std::string> echoed;
    runScript("intersection_for (i = [0:1], j = [0:i]) { echo(i, j); cube(1); }",
              [&](const std::string& msg) { echoed.push_back(msg); });
    EXPECT_EQ(echoed, (std::vector<std::string>{"ECHO: 0, 0", "ECHO: 1, 0", "ECHO: 1, 1"}));
}

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

TEST(IntersectionFor, TwoDeeChildrenExerciseSectionPath) {
    // combineBodies/generateIntersectionFor's 2D CrossSection branch --
    // every other intersection_for test here uses 3D cube() children.
    Evaluated e = evalSrc("intersection_for (i = [0,1]) translate([i,0]) square(3);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    // squares span x in [0,3] and [1,4]; intersection is [1,3] x [0,3].
    manifold::Rect bounds = e.bodies[0].section->Bounds();
    EXPECT_NEAR(bounds.min.x, 1.0, 1e-6);
    EXPECT_NEAR(bounds.max.x, 3.0, 1e-6);
}

TEST(IntersectionFor, MultipleStatementsPerIterationAreUnionedFirst) {
    // combineBodies' multi-body-per-iteration union branch: each iteration
    // here contributes TWO sibling cube() statements (not one), which must
    // be unioned together before intersecting across iterations.
    Evaluated e = evalSrc("intersection_for (i = [0,1]) { translate([i,0,0]) cube(3); translate([i,0,0]) cube(3); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.min.x, 1.0, 1e-6);
    EXPECT_NEAR(bbox.max.x, 3.0, 1e-6);
}

// -- render() ---------------------------------------------------------------

TEST(Render, PassesChildrenThroughUnchanged) {
    // render() is a display hint only -- no registered generate function,
    // so its result must be identical to the bare child geometry.
    Evaluated e = evalSrc("render() cube(2);");
    ASSERT_EQ(e.bodies.size(), 1u);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 2.0, 1e-9);
}

// -- breakpoint() -------------------------------------------------------

TEST(ControlBreakpoint, TruthyConditionStillForcesPause) {
    int forcedCalls = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool forced, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        if (forced) ++forcedCalls;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("breakpoint(true);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    EXPECT_EQ(forcedCalls, 1);
}

namespace {
int countForced(const std::string& src) {
    int forcedCalls = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool forced, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        if (forced) ++forcedCalls;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc(src);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    return forcedCalls;
}
} // namespace

TEST(ControlBreakpoint, NumericZeroAndOneCoerceLikeBooleans) {
    EXPECT_EQ(countForced("breakpoint(0);"), 0);
    EXPECT_EQ(countForced("breakpoint(1);"), 1);
}

TEST(ControlBreakpoint, NamedConditionArgument) {
    EXPECT_EQ(countForced("breakpoint(condition=true);"), 1);
    EXPECT_EQ(countForced("breakpoint(condition=false);"), 0);
}

TEST(ControlBreakpoint, VariableCondition) {
    EXPECT_EQ(countForced("x = 5;\nbreakpoint(x > 3);"), 1);
    EXPECT_EQ(countForced("x = 5;\nbreakpoint(x < 3);"), 0);
}

TEST(ControlBreakpoint, NoArgumentProducesNoGeometry) {
    Evaluated e = evalSrc("breakpoint();");
    EXPECT_TRUE(e.bodies.empty());
}

TEST(ControlBreakpoint, MultipleBreakpointsEachPauseOnce) {
    EXPECT_EQ(countForced("breakpoint();\ncube(1);\nbreakpoint();"), 2);
}

TEST(ControlBreakpoint, ConditionalInsideForLoopPausesOnlyWhenTrue) {
    EXPECT_EQ(countForced("for (i = [0:4]) { breakpoint(i >= 3); }"), 2); // i=3,4
}

// -- $-var visibility through children() -----------------------------------

TEST(DollarVarChildren, SetInsideModuleVisibleToCallSiteChildren) {
    std::string captured;
    runScript("module m() { $x = 42; children(); }\nm() echo($x);", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: 42");
}

TEST(DollarVarChildren, PerIterationForLoopVarVisibleToChildrenProducesDistinctGeometry) {
    // xcopies()-shaped pattern: each iteration's own $idx must be visible
    // to children() called from inside that same iteration, producing 3
    // spheres of 3 different sizes (d=$idx+1).
    Evaluated e = evalSrc("module xcopies(spacing, n=2) {"
                          "  for ($idx = [0:1:n-1]) {"
                          "    translate([($idx - n/2 + 0.5) * spacing, 0, 0]) children();"
                          "  }"
                          "}"
                          "xcopies(10, n=3) sphere(d=$idx+1, $fn=16);");
    ASSERT_EQ(e.bodies.size(), 3u);
    std::vector<double> widths;
    for (const auto& b : e.bodies) {
        manifold::Box bbox = b.body->BoundingBox();
        widths.push_back(bbox.max.x - bbox.min.x);
    }
    std::sort(widths.begin(), widths.end());
    EXPECT_LT(widths[0], widths[1]);
    EXPECT_LT(widths[1], widths[2]);
}

TEST(DollarVarChildren, AssignmentInForBodyVisiblePerIterationToChildren) {
    std::vector<std::string> echoed;
    runScript("module m() { for (i = [0:2]) { $val = i * 10; children(); } }\nm() echo($val);",
              [&](const std::string& msg) { echoed.push_back(msg); });
    ASSERT_EQ(echoed.size(), 3u);
    EXPECT_EQ(echoed[0], "ECHO: 0");
    EXPECT_EQ(echoed[1], "ECHO: 10");
    EXPECT_EQ(echoed[2], "ECHO: 20");
}

TEST(DollarVarChildren, InnermostNestedModuleOverrideWins) {
    std::string captured;
    runScript("module outer() { $x = 1; children(); }\n"
              "module inner() { $x = 2; children(); }\n"
              "outer() inner() echo($x);",
              [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: 2");
}

TEST(DollarVarChildren, TopLevelValueVisibleWithoutModuleOverride) {
    std::string captured;
    runScript("module m() { children(); }\n$x = 99;\nm() echo($x);", [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: 99");
}

TEST(DollarVarChildren, ChildrenInForLoopProducesOneBodyPerIteration) {
    Evaluated e = evalSrc("module triple() { for (i = [0:2]) { translate([i*10,0,0]) children(); } }\ntriple() cube(1);");
    EXPECT_EQ(e.bodies.size(), 3u);
}

TEST(DollarVarChildren, ChildrenInLetBlockPreservesDollarVars) {
    std::string captured;
    runScript("module m() { $v = 5; let (x = 1) { children(); } }\nm() echo($v);",
              [&](const std::string& msg) { captured = msg; });
    EXPECT_EQ(captured, "ECHO: 5");
}

TEST(DollarVarChildren, ChildrenIndexedWithModuleSetDollarVar) {
    Evaluated e = evalSrc("module m() { $x = 10; children(0); }\nm() { cube($x); cube(1); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    manifold::Box bbox = e.bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 10.0, 1e-9);
}

TEST(DollarVarChildren, ChildrenNIndexesStatementNotOutputBodyWhenAnEarlierStatementProducesZeroBodies) {
    // children(N) must count child *statements*, not output geometry
    // bodies -- a disabled (*-prefixed) earlier statement still occupies
    // its own statement slot. children(1) here must select cube(2), NOT
    // cube(3) (which a body-index-based scheme would incorrectly select
    // since *cube(1) contributes 0 bodies).
    Evaluated e = evalSrc("module m() { children(1); }\nm() { *cube(1); cube(2); cube(3); }");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-9); // cube(2)
}

// -- children(index): number, vector or range -----------------------------
//
// children() takes a number, a VECTOR of numbers, or a RANGE, so
// children([3:1:5]) is children(3); children(4); children(5), and
// children([3:-1:1]) is 3, 2, 1 in that order.
//
// Only a plain number used to be handled. toDoubleLenient collapses a list
// or a range to 0, so every vector/range form silently rendered child 0 --
// wrong geometry, no warning. Every case below was diffed against OpenSCAD
// 2026.02.01.

namespace {

// Which children ran, in order, as "c3,c2,c1". Each child echoes its own
// index, which survives ordering where a union of overlapping bodies would
// not -- and unlike geometry it also shows duplicates.
std::string childOrder(const std::string& picker) {
    std::string out;
    Evaluator ev([&](const std::string& m) {
        const std::string tag = "ECHO: \"c";
        const size_t at = m.find(tag);
        if (at == std::string::npos) return;
        if (!out.empty()) out += ",";
        out += "c" + m.substr(at + tag.size(), m.find('"', at + tag.size()) - at - tag.size());
    });
    const std::string src =
        "module pick() { " + picker + " }\n"
        "pick() { echo(\"c0\"); echo(\"c1\"); echo(\"c2\"); echo(\"c3\"); echo(\"c4\"); echo(\"c5\"); }\n";
    std::vector<std::unique_ptr<oscad::ASTNode>> ast = test::parseSrc(src);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    return out;
}

std::vector<std::string> childWarnings(const std::string& picker) {
    std::vector<std::string> warnings;
    Evaluator ev([&](const std::string& m) {
        if (m.rfind("WARNING:", 0) == 0) warnings.push_back(m);
    });
    const std::string src =
        "module pick() { " + picker + " }\n"
        "pick() { echo(\"c0\"); echo(\"c1\"); echo(\"c2\"); }\n";
    std::vector<std::unique_ptr<oscad::ASTNode>> ast = test::parseSrc(src);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    return warnings;
}

} // namespace

TEST(ChildrenIndex, PlainNumberStillWorks) {
    EXPECT_EQ(childOrder("children(2);"), "c2");
    EXPECT_EQ(childOrder("children();"), "c0,c1,c2,c3,c4,c5");
}

TEST(ChildrenIndex, AscendingRangeSelectsEachInTurn) {
    EXPECT_EQ(childOrder("children([3:1:5]);"), "c3,c4,c5");
    EXPECT_EQ(childOrder("children([1:3]);"), "c1,c2,c3");  // implicit step
}

TEST(ChildrenIndex, DescendingRangeRunsBackwards) {
    // The sign of the step decides direction, exactly as in a for-loop.
    EXPECT_EQ(childOrder("children([3:-1:1]);"), "c3,c2,c1");
    EXPECT_EQ(childOrder("children([5:-1:3]);"), "c5,c4,c3");
    EXPECT_EQ(childOrder("children([2:-1:0]);"), "c2,c1,c0");
}

TEST(ChildrenIndex, RangeStepIsHonoured) {
    EXPECT_EQ(childOrder("children([5:-2:0]);"), "c5,c3,c1");
    EXPECT_EQ(childOrder("children([0:2:5]);"), "c0,c2,c4");
}

TEST(ChildrenIndex, SingleElementRange) {
    EXPECT_EQ(childOrder("children([3:-1:3]);"), "c3");
    EXPECT_EQ(childOrder("children([3:1:3]);"), "c3");
}

TEST(ChildrenIndex, VectorSelectsInTheGivenOrder) {
    EXPECT_EQ(childOrder("children([3,4,5]);"), "c3,c4,c5");
    EXPECT_EQ(childOrder("children([5,0,2]);"), "c5,c0,c2");
}

TEST(ChildrenIndex, DuplicatesAreKept) {
    // children([2,2,2]) really does evaluate child 2 three times.
    EXPECT_EQ(childOrder("children([2,2,2]);"), "c2,c2,c2");
}

TEST(ChildrenIndex, FractionalIndicesTruncate) {
    EXPECT_EQ(childOrder("children([1.7]);"), "c1");
    EXPECT_EQ(childOrder("children([0:0.5:2]);"), "c0,c0,c1,c1,c2");
}

TEST(ChildrenIndex, EmptySelectionsProduceNothing) {
    EXPECT_EQ(childOrder("children([]);"), "");
    // Wrong direction for the step -- naturally empty, same as a for-loop.
    EXPECT_EQ(childOrder("children([1:0]);"), "");
    EXPECT_EQ(childOrder("children([3:1:1]);"), "");
}

TEST(ChildrenIndex, AnOutOfRangeIndexIsSkippedNotFatal) {
    // children([0,99]) still draws child 0.
    EXPECT_EQ(childOrder("children([0,99]);"), "c0");
    EXPECT_EQ(childOrder("children([99,1]);"), "c1");
    EXPECT_EQ(childOrder("children([-1,0]);"), "c0");
}

TEST(ChildrenIndex, OutOfRangeWarns) {
    // This used to return silently, for the plain-number form too.
    const std::vector<std::string> w = childWarnings("children(99);");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("Children index (99) out of bounds (3 children)"), std::string::npos) << w[0];
}

TEST(ChildrenIndex, NegativeIndexWarns) {
    const std::vector<std::string> w = childWarnings("children(-1);");
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("out of bounds"), std::string::npos) << w[0];
}

TEST(ChildrenIndex, EachBadIndexInAVectorWarnsSeparately) {
    const std::vector<std::string> w = childWarnings("children([99,98]);");
    EXPECT_EQ(w.size(), 2u);
}

TEST(ChildrenIndex, BadParameterTypeWarns) {
    // Quoted verbatim from the reference, trailing period included.
    for (const char* src : {"children(\"a\");", "children([\"a\"]);",
                             "children(true);", "children([true]);"}) {
        const std::vector<std::string> w = childWarnings(src);
        ASSERT_FALSE(w.empty()) << src;
        EXPECT_NE(w[0].find("for children, only accept: empty, number, vector, range."),
                  std::string::npos) << src << " -> " << w[0];
    }
}

TEST(ChildrenIndex, GeometryFollowsTheSelection) {
    // The echo-based checks above prove ordering; this one proves the
    // geometry really is the selected children and not child 0.
    // Written out rather than looped: children are counted as STATEMENTS,
    // and a for-loop is one statement however many bodies it emits.
    Evaluated e = evalSrc(
        "module pick() { children([3:1:5]); }\n"
        "pick() {\n"
        "  translate([0,0,0]) cube(1);  translate([2,0,0]) cube(1);\n"
        "  translate([4,0,0]) cube(1);  translate([6,0,0]) cube(1);\n"
        "  translate([8,0,0]) cube(1);  translate([10,0,0]) cube(1);\n"
        "}\n");
    double total = 0.0;
    manifold::Box all;
    bool first = true;
    for (const ColoredBody& b : e.bodies) {
        if (!b.body || b.body->IsEmpty()) continue;
        total += b.body->Volume();
        all = first ? b.body->BoundingBox() : all.Union(b.body->BoundingBox());
        first = false;
    }
    EXPECT_NEAR(total, 3.0, 1e-9);      // three unit cubes
    EXPECT_NEAR(all.min.x, 6.0, 1e-9);  // children 3, 4, 5 sit at x = 6, 8, 10
    EXPECT_NEAR(all.max.x, 11.0, 1e-9);
}

// -- range direction warnings ---------------------------------------------
//
// A range whose step points away from its end is naturally empty. The
// reference warns rather than iterating zero times in silence, since it is
// almost always a typo -- [1:0] where [1:-1:0] was meant. We did not warn
// anywhere. Wired into expandIterable, so every construct that iterates a
// range gets it: for, list comprehensions, intersection_for, children().

namespace {

std::vector<std::string> warningsFrom(const std::string& src) {
    std::vector<std::string> warnings;
    Evaluator ev([&](const std::string& m) {
        if (m.rfind("WARNING:", 0) == 0) warnings.push_back(m);
    });
    std::vector<std::unique_ptr<oscad::ASTNode>> ast = test::parseSrc(src);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    return warnings;
}

bool hasWarning(const std::vector<std::string>& w, const std::string& needle) {
    for (const std::string& m : w) {
        if (m.find(needle) != std::string::npos) return true;
    }
    return false;
}

const char* kGreater = "begin is greater than the end, but step is positive";
const char* kSmaller = "begin is smaller than the end, but step is negative";

} // namespace

TEST(RangeDirection, ImplicitStepPastTheEndWarns) {
    EXPECT_TRUE(hasWarning(warningsFrom("for (i=[1:0]) echo(i);"), kGreater));
    EXPECT_TRUE(hasWarning(warningsFrom("for (i=[3:1]) echo(i);"), kGreater));
}

TEST(RangeDirection, AnExplicitStepIsTakenAsDeliberate) {
    // Divergence from the reference, which warns for both of these. Writing
    // the step out is a statement of intent; the warning exists for the
    // author who wrote [3:1] meaning [3:-1:1] and got an empty loop.
    for (const char* src : {"for (i=[3:1:1]) echo(i);", "for (i=[0:-1:3]) echo(i);"}) {
        EXPECT_TRUE(warningsFrom(src).empty()) << src;
    }
}

TEST(RangeDirection, TheNegativeStepWordingIsUnreachable) {
    // An implicit step is always exactly 1, so no input can produce the
    // reference's second message. If this ever fires, the gate has been
    // widened back to explicit steps and this port needs that wording back.
    for (const char* src : {"for (i=[0:-1:3]) echo(i);", "for (i=[3:1]) echo(i);",
                             "x = [5:0];", "echo(chr([70:65]));"}) {
        EXPECT_FALSE(hasWarning(warningsFrom(src), kSmaller)) << src;
    }
}

TEST(RangeDirection, AWellFormedRangeIsSilent) {
    for (const char* src : {"for (i=[1:1:3]) echo(i);", "for (i=[3:-1:1]) echo(i);",
                             "for (i=[3:3]) echo(i);", "for (i=[0:0.5:2]) echo(i);",
                             "for (i=[1:3]) echo(i);"}) {
        EXPECT_TRUE(warningsFrom(src).empty()) << src;
    }
}

TEST(RangeDirection, AZeroStepIsNotReportedAsADirectionProblem) {
    // Different failure entirely -- the reference calls it "too many
    // elements". It shares rangeElementCount's nullopt with the
    // wrong-direction case only by coincidence.
    const std::vector<std::string> w = warningsFrom("for (i=[0:0:3]) echo(i);");
    EXPECT_FALSE(hasWarning(w, kGreater));
    EXPECT_FALSE(hasWarning(w, kSmaller));
}

TEST(RangeDirection, WarnsWhereTheRangeIsBuiltNotWhereItIsIterated) {
    // The reference reports this against the range literal, so a range that
    // is assigned and never iterated still warns -- matched here.
    EXPECT_TRUE(hasWarning(warningsFrom("r = [5:0];\ncube(1);"), kGreater));
    // And a range built once, iterated twice, warns once.
    const std::vector<std::string> w =
        warningsFrom("r = [5:0];\nfor (i=r) echo(i);\nfor (j=r) echo(j);");
    EXPECT_EQ(w.size(), 1u);
}

TEST(RangeDirection, EveryRangeBuildingConstructWarns) {
    // Construction is one shared site, but each of these reaches it by its
    // own route -- the interpreter's literal, the VM's Range op, and the
    // argument-evaluation path into a builtin.
    {
        const std::vector<std::string> w = warningsFrom("x = [for (i=[3:1]) i];");
        std::string got;
        for (const std::string& m : w) got += "\n    " + m;
        EXPECT_TRUE(hasWarning(w, kGreater)) << "list comprehension, got:" << got;
    }
    EXPECT_TRUE(hasWarning(warningsFrom("intersection_for (i=[3:1]) cube(1);"), kGreater))
        << "intersection_for";
    EXPECT_TRUE(hasWarning(warningsFrom("echo(chr([70:65]));"), kGreater)) << "chr";
    EXPECT_TRUE(hasWarning(warningsFrom(
        "module p() { children([1:0]); }\np() { cube(1); cube(2); }"), kGreater))
        << "children";
    EXPECT_TRUE(hasWarning(warningsFrom("function f() = [5:0];\nx = f();"), kGreater))
        << "returned from a function";
}

