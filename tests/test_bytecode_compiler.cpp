#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_evaluator/eval_error.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

// Targeted correctness tests for the Phase 1 bytecode compiler/VM
// (bytecode_compiler.cpp/bytecode_vm.cpp), forcing OSCAD_BYTECODE_VM on for
// just these tests (see Evaluator::setBytecodeVmEnabledForTesting -- the
// plain env var is cached forever after its first read, so a setenv() from
// inside a test body wouldn't reliably affect a shared test binary). The
// full 519-test suite is also run twice end-to-end (VM off / VM on via the
// real env var, two separate process invocations) as the differential
// "output matches" check this phase's plan calls for; these tests instead
// target the specific hard-part hazards a differential run could pass by
// accident -- self-referential/sibling-isolated parameter defaults, and
// sequential let() slot reassignment -- plus a couple of sanity checks that
// a non-compilable function (contains a call) still falls back correctly.
using namespace oscadeval;
using namespace oscadeval::test;

namespace {

class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

// Mirrors test_scoping.cpp's own captured-echo pattern: resolveTree() alone
// is enough to run every statement (echo included) without needing real
// geometry output. Accumulates every echo() call, newline-joined, so a
// multi-echo script's full output can be asserted in one go.
std::string runCapturingEcho(const std::string& code) {
    std::string captured;
    Evaluator ev([&](const std::string& msg) {
        if (!captured.empty()) captured += "\n";
        captured += msg;
    });
    auto ast = parseSrc(code);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    return captured;
}

} // namespace

TEST(BytecodeCompiler, BasicArithmeticAndComparison) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function add(a, b) = a + b;\necho(add(3, 4));"), "ECHO: 7");
    EXPECT_EQ(runCapturingEcho("function cmp(a, b) = a < b;\necho(cmp(3, 4));"), "ECHO: true");
    EXPECT_EQ(runCapturingEcho("function bits(x, y) = x & y;\necho(bits(6, 3));"), "ECHO: 2");
}

TEST(BytecodeCompiler, TernaryAndShortCircuitLogicalOps) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function pick(x) = x > 0 ? \"pos\" : \"neg\";\necho(pick(5));"), "ECHO: \"pos\"");
    EXPECT_EQ(runCapturingEcho("function pick(x) = x > 0 ? \"pos\" : \"neg\";\necho(pick(-5));"), "ECHO: \"neg\"");
    // is_undef(x) || (assert(...) ...) idiom -- must not evaluate the RHS
    // (which would throw) when the LHS is already true.
    EXPECT_EQ(runCapturingEcho("function safe(x) = is_undef(x) || x > 0;\necho(safe(undef));"), "ECHO: true");
}

TEST(BytecodeCompiler, PlainListLiteralCompiles) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function mk(x) = [x, x * 2, x * 3];\necho(mk(2));"), "ECHO: [2, 4, 6]");
    EXPECT_EQ(runCapturingEcho("function mk() = [];\necho(mk());"), "ECHO: []");
    // Nested plain list literals.
    EXPECT_EQ(runCapturingEcho("function mk(x) = [[x, 0], [0, x]];\necho(mk(5));"), "ECHO: [[5, 0], [0, 5]]");
}

TEST(BytecodeCompiler, ListLiteralWithRealComprehensionClauseStillBailsAndFallsBack) {
    ScopedVm vm(true);
    // `for` inside the vector is a real comprehension clause (deferred to
    // the original Phase 3) -- this whole function must bail compilation
    // and still produce the correct answer via the interpreter fallback.
    EXPECT_EQ(runCapturingEcho("function mk(n) = [for (i = [0:n]) i * i];\necho(mk(3));"), "ECHO: [0, 1, 4, 9]");
}

TEST(BytecodeCompiler, IndexAndMemberAccess) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function idx(v) = v[1];\necho(idx([10, 20, 30]));"), "ECHO: 20");
    EXPECT_EQ(runCapturingEcho("function xcomp(v) = v.x;\necho(xcomp([1, 2, 3]));"), "ECHO: 1");
}

// The self-referential-default hazard the plan's Phase 1 section calls out
// by name: a parameter's own default expression referencing its own name
// (or a sibling's) must NOT see that parameter's real slot value -- it must
// resolve to undef, exactly mirroring applyDefaults()'s existing
// let_->openChild(isolate=true) sibling isolation. My compiler reproduces
// this by compiling default expressions with a scope that has zero
// parameter-name bindings at all (see bytecode_compiler.cpp), not via any
// runtime special-casing -- these tests are the actual proof it works.
TEST(BytecodeCompiler, SelfReferentialParameterDefaultResolvesToUndef) {
    ScopedVm vm(true);
    // Referencing "x" inside its own default falls all the way through to
    // Scope::lookupVariable finding nothing usable there (a
    // ParameterDeclaration), producing the same "Ignoring unknown
    // variable" WARNING the plain interpreter does today (verified against
    // the unmodified CLI directly, not assumed) -- this test's own point is
    // that the compiled path reproduces that exactly, warning included, not
    // that the warning is otherwise desirable.
    EXPECT_EQ(runCapturingEcho("function foo(x = is_undef(x) ? 5 : x) = x;\necho(foo());"),
              "WARNING: Ignoring unknown variable 'x' in file <string>, line 1\nECHO: 5");
    // Caller-supplied value: default expression never runs at all, so no
    // warning either.
    EXPECT_EQ(runCapturingEcho("function foo(x = is_undef(x) ? 5 : x) = x;\necho(foo(10));"), "ECHO: 10");
}

TEST(BytecodeCompiler, SiblingParameterDefaultCannotSeeOtherParametersRealValue) {
    ScopedVm vm(true);
    // `b`'s default references `a` -- under applyDefaults' existing
    // isolation this is undef, not whatever `a` was actually bound to
    // (`undef + 1` -> undef, matching AdditionOp's non-numeric fallback).
    EXPECT_EQ(runCapturingEcho("function foo(a, b = a + 1) = b;\necho(foo(41));"),
              "WARNING: Ignoring unknown variable 'a' in file <string>, line 1\nECHO: undef");
}

TEST(BytecodeCompiler, SequentialLetReassignmentSeesPriorValue) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function bar(h) = let(h = h * 2) h;\necho(bar(3));"), "ECHO: 6");
    // BOSL2-style plain-variable-name reassignment via nested let().
    EXPECT_EQ(runCapturingEcho("function baz(x) = let(x = x + 1, x = x + 1) x;\necho(baz(1));"), "ECHO: 3");
}

TEST(BytecodeCompiler, LetCanOverrideDollarVar) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function withFn() = let($fn = 10) $fn;\necho(withFn());"), "ECHO: 10");
}

// `helper`'s call inside `outer`'s body means `outer` bails compilation
// entirely (see tryCompileFunction) -- this must still produce the correct
// value via the ordinary interpreter fallback, with the VM enabled for
// every OTHER function in the same run.
TEST(BytecodeCompiler, NonCompilableFunctionContainingACallStillFallsBackCorrectly) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function helper(x) = x + 1;\nfunction outer(y) = helper(y) * 2;\necho(outer(3));"),
              "ECHO: 8");
}

TEST(BytecodeCompiler, DollarPrefixedParameterBailsCompilationAndStillWorks) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function withFn($fn = 8) = $fn;\necho(withFn());"), "ECHO: 8");
    EXPECT_EQ(runCapturingEcho("function withFn($fn = 8) = $fn;\necho(withFn($fn = 20));"), "ECHO: 20");
}

TEST(BytecodeCompiler, UndeclaredDollarNamedArgumentReachesDynInsideCompiledFunction) {
    ScopedVm vm(true);
    // `plain` doesn't declare $fn as its own parameter, but a call-site
    // $fn= override must still reach the compiled body via LOAD_DYN.
    EXPECT_EQ(runCapturingEcho("function plain(x) = x + $fn;\necho(plain(1, $fn = 41));"), "ECHO: 42");
}

TEST(BytecodeCompiler, CallToBuiltinFunctionCompiles) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f(x) = is_undef(x) ? -1 : len(x);\necho(f([1, 2, 3]));"), "ECHO: 3");
    EXPECT_EQ(runCapturingEcho("function f(x) = sin(x);\necho(f(0));"), "ECHO: 0");
}

TEST(BytecodeCompiler, CallToOtherUserFunctionCompiles) {
    ScopedVm vm(true);
    // Both `helper` and `outer` are now compilable (Phase 1.5b) -- this is
    // the same script Phase 1's own fallback test used, but this time
    // BOTH functions actually compile rather than `outer` bailing.
    EXPECT_EQ(runCapturingEcho("function helper(x) = x + 1;\nfunction outer(y) = helper(y) * 2;\necho(outer(3));"),
              "ECHO: 8");
    // Named + positional arguments at a compiled call site.
    EXPECT_EQ(
        runCapturingEcho("function add(a, b) = a + b;\nfunction outer(x) = add(x, b = 10);\necho(outer(5));"),
        "ECHO: 15");
}

// Direct recursion through a compiled call site: each call gets its own
// fresh slot array (an ordinary C++ local, not pooled/shared storage), so
// nothing aliases between concurrently-active invocations -- this is the
// empirical proof for that reasoning, not just a plausibility argument.
TEST(BytecodeCompiler, DirectRecursionThroughCompiledCallSiteWorks) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function fact(n) = n <= 1 ? 1 : n * fact(n - 1);\necho(fact(6));"), "ECHO: 720");
}

TEST(BytecodeCompiler, MutualRecursionThroughCompiledCallSitesWorks) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function is_even(n) = n == 0 ? true : is_odd(n - 1);\n"
                                "function is_odd(n) = n == 0 ? false : is_even(n - 1);\n"
                                "echo(is_even(10));"),
              "ECHO: true");
}

TEST(BytecodeCompiler, CallToUnresolvableCalleeStillBailsAndFallsBack) {
    ScopedVm vm(true);
    // `g` is a function-literal *value* held in a variable -- not
    // statically resolvable to a FunctionDeclaration at compile time, so
    // `outer` must bail and still produce the correct answer.
    EXPECT_EQ(runCapturingEcho("g = function(x) x * 2;\nfunction outer(y) = g(y) + 1;\necho(outer(3));"), "ECHO: 7");
}

// -- Phase 3: real list-comprehension clauses ------------------------------
// Mirrors tests/test_control_flow.cpp's own ListComprehension suite
// (top-level assignment scripts there), wrapped in a compiled function's
// body here to exercise the compiler's own compileListElement/
// compileListCompBody instead of the AST interpreter.

TEST(BytecodeCompiler, ForClauseExpandsRange) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [for (i = [0:4]) i];\necho(f());"), "ECHO: [0, 1, 2, 3, 4]");
}

TEST(BytecodeCompiler, ForIfFiltersElements) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [for (i = [0:4]) if (i % 2 == 0) i];\necho(f());"), "ECHO: [0, 2, 4]");
}

TEST(BytecodeCompiler, ForIfElseClauseSyntax) {
    ScopedVm vm(true);
    EXPECT_EQ(
        runCapturingEcho("function f() = [for (i = [0:3]) if (i % 2 == 0) \"even\" else \"odd\"];\necho(f());"),
        "ECHO: [\"even\", \"odd\", \"even\", \"odd\"]");
}

TEST(BytecodeCompiler, EachFlattensNestedLists) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [each [1,2], each [3,4]];\necho(f());"), "ECHO: [1, 2, 3, 4]");
}

TEST(BytecodeCompiler, ListCompLetPrecedingForBindsIntoLaterClause) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [let (a = 1) for (i = [0:2]) i + a];\necho(f());"), "ECHO: [1, 2, 3]");
}

TEST(BytecodeCompiler, CStyleForAccumulates) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [for (a = 0, i = 0; i < 5; a = a + i, i = i + 1) a];\necho(f());"),
              "ECHO: [0, 0, 1, 3, 6]");
}

TEST(BytecodeCompiler, CStyleForExceedingMaxIterationsThrows) {
    ScopedVm vm(true);
    EXPECT_THROW(runCapturingEcho("function f() = [for (a = 0; true; a = a + 1) a];\necho(f());"), EvalError);
}

TEST(BytecodeCompiler, EachWrappingAForClauseFlattensIt) {
    ScopedVm vm(true);
    // Exercises AccumMergeEach specifically: the wrapped `for` clause's own
    // contribution is the thing being individually each-flattened.
    EXPECT_EQ(runCapturingEcho("function f() = [each for (i = [0:2]) i];\necho(f());"), "ECHO: [0, 1, 2]");
}

TEST(BytecodeCompiler, EachWrappingAForClauseOfListsFlattensEveryItemIndividually) {
    ScopedVm vm(true);
    // The specific hazard AccumMergeEach exists for: the inner clause's own
    // items are THEMSELVES lists ([i,i]) -- each one must be flattened
    // separately, not treated as one opaque unit.
    EXPECT_EQ(runCapturingEcho("function f() = [each for (i = [0:2]) [i, i]];\necho(f());"),
              "ECHO: [0, 0, 1, 1, 2, 2]");
}

TEST(BytecodeCompiler, EachWrappingAnIfElseClauseFlattensIt) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [each if (false) 5 else 6];\necho(f());"), "ECHO: [6]");
}

TEST(BytecodeCompiler, ForClauseWithNestedListLiteralBody) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [for (i = [0:1]) [i, i * 2]];\necho(f());"), "ECHO: [[0, 0], [1, 2]]");
}

TEST(BytecodeCompiler, CStyleForWithNestedListLiteralBody) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [for (a = 0; a < 2; a = a + 1) [a, a]];\necho(f());"),
              "ECHO: [[0, 0], [1, 1]]");
}

TEST(BytecodeCompiler, ForUndefIterableProducesEmptyList) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [for (i = undef) i];\necho(f());"), "ECHO: []");
}

TEST(BytecodeCompiler, ForOverUndefBodyKeepsUndefAsAnElement) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [for (i = [1,2]) undef];\necho(f());"), "ECHO: [undef, undef]");
}

TEST(BytecodeCompiler, ZeroStepRangeIterationYieldsNothing) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = [for (i = [1:0:5]) i];\necho(f());"), "ECHO: []");
}

TEST(BytecodeCompiler, MultiAssignmentForClauseIteratesCartesianProduct) {
    ScopedVm vm(true);
    // The genuinely new hazard for this phase: TWO assignments in one
    // `for(...)` -- nested iteration where the inner dimension must fully
    // restart (IterReset, not IterMaterialize again) on every outer step.
    EXPECT_EQ(runCapturingEcho("function f() = [for (i = [0:1], j = [0:1]) [i, j]];\necho(f());"),
              "ECHO: [[0, 0], [0, 1], [1, 0], [1, 1]]");
}

TEST(BytecodeCompiler, NonCompilableOuterFunctionCallingCompilableListCompFunctionStillWorks) {
    ScopedVm vm(true);
    // Sanity check that a compiled function containing a real
    // comprehension clause can still be CALLED from other compiled code
    // (via Phase 1.5b's CALL_USER_FN), not just invoked directly.
    EXPECT_EQ(runCapturingEcho("function evens(n) = [for (i = [0:n]) if (i % 2 == 0) i];\n"
                                "function outer(n) = len(evens(n));\n"
                                "echo(outer(6));"),
              "ECHO: 4");
}

TEST(BytecodeCompiler, VmOffAndVmOnAgreeOnListComprehensionCases) {
    const std::string script = "function f1() = [for (i = [0:4]) if (i % 2 == 0) i];\n"
                                "function f2() = [let (a = 1) for (i = [0:2]) i + a];\n"
                                "function f3() = [for (a = 0, i = 0; i < 5; a = a + i, i = i + 1) a];\n"
                                "function f4() = [each for (i = [0:2]) [i, i]];\n"
                                "function f5() = [for (i = [0:1], j = [0:1]) [i, j]];\n"
                                "echo(f1());\necho(f2());\necho(f3());\necho(f4());\necho(f5());\n";
    std::string offResult;
    {
        ScopedVm vm(false);
        offResult = runCapturingEcho(script);
    }
    std::string onResult;
    {
        ScopedVm vm(true);
        onResult = runCapturingEcho(script);
    }
    EXPECT_EQ(offResult, onResult);
}

// -- Phase 2: closures/upvalues --------------------------------------------
// A closure has no practical payoff unless something can call it BACK
// while its capturing frame is still active (this codebase has no
// escaping closures) -- the natural, most common shape is calling a
// locally-held function value directly (`g(5)` where `g` is a let/param
// bound to a FunctionLiteral), which needed its own dynamic-dispatch
// opcode (CALL_DYNAMIC) alongside LOAD_UPVALUE to be reachable at all; see
// bytecode_compiler.cpp's PrimaryCall case and bytecode_vm.cpp's
// CallDynamic handler.

TEST(BytecodeCompiler, ClosureCapturesParameterAndIsCalledDirectly) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function make(x) = let(g = function(y) y + x) g(5);\necho(make(10));"), "ECHO: 15");
}

TEST(BytecodeCompiler, ClosureCapturesLetBindingNotJustParameter) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function make(x) = let(n = x * 2, g = function(y) y + n) g(1);\necho(make(3));"),
              "ECHO: 7");
}

TEST(BytecodeCompiler, ClosurePassedThroughAnUnrelatedIntermediaryFunctionDoesNotCaptureAcrossIt) {
    ScopedVm vm(true);
    // `apply` is a genuinely SEPARATE, top-level function -- not lexically
    // nested inside `make` -- so real OpenSCAD's own "no escaping
    // closures, only reaches through an UNBROKEN chain of lexically-nested
    // calls" rule means `x` must NOT be visible when `f(v)` runs inside
    // apply's own (isolated) frame, even though make's frame is still
    // technically active on the call stack. Confirmed against the
    // unmodified interpreter directly (VM off) before writing this
    // expectation, not assumed -- this is exactly the case a naive
    // "scan the whole call stack for a matching declaration" upvalue
    // search gets wrong (see CallStackFrame::upvalueParent's own doc
    // comment for the story). `undef + 100` -> undef.
    EXPECT_EQ(runCapturingEcho("function apply(f, v) = f(v);\n"
                                "function make(x) = apply(function(y) y + x, 100);\n"
                                "echo(make(5));"),
              "ECHO: undef");
}

TEST(BytecodeCompiler, NestedClosureCapturesBothEnclosingLevels) {
    ScopedVm vm(true);
    // A closure inside a closure -- the innermost literal's own free
    // variables resolve to TWO different enclosing levels (`x` from the
    // outermost function, `y` from the middle closure), exercising the
    // multi-level enclosing chain, not just one hop.
    EXPECT_EQ(runCapturingEcho("function outer(x) = let(mid = function(y) let(inner = function(z) x + y + z) "
                                "inner(1)) mid(10);\n"
                                "echo(outer(100));"),
              "ECHO: 111");
}

TEST(BytecodeCompiler, RecursiveClosureCapturingOuterParameterWorks) {
    ScopedVm vm(true);
    // The closure itself recurses (direct recursion through CALL_DYNAMIC,
    // not CALL_FN, since the callee is the closure's own dynamically-held
    // value) while still correctly reading its captured upvalue `step` on
    // every recursive call.
    EXPECT_EQ(runCapturingEcho("function make(step) = let(count_to = function(n) n <= 0 ? 0 : step + "
                                "count_to(n - 1)) count_to(4);\n"
                                "echo(make(10));"),
              "ECHO: 40");
}

TEST(BytecodeCompiler, ClosureOverNonActiveEnclosingCallResolvesToUndef) {
    ScopedVm vm(true);
    // No escaping closures: `outer`'s own call has already returned by
    // the time `stored(3)` runs (a separate top-level statement) -- `n`
    // must resolve to undef, exactly matching the pre-existing
    // interpreter limitation (not something this phase is expected to
    // fix). `undef + 3` -> undef via AdditionOp's non-numeric fallback.
    EXPECT_EQ(runCapturingEcho("function outer(n) = function(y) y + n;\n"
                                "stored = outer(5);\n"
                                "echo(stored(3));"),
              "ECHO: undef");
}

TEST(BytecodeCompiler, VmOffAndVmOnAgreeOnClosureCases) {
    const std::string script = "function make(x) = let(g = function(y) y + x) g(5);\n"
                                "function apply(f, v) = f(v);\n"
                                "function make2(x) = apply(function(y) y + x, 100);\n"
                                "function make3(step) = let(count_to = function(n) n <= 0 ? 0 : step + "
                                "count_to(n - 1)) count_to(4);\n"
                                "echo(make(10));\necho(make2(5));\necho(make3(10));\n";
    std::string offResult;
    {
        ScopedVm vm(false);
        offResult = runCapturingEcho(script);
    }
    std::string onResult;
    {
        ScopedVm vm(true);
        onResult = runCapturingEcho(script);
    }
    EXPECT_EQ(offResult, onResult);
}

// -- Tail-call optimization, VM path (Phase B) -----------------------------
//
// Mirrors tests/test_tail_calls.cpp's own Phase A (interpreter) suite, but
// forcing the compiled path via ScopedVm(true). Before Phase B
// (CallFnTail/CallDynamicTail, runCompiledFunctionTrampoline/
// runCompiledFunctionFromBoundTrampoline), every compiled call -- tail
// position or not -- recursed genuinely (runChunk -> evalUserFunctionFromBound
// -> evalUserFunctionCore -> runCompiledFunctionFromBound -> runChunk),
// growing the real C++ stack once per OpenSCAD-level call; these are the
// tests that prove the VM path specifically needed its own fix, not just
// Phase A's interpreter-side one (a script forcing OSCAD_BYTECODE_VM=0 was
// already covered there).

TEST(BytecodeCompiler, DeepSelfTailRecursionUnderVmDoesNotOverflowTheNativeStack) {
    ScopedVm vm(true);
    const std::string script = "function sum(n, acc=0) = n == 0 ? acc : sum(n - 1, acc + n);\necho(sum(500000));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 1.25e+11");
}

TEST(BytecodeCompiler, MutualTailRecursionUnderVmAtDepthDoesNotOverflowTheNativeStack) {
    ScopedVm vm(true);
    const std::string script = "function is_even(n) = n == 0 ? true : is_odd(n - 1);\n"
                                "function is_odd(n) = n == 0 ? false : is_even(n - 1);\n"
                                "echo(is_even(300000));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: true");
}

TEST(BytecodeCompiler, MixedCompiledInterpretedTailChainStaysCorrect) {
    ScopedVm vm(true);
    // `relay` is compiled and tail-calls `uses_echo`, which contains an
    // echo() expression and so can never compile (NotCompilable, see
    // compileExpr's own default case) -- this hop crosses the compiled/
    // interpreted boundary, paying one real C++ frame there by design (see
    // runCompiledFunctionTrampoline's own doc comment), but must still
    // produce the correct value. `uses_echo` then tail-calls BACK into the
    // compiled `relay` for good measure.
    const std::string script = "function uses_echo(n) = echo(\"hop\") n == 0 ? 99 : relay(n - 1);\n"
                                "function relay(n) = n == 0 ? -1 : uses_echo(n - 1);\n"
                                "echo(relay(4));";
    // relay(4) -> uses_echo(3) [echo] -> relay(2) -> uses_echo(1) [echo] ->
    // relay(0) -> -1 (two hops through uses_echo, each crossing the
    // compiled/interpreted boundary once each way).
    EXPECT_EQ(runCapturingEcho(script), "ECHO: \"hop\"\nECHO: \"hop\"\nECHO: -1");
}

TEST(BytecodeCompiler, ClosureNestedTailCallUnderVmFallsBackAndStillResolves) {
    ScopedVm vm(true);
    // Same closure-nesting hazard as the interpreter path's own
    // ClosureNestedTailCallFallsBackToRealRecursionAndStillResolves
    // (test_tail_calls.cpp) -- g's own call is lexically nested inside
    // make's body (a closure over `x`, read via Op::LoadUpvalue), so
    // isolatedCallCtxFor must return nullopt for it, and CallDynamicTail's
    // handler must fall back to a real evalFunctionLiteralFromBound call
    // instead of trampolining.
    const std::string script = "function make(x) = let(g = function(y) y + x) g(5);\necho(make(10));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 15");
}

TEST(BytecodeCompiler, InfiniteTailRecursionUnderVmHitsTheIterationCapWithABoundedTrace) {
    // `loop`'s body (a plain PrimaryCall in tail position, no echo/assert)
    // compiles cleanly, so this exercises Op::CallFnTail's own runtime
    // isolation/chunk check and runCompiledFunctionTrampoline's loop
    // directly -- unlike a script using assert()/echo() in the chain
    // (those node kinds never compile at all, see compileExpr's own
    // default case, so a function using either always falls back to the
    // interpreter's own trampoline regardless of OSCAD_BYTECODE_VM).
    ScopedVm vm(true);
    Evaluator ev;
    auto ast = parseSrc("function loop(n) = loop(n + 1);\nresult = loop(0);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    try {
        ev.resolveTree(ast, ctx);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("loop"), std::string::npos);
        EXPECT_NE(msg.find("Recursion detected"), std::string::npos);
        // Bounded TRACE despite 1,000,000 tail hops: runCompiledFunctionTrampoline
        // mutates callStack_'s current frame in place per hop (never
        // pushes), exactly like the interpreter path's own trampoline.
        size_t count = 0;
        for (size_t pos = msg.find("TRACE:"); pos != std::string::npos; pos = msg.find("TRACE:", pos + 1)) ++count;
        EXPECT_LE(count, 2u);
    }
}

TEST(BytecodeCompiler, VmOffAndVmOnAgreeOnDefaultAndFallbackCases) {
    const std::string script = "function helper(x) = x + 1;\n"
                                "function outer(y) = helper(y) * 2;\n"
                                "function foo(x = is_undef(x) ? 5 : x) = x;\n"
                                "function fact(n) = n <= 1 ? 1 : n * fact(n - 1);\n"
                                "function mk(x) = [x, len([x, x])];\n"
                                "g = function(x) x * 2;\n"
                                "function dyncall(y) = g(y) + 1;\n"
                                "echo(outer(3));\n"
                                "echo(foo());\n"
                                "echo(foo(10));\n"
                                "echo(fact(6));\n"
                                "echo(mk(9));\n"
                                "echo(dyncall(4));\n";
    std::string offResult;
    {
        ScopedVm vm(false);
        offResult = runCapturingEcho(script);
    }
    std::string onResult;
    {
        ScopedVm vm(true);
        onResult = runCapturingEcho(script);
    }
    EXPECT_EQ(offResult, onResult);
}
