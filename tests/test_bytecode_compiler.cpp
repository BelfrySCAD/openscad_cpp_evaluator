#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_evaluator/eval_error.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
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

// Counts total debugHook invocations (statement- and expr-level combined)
// running `code` with a debugger attached. A VM-compiled function call
// gets exactly ONE stop -- evalUserFunctionCore's own unconditional
// body-entry checkDebug(), the only checkpoint compiled bytecode has left
// (see useBytecodeVm()/chunkEligibleNow's own doc comments, evaluator.hpp)
// -- while an interpreted call additionally hits every sub-expression
// checkpoint inside it (ternary condition + chosen branch, at minimum, for
// the functions these tests use). That gap is the only observable,
// black-box signal these tests have for "did this call actually run
// compiled or interpreted" -- there's no other public introspection point.
int countDebugHookStops(const std::string& code, std::optional<std::unordered_map<std::string, std::set<int>>> fastContinueBreakpoints) {
    int stops = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool, bool, const std::string&, const std::vector<CallStackFrame>&,
                          const DebugFramesFn&) {
        ++stops;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    if (fastContinueBreakpoints) ev.setFastContinueBreakpoints(std::move(fastContinueBreakpoints));
    auto ast = parseSrc(code);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    return stops;
}

std::filesystem::path tempPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("oscad_bytecode_compiler_test_" + name);
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

// TernaryAndShortCircuitLogicalOps above only asserts the RESULT, which an
// eager compiler computes just as correctly -- so it cannot actually fail if
// someone drops the JumpIfFalse/JumpIfTrue the compiler emits around the
// right operand / untaken branch. test_expr_eval.cpp's own
// RightSideNotEvaluatedWhenShortCircuited and OnlyChosenBranchEvaluates
// cover that for the AST interpreter only. These two do it for the compiled
// path, watching for a marker echo() from the side that must never run.
TEST(BytecodeCompiler, ShortCircuitDoesNotEvaluateRightOperandCompiled) {
    ScopedVm vm(true);
    const std::string defs = "function rhs() = echo(\"RHS RAN\") true;\n";
    // Left operand already decides it -- rhs() must stay unevaluated.
    EXPECT_EQ(runCapturingEcho(defs + "function f() = true || rhs();\necho(f());"), "ECHO: true");
    EXPECT_EQ(runCapturingEcho(defs + "function f() = false && rhs();\necho(f());"), "ECHO: false");
    // ...and the mirror image: it must still run when the left operand doesn't.
    EXPECT_EQ(runCapturingEcho(defs + "function f() = false || rhs();\necho(f());"),
              "ECHO: \"RHS RAN\"\nECHO: true");
    EXPECT_EQ(runCapturingEcho(defs + "function f() = true && rhs();\necho(f());"),
              "ECHO: \"RHS RAN\"\nECHO: true");
}

TEST(BytecodeCompiler, TernaryOnlyEvaluatesChosenBranchCompiled) {
    ScopedVm vm(true);
    const std::string defs = "function yes() = echo(\"TRUE-BRANCH\") 1;\n"
                             "function no() = echo(\"FALSE-BRANCH\") 2;\n"
                             "function pick(c) = c ? yes() : no();\n";
    EXPECT_EQ(runCapturingEcho(defs + "echo(pick(true));"), "ECHO: \"TRUE-BRANCH\"\nECHO: 1");
    EXPECT_EQ(runCapturingEcho(defs + "echo(pick(false));"), "ECHO: \"FALSE-BRANCH\"\nECHO: 2");
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
    // ParameterDeclaration) -- undef, so the ternary takes the 5. Silent:
    // is_undef() is a probe and does not warn about the name it asks about,
    // matching the reference, which is silent for this exact script. This
    // test's own point is that the compiled path reproduces the interpreter
    // exactly; it used to assert a warning both engines emitted and the
    // reference did not.
    EXPECT_EQ(runCapturingEcho("function foo(x = is_undef(x) ? 5 : x) = x;\necho(foo());"), "ECHO: 5");
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
              "WARNING: Ignoring unknown variable 'a' in file <string>, line 1\n"
              "WARNING: undefined operation (undefined + number) in file <string>, line 1\nECHO: undef");
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

TEST(BytecodeCompiler, DollarPrefixedParameterCompiles) {
    ScopedVm vm(true);
    // $-prefixed parameters used to bail compileFunctionLike outright
    // (bound through ctx.dyn, never slot-addressed) -- now they compile:
    // never declared as a local (skipped in the params/declareLocal loop),
    // so every reference inside the body still reaches Op::LoadDyn via the
    // Identifier case's own $-check, exactly as an undeclared override
    // already did.
    EXPECT_EQ(runCapturingEcho("function withFn($fn = 8) = $fn;\necho(withFn());"), "ECHO: 8");
    EXPECT_EQ(runCapturingEcho("function withFn($fn = 8) = $fn;\necho(withFn($fn = 20));"), "ECHO: 20");
    // Positional binding: a $-param still occupies its declared POSITION
    // among all parameters for a caller's positional arguments, exactly
    // like the interpreter's own bindArgs.
    EXPECT_EQ(runCapturingEcho("function withFn($fn, x) = $fn + x;\necho(withFn(10, 1));"), "ECHO: 11");
    // Declared without a default and not passed: shadows to undef locally
    // rather than inheriting an ancestor's dyn binding -- mirrors
    // Evaluator::applyDefaults' explicit "declaring a parameter always
    // creates a fresh binding" write.
    EXPECT_EQ(runCapturingEcho("function withFn($fn) = $fn;\necho(let($fn = 99) withFn());"), "ECHO: undef");
    // The proof this is actually running compiled, not silently falling
    // back: same ternary-bodied shape and technique as
    // FastContinueWithNoBreakpointInFunctionUsesVm above (2 stops =
    // compiled, 4 = interpreted), with $fn substituted in for the plain
    // parameter both as the declared name and every body reference.
    const int stops = countDebugHookStops("function f($fn) = $fn > 0 ? $fn + 1 : $fn - 1;\n"
                                           "echo(f(5));",
                                           std::unordered_map<std::string, std::set<int>>{{"<string>", {2}}});
    EXPECT_EQ(stops, 2);
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

TEST(BytecodeCompiler, CStyleForIncrCanIntroduceAndUseANewNameInTheSamePass) {
    // Real bug found via BOSL2's nurbs.scad hanging in BelfrySCAD (produced
    // no geometry there, silently, until traced to this): the compiler
    // assumed every incr assignment's target name was already declared in
    // the init list, dereferencing a disengaged scope.resolve() optional
    // (UB) whenever an incr introduced a brand-new name -- e.g. `k` here,
    // read by the very next incr in the same list. Verified against real
    // OpenSCAD and this port's own AST interpreter (both handle it
    // correctly) before fixing the compiler to match.
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho(
                  "function f() = [for (a = 0, i = 0; i < 5; k = i >= 2, a = k ? a + 10 : a + 1, i = i + 1) a];\n"
                  "echo(f());"),
              "ECHO: [0, 1, 2, 12, 22]");
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
// The natural, most common shape is calling a locally-held function value
// directly (`g(5)` where `g` is a let/param bound to a FunctionLiteral),
// which needed its own dynamic-dispatch opcode (CALL_DYNAMIC) alongside
// LOAD_UPVALUE to be reachable at all; see bytecode_compiler.cpp's
// PrimaryCall case and bytecode_vm.cpp's CallDynamic handler.
//
// Phase 2b (Op::MakeClosure) then made the CONTAINING function compilable
// even when it creates a closure that escapes (returned, stored, passed on)
// -- previously ANY FunctionLiteral referencing enclosing state bailed the
// whole container, unconditionally, since Op::LOAD_UPVALUE's live-call-
// stack walk (findUpvalue) can only resolve a still-active call frame,
// never survive past it. Op::MakeClosure instead snapshots the exact
// values a literal needs into a real TrailView<Value> at the moment of
// creation (see bytecode.hpp's own doc comment), reusing Closure::
// capturedLet/callCtxFor's existing capture-rooting unchanged -- so the
// closure's own BODY still always runs interpreted when invoked (never
// itself compiled), but the function that CREATES it no longer has to.
// Several tests below predate Phase 2b and were written to describe the
// OLD bail-and-fall-back behavior; their own comments now note what
// actually happens post-Phase-2b instead of describing stale behavior as
// if it were still current.

TEST(BytecodeCompiler, ClosureCapturesParameterAndIsCalledDirectly) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function make(x) = let(g = function(y) y + x) g(5);\necho(make(10));"), "ECHO: 15");
}

TEST(BytecodeCompiler, ClosureCapturesLetBindingNotJustParameter) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function make(x) = let(n = x * 2, g = function(y) y + n) g(1);\necho(make(3));"),
              "ECHO: 7");
}

TEST(BytecodeCompiler, ClosurePassedThroughAnUnrelatedIntermediaryFunctionStillCaptures) {
    ScopedVm vm(true);
    // `apply` is a genuinely SEPARATE, top-level function -- not lexically
    // nested inside `make` -- and its own call scope is isolated from
    // make's (an ordinary call boundary, see EvalContext::callCtx's own
    // isolate=true). Real OpenSCAD gives 105 here (verified against
    // ~/Desktop/OpenSCAD-dev.app), confirming a genuine closure carries
    // its captured environment through an arbitrary intermediary, not
    // just an unbroken chain of lexically-nested calls -- an earlier
    // version of this test asserted `undef`, describing this evaluator's
    // own bug as if it were real OpenSCAD's rule. This exact case is why
    // Evaluator::callCtxFor checks the closure's own capturedLet FIRST,
    // before any live-call-stack walk: make's own frame can still be
    // technically live on callStack_ while apply's own ctx is isolated
    // from it, so continuing ancestry from the CALLER's ctx (apply's)
    // rather than from the closure's own captured environment would
    // silently lose `x`.
    EXPECT_EQ(runCapturingEcho("function apply(f, v) = f(v);\n"
                                "function make(x) = apply(function(y) y + x, 100);\n"
                                "echo(make(5));"),
              "ECHO: 105");
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

TEST(BytecodeCompiler, ClosureOverNonActiveEnclosingCallResolvesCorrectly) {
    ScopedVm vm(true);
    // A genuine escaping closure: `outer`'s own call has already
    // returned by the time `stored(3)` runs (a separate top-level
    // statement), yet `n` must still resolve to 5 -- real OpenSCAD gives
    // 8 here (verified against ~/Desktop/OpenSCAD-dev.app). An earlier
    // version of this test asserted `undef`, describing a since-fixed gap
    // (no escaping-closure support) as if it were the intended contract.
    // The fix: Value's FunctionLiteral alternative became a real Closure
    // (node + capturedLet, a shared_ptr<TrailView<Value>> snapshot of
    // ctx.let_ at creation time, see value.hpp). Post-Phase-2b, `outer`
    // itself now compiles too (Op::MakeClosure builds an equivalent
    // snapshot from compiled code, see bytecode.hpp) -- only the returned
    // closure's own body (`function(y) y + n`) still always runs
    // interpreted when invoked, reading its capturedLet directly via the
    // same callCtxFor path an interpreter-created closure always has.
    EXPECT_EQ(runCapturingEcho("function outer(n) = function(y) y + n;\n"
                                "stored = outer(5);\n"
                                "echo(stored(3));"),
              "ECHO: 8");
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

TEST(BytecodeCompiler, MakeClosureLetsContainerCompileDespiteNonEscapingClosure) {
    ScopedVm vm(true);
    // `make` creates AND immediately calls `g` -- before Op::MakeClosure
    // this bailed `make` entirely (any non-empty upvalues did, escaping or
    // not). A debug hook alone forces the interpreter regardless of
    // compilability (see DebugAttachedWithoutFastContinueAlwaysInterprets
    // above), so fast-continue must be enabled (an empty breakpoint map --
    // fast-continue "on", nothing set) to actually observe whether `make`
    // runs compiled here.
    //
    // 2 stops: `make`'s own body-entry (1), `g`'s own body-entry (1) --
    // `g` itself now also runs compiled (its captures-having chunk is
    // registered too, not discarded -- see the FunctionLiteral case's own
    // doc comment, bytecode_compiler.cpp), so it costs exactly one stop
    // instead of the extra sub-expression checkpoint an interpreted call
    // used to add on top. NOT a 3rd stop for the top-level echo()
    // statement's own call-site anymore either (Phase 1, module/top-level
    // compilation): `echo(make(10))`'s own argument now compiles too (see
    // evalExprMaybeCompiled, evaluator.hpp), so make(10)'s call-site
    // checkpoint (previously fired by evalFunctionCall's own checkDebug,
    // evaluated as a plain interpreted expression) no longer fires --
    // exactly the same "compiled code has no sub-expression checkpoints"
    // trade-off a compiled FUNCTION body's own internal calls already had,
    // now also applying to a statement's own top-level expression.
    const int stops = countDebugHookStops("function make(x) = let(g = function(y) y + x) g(5);\n"
                                           "echo(make(10));",
                                           std::unordered_map<std::string, std::set<int>>{});
    EXPECT_EQ(stops, 2);
}

TEST(BytecodeCompiler, MakeClosureLetsContainerCompileDespiteEscapingClosure) {
    ScopedVm vm(true);
    // `outer` returns a closure that escapes past its own call entirely
    // (called later, from a separate top-level statement, see
    // ClosureOverNonActiveEnclosingCallResolvesCorrectly above for the
    // correctness side of this same shape) -- `outer` itself now compiles
    // too (fast-continue enabled for the same reason as the non-escaping
    // case above). The ternary is needed here purely to make compiled-vs-
    // interpreted OBSERVABLE via stop count: a body that's nothing but
    // "return a closure" has no sub-expression checkpoints in the
    // interpreter either way (a FunctionLiteral's own creation isn't itself
    // a checkpoint), so it produces the SAME stop count whether `outer`
    // compiled or not -- confirmed empirically, not assumed, by diffing
    // this test's own result against the pre-Op::MakeClosure compiler.
    //
    // 4, not 6: BOTH top-level statements' own expressions now compile too
    // (Phase 1, module/top-level compilation). Confirmed directly via a
    // standalone debug-hook trace, not derived by hand: (1) `stored =
    // outer(5);`'s own STATEMENT-level checkpoint (line 2, depth 0) --
    // now the evalChildren top-level list compiles as ONE chunk
    // (Evaluator::tryRunCompiledChildren, the "NativeStatement gap" fix),
    // this assignment runs as an ordinary Op::NativeStatement entry,
    // which DOES fire its own statement-level checkDebug -- unlike the
    // narrower, single-purpose tryRunCompiledAssignmentBlock path this
    // used to take instead (a real, single-assignment "block"), which
    // skips that checkpoint entirely. `outer(5)`'s own call-site
    // checkpoint is still lost either way (compiled either path). (2)
    // `outer`'s own body-entry (line 1, depth 1). (3) `echo(stored(3));`'s
    // own statement-level checkpoint (line 3, depth 0) -- unchanged from
    // before, echo was never part of the assignment-block compile. (4)
    // `stored(3)`'s own body-entry (line 1, depth 1) -- `stored` is an
    // ESCAPING closure (captures `n`), never compiles regardless (see
    // this test's own title), so this is the interpreter's usual
    // body-entry stop for a genuine call.
    const int stops = countDebugHookStops(
        "function outer(n) = n > 0 ? function(y) y + n : function(y) y - n;\n"
        "stored = outer(5);\n"
        "echo(stored(3));",
        std::unordered_map<std::string, std::set<int>>{});
    EXPECT_EQ(stops, 4);
}

TEST(BytecodeCompiler, MakeClosureCapturesFreshValuePerInvocation) {
    ScopedVm vm(true);
    // Op::MakeClosure runs fresh every time the instruction is actually
    // reached (never a compile-time constant) -- a closure built once per
    // list-comprehension iteration must capture THAT iteration's own `i`,
    // not all share whatever `i` ended at. `make_adders(3)` returns
    // [function(y) y+0, function(y) y+1, function(y) y+2]; calling each
    // with 10 and summing proves each closed over its OWN value.
    EXPECT_EQ(runCapturingEcho("function make_adders(n) = [for (i = [0:1:n-1]) function(y) y + i];\n"
                                "fns = make_adders(3);\n"
                                "echo(fns[0](10) + fns[1](10) + fns[2](10));"),
              "ECHO: 33");
}

TEST(BytecodeCompiler, MakeClosureBubblesCaptureThroughAnIntermediateClosureLevel) {
    ScopedVm vm(true);
    // `inner` (nested inside `mid`, nested inside `outer`) references
    // `outer`'s own `x` DIRECTLY, skipping `mid` entirely -- exercising
    // bubbleEscapingCaptures' own transitive merge (bytecode_compiler.cpp):
    // `mid`'s own ClosureSite must bubble this capture up so `outer`'s own
    // Op::MakeClosure (for `mid`) snapshots `x` too, even though `mid`
    // itself never reads `x`. Same values/shape as
    // NestedClosureCapturesBothEnclosingLevels above, but this time
    // `mid` escapes `outer` (returned, called from a separate statement)
    // rather than being called back immediately -- so `outer` itself must
    // now compile, and the correctness depends on the bubbled capture
    // actually reaching `mid`'s own snapshot correctly.
    const std::string script = "function outer(x) = let(mid = function(y) let(inner = function(z) x + y + z) "
                                "inner(1)) mid;\n"
                                "stored = outer(100);\n"
                                "echo(stored(10));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 111");
}

TEST(BytecodeCompiler, MultiLevelCurriedClosureInvocationRunsCompiled) {
    ScopedVm vm(true);
    // fnliterals.scad's f_1arg()-style curry adapter: a closure that
    // returns ANOTHER closure nested inside it, both with real captures.
    // This is exactly the two-level-nesting case Op::MakeClosure's own
    // runtime handler (bytecode_vm.cpp) has to get right when a captures-
    // having chunk's OWN body is registered too -- the inner literal's own
    // MakeClosure instruction runs as part of the OUTER closure's later,
    // separate invocation (not inline within f_1arg's own call, the way a
    // single-level closure's capture always does), so its own captures
    // (bubbled up from `f_1arg` at compile time) are no longer local to
    // whatever frame happens to be running -- they resolve via
    // Evaluator::findUpvalue/`ctx.let_`, not `slots` directly. Correctness
    // (not a stop count -- several calls chain here, and not every shape
    // adds its own extra interpreted-only checkpoint, so the total isn't a
    // reliable compiled-vs-interpreted signal for this particular script)
    // is what actually matters: a wrong result here is exactly what a
    // missed/misdirected capture read would produce.
    EXPECT_EQ(runCapturingEcho("function f_1arg(target_func) = function(a) a==undef? function(x) target_func(x) : "
                                "function() target_func(a);\n"
                                "function double(x) = x * 2;\n"
                                "adder = f_1arg(function(x) double(x));\n"
                                "echo(adder(21)());\n"
                                "echo(adder(undef)(10));"),
              "ECHO: 42\nECHO: 20");
}

TEST(BytecodeCompiler, SelfReferentialRecursiveClosureNowResolvesAsUpvalueAndRunsCompiled) {
    ScopedVm vm(true);
    // reduce()'s own idiom: `a` calls itself by name. A direct
    // `let(a = function(...) ... a(...) ...)` RHS gets `a`'s own slot
    // pre-declared BEFORE compiling that RHS (bytecode_compiler.cpp's
    // LetOp case), letrec-style -- so this self-reference now resolves as
    // a genuine upvalue (Op::LoadUpvalue) instead of falling through to
    // Op::LoadFree, and `a`'s own body is eligible for registration
    // (containsLoadFree no longer sees any unresolved reference in it).
    // Op::MakeClosure's own runtime handler defers this ONE capture
    // (the closure being built doesn't exist yet when its own captures
    // are normally snapshotted) and patches it in immediately after
    // constructing the real closure -- see its own doc comment,
    // bytecode_vm.cpp. This is also what fixed a real O(n) -> O(n^2)
    // regression found while building this feature: every recursive call
    // used to re-derive a FRESH closure via Evaluator::evalIdentifier's
    // ctx.scope->lookupVariable() fallback, nesting its own capturedLet
    // one level deeper than the last -- confirmed via the CLI against a
    // 32,000-element list (7+ seconds -> 0.05s once `a` genuinely
    // resolves itself as an upvalue instead).
    //
    // 6 stops (was 11 before Phase 1 module/top-level compilation --
    // confirmed via a line/depth debug-hook dump, not re-derived by hand):
    // the top-level echo() statement's own stop (1, line 2, depth 0) +
    // `reduce`'s own body-entry (1, line 1, depth 1) + `func`'s own body-
    // entry, once per call from within `a`'s 5-iteration chain (4, line 2,
    // depth 2 -- `function(p,q) p+q` is textually written on line 2, as
    // part of echo's own argument expression). `a`'s own 5 self-recursive
    // hops contribute NOTHING of their own: reduce's own tail-position call
    // INTO `a` (`a(init, 0)`, reduce's whole body) and every one of `a`'s
    // own tail-recursive hops are all part of the SAME trampolined chain,
    // so only the very first evalUserFunctionCore invocation (reduce's own)
    // ever fires a body-entry checkDebug -- exactly the same collapse
    // MutualRecursionBetweenSiblingLetBoundClosuresResolvesCorrectly, below,
    // documents for isEven<->isOdd. What actually changed here: echo's own
    // argument -- the WHOLE reduce(...) call -- now compiles as a bare
    // statement-context chunk too (Phase 1, see evalExprMaybeCompiled,
    // evaluator.hpp), so it no longer gets a separate call-site checkpoint
    // of its own either, on top of everything already collapsed above.
    const int stops = countDebugHookStops(
        "function reduce(func, list, init=0) = let(l = len(list), a = function (x,i) i<l? "
        "a(func(x,list[i]), i+1) : x) a(init,0);\n"
        "echo(reduce(function(p,q) p+q, [1,2,3,4], 0));",
        std::unordered_map<std::string, std::set<int>>{});
    EXPECT_EQ(stops, 6);
    EXPECT_EQ(runCapturingEcho("function reduce(func, list, init=0) = let(l = len(list), a = function (x,i) "
                                "i<l? a(func(x,list[i]), i+1) : x) a(init,0);\n"
                                "echo(reduce(function(p,q) p+q, [1,2,3,4], 0));"),
              "ECHO: 10");
}

TEST(BytecodeCompiler, PlainLetSelfReferenceStillSeesTheOuterBindingNotItself) {
    ScopedVm vm(true);
    // The letrec pre-declare above is gated on the RHS being a DIRECT
    // FunctionLiteral specifically because a non-function `let(x = x + 1)`
    // must keep seeing the OUTER x (real OpenSCAD/letOp shadowing
    // semantics -- each assignment's RHS sees only what preceded it in the
    // same let, never itself), not a fresh, not-yet-assigned local
    // shadowing it. Confirms that ordinary case is untouched by the
    // FunctionLiteral-only pre-declare.
    EXPECT_EQ(runCapturingEcho("x = 100;\nfunction f() = let(x = x + 1) x;\necho(f());"), "ECHO: 101");
}

TEST(BytecodeCompiler, MutualRecursionBetweenSiblingLetBoundClosuresResolvesCorrectly) {
    ScopedVm vm(true);
    // fnliterals.scad-style isEven()/isOdd(): TWO sibling closures in the
    // SAME let(), each calling the OTHER by name. `isEven` (compiled
    // FIRST) references `isOdd`, which doesn't exist at all yet at that
    // point -- unlike a self-reference, there's nothing to read OR self-
    // patch into (see Op::PatchClosureCapture's own doc comment,
    // bytecode.hpp): resolved instead by a real patch instruction emitted
    // right after `isOdd`'s own StoreLocal, once it actually exists.
    // 2 stops, not one per logical call (isEvenTest's own entry, plus one
    // per level of isEven(6)->isOdd(5)->isEven(4)->...->isEven(0)):
    // isEven<->isOdd calling each other in tail position is exactly what
    // the tail-call trampoline collapses into ONE evalUserFunctionCore
    // invocation for the WHOLE chain (see runCompiledFunctionTrampoline,
    // bytecode_vm.cpp) -- its own unconditional body-entry checkDebug()
    // fires once for that entire loop, not once per hop. isEvenTest's own
    // entry (1) + that one trampoline loop (1) = 2. NOT a 3rd stop for the
    // top-level echo() statement's own call-site anymore either (Phase 1,
    // module/top-level compilation): `echo(isEvenTest(6))`'s own argument
    // now compiles too (see evalExprMaybeCompiled, evaluator.hpp), so
    // isEvenTest(6)'s call-site checkpoint -- previously fired by plain
    // interpreted evalFunctionCall -- no longer does.
    const int stops = countDebugHookStops(
        "function isEvenTest(n) = let(isEven = function(k) k==0 ? true : isOdd(k-1), isOdd = function(k) "
        "k==0 ? false : isEven(k-1)) isEven(n);\n"
        "echo(isEvenTest(6));",
        std::unordered_map<std::string, std::set<int>>{});
    EXPECT_EQ(stops, 2);
    EXPECT_EQ(runCapturingEcho("function isEvenTest(n) = let(isEven = function(k) k==0 ? true : isOdd(k-1), "
                                "isOdd = function(k) k==0 ? false : isEven(k-1)) isEven(n);\n"
                                "echo(isEvenTest(6));\necho(isEvenTest(7));"),
              "ECHO: true\nECHO: false");
}

TEST(BytecodeCompiler, ThreeWaySiblingMutualRecursionResolvesCorrectly) {
    ScopedVm vm(true);
    // The patch mechanism isn't hardcoded to pairs -- a THIRD sibling
    // (f3, itself forward-referencing f1, the FIRST one compiled) proves
    // pendingWaiters/Op::PatchClosureCapture generalizes to an arbitrary
    // cycle length, not just mutual (2-closure) recursion specifically.
    EXPECT_EQ(runCapturingEcho("function test(n) = let(f1 = function(k) k==0 ? \"f1\" : f2(k-1), f2 = "
                                "function(k) k==0 ? \"f2\" : f3(k-1), f3 = function(k) k==0 ? \"f3\" : "
                                "f1(k-1)) f1(n);\n"
                                "echo(test(9));\necho(test(10));\necho(test(11));"),
              "ECHO: \"f1\"\nECHO: \"f2\"\nECHO: \"f3\"");
}

TEST(BytecodeCompiler, TernaryWrappedSelfReferenceResolvesAsUpvalue) {
    ScopedVm vm(true);
    // The letrec pre-declare's own eligibility check isn't limited to a
    // bare `name = function(...) ...` RHS -- `cond ? function(...) ... :
    // function(...) ...` (EVERY reachable branch is unambiguously a
    // closure, see collectLetrecCandidateLiterals' own doc comment,
    // bytecode_compiler.cpp) also pre-declares, and BOTH branches'
    // FunctionLiteral nodes are independently found and marked afterward
    // (a ternary can produce two structurally distinct closures, not just
    // one). Deep, genuinely TAIL-recursive (`acc` accumulates on the way
    // down, not after the recursive call returns -- see this file's own
    // DeepSelfTailRecursionUnderVmDoesNotOverflowTheNativeStack, above, for
    // why that distinction matters at this depth) to prove it actually
    // runs compiled, not just correctly-but-slowly interpreted.
    const std::string script =
        "function make(n) = let(a = n > 0 ? function(k, acc) k<=0 ? acc : a(k-1, acc+n) : function(k, acc) acc) "
        "a(200000, 0);\necho(make(1));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 200000");
}

TEST(BytecodeCompiler, TernaryWrappedMutualRecursionResolvesCorrectly) {
    ScopedVm vm(true);
    // Same ternary-wrapped-RHS eligibility as the self-reference case
    // above, but for TWO siblings that reference each other -- both
    // Op::PatchClosureCapture's forward-reference deferral and the
    // ternary's own multi-candidate handling apply simultaneously.
    const std::string script =
        "function test(n) = let(isEven = n >= 0 ? function(k) k==0 ? true : isOdd(k-1) : function(k) undef, "
        "isOdd = n >= 0 ? function(k) k==0 ? false : isEven(k-1) : function(k) undef) isEven(n);\n"
        "echo(test(10));\necho(test(7));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: true\nECHO: false");
}

TEST(BytecodeCompiler, ClosureWithDollarParameterNowCompilesToo) {
    ScopedVm vm(true);
    // A FunctionLiteral with its OWN dollar-prefixed parameter used to fail
    // compileFunctionLike outright, bailing the WHOLE containing
    // declaration (see DollarPrefixedParameterCompiles above for the plain-
    // function case) -- now compileFunctionLike has no $-specific bail left
    // at all, so this compiles too, container and closure both.
    const std::string script = "function outer(x) = let(g = function(y, $fn) y + x + $fn) g(5, $fn=2);\n"
                                "echo(outer(10));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 17");
    // Proof via the same fast-continue stop-count technique used
    // throughout this file (nullopt is NOT a reliable signal here -- it
    // forces every function to interpret regardless of eligibility, see
    // DebugAttachedWithoutFastContinueAlwaysInterprets above). The
    // breakpoint sits on line 2 -- echo's own statement line -- so it
    // deliberately forces THAT one call (echo's `outer(10)` argument) to
    // interpret, while outer's and g's own bodies (line 1) stay eligible
    // and run compiled. 3 stops: the echo statement's own per-statement
    // checkpoint (line 2), the interpreted call-site checkpoint for
    // calling outer (line 2), and outer's own body-entry (line 1) -- g's
    // own tail call, reached from INSIDE outer's now-compiled body, hops
    // in place silently (no separate stop), proving the tail hop stays
    // silent even for a closure capturing an enclosing binding with its
    // own $-parameter.
    const int stops = countDebugHookStops(script, std::unordered_map<std::string, std::set<int>>{{"<string>", {2}}});
    EXPECT_EQ(stops, 2);
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
    // `g` (a closure over `make`'s own `x`) is called in tail position from
    // `make`'s own compiled body -- but `g`'s own body is never itself
    // compiled post-Phase-2b (Op::MakeClosure's whole point is to let
    // `make` compile despite creating `g`; `g` still always runs
    // interpreted when invoked, see Op::MakeClosure's own doc comment,
    // bytecode.hpp). CallDynamicTail's handler therefore finds no compiled
    // chunk for `g` (Evaluator::lookupCompiledLiteralChunk returns nullptr)
    // and falls back to a real evalFunctionLiteralFromBound call instead of
    // trampolining, exactly like the interpreter path's own
    // ClosureNestedTailCallFallsBackToRealRecursionAndStillResolves
    // (test_tail_calls.cpp).
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

// -- Compiling echo()/assert()/import()/object() ---------------------------
//
// These four previously bailed compilation of their whole containing
// function (fell through to `default: throw NotCompilable{}` for echo/
// assert, or an explicit bail for import/object) -- see bytecode_compiler.cpp's
// PrimaryCall/EchoOp/AssertOp cases and bytecode_vm.cpp's Op::Echo/
// Op::AssertFail handlers and CallFn's isImport/object-name dispatch.

TEST(BytecodeCompiler, EchoExpressionCompiles) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f(x) = echo(\"got\", x) x + 1;\necho(f(5));"), "ECHO: \"got\", 5\nECHO: 6");
    // Named argument formatting ("name = value") and chained echo(...)
    // echo(...) body (each inherits `tail` from the enclosing one, no jump
    // between them -- straight-line fallthrough).
    EXPECT_EQ(runCapturingEcho("function g(x) = echo(x, label=\"y\") echo(\"again\") x * 2;\necho(g(3));"),
              "ECHO: 3, label = \"y\"\nECHO: \"again\"\nECHO: 6");
}

TEST(BytecodeCompiler, AssertExpressionCompiles) {
    ScopedVm vm(true);
    // Passing condition: falls through to body untouched.
    EXPECT_EQ(runCapturingEcho("function f(x) = assert(x > 0) x;\necho(f(5));"), "ECHO: 5");
    // Zero-argument assert() is unconditionally true (compiled with no
    // check at all -- see the compiler's own AssertOp case).
    EXPECT_EQ(runCapturingEcho("function h() = assert() 42;\necho(h());"), "ECHO: 42");
}

TEST(BytecodeCompiler, AssertExpressionFailureThrowsWithCorrectMessage) {
    ScopedVm vm(true);
    Evaluator ev;
    auto ast = parseSrc("function f(x) = assert(x > 0) x;\nresult = f(-1);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    try {
        ev.resolveTree(ast, ctx);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        // condText baked in at compile time via the condition's own
        // toString() -- must match the source text exactly.
        EXPECT_NE(std::string(e.what()).find("Assertion 'x > 0' failed"), std::string::npos);
    }
}

TEST(BytecodeCompiler, AssertExpressionFailureWithMessageThrowsWithMessage) {
    ScopedVm vm(true);
    Evaluator ev;
    auto ast = parseSrc("function f(x) = assert(x > 0, \"must be positive\") x;\nresult = f(-1);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    try {
        ev.resolveTree(ast, ctx);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("Assertion 'x > 0' failed"), std::string::npos);
        EXPECT_NE(msg.find("must be positive"), std::string::npos);
    }
}

TEST(BytecodeCompiler, AssertMessageArgumentReferencesEarlierLetBinding) {
    // The message argument is lazily compiled (only reachable on the
    // condition-false path, behind a JumpIfTrue) -- its own slot
    // references must still resolve correctly, since CompileScope is a
    // purely compile-time name/index table, unaffected by which runtime
    // path actually reaches this code.
    ScopedVm vm(true);
    EXPECT_EQ(
        runCapturingEcho(
            "function f(x) = let(lo = 0) assert(x > lo, str(\"must exceed \", lo)) x;\necho(f(5));"),
        "ECHO: 5");
    Evaluator ev;
    auto ast = parseSrc("function f(x) = let(lo = 0) assert(x > lo, str(\"must exceed \", lo)) x;\nresult = f(-1);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    try {
        ev.resolveTree(ast, ctx);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        EXPECT_NE(std::string(e.what()).find("must exceed 0"), std::string::npos);
    }
}

TEST(BytecodeCompiler, ImportCompilesAsTailPositionBody) {
    // The specific case that crashes without the CallFnTail exclusion fix
    // (bytecode_compiler.cpp's tail-opcode-selection condition): import()
    // as a function's ENTIRE body is a tail-position PrimaryCall resolving
    // to isImport=true, isBuiltin=false -- without `!site.isImport` in that
    // condition this would emit CallFnTail, whose handler unconditionally
    // dereferences site.decl (null for an import site).
    ScopedVm vm(true);
    const auto path = tempPath("data.json");
    {
        std::ofstream out(path);
        out << R"({"a": 1, "b": 2})";
    }
    EXPECT_EQ(runCapturingEcho("function f() = import(\"" + path.generic_string() + "\");\necho(f());"),
              "ECHO: { a = 1; b = 2; }");
    std::filesystem::remove(path);
}

TEST(BytecodeCompiler, ObjectCompilesWithExactCallSiteInterleavedOrder) {
    ScopedVm vm(true);
    // a=1 (named) -> b spread from an existing object's own entries ->
    // c=3 (named) -> a=4 (named, overrides the earlier a=1) -- later
    // writes for the same key must win, in exact call-site order, matching
    // builtinObject's own documented semantics.
    EXPECT_EQ(runCapturingEcho("existing = object(b = 2);\n"
                                "function f() = object(a = 1, existing, c = 3, a = 4);\n"
                                "echo(f());"),
              "ECHO: { a = 4; b = 2; c = 3; }");
    // Positional list-of-pairs spread.
    EXPECT_EQ(runCapturingEcho("function g() = object(a = 1, [[\"b\", 2]], a = 3);\necho(g());"),
              "ECHO: { a = 3; b = 2; }");
}

TEST(BytecodeCompiler, VmOffAndVmOnAgreeOnEchoAssertImportObjectCases) {
    const auto path = tempPath("combined_data.json");
    {
        std::ofstream out(path);
        out << R"({"k": 9})";
    }
    const std::string script = "function withEcho(x) = echo(\"trace\", x) x + 1;\n"
                                "function withAssert(x) = assert(x >= 0, \"must be non-negative\") x * 2;\n"
                                "function withImport() = import(\"" +
                                path.generic_string() +
                                "\");\n"
                                "existing = object(m = 1);\n"
                                "function withObject() = object(n = 2, existing, m = 3);\n"
                                "function combined(x) = echo(\"start\") assert(x > 0) let(y = withEcho(x)) "
                                "withAssert(y) + withObject().n;\n"
                                "echo(combined(5));\n"
                                "echo(withImport());";
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
    std::filesystem::remove(path);
}

TEST(BytecodeCompiler, RangeBasedForListCompDoesNotReMaterializeRangeSizePerIteration) {
    ScopedVm vm(true);
    // Op::IterNext used to bound-check via `il.index < il.values.size()`
    // every single iteration. IterableValues::size() is O(1) for a
    // materialized list, but for a RANGE it walks the whole sequence from
    // scratch (its own doc comment already flagged this as a stale
    // assumption: "no caller in this codebase actually calls size() on a
    // range today" -- Op::IterNext was exactly that caller). Recomputing an
    // O(n) size every one of n iterations is O(n^2) overall -- a
    // 100,000-element range list comprehension took ~9.5s compiled vs
    // ~0.03s interpreted before the fix (caught via a BOSL2 corpus sweep,
    // test_math.scadtest's gaussian_rands()). Fixed by caching the size
    // once at IterMaterialize time (IterList::total). 200,000 elements
    // finishes in well under a second now; the old behavior would have
    // taken tens of seconds, so a generous 5s bound cleanly separates
    // "fixed" from "regressed" without making this test itself slow.
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(runCapturingEcho("function myrange(n) = [for (i=[0:1:n-1]) i];\n"
                                "r = myrange(200000);\n"
                                "echo(len(r));"),
              "ECHO: 200000");
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 5000);
}

// -- Debug "fast continue" per-function VM gating --------------------------
//
// A debugger attached AND actively single-stepping needs every checkDebug()
// checkpoint the interpreter provides (a compiled function has none for a
// step to land on) -- that's the pre-existing, always-safe, never-changed
// behavior when Evaluator::setFastContinueBreakpoints is never called at
// all (fastContinueBreakpoints_ stays nullopt). But a plain "Continue,
// pause only at a known breakpoint line" doesn't need that: a function
// whose own compiled span contains none of the currently-set breakpoint
// lines can safely run on the VM, since nothing inside it could possibly
// need a checkpoint right now.

// Expected stop counts below are empirically verified (via a temporary
// direct instrumentation pass through lookupOrCompileChunk/evalUserFunction,
// not guessed): calling `f(5)` for `function f(x) = x > 0 ? x + 1 : x - 1;`
// as the lone argument of a top-level `echo(...)` produces 3 checkDebug
// stops when f runs compiled (the top-level echo statement's own stop,
// f's call-site stop from evalFunctionCall, and evalUserFunctionCore's
// unconditional body-entry stop -- no others, since compiled bytecode has
// no sub-expression checkpoints) versus 5 when f runs interpreted (those
// same 3, plus the ternary's own condition-check and chosen-branch-entry
// sub-expression stops). A FunctionDeclaration statement itself never adds
// its own top-level stop (evalChildren's per-statement checkpoint skips
// declarations, mirroring $children's own count -- see
// Evaluator::evalUserModule's childrenCount loop), so adding a second,
// uncalled function declaration to a script doesn't change either count.

TEST(BytecodeCompiler, DebugAttachedWithoutFastContinueAlwaysInterprets) {
    ScopedVm vm(true);
    // No setFastContinueBreakpoints call at all (nullopt, the default):
    // must behave exactly as before this feature existed -- always
    // interpreted -- regardless of the function's own line having no
    // breakpoint anywhere near it.
    const int stops = countDebugHookStops("function f(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "echo(f(5));",
                                           std::nullopt);
    EXPECT_EQ(stops, 4);
}

TEST(BytecodeCompiler, FastContinueWithNoBreakpointInFunctionUsesVm) {
    ScopedVm vm(true);
    // Breakpoint set, but only on line 2 (the echo statement) -- f's own
    // body is entirely on line 1, outside that set, so it's eligible to
    // run compiled.
    const int stops = countDebugHookStops("function f(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "echo(f(5));",
                                           std::unordered_map<std::string, std::set<int>>{{"<string>", {2}}});
    EXPECT_EQ(stops, 2);
}

TEST(BytecodeCompiler, FastContinueWithBreakpointInsideFunctionStillInterprets) {
    ScopedVm vm(true);
    // Breakpoint on line 1 itself -- inside f's own compiled span -- must
    // still force the interpreter for f specifically, even in fast-continue
    // mode: something on that exact line could need a real checkpoint. 4,
    // not 5 (Phase 1, module/top-level compilation): the breakpoint is on
    // line 1 (f's own body), not line 2, so it doesn't affect echo(f(5))'s
    // OWN argument expression -- a SEPARATE chunk with its own SEPARATE
    // [minLine,maxLine] span (line 2 only) -- which is still eligible to
    // compile and so loses its own call-site checkpoint, exactly like
    // every other statement-context expression now does when eligible.
    // f's OWN body still correctly interprets either way (the actual
    // guarantee this test exists to prove), just via one fewer surrounding
    // stop than before.
    const int stops = countDebugHookStops("function f(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "echo(f(5));",
                                           std::unordered_map<std::string, std::set<int>>{{"<string>", {1}}});
    EXPECT_EQ(stops, 4);
}

TEST(BytecodeCompiler, FastContinueWithBreakpointInDifferentFunctionUsesVmForThisOne) {
    ScopedVm vm(true);
    // Two ternary-bodied functions (so both would show the same VM-vs-
    // interpreter stop-count gap if called), breakpoint only inside g's
    // body (line 2) -- f (line 1) must still run compiled since its own
    // span doesn't contain line 2, even though SOME breakpoint exists in
    // this same file. Confirms the gating is genuinely per-function, not
    // "any breakpoint anywhere disables the whole file." Only f is ever
    // called, so g's own eligibility is never directly observed here.
    // 2, not 3 (Phase 1, module/top-level compilation): echo(f(5))'s own
    // argument (line 3, outside the breakpoint's line-2 span) is a
    // separate chunk from f's own body, independently eligible to compile
    // -- losing its own call-site checkpoint like every other eligible
    // statement-context expression now does.
    const int stops = countDebugHookStops("function f(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "function g(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "echo(f(5));",
                                           std::unordered_map<std::string, std::set<int>>{{"<string>", {2}}});
    EXPECT_EQ(stops, 2);
}

TEST(BytecodeCompiler, CStyleForIncrNameCanReadItsOwnPriorValueInTheSameAssignment) {
    ScopedVm vm(true);
    // A C-style for's incr list can introduce a name that reads its OWN
    // not-yet-assigned-this-iteration value on the right-hand side (its
    // prior iteration's value) -- e.g. BOSL2's skin.scad: `best_i =
    // result[0]<bestcost ? i : best_i,`. The compiler used to compile each
    // incr assignment's RHS BEFORE declaring that assignment's own local
    // slot (declareLocal ran only after compileExpr), so a self-reference
    // like this -- and the loop BODY's own read, compiled even earlier --
    // resolved via Op::LoadFree (the "not a known local, might be dynamic"
    // fallback) instead of Op::LoadLocal. That's not just an extra
    // "unknown variable" warning: LoadFree runs identically on every
    // iteration once compiled, so it NEVER saw what Op::StoreLocal had
    // written moments earlier at runtime -- a genuinely wrong final
    // result (undef), not merely a cosmetic warning-count mismatch. Real
    // OpenSCAD gives [4] here (one "unknown variable" warning for the
    // first, genuinely-unassigned read, then the accumulator works
    // correctly from there). Fixed by pre-declaring every incr name's slot
    // before compiling the condition/body/incr expressions at all.
    EXPECT_EQ(runCapturingEcho("function myloop(n) =\n"
                                "    [for (i=0, bestcost=1/0;\n"
                                "          i<=n;\n"
                                "          bestcost = (i==2||i==4) ? i : bestcost,\n"
                                "          best_i = (i==2||i==4) ? i : best_i,\n"
                                "          i=i+1)\n"
                                "          if (i==n) best_i];\n"
                                "echo(myloop(5));"),
              "ECHO: [4]");
}

// -- Module-body compilation (Stage 2) -------------------------------------
//
// Mirrors the function-side Phase B section above: module bodies now
// compile too (Op::CallModule/NativeStatement/NativeCondJumpIfFalse/
// NativeIterMaterialize/ForIterNext/ForIterEnd -- see bytecode.hpp and
// tryCompileModuleBody, bytecode_compiler.cpp), specifically so a
// recursive module call no longer makes an unbounded native C++ recursive
// call the way evalUserModule always used to (no trampoline exists for
// modules, unlike functions' own tail-call machinery). assignment/echo/
// assert/let-blocks/modifiers/intersection_for still delegate to a native
// evalStatement call for one node at a time -- only if/for/a resolved
// user-module call get real bytecode, since those are the shapes that can
// hide a recursive call behind a native call boundary.

TEST(ModuleBodyCompiles, ForLoopProducesCorrectGeometryCompiled) {
    // Each iteration's translate()/cube() stays its own separate top-level
    // body (four disjoint cubes, not boolean-unioned into one manifold --
    // confirmed against the interpreted path directly: byte-identical STL
    // output either way for this exact script). Asserting bodies.size()
    // and the outermost bounding box across all of them is what's actually
    // load-bearing here, not a specific body count guess.
    ScopedVm vm(true);
    Evaluated e = evalSrc("module row(n) { for (i = [0:n-1]) translate([i * 2, 0, 0]) cube(1); }\nrow(4);");
    ASSERT_EQ(e.bodies.size(), 4u);
    double maxX = 0.0;
    for (const auto& body : e.bodies) maxX = std::max(maxX, body.body->BoundingBox().max.x);
    EXPECT_NEAR(maxX, 7.0, 1e-9); // last cube at i=3 -> translate([6,0,0]), cube(1) -> max.x = 7
}

TEST(ModuleBodyCompiles, IfElseInModuleBodyPicksCorrectBranchCompiled) {
    ScopedVm vm(true);
    Evaluated eTrue = evalSrc("module pick(n) { if (n > 0) cube(5); else sphere(1); }\npick(1);");
    ASSERT_EQ(eTrue.bodies.size(), 1u);
    EXPECT_NEAR(eTrue.bodies[0].body->BoundingBox().max.x, 5.0, 1e-9);

    Evaluated eFalse = evalSrc("module pick(n) { if (n > 0) cube(5); else sphere(1); }\npick(-1);");
    ASSERT_EQ(eFalse.bodies.size(), 1u);
    EXPECT_NEAR(eFalse.bodies[0].body->BoundingBox().max.x, 1.0, 1e-9);
}

TEST(ModuleBodyCompiles, ReassignmentInsideForLoopBodyDoesNotSpuriouslyWarn) {
    ScopedVm vm(true);
    // Op::ForIterNext pushes a FRESH child ctx per iteration (see its own
    // doc comment, bytecode.hpp) specifically so an ordinary local
    // assignment inside the loop body -- reusing the same NAME every
    // iteration, a common and unremarkable pattern -- doesn't trip
    // evalAssignment's own "was assigned on line N but overwritten"
    // warning from iteration 2 onward. Reusing the SAME ctx across
    // iterations (the wrong, simpler design) would warn 9 times here.
    std::vector<std::string> warnings;
    Evaluator ev([&](const std::string& msg) { warnings.push_back(msg); });
    auto ast = parseSrc("module row(n) { for (i = [0:n-1]) { r = i * 2; translate([r, 0, 0]) cube(1); } }\nrow(10);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx, {});
    for (const std::string& w : warnings) EXPECT_EQ(w.find("overwritten"), std::string::npos) << w;
}

// Regression test for a real cache-collision bug: childrenListChunkCache_
// was keyed by the list's FIRST element alone (a convention borrowed from
// assignBlockChunkCache_, where it IS sufficient) -- but children()'s
// forwarding produces two DIFFERENT lists sharing a first element: bare
// `children()` forwards the caller's whole list, `children(0)` a single-
// element slice of it. Whichever form ran first poisoned the cache for
// the other: here, bare children() cached a two-statement chunk keyed by
// &cubeStmt, then children(0)'s single-element {&cubeStmt} lookup HIT
// that key and emitted BOTH shapes. Fixed by keying on (front, size).
TEST(ModuleBodyCompiles, BareAndIndexedChildrenForwardingDoNotShareACachedChunk) {
    ScopedVm vm(true);
    // m() emits: children() -> cube+sphere, then children(0) -> cube only.
    // 3 bodies total; the collision bug produced 4 (children(0) emitting
    // sphere too).
    Evaluated e = evalSrc("module m() { children(); children(0); }\n"
                          "m() { cube(1); sphere(r=1, $fn=8); }");
    EXPECT_EQ(e.bodies.size(), 3u);
}

// Same collision, opposite statement order -- children(0) caching its
// single-element chunk first used to TRUNCATE the later bare children()
// (2 bodies where 3 belong).
TEST(ModuleBodyCompiles, IndexedThenBareChildrenForwardingDoNotShareACachedChunk) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module m() { children(0); children(); }\n"
                          "m() { cube(1); sphere(r=1, $fn=8); }");
    EXPECT_EQ(e.bodies.size(), 3u);
}

TEST(ModuleBodyCompiles, RecursiveModuleWithIfSucceedsWellPastTheOldNativeLimitCompiled) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\nrecur(10000);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// The "NativeStatement gap" this used to prove (Evaluator::
// tryRunCompiledChildren): a builtin-with-children statement wrapping a
// RECURSIVE call used to fall to Op::NativeStatement every single level --
// a genuine, repeated native C++ re-entry into the VM (evalChildren ->
// tryRunCompiledChildren -> runCompiledModuleBody -> driveVm), unlike a
// bare `recur(n-1);` statement, which compiles straight to Op::CallModule
// and costs zero native stack. Before driveVmNativeDepth_/
// kMaxDriveVmNativeDepth existed, this pattern segfaulted (exit 139)
// around n=3000. Op::PushBuiltinWrap (bytecode_compiler.cpp/bytecode_vm.cpp)
// closes that gap for translate/rotate/scale/mirror/multmatrix/resize/
// color/#/%/! specifically -- this construct now compiles to real bytecode
// that runs entirely on the heap-based vmCallStack_, so it SUCCEEDS instead
// of hitting the native-reentry guard at all. Depth 1500 (not the old
// 3000): comfortably past the old ~40-55 native-reentry ceiling, but under
// kMaxCsgTreeDepth's own 2000 -- translate() genuinely nests real tree
// structure (unlike a bare module call, which splices flat), so 3000 would
// now hit THAT unrelated guard instead, which isn't what this test is
// about; see CsgTree's own depth-guard tests (test_csg_tree.cpp) for that
// one. See UnionWrappedRecursionStillHitsTheNativeReentryGuardControlled-
// Error, below, for proof the guard itself still works for a construct
// this fix deliberately doesn't cover.
TEST(ModuleBodyCompiles, RecursiveTranslateWrappedCallSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { translate([0,0,n]) recur2(n); }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// Same shape, multmatrix() instead of translate() -- both share
// resolveTransform/computeTransformParams, but detected/dispatched
// separately at compile time (a different registry.cpp entry), so this is
// a real, independent proof it's not just translate() that got covered.
TEST(ModuleBodyCompiles, RecursiveMultmatrixWrappedCallSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e =
        evalSrc("module recur(n) { multmatrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) recur2(n); }\n"
                "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// Same shape, color() -- the Color-kind branch (computeColorParams, a
// separate ctxChain-pushing path from Transform's own), not just Transform.
TEST(ModuleBodyCompiles, RecursiveColorWrappedCallSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { color(\"red\") recur2(n); }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// Same shape, `#` (highlight) -- the Modifier-kind branch (no arg
// resolution, no ctxChain push at all), and the ONLY one of the 3 kinds
// reached via a NodeKind case rather than resolveDispatch() lookup.
TEST(ModuleBodyCompiles, RecursiveModifierWrappedCallSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { #recur2(n); }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// union()/difference()/intersection() now have their own real bytecode
// (Op::PushCsgWrap/CsgGroupStart/CsgGroupEnd/PopCsgWrap) instead of falling
// to Op::NativeStatement -- see that op's own doc comment (bytecode.hpp)
// for why they needed a bespoke bracket rather than just reusing
// Op::PushBuiltinWrap's. A union()/difference()/intersection()-wrapped
// recursive call no longer costs any native reentry at all: `recur2(n);`
// (a resolved user-module call) compiles inline to Op::CallModule, a pure
// in-VM push, so the whole chain never touches driveVmNativeDepth_.
// Depth 1500 (same figure the PushBuiltinWrap/CallChildren tests above use)
// comfortably clears the old 40-level Windows-unsafe native-reentry ceiling
// while staying under kMaxCsgTreeDepth=2000 -- each `recur` level's own
// union() still contributes one real CSGNode to the tree (this fix
// eliminates native-stack cost, not CSG-tree depth, which is an orthogonal,
// unrelated cap on the RESULT shape, not the call chain).
TEST(ModuleBodyCompiles, UnionWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { union() { recur2(n); } }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// The SAME chain pushed deep enough (3000 levels, one union() CSGNode per
// `recur` level) now hits the orthogonal, pre-existing CSG-TREE-depth guard
// (kMaxCsgTreeDepth=2000, csg_resolve.cpp) instead of a native-reentry
// guard -- proves the native-reentry elimination didn't silently trade one
// crash risk for an unguarded one; something still stops an unreasonably
// deep result, just a different (and correctly-named) error now.
TEST(ModuleBodyCompiles, UnionWrappedRecursionPastCsgTreeDepthLimitStillErrorsCleanly) {
    ScopedVm vm(true);
    try {
        evalSrc("module recur(n) { union() { recur2(n); } }\n"
                "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                "recur(3000);");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("Recursion too deep while building geometry"), std::string::npos) << what;
    }
}

// difference()/intersection() get the same treatment -- one representative
// test each rather than the full union() coverage above, since all three
// share emitCsgWrap/Op::PushCsgWrap verbatim (only the runtime `op` string
// differs, consumed solely by generateCsg at generate time, never by the
// compiled path itself).
TEST(ModuleBodyCompiles, DifferenceWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { difference() { recur2(n); } }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

TEST(ModuleBodyCompiles, IntersectionWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { intersection() { recur2(n); } }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// group_sizes/multi-body-operand grouping correctness under the COMPILED
// path (test_booleans.cpp's own Difference.MultipleBodiesInFirstStatement
// FormOnePositiveOperand/Union.MultipleBodiesInFirstStatementAllSurvive
// already cover this for the default ambient VM state; this pins it
// explicitly under ScopedVm(true) plus a MODULE body, not just a top-level
// script, since that's the shape emitCsgWrap actually compiles).
TEST(ModuleBodyCompiles, CsgWrapGroupingSurvivesInsideACompiledModuleBody) {
    ScopedVm vm(true);
    // First statement's own union() contributes TWO disjoint bodies (one
    // group, size 2); the second statement subtracts a cube overlapping
    // only the first of those two by 1. A flat (non-grouped) evaluation
    // would instead treat the second cube as its own separate operand.
    Evaluated e = evalSrc("module m() {"
                          "  difference() {"
                          "    union() { cube(2); translate([10,0,0]) cube(2); }"
                          "    translate([1,0,0]) cube(2);"
                          "  }"
                          "}"
                          "m();");
    ASSERT_EQ(e.bodies.size(), 1u);
    // 16 - overlap(1*2*2=4) = 12, same expected volume as the top-level test.
    EXPECT_NEAR(e.bodies[0].body->Volume(), 12.0, 1e-6);
}

// $-named-argument propagation into a compiled CSG wrap's children --
// union/difference/intersection take no positional args in real OpenSCAD,
// but `difference($fn=8) {...}` is legal and must still reach its children,
// exactly like resolveCsg's own resolveCallArgs call preserves natively
// (booleans.cpp). Verifies Op::PushCsgWrap's ctx push (unconditional,
// unlike PushBuiltinWrap's Transform/Color-only push) actually carries it.
TEST(ModuleBodyCompiles, DollarArgPropagatesIntoCompiledCsgWrapChildren) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("module m() { union($fn = 9) { echo($fn); cube(1); } }\n"
                               "m();"),
              "ECHO: 9");
}

// Assignment-before-geometry ordering: resolveCsg evaluates EVERY
// assignment child first, regardless of source interleaving with geometry
// statements, THEN each geometry statement in its own group -- emitCsgWrap
// mirrors this with a separate compile pass, not source order. A geometry
// statement referencing a LATER-in-source assignment must still see it.
TEST(ModuleBodyCompiles, CompiledCsgWrapEvaluatesAllAssignmentsBeforeAnyGeometryStatement) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module m() {"
                          "  union() {"
                          "    translate([sz, 0, 0]) cube(1);" // geometry statement first in SOURCE order
                          "    sz = 5;"                        // assignment second in source order
                          "  }"
                          "}"
                          "m();");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0, 1e-6);
}

// -- hull()/minkowski()/render()/linear_extrude()/rotate_extrude()/ --------
// -- projection()/offset()/roof() -- closing the last real native-reentry --
// -- gap, via 6 new BuiltinWrapSite::Kind values sharing Op::PushBuiltinWrap
// -- itself rather than a new bracket (see that op's own doc comment). ------
// Every test here mirrors UnionWrappedRecursionSucceedsWellPastTheOldNative
// ReentryLimit's own shape/depth (1500 -- comfortably past the old 40-level
// native ceiling, comfortably under kMaxCsgTreeDepth=2000). Dimension-
// mismatched constructs (linear_extrude/rotate_extrude/projection/offset/
// roof all expect 2D children, but nesting one inside itself recursively
// means every non-leaf level's own "child" is really the 3D/2D RESULT of
// the next level down, not raw 2D input) intentionally don't assert a
// specific body count/volume -- recursively self-nesting one of these
// isn't an idiomatic real pattern the way union()/hull()/translate() are;
// the only thing worth proving here is that deep recursion no longer
// throws "Recursion too deep (native call stack)".

TEST(ModuleBodyCompiles, HullWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { hull() { recur2(n); } }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

TEST(ModuleBodyCompiles, MinkowskiWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { minkowski() { cube(0.1); recur2(n); } }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

TEST(ModuleBodyCompiles, RenderWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { render() { recur2(n); } }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

TEST(ModuleBodyCompiles, LinearExtrudeWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    evalSrc("module recur(n) { linear_extrude(1) { recur2(n); } }\n"
            "module recur2(n) { if (n > 0) { recur(n - 1); } else { square(1); } }\n"
            "recur(1500);");
}

TEST(ModuleBodyCompiles, RotateExtrudeWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    evalSrc("module recur(n) { rotate_extrude() { recur2(n); } }\n"
            "module recur2(n) { if (n > 0) { recur(n - 1); } else { translate([1,0]) square(1); } }\n"
            "recur(1500);");
}

TEST(ModuleBodyCompiles, ProjectionWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    evalSrc("module recur(n) { projection() { recur2(n); } }\n"
            "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
            "recur(1500);");
}

// offset() is 2D->2D -- unlike the dimension-changing group above, nesting
// it inside itself really does feed each level's REAL 2D output into the
// next level's real offsetting work at generate time, unlike linear_
// extrude/rotate_extrude/projection/roof (whose intermediate levels see a
// dimension-mismatched, effectively-empty child). offset(r=1) with a
// nonzero radius compounds: each level's rounded-join offsetting grows
// both the shape's size AND its vertex count (more arc segments per
// corner, applied to an already-larger corner count from the level below)
// -- genuinely exponential blowup by depth 1500, confirmed by hanging for
// minutes before this test was rewritten. delta=0 (Square/Miter join,
// never the Round path that adds arc segments) is a real geometric no-op
// at every level -- exercises the exact same compiled opcode path with
// none of the compounding cost.
TEST(ModuleBodyCompiles, OffsetWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    evalSrc("module recur(n) { offset(delta=0) { recur2(n); } }\n"
            "module recur2(n) { if (n > 0) { recur(n - 1); } else { square(1); } }\n"
            "recur(1500);");
}

TEST(ModuleBodyCompiles, RoofWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    evalSrc("module recur(n) { roof() { recur2(n); } }\n"
            "module recur2(n) { if (n > 0) { recur(n - 1); } else { square(4, center=true); } }\n"
            "recur(1500);");
}

// $-named-argument propagation into a compiled Passthrough-kind wrap's
// children -- mirrors DollarArgPropagatesIntoCompiledCsgWrapChildren, above,
// for the OTHER kind (Passthrough/LinearExtrude/etc.) that also pushes a
// possibly-$-scoped ctx unconditionally.
TEST(ModuleBodyCompiles, DollarArgPropagatesIntoCompiledHullWrapChildren) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("module m() { hull($fn = 9) { echo($fn); cube(1); } }\n"
                               "m();"),
              "ECHO: 9");
}

// The ONE behavioral subtlety Op::PushBuiltinWrap's Roof kind introduces:
// computeRoofParams's own "Unknown roof method" warning is computed at POP
// time (after children), deliberately, so it stays ordered AFTER any
// echo()/warn() a child produces -- exactly matching native resolveRoof's
// own evalChildren-then-compute-params order. Pins that ordering under the
// compiled path specifically (every other kind in this group computes its
// params at PUSH time instead, since none of them has an order-sensitive
// side effect to preserve).
TEST(ModuleBodyCompiles, CompiledRoofUnknownMethodWarningStaysOrderedAfterChildrensOwnEcho) {
    ScopedVm vm(true);
    const std::string out = runCapturingEcho(
        "module m() { roof(method=\"bogus\") { echo(\"child\"); square(4, center=true); } }\n"
        "m();");
    // Asserted as an ordering rather than one exact string: the warning also
    // carries call-site attribution (", from ... line N" plus TRACE lines,
    // see formatWarning), which is orthogonal to what this test pins.
    const size_t echoAt = out.find("ECHO: \"child\"");
    const size_t warnAt = out.find("WARNING: Unknown roof method 'bogus'. Using 'voronoi'.");
    ASSERT_NE(echoAt, std::string::npos) << out;
    ASSERT_NE(warnAt, std::string::npos) << out;
    EXPECT_LT(echoAt, warnAt) << out;
}

// -- intersection_for -- the TRUE last native-reentry gap, closed by -------
// -- reusing Op::PushCsgWrap around a compiled cartesian-product loop. -----
// DebugHooksParity.IntersectionForMarksBodyEntryOnlyNotEachBinding
// (test_debug_hooks.cpp) and every existing IntersectionFor.* test
// (test_control_flow.cpp -- including the multi-body-per-iteration
// grouping case, combineBodies' own union branch) already exercise this
// new compiled path under the ambient VM-on default and pass unchanged;
// these two are the NEW behavior this fix specifically adds.

// A recursive call wrapped in intersection_for() used to fall to
// Op::NativeStatement like every other uncovered construct -- one real
// native reentry per level, capped at the old kMaxDriveVmNativeDepth=40.
// `i=[0:0]` (a single-element range) keeps this to one group per level,
// same shape/depth as the union/difference/intersection depth tests above
// (comfortably under kMaxCsgTreeDepth=2000).
TEST(ModuleBodyCompiles, IntersectionForWrappedRecursionSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) { intersection_for (i = [0:0]) { recur2(n); } }\n"
                          "module recur2(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// Nested (2-dimensional) cartesian product, compiled -- proves
// compileIntersectionForLoop's reuse of compileForLoop's own multi-
// dimension IterReset/ForIterNext/ForIterEnd scaffold is correct for MORE
// than one dimension, not just the single-assignment shape every other
// existing intersection_for test uses. i in {0,1}, j in {0,1} -> 4 total
// iterations, each contributing exactly one echo() call.
TEST(ModuleBodyCompiles, CompiledIntersectionForHandlesTwoDimensionalCartesianProduct) {
    ScopedVm vm(true);
    int echoCount = 0;
    Evaluator ev([&](const std::string&) { ++echoCount; });
    auto ast = parseSrc("intersection_for (i = [0,1], j = [0,1]) { echo(i, j); cube(1); }");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    EXPECT_EQ(echoCount, 4);
}

// The BOSL2 attachable() shape this whole effort targets: a wrapper module
// whose body is just `children();`, applied at every level of a recursive
// chain. children() used to fall to Op::NativeStatement -- one genuine
// native C++ reentry (evalStatement -> evalModularCall -> builtinChildren
// -> evalChildren -> runCompiledModuleBody -> a fresh nested driveVm) per
// level, measured as 85 of 93 native-reentry hits in a real BOSL2 script.
// Op::CallChildren resolves the forwarded list at runtime and pushes its
// chunk onto vmCallStack_ directly (zero native frames), so this now
// SUCCEEDS well past the old ~40-level Windows-safe ceiling. Depth 1500:
// same figure the PushBuiltinWrap per-construct tests use -- comfortably
// past the old ceiling, and tree depth stays flat regardless (children()
// and user-module calls both splice, no kMaxCsgTreeDepth interaction).
TEST(ModuleBodyCompiles, RecursiveChildrenForwardingChainSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module wrap() { children(); }\n"
                          "module recur(n) { if (n > 0) { wrap() recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// Same chain through the INDEXED form -- children(0) shares Op::
// CallChildren with the bare form (the index is resolved at runtime by
// the same prepareChildrenForward helper the native path uses), so it
// gets the same zero-native-reentry treatment, not just bare children().
TEST(ModuleBodyCompiles, RecursiveIndexedChildrenForwardingChainSucceedsWellPastTheOldNativeReentryLimit) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module wrap() { children(0); }\n"
                          "module recur(n) { if (n > 0) { wrap() recur(n - 1); } else { cube(1); } }\n"
                          "recur(1500);");
    ASSERT_EQ(e.bodies.size(), 1u);
}

// $-forwarding parity for the compiled children() path: a wrapper module's
// own $-writes (`$fn = 7; children();`) and a children($fn=9)-style
// named-$ override must both reach the forwarded child, exactly as the
// native builtinChildren path forwards them (prepareChildrenForward's own
// $-loop, shared by both paths precisely so they can't drift -- these
// tests are the proof it actually holds end-to-end through the compiled
// opcode, not just by construction).
TEST(ModuleBodyCompiles, ChildrenForwardingCarriesWrapperDollarWritesCompiled) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("module w() { $fn = 7; children(); }\n"
                               "w() echo($fn);"),
              "ECHO: 7");
}

TEST(ModuleBodyCompiles, ChildrenForwardingCarriesNamedDollarArgCompiled) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("module w() { children($fn = 9); }\n"
                               "w() echo($fn);"),
              "ECHO: 9");
}

// Regression test for a real bug this same investigation caught:
// enterUserCall's own depth guard used to check callStack_.size()
// directly -- but callStack_ also grows from cheap, zero-native-cost
// COMPILED module-call pushes (skipDepthGuard=true), which must not
// count toward the native-stack-safety ceiling a LATER genuinely
// interpreted call (skipDepthGuard=false) is guarded by. Caught for
// real: a live BOSL2 script's own ambient callStack_ depth (mostly
// compiled Op::CallModule pushes) reached the mid-30s, tripping
// kMaxUserCallDepth=30 for a plain function call even though the REAL
// native C++ nesting at that point was nowhere near it. Fixed via
// nativeUserCallDepth_ (evaluator.hpp), a separate counter incremented
// only for skipDepthGuard=false pushes.
//
// recur() recurses 100 levels deep via bare `recur(n-1);` (pure
// Op::CallModule, skipDepthGuard=true, zero native cost, well past the
// old kMaxUserCallDepth=30) before calling leaf() at the base case --
// forced to run INTERPRETED (skipDepthGuard=false) via a fast-continue
// breakpoint on its own declaration line (mirrors countDebugHookStops'
// own "force one specific chunk native" trick, above this file's first
// TEST). Before the fix, this leaf() call -- genuinely nested only 1
// real native frame deep -- would have falsely tripped the guard purely
// from callStack_'s own ambient depth.
TEST(ModuleBodyCompiles, DeepCompiledModuleRecursionDoesNotFalselyTripTheInterpretedFunctionGuard) {
    ScopedVm vm(true);
    DebugHooks hooks;
    hooks.debugHook = [](int, int, bool, bool, const std::string&, const std::vector<CallStackFrame>&,
                          const DebugFramesFn&) { return DebugAction{}; };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    ev.setFastContinueBreakpoints(std::unordered_map<std::string, std::set<int>>{{"<string>", {2}}});
    auto ast = parseSrc("module recur(n) { if (n > 0) { recur(n - 1); } else { cube(leaf()); } }\n"
                        "function leaf() = 1;\n"
                        "recur(100);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    std::vector<std::unique_ptr<CSGNode>> tree = ev.resolveTree(ast, ctx);
    ASSERT_EQ(tree.size(), 1u);
    EXPECT_EQ(tree[0]->kind, "cube");
}

// The realistic case the NativeStatement-gap fix actually targets: the
// RECURSIVE call itself is a bare statement (pure Op::CallModule, zero
// native stack, bounded only by the heap-sized kMaxVmCallStackDepth), and
// only a non-recursive LEAF statement per level is builtin-wrapped. Each
// such native re-entry completes immediately (no further recursion inside
// it), so driveVmNativeDepth_ never nests past 1-2 regardless of how deep
// `n` goes -- proving driveVmNativeDepth_'s new guard doesn't wrongly cap
// this well past kMaxUserCallDepth=30/kMaxDriveVmNativeDepth=30.
TEST(ModuleBodyCompiles, DeepPureVmRecursionWithNonRecursingBuiltinWrappedLeafSucceeds) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) {"
                          "  if (n > 0) {"
                          "    translate([0,0,n]) cube(1);"
                          "    recur(n - 1);"
                          "  } else {"
                          "    cube(1);"
                          "  }"
                          "}"
                          "recur(100);");
    ASSERT_EQ(e.bodies.size(), 101u);
}

// evalModifier used to dispatch its single wrapped child via a hand-rolled
// checkDebug()+evalStatement() pair, bypassing evalChildren entirely -- so
// a `#`/`!`-wrapped recursive module call never got the NativeStatement-gap
// fix's Op::CallModule treatment, unlike an otherwise-identical `translate`-
// wrapped call. Same shape as
// DeepPureVmRecursionWithNonRecursingBuiltinWrappedLeafSucceeds, just with
// `#` instead of `translate` wrapping the non-recursing leaf, proving the
// fix now applies here too.
TEST(ModuleBodyCompiles, DeepPureVmRecursionWithNonRecursingModifierWrappedLeafSucceeds) {
    ScopedVm vm(true);
    Evaluated e = evalSrc("module recur(n) {"
                          "  if (n > 0) {"
                          "    #translate([0,0,n]) cube(1);"
                          "    recur(n - 1);"
                          "  } else {"
                          "    cube(1);"
                          "  }"
                          "}"
                          "recur(100);");
    ASSERT_EQ(e.bodies.size(), 101u);
}

// -- Leaf-statement compilation (Assignment/ModularEcho/ModularAssert/ -----
// -- ModularLet get real bytecode instead of Op::NativeStatement) ---------

TEST(ModuleBodyCompiles, AssignmentStatementCompilesWithDollarVarAndOverwrittenWarning) {
    ScopedVm vm(true);
    std::string captured = runCapturingEcho("$fn = 6;\n"
                                             "function f() = $fn;\n"
                                             "echo(f());\n"
                                             "x = 1;\n"
                                             "x = 2;\n" // overwritten warning
                                             "echo(x);\n");
    EXPECT_NE(captured.find("ECHO: 6"), std::string::npos);
    EXPECT_NE(captured.find("was assigned on line"), std::string::npos);
    EXPECT_NE(captured.find("but was overwritten"), std::string::npos);
    EXPECT_NE(captured.find("ECHO: 2"), std::string::npos);
}

TEST(ModuleBodyCompiles, EchoStatementCompilesWithNamedAndPositionalArgs) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("echo(1, b=2, 3);"), "ECHO: 1, b = 2, 3");
}

TEST(ModuleBodyCompiles, AssertStatementCompilesWithNamedArgsMessageAndEagerSideEffects) {
    ScopedVm vm(true);
    // Named args in reverse order (message before condition), PLUS a 3rd,
    // logically-unused positional argument -- must still be evaluated
    // eagerly for its own side effect (mirrors evalAssertStatement's own
    // resolveArgs() call, unlike AssertOp's lazy message).
    std::string captured =
        runCapturingEcho("assert(message=\"unused, condition true\", condition=true, echo(\"side effect\"));\n"
                          "echo(\"reached\");\n");
    EXPECT_NE(captured.find("ECHO: \"side effect\""), std::string::npos);
    EXPECT_NE(captured.find("ECHO: \"reached\""), std::string::npos);
}

TEST(ModuleBodyCompiles, AssertStatementCompiledFormFailsWithNamedMessage) {
    ScopedVm vm(true);
    try {
        evalSrc("assert(condition=false, message=\"custom\");");
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("Assertion 'false' failed"), std::string::npos) << what;
        EXPECT_NE(what.find("\"custom\""), std::string::npos) << what;
    }
}

TEST(ModuleBodyCompiles, LetBlockStatementDoesNotSeeItsOwnEarlierSiblingAssignment) {
    ScopedVm vm(true);
    // The statement form's own documented divergence from let-EXPRESSION
    // sequential visibility: b's RHS must see the OUTER a (1), not this
    // same let-block's own a=2 -- b should be 2, not 3.
    EXPECT_EQ(runCapturingEcho("a = 1;\n"
                               "let (a = 2, b = a + 1) { echo(a, b); }\n"
                               "echo(a);\n"),
              "ECHO: 2, 2\nECHO: 1");
}

TEST(ModuleBodyCompiles, LetBlockStatementIsolatesDollarVarOverrideFromLaterStatements) {
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("function f() = $fn;\n"
                               "$fn = 10;\n"
                               "let ($fn = 99) { echo(f()); }\n"
                               "echo(f());\n"),
              "ECHO: 99\nECHO: 10");
}

TEST(ModuleBodyCompiles, NestedLetExpressionInsideCompiledStatementArgumentAllocatesSlotsCorrectly) {
    // Exercises the numSlots/frame->slots fix -- a compiled module-body
    // statement's own inline argument can itself contain a LetOp
    // EXPRESSION (as opposed to the ModularLet STATEMENT form, above),
    // which DOES use real local slots (unlike anything else in a module
    // chunk) -- without frame->slots sized to chunk.numSlots, this
    // indexes out of bounds.
    ScopedVm vm(true);
    EXPECT_EQ(runCapturingEcho("echo(let(x = 5, y = x + 1) x + y);\n"
                               "v = let(q = 3) q * q;\n"
                               "echo(v);\n"),
              "ECHO: 11\nECHO: 9");
}

TEST(ModuleBodyCompiles, RecursiveModuleThrowingPartwayUnwindsCleanlyAndVmFrameRunsAgain) {
    ScopedVm vm(true);
    Evaluator ev;
    auto ast1 =
        parseSrc("module recur(n) { if (n > 0) { recur(n - 1); } else { assert(false, \"boom\"); } }\nrecur(5000);");
    auto scope1 = oscad::buildScopes(ast1);
    EvalContext ctx1 = EvalContext::makeRoot(scope1.get());
    EXPECT_THROW(ev.evaluate(ast1, ctx1, {}), EvalError);

    // A second, unrelated script on the SAME Evaluator afterward -- if any
    // VmFrame/treeStack_ frame/callStack_ entry leaked from the throw
    // above, this would crash, misbehave, or show a corrupted pool.
    auto ast2 = parseSrc("module recur(n) { if (n > 0) { recur(n - 1); } else { cube(1); } }\nrecur(2000);");
    auto scope2 = oscad::buildScopes(ast2);
    EvalContext ctx2 = EvalContext::makeRoot(scope2.get());
    std::vector<ColoredBody> bodies = ev.evaluate(ast2, ctx2, {});
    ASSERT_EQ(bodies.size(), 1u);
}

TEST(ModuleBodyCompiles, DollarVarSetEarlyStaysVisibleDeepInARecursiveModuleChainCompiled) {
    ScopedVm vm(true);
    // The module-call analog of TailCalls.DollarVarSetEarlyInANonTailCompiled
    // ChainStaysVisibleManyLevelsDeep (test_tail_calls.cpp) -- each
    // recursive level is a genuine Op::CallModule push (its own VmFrame,
    // its own ctxChain entry via buildModuleChildCtx's callCtxFor), not a
    // hop -- $probe (dyn, isolate=false) must still resolve 2000 real
    // pushes deep.
    EXPECT_EQ(runCapturingEcho("module recur(n) { if (n > 0) { recur(n - 1); } else { echo($probe); } }\n"
                                "let($probe = 77) recur(2000);"),
              "ECHO: 77");
}

TEST(ModuleBodyCompiles, NestedModuleClosureOverReassignedParameterCompiles) {
    ScopedVm vm(true);
    // Regression test for a real bug caught while building this feature:
    // evalUserModule's own compiled branch used to skip the callStack_
    // bracket entirely (assuming, wrongly, that some OUTER caller already
    // did it -- true for a NESTED Op::CallModule call, never true for
    // THIS, the outermost entry point) -- silently breaking callCtxFor's
    // span-containment closure search for any module declared INSIDE
    // another module's body, which hung (an infectious infinite loop, not
    // a clean failure) rather than crashing outright.
    Evaluated e = evalSrc("module outer(edges) {"
                          "  edges = edges * 2;"
                          "  module inner() { cube(edges); }"
                          "  inner();"
                          "}"
                          "outer(3);");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->BoundingBox().max.x, 6.0, 1e-9);
}

TEST(ModuleBodyCompiles, ParentModulesCountIsAccurateAtEachNestingLevelCompiled) {
    // Evaluator::moduleCallDepth_ replaced a full callStack_ rescan per
    // module call (buildModuleChildCtx's own $parent_modules) with an
    // incrementally maintained counter -- found necessary because the
    // rescan is O(depth), turning a script recursing 64,000 modules deep
    // (no geometry cost at all, just the recursion itself) from an
    // expected-linear ~1s into an actual ~5.5s, and 128,000 deep into
    // ~22s (quadratic, confirmed by doubling depth and watching wall time
    // roughly quadruple) -- entirely invisible at the small depths any of
    // this session's OTHER tests exercise. This test is the correctness
    // half of that fix: increment/decrement must land on exactly the
    // right call, at every depth, or $parent_modules silently drifts.
    EXPECT_EQ(runCapturingEcho("module inner() { echo($parent_modules); }\n"
                                "module mid() { inner(); }\n"
                                "module outer() { mid(); }\n"
                                "outer();"),
              "ECHO: 2");
}

TEST(ModuleBodyCompiles, ParentModulesCountRecoversCorrectlyAfterACaughtExceptionCompiled) {
    // The exception-unwind path (Evaluator::exitUserCallException, and
    // driveVm's own teardownVmCallStackDownTo for a compiled chain) must
    // decrement moduleCallDepth_ exactly as often as the success path
    // does -- otherwise a caught error inside a deep module chain would
    // leave every SUBSEQUENT $parent_modules read permanently wrong (an
    // off-by-N leak, not a crash, so nothing else would catch it).
    std::string captured;
    Evaluator ev([&](const std::string& msg) { captured = msg; });
    auto ast1 = parseSrc("module inner() { assert(false, \"boom\"); }\n"
                          "module outer() { inner(); }\n"
                          "outer();");
    auto scope1 = oscad::buildScopes(ast1);
    EvalContext ctx1 = EvalContext::makeRoot(scope1.get());
    EXPECT_THROW(ev.resolveTree(ast1, ctx1), EvalError);

    // Same Evaluator instance, a second, unrelated script -- if the throw
    // above leaked moduleCallDepth_ increments, THIS $parent_modules read
    // would be wrong even though nothing here is nested at all.
    auto ast2 = parseSrc("module inner() { echo($parent_modules); }\n"
                          "module outer() { inner(); }\n"
                          "outer();");
    auto scope2 = oscad::buildScopes(ast2);
    EvalContext ctx2 = EvalContext::makeRoot(scope2.get());
    ev.resolveTree(ast2, ctx2);
    EXPECT_EQ(captured, "ECHO: 1");
}

TEST(ModuleBodyCompiles, NestedModuleClosureStillWorksAfterADeepUnrelatedRecursiveDetour) {
    ScopedVm vm(true); // 5000-deep non-tail recursion needs the compiled path (see test_tail_calls.cpp)
    // Regression test for Evaluator::activeDeclRefcount_ (callCtxFor's own
    // O(depth) callStack_ scan replaced with a refcounted set of DISTINCT
    // active declarations -- see its own doc comment, evaluator.hpp, for
    // the O(depth^2) perf cliff this fixes). The refcount for `deep`
    // should go all the way back to exactly 0 once its 5000-deep
    // recursive detour unwinds -- if the enter/exit accounting were even
    // slightly off (e.g. missing a decrement somewhere), `deep`'s own
    // entry would leak into activeDeclRefcount_ forever, which wouldn't
    // break correctness here (deep's own span can't contain outer/inner's)
    // but WOULD mean this test is exercising a real leak undetected by
    // simpler tests. Bounding `deep`'s own recursion at a shallow depth
    // first (to confirm it terminates cleanly) then going deep is the
    // point: it must not corrupt the SEPARATE outer/inner closure check
    // that runs after it, on the same Evaluator.
    std::string captured;
    Evaluator ev([&](const std::string& msg) { captured = msg; });
    auto ast = parseSrc("function deep(n) = n <= 0 ? 0 : 1 + deep(n - 1);\n"
                        "module outer(edges) {"
                        "  edges = edges * 2;"
                        "  module inner() { echo(edges); }"
                        "  inner();"
                        "}"
                        "echo(deep(5000));"
                        "outer(3);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
    EXPECT_EQ(captured, "ECHO: 6"); // 3*2, seen via inner's own closure over outer's reassigned edges
}
