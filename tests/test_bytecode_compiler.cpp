#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_evaluator/eval_error.hpp"

#include "test_helpers.hpp"

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
    // 3 stops, not 4: `make`'s own body-entry (1), `g`'s own body-entry
    // (1) -- `g` itself now also runs compiled (its captures-having chunk
    // is registered too, not discarded -- see the FunctionLiteral case's
    // own doc comment, bytecode_compiler.cpp), so it costs exactly one
    // stop instead of the extra sub-expression checkpoint an interpreted
    // call used to add on top -- plus the top-level echo() statement's own
    // stop (module-level code is never compiled).
    const int stops = countDebugHookStops("function make(x) = let(g = function(y) y + x) g(5);\n"
                                           "echo(make(10));",
                                           std::unordered_map<std::string, std::set<int>>{});
    EXPECT_EQ(stops, 3);
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
    const int stops = countDebugHookStops(
        "function outer(n) = n > 0 ? function(y) y + n : function(y) y - n;\n"
        "stored = outer(5);\n"
        "echo(stored(3));",
        std::unordered_map<std::string, std::set<int>>{});
    EXPECT_EQ(stops, 6);
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

TEST(BytecodeCompiler, SelfReferentialRecursiveClosureStillRunsInterpretedAndCorrectly) {
    ScopedVm vm(true);
    // reduce()'s own idiom: `a` calls itself by name, which LetOp's own
    // declareLocal-after-compiling-the-RHS ordering means never resolves as
    // a genuine upvalue at compile time (falls through to Op::LoadFree
    // instead -- see containsLoadFree's own doc comment, bytecode_compiler.cpp).
    // Registering such a chunk for compiled invocation is what previously
    // (empirically, against a real list) turned an O(n) reduce() into
    // O(n^2): every recursive call re-derives a fresh closure via
    // Evaluator::evalIdentifier's ctx.scope->lookupVariable() fallback,
    // nesting its own capturedLet one level deeper than the last every
    // time. `containsLoadFree` excludes exactly this case from
    // registration, so `a` keeps running interpreted -- more than 1 stop
    // for its own call proves that, and the actual sum must still come out
    // right either way.
    const int stops = countDebugHookStops(
        "function reduce(func, list, init=0) = let(l = len(list), a = function (x,i) i<l? "
        "a(func(x,list[i]), i+1) : x) a(init,0);\n"
        "echo(reduce(function(p,q) p+q, [1,2,3,4], 0));",
        std::unordered_map<std::string, std::set<int>>{});
    EXPECT_GT(stops, 2);
    EXPECT_EQ(runCapturingEcho("function reduce(func, list, init=0) = let(l = len(list), a = function (x,i) "
                                "i<l? a(func(x,list[i]), i+1) : x) a(init,0);\n"
                                "echo(reduce(function(p,q) p+q, [1,2,3,4], 0));"),
              "ECHO: 10");
}

TEST(BytecodeCompiler, ClosureWithDollarParameterStillBailsContainer) {
    ScopedVm vm(true);
    // Residual, deliberate limitation: a FunctionLiteral with its OWN
    // dollar-prefixed parameter still fails compileFunctionLike outright
    // (same $-prefixed-parameter rule as a plain named function, see
    // DollarPrefixedParameterBailsCompilationAndStillWorks above) -- there's
    // no lighter way left to discover what it captures in that case, so the
    // WHOLE containing declaration still bails (throw NotCompilable), same
    // as before Op::MakeClosure. Value must still be correct via the
    // interpreter fallback.
    const int stops = countDebugHookStops(
        "function outer(x) = let(g = function(y, $fn) y + x + $fn) g(5, $fn=2);\n"
        "echo(outer(10));",
        std::nullopt);
    // `outer` never compiles at all here (more than 1 stop for its own
    // call), unlike MakeClosureLetsContainerCompileDespiteNonEscapingClosure
    // above where the analogous ($-free) container compiles to 1.
    EXPECT_GT(stops, 2);
    EXPECT_EQ(runCapturingEcho("function outer(x) = let(g = function(y, $fn) y + x + $fn) g(5, $fn=2);\n"
                                "echo(outer(10));"),
              "ECHO: 17");
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
    EXPECT_EQ(runCapturingEcho("function f() = import(\"" + path.string() + "\");\necho(f());"),
              "ECHO: object(a = 1, b = 2)");
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
              "ECHO: object(a = 4, b = 2, c = 3)");
    // Positional list-of-pairs spread.
    EXPECT_EQ(runCapturingEcho("function g() = object(a = 1, [[\"b\", 2]], a = 3);\necho(g());"),
              "ECHO: object(a = 3, b = 2)");
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
                                path.string() +
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
    EXPECT_EQ(stops, 5);
}

TEST(BytecodeCompiler, FastContinueWithNoBreakpointInFunctionUsesVm) {
    ScopedVm vm(true);
    // Breakpoint set, but only on line 2 (the echo statement) -- f's own
    // body is entirely on line 1, outside that set, so it's eligible to
    // run compiled.
    const int stops = countDebugHookStops("function f(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "echo(f(5));",
                                           std::unordered_map<std::string, std::set<int>>{{"<string>", {2}}});
    EXPECT_EQ(stops, 3);
}

TEST(BytecodeCompiler, FastContinueWithBreakpointInsideFunctionStillInterprets) {
    ScopedVm vm(true);
    // Breakpoint on line 1 itself -- inside f's own compiled span -- must
    // still force the interpreter for f specifically, even in fast-continue
    // mode: something on that exact line could need a real checkpoint.
    const int stops = countDebugHookStops("function f(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "echo(f(5));",
                                           std::unordered_map<std::string, std::set<int>>{{"<string>", {1}}});
    EXPECT_EQ(stops, 5);
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
    const int stops = countDebugHookStops("function f(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "function g(x) = x > 0 ? x + 1 : x - 1;\n"
                                           "echo(f(5));",
                                           std::unordered_map<std::string, std::set<int>>{{"<string>", {2}}});
    EXPECT_EQ(stops, 3);
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
