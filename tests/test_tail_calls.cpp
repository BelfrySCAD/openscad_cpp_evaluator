// Tail-call optimization (TCO) tests for the AST-walking interpreter path
// (evalFunctionBodyTrampoline/simplifyTailStep, src/user_calls.cpp) --
// mirrors real upstream OpenSCAD's own FunctionCall::evaluate()/
// simplify_function_body trampoline. See these functions' own doc comments
// in evaluator.hpp for the full design rationale, including why a
// closure-nested tail hop deliberately falls back to a real recursive call
// instead of being trampolined.
//
// Every test here explicitly forces the interpreter path via ScopedVm(false)
// (not the OSCAD_BYTECODE_VM env var, which is cached forever after its
// first read within a test binary -- see Evaluator::
// setBytecodeVmEnabledForTesting's own doc comment): the bytecode VM's own
// call machinery (Phase B, not yet implemented as of this file) still does
// genuine C++ recursion for every call, so a deep-recursion test run under
// the default VM-on path would crash for the wrong reason.

#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <chrono>
#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

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

TEST(TailCalls, DeepSelfTailRecursionDoesNotOverflowTheNativeStack) {
    ScopedVm vm(false);
    // n*(n+1)/2 via an accumulator-style tail call -- the recursive call
    // IS the ternary's whole false-branch, a genuine tail position. Before
    // this trampoline existed, this depth reliably segfaulted (real C++
    // recursion through evalUserFunctionCore/evalExpr, ~5-6 native frames
    // per OpenSCAD-level call).
    const std::string script = "function sum(n, acc=0) = n == 0 ? acc : sum(n - 1, acc + n);\necho(sum(200000));";
    // n*(n+1)/2 = 20000100000 -- OpenSCAD's own number formatting switches
    // to scientific notation past a magnitude threshold, matching every
    // other echo-format test in this suite.
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 2.00001e+10");
}

TEST(TailCalls, MutualTailRecursionAtDepthDoesNotOverflowTheNativeStack) {
    ScopedVm vm(false);
    const std::string script = "function is_even(n) = n == 0 ? true : is_odd(n - 1);\n"
                                "function is_odd(n) = n == 0 ? false : is_even(n - 1);\n"
                                "echo(is_even(100000));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: true");
}

TEST(TailCalls, ClosureNestedTailCallFallsBackToRealRecursionAndStillResolves) {
    ScopedVm vm(false);
    // g's own tail call is lexically nested inside make's body -- a
    // closure over `x`. tryTailStepFor must NOT trampoline this hop
    // (callCtxFor's usedChildCtx==true): this port's closure lookup
    // (findUpvalue) walks CallStackFrame::upvalueParent as an index chain
    // across distinct call-stack slots, and collapsing this into the
    // trampoline's single mutated frame would make that frame's own
    // upvalueParent point at itself, looping findUpvalue forever the first
    // time such a closure reads an ancestor two or more levels up. This is
    // a permanent regression guard for that hazard, not a hypothetical --
    // caught for real during development (this exact script came back
    // "undef" instead of 15 before a related EvalContext lifetime bug was
    // also fixed).
    const std::string script = "function make(x) = let(g = function(y) y + x) g(5);\necho(make(10));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 15");
}

TEST(TailCalls, LetLetLetChainWithinOneTailStepKeepsEveryLevelReadable) {
    ScopedVm vm(false);
    // Multiple LetOp unwraps within ONE logical call's own tail-position
    // chain, each opening a new (non-isolated) trail level -- regression
    // test for the "reused ctx variable destroys an ancestor level's
    // shared_ptr, popping bindings a later step still needs" bug fixed
    // alongside this trampoline (see evalFunctionBodyTrampoline's own
    // `chain` comment). Every let()-bound name must stay readable by the
    // final body expression regardless of how many Let unwraps preceded it.
    const std::string script = "function f() = let(a = 1) let(b = 2) let(c = 3) a + b + c;\necho(f());";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 6");
}

TEST(TailCalls, DollarVarSetEarlyInATailChainStaysVisibleManyHopsLater) {
    ScopedVm vm(false);
    // $-vars stay dynamically scoped THROUGH an isolated call (callCtx()'s
    // dyn uses isolate=false, unlike let_'s isolate=true) -- a $fn set via
    // let() before a long tail-recursive chain begins must still resolve
    // deep inside it. This is exactly the property the "keep every
    // EvalContext alive for the whole trampoline run" fix (as opposed to
    // dropping earlier levels at each isolated-call boundary) protects.
    const std::string script = "function depth(n) = n == 0 ? $fn : depth(n - 1);\n"
                                "echo(let($fn = 77) depth(50000));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 77");
}

TEST(TailCalls, NonTailRecursionIsUnaffectedByTheTrampoline) {
    ScopedVm vm(false);
    // `n * fact(n-1)` is NOT a tail position -- the multiply happens after
    // the recursive call returns -- so this must keep working exactly as
    // it always has, via genuine (bounded-depth) recursion. Matches real
    // upstream OpenSCAD's own identical limitation for the identical
    // reason (simplify_function_body's `else` branch is a real recursive
    // evaluate() call too).
    const std::string script = "function fact(n) = n <= 1 ? 1 : n * fact(n - 1);\necho(fact(10));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 3.6288e+6");
}

TEST(TailCalls, InfiniteTailRecursionHitsTheIterationCapInsteadOfHanging) {
    ScopedVm vm(false);
    Evaluator ev;
    auto ast = parseSrc("function loop(n) = loop(n + 1);\nresult = loop(0);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    // Mirrors upstream's own 1,000,000-iteration RecursionException cap
    // (Expression.cc) -- a tail-recursive function with no base case must
    // eventually fail with a controlled error, not hang or silently loop
    // forever in O(1) stack space.
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
}

TEST(TailCalls, InfiniteTailRecursionErrorMentionsTheFunctionName) {
    ScopedVm vm(false);
    Evaluator ev;
    auto ast = parseSrc("function loop(n) = loop(n + 1);\nresult = loop(0);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    try {
        ev.resolveTree(ast, ctx);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        EXPECT_NE(std::string(e.what()).find("loop"), std::string::npos);
        EXPECT_NE(std::string(e.what()).find("Recursion detected"), std::string::npos);
    }
}

TEST(TailCalls, InfiniteTailRecursionHitsTheCapInBoundedWallTime) {
    // Regression tripwire for issue #50: evalFunctionBodyTrampoline's own
    // `chain` (every hop's EvalContext, kept alive for $-var ancestry --
    // see its own doc comment) used to rely on std::vector<EvalContext>'s
    // OWN destructor to tear itself down, whose element-destruction order
    // is unspecified by the standard. libstdc++ (GCC) and, per CI evidence,
    // MSVC destroy front-to-back -- the adversarial order for
    // ScopeTrailStorage::popLevel's "scan from the back" optimization
    // (its own doc comment), turning an intended O(N) teardown into a
    // real, measured O(N^2): the exact same 1,000,000-iteration script
    // below took 25-50+ minutes on GCC/MSVC CI runners before the fix
    // (an explicit back-to-front teardown, immune to the underlying
    // vector's own unspecified order), vs ~1s on Clang/libc++ throughout.
    // ponytail: generous fixed ceiling, not a strict perf target -- trips
    // only on a real regression (this exact O(N^2) reintroduced), not
    // machine noise; raise if a slower CI runner ever needs it.
    ScopedVm vm(false);
    Evaluator ev;
    auto ast = parseSrc("function loop(n) = loop(n + 1);\nresult = loop(0);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    const auto start = std::chrono::steady_clock::now();
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    constexpr double kCeilingMs = 30000.0;
    EXPECT_LT(ms, kCeilingMs) << "took " << ms << "ms -- possible O(N^2) regression, see issue #50";
}

TEST(TailCalls, ErrorThrownDeepInATailChainProducesABoundedTrace) {
    ScopedVm vm(false);
    Evaluator ev;
    // assert() past n==0 always fails -- the error fires after a real tail
    // chain of several thousand hops. Since the trampoline mutates ONE
    // CallStackFrame in place per hop (never pushes), the resulting TRACE
    // must stay small regardless of how deep the chain was -- proportional
    // to actual C++/module call nesting (here: none), not to the number of
    // tail-call iterations that happened before the error.
    auto ast = parseSrc("function f(n) = n == 0 ? assert(false, \"boom\") 0 : f(n - 1);\nresult = f(5000);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    try {
        ev.resolveTree(ast, ctx);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        const std::string msg = e.what();
        size_t count = 0;
        for (size_t pos = msg.find("TRACE:"); pos != std::string::npos; pos = msg.find("TRACE:", pos + 1)) ++count;
        EXPECT_LE(count, 2u);
    }
}

TEST(TailCalls, TailRecursiveProfilingCallCountIsAccuratePerSite) {
    ScopedVm vm(false);
    Evaluator ev(EchoFn{}, nullptr, nullptr, DebugHooks{}, /*profiling=*/true);
    auto ast = parseSrc("function is_even(n) = n == 0 ? true : is_odd(n - 1);\n"
                         "function is_odd(n) = n == 0 ? false : is_even(n - 1);\n"
                         "result = is_even(200);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);

    ASSERT_TRUE(ev.profileResult.has_value());
    // Every hop through the tail chain is a DIFFERENT call site
    // (is_even's own call to is_odd, and vice versa) -- profileRecordTailHop
    // must bump each site's callCount even though only ONE real
    // profileEnter/profileExit bracket (the outermost is_even(200) call)
    // ever runs. Sequence: is_even(200), is_odd(199), is_even(198), ...,
    // is_even(0) -- n=200,198,...,0 is 101 is_even invocations (the
    // initial outer call plus 100 recursive hops back into it), n=199,
    // 197,...,1 is 100 is_odd invocations.
    int isOddCalls = 0, isEvenCalls = 0;
    for (const auto& s : ev.profileResult->callSites) {
        if (s.kind != "function") continue;
        if (s.name == "is_odd") isOddCalls += s.callCount;
        if (s.name == "is_even") isEvenCalls += s.callCount;
    }
    EXPECT_EQ(isOddCalls, 100);
    EXPECT_EQ(isEvenCalls, 101);
}

TEST(TailCalls, DebugHookFiresPerHopInsideATailChain) {
    ScopedVm vm(false);
    int calls = 0, bodyEntry = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int line, int, bool, bool exprLevel, const std::string&, const std::vector<CallStackFrame>&,
                           const DebugFramesFn&) {
        ++calls;
        if (line == 1 && !exprLevel) ++bodyEntry;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("function f(n, acc=0) = n == 0 ? acc : f(n - 1, acc + 1);\necho(f(500));");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    // Each of the 500 logical hops fires its own ternary stop, chosen-branch
    // stop and (for the 499 recursive ones) call-site stop, matching the
    // reference's per-call _check_debug placement -- a tail chain is NOT
    // debug-invisible just because it runs on the trampoline instead of the
    // C++ stack. (The reference can't be run as a baseline at this depth:
    // 500 nested _eval_function_call frames blow Python's recursion limit.)
    //
    // The one thing the trampoline still collapses is
    // evalUserFunctionCore's body-entry stop: pushed once for the whole
    // chain, since the chain never leaves that single core call. Documented
    // divergence, not an accident -- restoring it per-hop would mean
    // abandoning TCO whenever a debugger is attached.
    EXPECT_GT(calls, 1000);
    // All statement-level stops on line 1 (the whole function declaration):
    // 501 ternary stops (n = 500..0), 500 recursive call-site stops, and the
    // single body-entry stop.
    EXPECT_EQ(bodyEntry, 501 + 500 + 1);
}
