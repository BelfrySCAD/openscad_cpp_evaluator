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
// setBytecodeVmEnabledForTesting's own doc comment). Both the interpreter
// AND the bytecode VM's own call machinery (Phase B) do genuine C++
// recursion for a NON-tail call (there is no other way to implement one --
// see NonTailRecursionIsUnaffectedByTheTrampoline, below) -- a sufficiently
// deep one used to silently overflow the native stack and crash the whole
// process; evalUserFunctionCore's own depth guard (evaluator.hpp) now turns
// that into a controlled EvalError well before that point, shared by both
// paths, exercised by both compiled- and interpreted-path variants of the
// same test at the bottom of this file.

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

TEST(TailCalls, DollarParameterThreadsThroughCompiledTailRecursion) {
    ScopedVm vm(true);
    // $fn declared as its own PARAMETER (not just read as an ambient dyn
    // var, unlike the test above) rebinds via childCtx.dyn at every tail
    // hop -- compileFunctionLike no longer bails on a $-prefixed parameter
    // (see its own doc comment, bytecode_compiler.cpp). No tail-call-
    // specific hazard: every hop's bindCompiledArgs call is identical to
    // what a genuine non-tail call already does, and dyn already threads
    // correctly across hops (proved by the test above).
    const std::string script =
        "function sum_with_fn($fn, n, acc=0) = n == 0 ? $fn + acc : sum_with_fn($fn, n - 1, acc + n);\n"
        "echo(sum_with_fn(1000, 50000));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 1.25003e+9");
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

TEST(TailCalls, DeepNonTailRecursionHitsAControlledErrorInsteadOfCrashingInterpreted) {
    ScopedVm vm(false);
    // `1 + f(n-1)` is NOT a tail position (the addition happens after the
    // recursive call returns) -- there is no trampoline for this, ever
    // (see NonTailRecursionIsUnaffectedByTheTrampoline, above): every
    // logical call is a real, unavoidable C++ recursive call through
    // evalUserFunctionCore. Before its own depth guard existed, a script
    // shaped exactly like this one reliably segfaulted the whole process
    // (confirmed present in the wild -- a plain, closure-free
    // FunctionDeclaration, nothing specific to closures/letrec/tail-call
    // machinery at all). Interpreted-path variant; see the compiled-path
    // one below. f(500) is comfortably past kMaxUserCallDepth (30, see its
    // own doc comment, evaluator.hpp) -- the actual native recursion never
    // gets anywhere near 500, since the guard fires at 30 regardless of
    // what depth the script asks for.
    Evaluator ev;
    auto ast = parseSrc("function f(n) = n <= 0 ? 0 : 1 + f(n - 1);\nresult = f(500);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    try {
        ev.resolveTree(ast, ctx);
        FAIL() << "expected EvalError";
    } catch (const EvalError& e) {
        EXPECT_NE(std::string(e.what()).find("Recursion too deep"), std::string::npos);
        EXPECT_NE(std::string(e.what()).find("'f'"), std::string::npos);
    }
}

TEST(TailCalls, DeepNonTailRecursionUnderVmDoesNotHitTheNativeDepthGuard) {
    ScopedVm vm(true);
    // Same script as the interpreted test above, but this is exactly the
    // case the explicit frame-stack VM (see bytecode_vm.cpp's driveVm)
    // exists to fix: a NON-tail call between two compiled chunks
    // (Op::CallFn, via pushBracketedCallFrame) no longer makes a genuine
    // native C++ call -- it's pushed onto vmCallStack_ and serviced by
    // driveVm's own loop instead, so kMaxUserCallDepth (50, a NATIVE
    // stack margin -- see its own doc comment) no longer applies here at
    // all (enterUserCall's skipDepthGuard=true, set only by
    // pushBracketedCallFrame). f(500) now succeeds where it used to throw
    // -- the direct, measurable proof the redesign works, not just "no
    // regression." Bounded only by the new, much larger
    // kMaxVmCallStackDepth (1,000,000, a memory/runaway guard, not a
    // stack-safety one).
    Evaluator ev;
    auto ast = parseSrc("function f(n) = n <= 0 ? 0 : 1 + f(n - 1);\necho(f(500));");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.resolveTree(ast, ctx);
}

TEST(TailCalls, DeepNonTailRecursionUnderVmSucceedsWellPastTheOldNativeLimit) {
    ScopedVm vm(true);
    // 10,000 is two orders of magnitude past the old native-stack-bound
    // kMaxUserCallDepth=50 -- the explicit-stack VM (vmCallStack_, heap-
    // allocated) has no trouble with it.
    const std::string script = "function f(n) = n <= 0 ? 0 : 1 + f(n - 1);\necho(f(10000));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 10000");
}

TEST(TailCalls, DeepNonTailRecursionThrowingPartwayUnwindsCleanly) {
    ScopedVm vm(true);
    // An assert() partway down a long NON-tail compiled call chain throws
    // through N pushed, bracketed VmFrames -- driveVm's own outer
    // try/catch (teardownVmCallStackDownTo) must walk vmCallStack_ back
    // to the floor, releasing every VmFrame to the pool and closing out
    // every callStack_/profiling bracket WITHOUT firing returnHook,
    // exactly mirroring the pre-redesign per-frame native unwind. Proven
    // not by inspecting internals directly but by running a second,
    // unrelated script on the SAME Evaluator afterward -- if any frame,
    // pool slot, or callStack_ entry leaked, this second run would either
    // crash, wrongly inherit stale state, or the pool would be visibly
    // exhausted/corrupted.
    Evaluator ev;
    auto ast = parseSrc("function f(n) = n <= 0 ? assert(false, \"boom\") : 1 + f(n - 1);\necho(f(5000));");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.resolveTree(ast, ctx), EvalError);

    auto ast2 = parseSrc("function g(n) = n <= 0 ? 0 : 1 + g(n - 1);\necho(g(2000));");
    auto scope2 = oscad::buildScopes(ast2);
    EvalContext ctx2 = EvalContext::makeRoot(scope2.get());
    ev.resolveTree(ast2, ctx2);
}

TEST(TailCalls, DollarVarSetEarlyInANonTailCompiledChainStaysVisibleManyLevelsDeep) {
    ScopedVm vm(true);
    // The ctxChain analog of DollarVarSetEarlyInATailChainStaysVisible
    // ManyHopsLater above, but for a NON-tail chain -- each level is a
    // genuine pushBracketedCallFrame push (its own VmFrame, its own
    // ctxChain entry), not a tail hop mutating one frame in place. $probe
    // (dyn, isolate=false) must still resolve 2000 real pushes deep.
    const std::string script = "function depth(n) = n <= 0 ? $probe : 1 + depth(n - 1);\n"
                                "echo(let($probe = 77) depth(2000));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 2077");
}

TEST(TailCalls, CollatzStepsRecursesFromBothTernaryBranchesInterpreted) {
    ScopedVm vm(false);
    // Real-world example exercising tail calls from BOTH ternary branches at
    // once (the even and odd cases each recurse) -- came up when auditing
    // for un-trampolined tail positions (see this file's header). Scans
    // every n < 10000 and takes the longest chain; the known record holder
    // in that range is n=6171 at 261 steps, calling collatz_steps 10000
    // times over.
    const std::string script =
        "function collatz_steps(n, steps=0) = n == 1 ? steps : "
        "(n % 2 == 0 ? collatz_steps(n / 2, steps + 1) : collatz_steps(3 * n + 1, steps + 1));\n"
        "lengths = [for (i = [1:10000]) collatz_steps(i)];\n"
        "echo(max(lengths));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 261");
}

TEST(TailCalls, CollatzStepsRecursesFromBothTernaryBranchesCompiled) {
    ScopedVm vm(true);
    const std::string script =
        "function collatz_steps(n, steps=0) = n == 1 ? steps : "
        "(n % 2 == 0 ? collatz_steps(n / 2, steps + 1) : collatz_steps(3 * n + 1, steps + 1));\n"
        "lengths = [for (i = [1:10000]) collatz_steps(i)];\n"
        "echo(max(lengths));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 261");
}

TEST(TailCalls, ShallowNonTailRecursionStillWorksUnderTheDepthGuard) {
    ScopedVm vm(true);
    // The guard must not fire for perfectly ordinary, shallow non-tail
    // recursion -- the overwhelmingly common real-world shape (a handful
    // to a few dozen levels, e.g. walking a small nested data structure).
    const std::string script = "function fact(n) = n <= 1 ? 1 : n * fact(n - 1);\necho(fact(10));";
    EXPECT_EQ(runCapturingEcho(script), "ECHO: 3.6288e+6");
}
