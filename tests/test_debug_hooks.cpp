#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {
// Mirrors test_bytecode_compiler.cpp's/test_tail_calls.cpp's own ScopedVm --
// the plain OSCAD_BYTECODE_VM env var is cached forever after its first
// read, so a test whose own expected stop-count depends on the compiled
// path actually running (see FastContinueNotHookSkippableStillFiresEvery
// Checkpoint, below) must force it explicitly rather than relying on
// whatever the process default happens to be.
class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};
} // namespace

// Records (line, exprLevel, forced) for every debug-hook call, in order --
// the shape every parity test below asserts on. Sequences here were taken
// from the Python reference (openscad_evaluator) run over the same source
// with an equivalent recording hook, not derived from this port's own
// behavior.
struct Stop {
    int line;
    bool exprLevel;
    bool forced;
    bool operator==(const Stop&) const = default;
};

std::ostream& operator<<(std::ostream& os, const Stop& s) {
    return os << "{" << s.line << ", " << (s.exprLevel ? "expr" : "stmt") << (s.forced ? ", forced" : "") << "}";
}

std::vector<Stop> recordStops(const std::string& src) {
    std::vector<Stop> stops;
    DebugHooks hooks;
    hooks.debugHook = [&](int line, int, bool forced, bool exprLevel, const std::string&,
                           const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        stops.push_back(Stop{line, exprLevel, forced});
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc(src);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    return stops;
}

// Just the statement-level (non-exprLevel) stops -- what a stepping
// debugger actually pauses on.
std::vector<Stop> stmtStops(const std::vector<Stop>& stops) {
    std::vector<Stop> out;
    for (const Stop& s : stops) {
        if (!s.exprLevel) out.push_back(s);
    }
    return out;
}

// Stops filtered to a set of source lines, for tests where unrelated lines
// (an enclosing assignment, a function body elsewhere) would add noise.
std::vector<Stop> stopsOnLines(const std::vector<Stop>& stops, const std::vector<int>& lines) {
    std::vector<Stop> out;
    for (const Stop& s : stops) {
        if (std::find(lines.begin(), lines.end(), s.line) != lines.end()) out.push_back(s);
    }
    return out;
}

size_t countOnLine(const std::vector<Stop>& stops, int line) {
    return static_cast<size_t>(std::count_if(stops.begin(), stops.end(), [line](const Stop& s) { return s.line == line; }));
}

TEST(DebugHooks, HookFiresOnceForEachStatementAtEveryNestingLevel) {
    // 4 statement-level stops, not 3: `translate(...) sphere(...)` is two
    // statements from checkDebug's perspective -- the top-level
    // translate() ModularCall itself (checked by the outer evalChildren)
    // AND its nested sphere() child (checked again when resolveTransform's
    // own resolve function calls evalChildren on its .children).
    //
    // The 3 extra expr-level stops on line 2 are `[1,0,0]`'s own list
    // elements: a list literal is a ListComprehension of three bare
    // element expressions, each of which gets _eval_list_comp's
    // `_check_debug(elem, ctx, expr_level=True)`. Verified against the
    // reference: identical 7-stop sequence.
    EXPECT_EQ(recordStops("cube(1);\ntranslate([1,0,0]) sphere(r=1,$fn=8);\necho(\"x\");"),
              (std::vector<Stop>{{1, false, false},
                                 {2, false, false},
                                 {2, true, false},
                                 {2, true, false},
                                 {2, true, false},
                                 {2, false, false},
                                 {3, false, false}}));
}

TEST(DebugHooks, StopAbortsEvaluation) {
    DebugHooks hooks;
    hooks.debugHook = [](int line, int, bool, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        DebugAction a;
        a.stop = (line == 2);
        return a;
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("cube(1);\nsphere(r=1,$fn=8);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.evaluate(ast, ctx), EvalError);
}

TEST(DebugHooks, ModsOverrideVariableForThatScope) {
    // Mirrors examples/minimal_debugger.py's override_variable_via_mods
    // demo exactly: override `width` just before the cube() statement
    // that uses it, verify the resulting geometry reflects the override.
    DebugHooks hooks;
    hooks.debugHook = [](int line, int, bool, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        DebugAction a;
        if (line == 2) a.mods["width"] = Value{2.0};
        return a;
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("width = 10;\ncube([width, width, width]);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    std::vector<ColoredBody> bodies = ev.evaluate(ast, ctx);
    ASSERT_EQ(bodies.size(), 1u);
    ASSERT_TRUE(bodies[0].body.has_value());
    manifold::Box bbox = bodies[0].body->BoundingBox();
    EXPECT_NEAR(bbox.max.x, 2.0, 1e-9); // script said 10, override said 2
}

TEST(DebugHooks, FiresInsideModuleBodyWithCorrectDepthAndLocals) {
    struct Hit {
        int line = 0, depth = 0;
        bool sawH = false;
        double hValue = 0;
    };
    Hit hit;
    DebugHooks hooks;
    hooks.debugHook = [&](int line, int depth, bool, bool, const std::string&, const std::vector<CallStackFrame>&,
                           const DebugFramesFn& getFrame) {
        if (line == 3) { // the `cube([w, h, 3]);` statement inside module bracket
            hit.line = line;
            hit.depth = depth;
            std::vector<DebugFrame> frames = getFrame();
            DebugFrame frame = frames.empty() ? DebugFrame{} : frames.front();
            auto it = frame.locals.find("h");
            if (it != frame.locals.end()) {
                hit.sawH = true;
                hit.hValue = std::get<double>(it->second);
            }
        }
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("module bracket(w) {\n  h = w * 2;\n  cube([w, h, 3]);\n}\nbracket(10);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);

    EXPECT_EQ(hit.line, 3);
    EXPECT_EQ(hit.depth, 1); // inside one active module call
    ASSERT_TRUE(hit.sawH);
    EXPECT_DOUBLE_EQ(hit.hValue, 20.0); // w=10, h=w*2 -- matches the worked example's `print h` -> $1 = 20
}

TEST(DebugHooks, ForcedBreakpointFiresExactlyOnce) {
    int forcedCalls = 0, normalCalls = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool forced, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        (forced ? forcedCalls : normalCalls)++;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("cube(1);\nbreakpoint();\nsphere(r=1,$fn=8);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    EXPECT_EQ(forcedCalls, 1);
    EXPECT_EQ(normalCalls, 3); // cube, breakpoint() itself (as an ordinary statement), sphere
}

TEST(DebugHooks, BreakpointConditionFalseSkipsThePause) {
    int forcedCalls = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool forced, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        if (forced) ++forcedCalls;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("breakpoint(false);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    EXPECT_EQ(forcedCalls, 0);
}

TEST(DebugHooks, ErrorBreakFiresBeforeThrowingOnAssertFailure) {
    std::string capturedHeader;
    int errorBreakCalls = 0;
    DebugHooks hooks;
    hooks.errorBreak = [&](int, const std::string& header, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        ++errorBreakCalls;
        capturedHeader = header;
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("assert(false, \"boom\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_THROW(ev.evaluate(ast, ctx), EvalError);
    EXPECT_EQ(errorBreakCalls, 1);
    EXPECT_NE(capturedHeader.find("boom"), std::string::npos);
}

TEST(DebugHooks, ReturnHookFiresForUserFunctionCallsWithCorrectValue) {
    std::vector<std::pair<std::string, double>> returns;
    DebugHooks hooks;
    hooks.returnHook = [&](const std::string& name, const Value& v, int) { returns.emplace_back(name, std::get<double>(v)); };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("function sq(x) = x * x;\nresult = sq(6);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    ASSERT_EQ(returns.size(), 1u);
    EXPECT_EQ(returns[0].first, "sq");
    EXPECT_DOUBLE_EQ(returns[0].second, 36.0);
}

TEST(DebugHooks, NoHooksInstalledMeansZeroOverheadCodePathStillWorks) {
    // Bare Evaluator() (no DebugHooks at all) must behave exactly as
    // before this phase -- checkDebug()'s null-hook early return.
    Evaluated e = evalSrc("cube(2);");
    EXPECT_EQ(e.bodies.size(), 1u);
}

// ---------------------------------------------------------------------------
// Fast-continue's hook-skippable mode (issue found via BelfrySCAD's own
// nurbs.scad-heavy debug session: 462k checkDebug() calls survived even
// with per-function VM fast-continue fully engaged, since module/geometry
// evaluation never compiles and every one of those still crossed into
// Python just to be told "continue"). See setFastContinueBreakpoints's own
// doc comment (evaluator.hpp) for the full contract.
// ---------------------------------------------------------------------------

TEST(DebugHooks, FastContinueHookSkippableSkipsCheckpointsWithNoMatchingBreakpoint) {
    int calls = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        ++calls;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    ev.setFastContinueBreakpoints(std::unordered_map<std::string, std::set<int>>{}, /*hookSkippable=*/true);
    auto ast = parseSrc("cube(1);\ntranslate([1,0,0]) sphere(r=1,$fn=8);\necho(\"x\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    // Same script as HookFiresOnceForEachStatementAtEveryNestingLevel (7
    // checkpoints without fast-continue) -- with hook-skippable mode and no
    // breakpoints anywhere, every single one is a plain C++ skip.
    EXPECT_EQ(calls, 0);
}

TEST(DebugHooks, FastContinueHookSkippableStillFiresAtMatchingBreakpointLine) {
    std::vector<int> firedLines;
    DebugHooks hooks;
    hooks.debugHook = [&](int line, int, bool, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        firedLines.push_back(line);
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    ev.setFastContinueBreakpoints(std::unordered_map<std::string, std::set<int>>{{"<string>", {2}}}, /*hookSkippable=*/true);
    auto ast = parseSrc("cube(1);\ntranslate([1,0,0]) sphere(r=1,$fn=8);\necho(\"x\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    // Every line-2 checkpoint (translate itself, its 3 expr-level [1,0,0]
    // list elements, and the nested sphere child -- 5 total, matching
    // HookFiresOnceForEachStatementAtEveryNestingLevel's own sequence)
    // survives the skip; line 1 (cube) and line 3 (echo) have no
    // breakpoint and are skipped entirely.
    EXPECT_EQ(firedLines, (std::vector<int>{2, 2, 2, 2, 2}));
}

TEST(DebugHooks, FastContinueNotHookSkippableStillFiresEveryCheckpoint) {
    // This test's own expected count (4, not 7) depends on the compiled
    // path actually running (translate()'s own [1,0,0] argument becomes
    // compile-eligible, collapsing 3 sub-expression stops -- see below) --
    // force it rather than relying on the process-default OSCAD_BYTECODE_VM,
    // or this test is silently wrong half the time CI runs the suite with
    // it forced off (caught for real: both macOS/Ubuntu CI legs failed on
    // exactly this, PR #59).
    ScopedVm vm(true);
    int calls = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        ++calls;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    // Mirrors step_over/step_out: a real (here, empty) breakpoints set is
    // provided -- so chunkEligibleNow/useBytecodeVm still apply -- but
    // hookSkippable stays false, since both need checkDebug() to keep
    // calling into the hook on every statement so their own step_hit logic
    // (line/depth comparison against the step's own starting point) can
    // run; there is no way to decide that in advance.
    ev.setFastContinueBreakpoints(std::unordered_map<std::string, std::set<int>>{}, /*hookSkippable=*/false);
    auto ast = parseSrc("cube(1);\ntranslate([1,0,0]) sphere(r=1,$fn=8);\necho(\"x\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    // 4, not 7 (Phase 1, module/top-level compilation): hookSkippable is a
    // narrower claim than "breakpoints is accurate" (see
    // setFastContinueBreakpoints' own doc comment, evaluator.hpp) -- it
    // does NOT disable useBytecodeVm()/chunkEligibleNow, only whether
    // checkDebug() can skip calling the hook at all for an eligible line.
    // With an empty (but real) breakpoint set, translate()'s own argument
    // `[1,0,0]` is now eligible to compile too (evalExprMaybeCompiled,
    // called from resolveArgs) -- as a single Op::BuildList instead of
    // three separate _eval_list_comp element evaluations, its own 3
    // expr-level element checkpoints (see
    // HookFiresOnceForEachStatementAtEveryNestingLevel's own doc comment,
    // above, for where those 3 came from) no longer fire: 7 - 3 = 4.
    EXPECT_EQ(calls, 4);
}

TEST(DebugHooks, FastContinueInterruptFlagForcesTheNextCheckpointThrough) {
    std::vector<int> firedLines;
    DebugHooks hooks;
    hooks.debugHook = [&](int line, int, bool, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        firedLines.push_back(line);
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    ev.setFastContinueBreakpoints(std::unordered_map<std::string, std::set<int>>{}, /*hookSkippable=*/true);
    // Pre-armed, as if Pause (or a breakpoint edit) was requested from the
    // main thread before this run even started -- see
    // setFastContinueInterruptFlag's own doc comment (evaluator.hpp) for
    // why this exists: there is no other way to reach a running
    // debug_evaluate() call from outside a hook invocation.
    auto flag = std::make_shared<std::atomic<bool>>(true);
    ev.setFastContinueInterruptFlag(flag);
    auto ast = parseSrc("cube(1);\ntranslate([1,0,0]) sphere(r=1,$fn=8);\necho(\"x\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    // Only the FIRST checkpoint (line 1, cube) fires -- checkDebug's own
    // test-and-clear means the flag is consumed there, so every subsequent
    // checkpoint goes back to being skipped normally (no breakpoints
    // anywhere).
    EXPECT_EQ(firedLines, (std::vector<int>{1}));
    EXPECT_FALSE(flag->load());
}

TEST(DebugHooks, ForcedBreakpointBuiltinAlwaysFiresEvenInHookSkippableMode) {
    int forcedCalls = 0, normalCalls = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool forced, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        (forced ? forcedCalls : normalCalls)++;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    ev.setFastContinueBreakpoints(std::unordered_map<std::string, std::set<int>>{}, /*hookSkippable=*/true);
    auto ast = parseSrc("cube(1);\nbreakpoint();\nsphere(r=1,$fn=8);");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    EXPECT_EQ(forcedCalls, 1); // breakpoint() always bypasses hook-skippable mode
    EXPECT_EQ(normalCalls, 0); // cube/breakpoint()-as-statement/sphere: no breakpoint line matches, all skipped
}

// Regression test for a real dangling-pointer bug (found via a targeted
// ASan build reproducing BelfrySCAD's own "step over a call, but honor a
// breakpoint set inside it" debug scenario): pushBracketedCallFrame/
// pushBracketedModuleFrame (bytecode_vm.cpp) used to point
// CallStackFrame::bodyCtx at their own LOCAL `childCtx` variable, which is
// destroyed the instant they return -- but bodyCtx is read LATER, by any
// subsequent checkDebug() call that walks the WHOLE call stack to build
// per-frame debugger locals (getFrame(), below). A compiled caller
// (`stepped_over`, no breakpoint of its own, so it runs compiled) calling
// a callee forced into the interpreter by an active breakpoint on ITS OWN
// line (`heavy`) reproduces this exactly: pausing deep inside `heavy` and
// calling getFrame() used to segfault reading the compiled caller's own
// already-dangling frame.
TEST(DebugHooks, GetFrameDoesNotCrashWalkingACompiledCallerAboveAForcedInterpretedCallee) {
    ScopedVm vm(true);
    std::vector<size_t> frameCountsAtLine1;
    DebugHooks hooks;
    hooks.debugHook = [&](int line, int depth, bool, bool exprLevel, const std::string&,
                           const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame) {
        if (line == 1 && depth == 2 && !exprLevel) {
            std::vector<DebugFrame> frames = getFrame();
            frameCountsAtLine1.push_back(frames.size());
            EXPECT_EQ(callStack.size(), 2u);
        }
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    // Mirrors BelfrySCAD's own _apply_fast_continue for an active step:
    // a real breakpoint set (on heavy's own line 1, forcing IT specifically
    // to stay interpreted via chunkEligibleNow) with hookSkippable=false.
    ev.setFastContinueBreakpoints(std::unordered_map<std::string, std::set<int>>{{"<string>", {1}}},
                                   /*hookSkippable=*/false);
    auto ast = parseSrc("function heavy(n) = [for (i=[0:1:n-1]) i*i];\n"
                        "function stepped_over() = let(r = heavy(3)) len(r);\n"
                        "b = stepped_over();\n");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    // The actual point of this test is that getFrame() doesn't crash
    // reading the compiled `stepped_over` frame's own (previously
    // dangling) bodyCtx -- a consistent, non-empty frame list is enough
    // evidence it read something real rather than garbage each time.
    ASSERT_FALSE(frameCountsAtLine1.empty());
    for (size_t count : frameCountsAtLine1) EXPECT_EQ(count, frameCountsAtLine1.front());
    EXPECT_GT(frameCountsAtLine1.front(), 0u);
}

// ---------------------------------------------------------------------------
// Full parity with the Python reference's _check_debug call sites.
//
// Every expected sequence below was captured by running the reference
// (openscad_evaluator's Evaluator) over the exact same source with a hook
// recording (line, expr_level, forced), then pasted here verbatim. They are
// not this port's own output written down after the fact.
// ---------------------------------------------------------------------------

// (1) ModularIf / ModularIfElse: an expr-level branch-entry marker on the
// chosen branch's first statement, after the condition, before the branch's
// own statement checks.
TEST(DebugHooksParity, ModularIfFiresExprLevelBranchEntryMarker) {
    EXPECT_EQ(recordStops("if (true)\n  echo(\"yes\");\n"),
              (std::vector<Stop>{{1, false, false}, {2, true, false}, {2, false, false}}));
}

TEST(DebugHooksParity, ModularIfElseMarksWhicheverBranchWasChosen) {
    // false -> the else branch (line 4), never the true branch (line 2).
    EXPECT_EQ(recordStops("if (false)\n  echo(\"yes\");\nelse\n  echo(\"no\");\n"),
              (std::vector<Stop>{{1, false, false}, {4, true, false}, {4, false, false}}));
}

// (2) breakpoint() -- already ported; guards against the forced flag being
// lost while everything around it was rewired.
TEST(DebugHooksParity, BreakpointStillForcesExactlyOnePauseAfterItsOwnStatementStop) {
    // Assignments run before other statements in a scope, hence 1, 3, 2, 2.
    EXPECT_EQ(recordStops("a = 1;\nbreakpoint();\nb = 2;\n"),
              (std::vector<Stop>{{1, false, false}, {3, false, false}, {2, false, false}, {2, false, true}}));
}

// (3) Statement for() with two loop variables: one statement-level stop per
// variable binding per combination, PLUS an expr-level body-entry marker per
// full iteration.
TEST(DebugHooksParity, ModularForStopsAtEachVariableBindingAndEachBodyEntry) {
    const std::vector<Stop> stops = recordStops("for(\ni=[0:2],\nj=[0:1]\n)\necho(i*3+j);\n");
    // i binds 3 times (line 2); j binds twice per i (line 3).
    EXPECT_EQ(countOnLine(stmtStops(stops), 2), 3u);
    EXPECT_EQ(countOnLine(stmtStops(stops), 3), 6u);
    // Outer variable advances only after the inner one exhausts its range.
    EXPECT_EQ(stopsOnLines(stops, {2, 3}),
              (std::vector<Stop>{{2, false, false}, {3, false, false}, {3, false, false},
                                 {2, false, false}, {3, false, false}, {3, false, false},
                                 {2, false, false}, {3, false, false}, {3, false, false}}));
    // 6 iterations, each with one expr-level body-entry marker on the echo.
    EXPECT_EQ(std::count(stops.begin(), stops.end(), Stop{5, true, false}), 6);
}

// (4) intersection_for: ONE expr-level body-entry marker per full cartesian
// iteration and nothing per individual variable binding -- deliberately
// unlike (3), matching _resolve_intersection_for.
TEST(DebugHooksParity, IntersectionForMarksBodyEntryOnlyNotEachBinding) {
    EXPECT_EQ(recordStops("intersection_for(\ni=[0:1]\n)\ncube(1);\n"),
              (std::vector<Stop>{{1, false, false},
                                 {4, true, false},
                                 {4, false, false},
                                 {4, true, false},
                                 {4, false, false}}));
}

// (5) Statement-form let() { }: one statement-level stop per assignment.
TEST(DebugHooksParity, LetBlockStopsAtEachAssignment) {
    EXPECT_EQ(recordStops("let(a = 1, b = 2)\n  echo(a + b);\n"),
              (std::vector<Stop>{{1, false, false}, {1, false, false}, {2, false, false}}));
}

// (6) Ternary: statement-level on the whole ternary, then expr-level on the
// chosen branch.
TEST(DebugHooksParity, TernaryStopsOnConditionThenChosenBranch) {
    EXPECT_EQ(recordStops("a = true ? 1 : 2;\n"),
              (std::vector<Stop>{{1, false, false}, {1, false, false}, {1, true, false}}));
}

// (7) Expression-form let(): one statement-level stop per assignment, no
// expr-level body marker.
TEST(DebugHooksParity, LetExprStopsAtEachAssignment) {
    EXPECT_EQ(recordStops("a = let(x = 10, y = 20) x + y;\n"),
              (std::vector<Stop>{{1, false, false}, {1, false, false}, {1, false, false}}));
}

// (8)/(9) Expression-form echo()/assert(): one statement-level stop each.
TEST(DebugHooksParity, EchoAndAssertExpressionFormsEachAddOneStatementStop) {
    EXPECT_EQ(recordStops("a = echo(\"hi\") 42;\n"), (std::vector<Stop>{{1, false, false}, {1, false, false}}));
    EXPECT_EQ(recordStops("a = assert(true) 42;\n"), (std::vector<Stop>{{1, false, false}, {1, false, false}}));
}

// (10a) List-comp if: statement-level on the clause before its condition,
// then expr-level on the body when the condition passes. The body's
// expr-level stop appears TWICE (the clause's own marker plus the
// bare-element marker the body dispatch adds) -- the reference does the
// same; not a bug.
TEST(DebugHooksParity, ListCompIfStopsPerIterationAndOnlyEntersBodyWhenTrue) {
    // i in 0..2, condition true for 0 and 1 only.
    EXPECT_EQ(recordStops("a = [\n    for (i = [0:2])\n        if (i <= 1)\n            i * 10\n];\n"),
              (std::vector<Stop>{{1, false, false},
                                 {2, false, false}, {3, false, false}, {4, true, false}, {4, true, false},
                                 {2, false, false}, {3, false, false}, {4, true, false}, {4, true, false},
                                 {2, false, false}, {3, false, false}}));
}

// (10b) List-comp if/else: same, but always takes a branch.
TEST(DebugHooksParity, ListCompIfElseMarksWhicheverBranchWasChosen) {
    EXPECT_EQ(recordStops("a = [\n    for (i = [0:1])\n        if (i == 1)\n            99\n        else\n            i\n];\n"),
              (std::vector<Stop>{{1, false, false},
                                 {2, false, false}, {3, false, false}, {6, true, false}, {6, true, false},
                                 {2, false, false}, {3, false, false}, {4, true, false}, {4, true, false}}));
}

// (10c) List-comp let: per-assignment statement stops, and (unlike if/each)
// no expr-level marker of its own -- the body's bare-element check supplies
// the only expr-level stop.
TEST(DebugHooksParity, ListCompLetStopsAtAssignmentsWithNoOwnBodyMarker) {
    EXPECT_EQ(recordStops("a = [let(x=1) x];\n"),
              (std::vector<Stop>{{1, false, false}, {1, true, false}, {1, false, false}}));
}

// (10d) `each`: exactly one expr-level marker, before the body.
TEST(DebugHooksParity, ListCompEachFiresOneExprLevelMarker) {
    // 4 = assignment, the `each` marker, and the inner list's 2 elements.
    EXPECT_EQ(recordStops("a = [each [1,2]];\n"),
              (std::vector<Stop>{{1, false, false}, {1, true, false}, {1, true, false}, {1, true, false}}));
}

// (11) List-comp for() with two variables: per-binding statement stops, and
// -- unlike the statement for() in (3) -- NO separate body-entry marker.
TEST(DebugHooksParity, ListCompForStopsPerBindingWithNoSeparateBodyEntryMarker) {
    EXPECT_EQ(recordStops("x = [\nfor(\ni=[0:2],\nj=[0:1]\n)\ni*3+j\n];\n"),
              (std::vector<Stop>{{1, false, false},
                                 {3, false, false}, {4, false, false}, {6, true, false},
                                                    {4, false, false}, {6, true, false},
                                 {3, false, false}, {4, false, false}, {6, true, false},
                                                    {4, false, false}, {6, true, false},
                                 {3, false, false}, {4, false, false}, {6, true, false},
                                                    {4, false, false}, {6, true, false}}));
}

// (12) List-comp C-style for -- the most intricate case, and the one that
// previously got ZERO debug checks anywhere inside it. Exact per-iteration
// order: init(s) -> [cond(true) -> body -> incr(s)]* -> cond(false).
TEST(DebugHooksParity, ListCompCForStopOrderAcrossTwoIterations) {
    //   line 1  a = [
    //   line 2      for (            <- body-entry stop uses the CFor node
    //   line 3          i = 0;       <- init assignment
    //   line 4          i < 2;       <- condition (expr-level, every check)
    //   line 5          i = i + 1    <- incr assignment
    //   line 6      ) i              <- body expression
    //   line 7  ];
    EXPECT_EQ(recordStops("a = [\n    for (\n        i = 0;\n        i < 2;\n        i = i + 1\n    ) i\n];\n"),
              (std::vector<Stop>{{1, false, false},
                                 {3, false, false},                                    // init, once
                                 {4, true, false}, {2, false, false}, {6, true, false}, // iter 1
                                 {5, false, false},                                    // iter 1 incr
                                 {4, true, false}, {2, false, false}, {6, true, false}, // iter 2
                                 {5, false, false},
                                 {4, true, false}})); // final false check ends the loop
}

TEST(DebugHooksParity, ListCompCForFiresEveryInitAndIncrSeparately) {
    // 2 inits (once each) + 2 incrs per iteration x 2 iterations.
    const std::vector<Stop> stops = recordStops("a = [for (i = 0, j = 0; i < 2; i = i + 1, j = j + 2) i + j];\n");
    EXPECT_EQ(stmtStops(stops).size(), 9u); // outer assign + 2 inits + 2 body entries + 4 incrs
    EXPECT_EQ(stops.size(), 14u);           // + 3 condition checks + 2 body expressions
}

// (13) User function / function-literal call sites: a stop in the CALLER's
// context, before the callee's own body-entry stop.
//
// DELIBERATE DIVERGENCE from the reference, which reports this stop as
// statement-level. A call is not a statement of its own: when one sits
// inside an assignment, both checkpoints land on the same line at the same
// depth, so every consumer that treats statement-level stops as breakpoint
// hits fires twice per execution -- BelfrySCAD's debugger stuttered on
// Continue and reported the same loop iteration twice. Consumers cannot
// collapse it themselves: fast-continue skips the checkpoints in between,
// so a duplicate is indistinguishable from a genuine second visit.
//
// The stop itself is kept -- stepping into a call still needs it -- it is
// simply labelled for what it is.
TEST(DebugHooksParity, UserFunctionCallSiteStopsBeforeDescendingIntoTheCallee) {
    // line 2 (assignment), the call site as expression-level, then line 1
    // (body entry).
    EXPECT_EQ(recordStops("function double(x) = x * 2;\na = double(5);\n"),
              (std::vector<Stop>{{2, false, false}, {2, true, false}, {1, false, false}}));
    // The line a breakpoint would be set on is reached once, not twice.
    EXPECT_EQ(stmtStops(recordStops("function double(x) = x * 2;\na = double(5);\n")).size(), 2u);
}

TEST(DebugHooksParity, FunctionLiteralCallSiteAlsoStops) {
    EXPECT_EQ(recordStops("f = function(x) x * 2;\na = f(5);\n"),
              (std::vector<Stop>{{1, false, false}, {2, false, false}, {2, true, false}, {1, false, false}}));
}

TEST(DebugHooksParity, BuiltinFunctionCallGetsNoCallSiteStop) {
    // Only the assignment is statement-level; the 3 expr-level stops are
    // the argument list literal's own elements, not a call-site stop.
    const std::vector<Stop> stops = recordStops("a = len([1,2,3]);\n");
    EXPECT_EQ(stmtStops(stops), (std::vector<Stop>{{1, false, false}}));
    EXPECT_EQ(stops.size(), 4u);
}

// A modifier's wrapped child gets its own statement-level stop, in addition
// to the modifier node's own -- `#cube(1);` pauses twice, not once.
TEST(DebugHooksParity, ModifierAndItsWrappedChildBothStop) {
    EXPECT_EQ(recordStops("#cube(1);\n"), (std::vector<Stop>{{1, false, false}, {1, false, false}}));
}
