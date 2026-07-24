#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

TEST(DebugHooks, HookFiresOnceForEachStatementAtEveryNestingLevel) {
    // 4, not 3: `translate(...) sphere(...)` is two statements from
    // checkDebug's perspective -- the top-level translate() ModularCall
    // itself (checked by the outer evalChildren) AND its nested sphere()
    // child (checked again when resolveTransform's own resolve function
    // calls evalChildren on its .children) -- see evalChildren's own doc
    // comment on why this is the port's single, but recursively-applied,
    // statement-level checkpoint.
    int calls = 0;
    DebugHooks hooks;
    hooks.debugHook = [&](int, int, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
        ++calls;
        return DebugAction{};
    };
    Evaluator ev(EchoFn{}, nullptr, nullptr, hooks);
    auto ast = parseSrc("cube(1);\ntranslate([1,0,0]) sphere(r=1,$fn=8);\necho(\"x\");");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    ev.evaluate(ast, ctx);
    EXPECT_EQ(calls, 4);
}

TEST(DebugHooks, StopAbortsEvaluation) {
    DebugHooks hooks;
    hooks.debugHook = [](int line, int, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
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
    hooks.debugHook = [](int line, int, bool, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
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
    hooks.debugHook = [&](int line, int depth, bool, const std::string&, const std::vector<CallStackFrame>&,
                           const DebugFramesFn& getFrame) {
        if (line == 3) { // the `cube([w, h, 3]);` statement inside module bracket
            hit.line = line;
            hit.depth = depth;
            DebugFrame frame = getFrame();
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
    hooks.debugHook = [&](int, int, bool forced, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
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
    hooks.debugHook = [&](int, int, bool forced, const std::string&, const std::vector<CallStackFrame>&, const DebugFramesFn&) {
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
