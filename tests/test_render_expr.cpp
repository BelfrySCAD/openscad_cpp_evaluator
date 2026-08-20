// `render()` in EXPRESSION position -- `obj = render() { cube(10); };`
//
// It measures and returns; it never draws. Most of these tests therefore
// assert on TWO things at once: the object's contents, and that the drawn
// body count is unchanged.

#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

// Captures echo output; every measurement assertion reads it back, since
// the object only exists inside the script.
struct Measured {
    Evaluated e;
    std::vector<std::string> echoes;
};

Measured runScript(const std::string& code) {
    std::vector<std::string> echoes;
    // The lambda outlives this call inside the Evaluator, so it captures a
    // pointer to storage the caller keeps.
    auto captured = std::make_shared<std::vector<std::string>>();
    Evaluated e = evalSrc(code, [captured](const std::string& m) { captured->push_back(m); });
    return Measured{std::move(e), *captured};
}

size_t drawnBodies(const Evaluated& e) {
    size_t n = 0;
    for (const ColoredBody& b : e.bodies) {
        if ((b.body && !b.body->IsEmpty()) || b.section || b.isDisplayOnly()) ++n;
    }
    return n;
}

} // namespace

// -- Measurements against known solids ------------------------------------

TEST(RenderExpr, MeasuresACube) {
    Measured r = runScript("o = render() { cube(10); };\n"
                "echo(o.volume, o.area, o.genus, o.dim);\n"
                "echo(o.boundingbox);\n"
                "echo(len(o.vertices), len(o.faces));");
    ASSERT_EQ(r.echoes.size(), 3u);
    EXPECT_EQ(r.echoes[0], "ECHO: 1000, 600, 0, 3");
    EXPECT_EQ(r.echoes[1], "ECHO: [[0, 0, 0], [10, 10, 10]]");
    // 8 and 12, not 24 and 12: the exact-position weld ran. Without it the
    // round-trip below would build an OPEN mesh.
    EXPECT_EQ(r.echoes[2], "ECHO: 8, 12");
}

TEST(RenderExpr, MeasuresABooleanResult) {
    Measured r = runScript("o = render() { difference() { cube(10); cube(5); } };\necho(o.volume, o.genus);");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: 875, 0");
}

TEST(RenderExpr, ReportsGenusOfATorus) {
    // genus is the one measurement no other code path in this repo produces,
    // so it gets its own case. A torus is genus 1 however coarsely faceted.
    Measured r = runScript("o = render() { rotate_extrude($fn=24) translate([5,0]) circle(1,$fn=12); };\necho(o.genus, o.dim);");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: 1, 3");
}

TEST(RenderExpr, HonoursDollarVariableArguments) {
    Measured coarse = runScript("o = render($fn=8) { sphere(10); };\necho(len(o.faces));");
    Measured fine = runScript("o = render($fn=64) { sphere(10); };\necho(len(o.faces));");
    ASSERT_EQ(coarse.echoes.size(), 1u);
    ASSERT_EQ(fine.echoes.size(), 1u);
    EXPECT_NE(coarse.echoes[0], fine.echoes[0]) << "$fn did not reach the children";
}

TEST(RenderExpr, AcceptsAndIgnoresConvexity) {
    Measured r = runScript("o = render(convexity=4) { cube(2); };\necho(o.volume);");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: 8");
}

// -- The mesh round-trips -------------------------------------------------

TEST(RenderExpr, VnfRoundTripsThroughPolyhedron) {
    // THE winding test. A reversed mesh still builds -- Manifold reports
    // Status()==NoError -- but its volume comes back NEGATIVE. So assert a
    // POSITIVE volume, never abs(): the sign is the whole signal.
    Measured r = runScript("o = render() { sphere(10, $fn=16); };\npolyhedron(o.vertices, o.faces);");
    ASSERT_EQ(drawnBodies(r.e), 1u);
    const ColoredBody& b = r.e.bodies.front();
    ASSERT_TRUE(b.body.has_value());
    EXPECT_EQ(b.body->Status(), manifold::Manifold::Error::NoError) << "open mesh -- the vertex weld regressed";
    EXPECT_GT(b.body->Volume(), 0.0) << "NEGATIVE volume -- face winding is reversed";

    // Fidelity: against the SAME sphere drawn directly, not against echo
    // output (which rounds to 6 significant digits).
    Evaluated direct = evalSrc("sphere(10, $fn=16);");
    ASSERT_EQ(drawnBodies(direct), 1u);
    EXPECT_NEAR(b.body->Volume(), direct.bodies.front().body->Volume(), 1e-9);
    EXPECT_EQ(b.body->NumTri(), direct.bodies.front().body->NumTri());
}

TEST(RenderExpr, PolyhedronAcceptsTheObjectDirectly) {
    Measured r = runScript("o = render() { difference() { cube(10); cube(5); } };\npolyhedron(o);");
    ASSERT_EQ(drawnBodies(r.e), 1u);
    const ColoredBody& b = r.e.bodies.front();
    ASSERT_TRUE(b.body.has_value());
    EXPECT_EQ(b.body->Status(), manifold::Manifold::Error::NoError);
    EXPECT_NEAR(b.body->Volume(), 875.0, 1e-6);
}

TEST(RenderExpr, PolyhedronAcceptsAnyObjectWithTheKeys) {
    // Nothing about this is render()-specific -- a script can build its own.
    Measured r = runScript("v = [[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]];\n"
                "f = [[0,1,2,3],[7,6,5,4],[0,4,5,1],[1,5,6,2],[2,6,7,3],[3,7,4,0]];\n"
                "polyhedron(object(vertices=v, faces=f));");
    ASSERT_EQ(drawnBodies(r.e), 1u);
    EXPECT_NEAR(r.e.bodies.front().body->Volume(), 1.0, 1e-9);
}

TEST(RenderExpr, PolyhedronNamesTheMissingKey) {
    EXPECT_THROW(evalSrc("polyhedron(object(vertices=[[0,0,0]]));"), std::exception);
    EXPECT_THROW(evalSrc("polyhedron(object(faces=[[0,1,2]]));"), std::exception);
}

TEST(RenderExpr, ExposesVnfAsTheTwoList) {
    // BOSL2 functions take the 2-list, not two arguments.
    Measured r = runScript("o = render() { cube(3); };\necho(len(o.vnf), o.vnf[0] == o.vertices, o.vnf[1] == o.faces);");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: 2, true, true");
}

// -- 2D -------------------------------------------------------------------

TEST(RenderExpr, MeasuresA2dShape) {
    Measured r = runScript("o = render() { square([4,3]); };\n"
                "echo(o.dim, o.area, o.perimeter);\n"
                "echo(o.boundingbox);\n"
                "echo(len(o.vertices), len(o.paths));");
    ASSERT_EQ(r.echoes.size(), 3u);
    EXPECT_EQ(r.echoes[0], "ECHO: 2, 12, 14");
    EXPECT_EQ(r.echoes[1], "ECHO: [[0, 0], [4, 3]]");
    EXPECT_EQ(r.echoes[2], "ECHO: 4, 1");
}

TEST(RenderExpr, TwoDShapeWithAHoleHasTwoPaths) {
    // The inner square must be strictly INSIDE: at the origin it would cut a
    // corner notch, which is still a single contour.
    Measured r = runScript("o = render() { difference() { square(10); translate([4,4]) square(2); } };\n"
                            "echo(o.area, len(o.paths));");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: 96, 2");
}

TEST(RenderExpr, PolygonAcceptsTheObjectDirectly) {
    Measured r = runScript("o = render() { difference() { square(10); translate([4,4]) square(2); } };\npolygon(o);");
    ASSERT_EQ(drawnBodies(r.e), 1u);
    ASSERT_TRUE(r.e.bodies.front().section.has_value());
    EXPECT_NEAR(r.e.bodies.front().section->Area(), 96.0, 1e-9);
}

// -- Degenerate input -----------------------------------------------------

TEST(RenderExpr, EmptyGeometryYieldsDimZeroAndUndefBounds) {
    // boundingbox must be undef, NOT Manifold's empty Box -- that is
    // {+inf, -inf} and would poison any arithmetic downstream.
    for (const char* src : {"o = render() { };", "o = render() { if (false) cube(1); };"}) {
        Measured r = runScript(std::string(src) + "\necho(o.dim, o.volume, o.boundingbox, len(o.vertices));");
        ASSERT_EQ(r.echoes.size(), 1u) << src;
        EXPECT_EQ(r.echoes[0], "ECHO: 0, 0, undef, 0") << src;
    }
}

// -- The headline property: nothing is drawn ------------------------------

TEST(RenderExpr, DrawsNothing) {
    Measured r = runScript("o = render() { cube(100); };\ncube(1);");
    ASSERT_EQ(drawnBodies(r.e), 1u) << "the measured cube(100) leaked into the model";
    EXPECT_NEAR(r.e.bodies.front().body->Volume(), 1.0, 1e-9);
}

TEST(RenderExpr, DrawsNothingFromAnyContext) {
    // Same rule everywhere -- no special case for functions, loops, or
    // branches, precisely because there are no side effects to sequence.
    const char* cases[] = {
        "module m() { o = render() { cube(100); }; } m();",
        "function f() = render() { cube(100); }.volume; x = f();",
        "v = [for (i = [0:2]) render() { cube(i + 1); }];",
        "x = true ? render() { cube(100); }.volume : 0;",
        "o = render() { cube(100); };",
    };
    for (const char* src : cases) {
        Measured r = runScript(std::string(src) + "\ncube(1);");
        EXPECT_EQ(drawnBodies(r.e), 1u) << src;
    }
}

TEST(RenderExpr, EvaluatesOncePerEvaluationInAComprehension) {
    Measured r = runScript("v = [for (i = [0:2]) render() { cube(i + 1); }];\necho(len(v), v[0].volume, v[2].volume);");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: 3, 1, 27");
}

TEST(RenderExpr, WorksInsideAFunctionBody) {
    // Function purity is preserved because nothing is drawn -- this is the
    // use case that a draw-and-measure design would have had to forbid.
    Measured r = runScript("function fits(w) = render() { cube(w); }.volume < 500;\necho(fits(5), fits(10));");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: true, false");
}

// -- Provenance is not polluted -------------------------------------------

TEST(RenderExpr, LeavesNoProvenanceEntriesBehind) {
    // Discarded geometry must not register originalID -> AST node entries:
    // those tables are cleared once per pass, so a leak here is permanent
    // and shows up much later as wrong click-to-source. This fails if ANY
    // of the four guards in csg_generate.cpp is missed.
    Evaluated without = evalSrc("sphere(5);");
    Evaluated with = evalSrc("o = render() { cube(10); };\nsphere(5);");
    EXPECT_EQ(with.ev.idToNode.size(), without.ev.idToNode.size());
    EXPECT_EQ(with.ev.idToColor.size(), without.ev.idToColor.size());
}

TEST(RenderExpr, RestoresItsInternalStateAfterAThrow) {
    // A throw inside the measured children must unwind the pushed
    // treeStack_ frame AND clear measuring_, or every later resolve
    // accumulates into a dead level and provenance stays suppressed.
    Evaluated e{parseSrc("cube(1);"), nullptr, Evaluator(), {}, {}};
    e.scope = oscad::buildScopes(e.ast);
    const size_t depthBefore = e.ev.treeStackDepthForTesting();
    EXPECT_FALSE(e.ev.measuringForTesting());

    EXPECT_THROW(evalSrc("o = render() { assert(false); cube(1); };"), std::exception);
    EXPECT_THROW(evalSrc("translate([5,0,0]) { o = render() { assert(false); }; }"), std::exception);

    // And the machine still works afterwards.
    Measured after = runScript("o = render() { cube(3); };\necho(o.volume);");
    EXPECT_EQ(after.echoes.size(), 1u);
    EXPECT_EQ(after.echoes[0], "ECHO: 27");
    EXPECT_EQ(after.e.ev.treeStackDepthForTesting(), depthBefore);
    EXPECT_FALSE(after.e.ev.measuringForTesting());
}

// -- The statement form is untouched --------------------------------------

TEST(RenderExpr, StatementFormStillDraws) {
    Evaluated e = evalSrc("render() cube(2);");
    ASSERT_EQ(drawnBodies(e), 1u);
    EXPECT_NEAR(e.bodies.front().body->Volume(), 8.0, 1e-9);
}

TEST(RenderExpr, MeasuredSubtreeUsesItsOwnCoordinates) {
    // An enclosing transform applies to what is DRAWN, not to what a
    // render() expression measures -- the expression resolves its own
    // children in its own frame.
    Measured r = runScript("translate([5,0,0]) { o = render() { cube(2); }; cube(o.volume); }\n"
                "echo(o.boundingbox);");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: [[0, 0, 0], [2, 2, 2]]");
}

// -- Both engines, same answers -------------------------------------------
//
// The VM compiles a render expression to a real Kind::Measure bracket
// (Op::PushBuiltinWrap/PopBuiltinWrap) rather than declining to compile and
// letting the interpreter take over -- the whole containing declaration
// would otherwise run interpreted, which is many times slower.

namespace {

class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

std::vector<std::string> echoesUnder(bool vm, const std::string& code) {
    ScopedVm guard(vm);
    return runScript(code).echoes;
}

} // namespace

TEST(RenderExprEngines, InterpreterAndVmAgree) {
    const char* cases[] = {
        "o = render() { cube(10); };\necho(o.volume, o.area, o.genus, o.dim, o.boundingbox);",
        "o = render() { difference() { cube(10); cube(5); } };\necho(o.volume, len(o.vertices), len(o.faces));",
        "o = render($fn=16) { sphere(10); };\necho(o.volume, len(o.faces));",
        "o = render() { square([4,3]); };\necho(o.dim, o.area, o.perimeter, o.boundingbox);",
        "o = render() { };\necho(o.dim, o.boundingbox);",
        "function f(w) = render() { cube(w); }.volume;\necho(f(2), f(3));",
        "module m() { o = render() { cube(4); }; echo(o.volume); } m();",
        "v = [for (i = [1:3]) render() { cube(i); }.volume];\necho(v);",
        "x = true ? render() { cube(2); }.volume : 0;\necho(x);",
        "g = function(a) a + render() { cube(a); }.volume;\necho(g(2));",
        "o = render() { translate([5,0,0]) cube(2); };\necho(o.boundingbox);",
        "o = render() { render() { cube(3); } };\necho(o.volume);",
    };
    for (const char* src : cases) {
        EXPECT_EQ(echoesUnder(false, src), echoesUnder(true, src)) << "engines diverge for:\n" << src;
    }
}

TEST(RenderExprEngines, DrawsNothingUnderEitherEngine) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        Evaluated e = evalSrc("function f(w) = render() { cube(w); }.volume;\nx = f(9);\ncube(1);");
        ASSERT_EQ(drawnBodies(e), 1u) << "vm=" << vm;
        EXPECT_NEAR(e.bodies.front().body->Volume(), 1.0, 1e-9) << "vm=" << vm;
    }
}

TEST(RenderExprEngines, TheDeclarationActuallyCompiles) {
    // The regression guard for "someone reintroduced a NotCompilable bail".
    // If compileExpr ever declines a RenderExpression again, these chunks
    // come back null and the containing declaration runs interpreted.
    ScopedVm guard(true);
    Evaluated e = evalSrc("function f(w) = render() { cube(w); }.volume;\n"
                          "module m() { o = render() { cube(2); }; }\n"
                          "x = f(3);\nm();");
    const oscad::FunctionDeclaration* fn = nullptr;
    const oscad::ModuleDeclaration* mod = nullptr;
    for (const auto& n : e.ast) {
        if (auto* d = dynamic_cast<const oscad::FunctionDeclaration*>(n.get())) fn = d;
        if (auto* d = dynamic_cast<const oscad::ModuleDeclaration*>(n.get())) mod = d;
    }
    ASSERT_NE(fn, nullptr);
    ASSERT_NE(mod, nullptr);
    EXPECT_NE(e.ev.lookupOrCompileChunk(*fn), nullptr) << "function containing render() fell back to the interpreter";
    EXPECT_NE(e.ev.lookupOrCompileModuleChunk(*mod), nullptr) << "module containing render() fell back to the interpreter";
}

TEST(RenderExprEngines, CapturedLocalsReachTheChildren) {
    // A compiled function keeps parameters and lets in frame SLOTS, which
    // the children's EvalContext cannot see -- Kind::Measure republishes
    // them. Without that, every one of these measures an undef-sized shape.
    ScopedVm guard(true);
    Measured r = runScript("function f(w) = render() { cube(w); }.volume;\n"
                            "function g(a, b) = let (s = a * b) render() { cube(s); }.volume;\n"
                            "echo(f(2), f(3), g(2, 2));");
    ASSERT_EQ(r.echoes.size(), 1u);
    EXPECT_EQ(r.echoes[0], "ECHO: 8, 27, 64");
}

TEST(RenderExprEngines, UnwindsCleanlyUnderTheVm) {
    // Both nesting orders, plus a throw originating DEEP inside a nested
    // module frame so teardownVmCallStackDownTo's multi-frame loop runs --
    // that is the path where a mismatched pop would corrupt treeStack_.
    ScopedVm guard(true);
    const char* throwing[] = {
        "o = render() { assert(false); cube(1); };",
        "o = render() { translate([1,0,0]) { assert(false); } };",
        "translate([5,0,0]) { o = render() { assert(false); }; }",
        "module deep(n) { if (n > 0) deep(n - 1); else assert(false); }\no = render() { deep(20); };",
        "function f(w) = render() { assert(false); cube(w); }.volume;\nx = f(2);",
    };
    for (const char* src : throwing) {
        EXPECT_THROW(evalSrc(src), std::exception) << src;
    }
    // The machine still works, and both invariants are back where they started.
    Measured after = runScript("o = render() { cube(3); };\necho(o.volume);");
    ASSERT_EQ(after.echoes.size(), 1u);
    EXPECT_EQ(after.echoes[0], "ECHO: 27");
    EXPECT_EQ(after.e.ev.treeStackDepthForTesting(), 0u);
    EXPECT_FALSE(after.e.ev.measuringForTesting());
}

TEST(RenderExprEngines, ProvenanceStaysCleanUnderTheVm) {
    // The RestoreMeasuring guard in Op::PopBuiltinWrap must wrap the
    // generate, not precede it. Restoring measuring_ early leaves the
    // provenance guards inert on the VM path ONLY -- no crash, no wrong
    // geometry, just silently wrong click-to-source. This is the only test
    // that catches that.
    ScopedVm guard(true);
    Evaluated without = evalSrc("sphere(5);");
    Evaluated with = evalSrc("function f(w) = render() { cube(w); }.volume;\nx = f(9);\nsphere(5);");
    EXPECT_EQ(with.ev.idToNode.size(), without.ev.idToNode.size());
    EXPECT_EQ(with.ev.idToColor.size(), without.ev.idToColor.size());
}
