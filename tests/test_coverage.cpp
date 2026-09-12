#include "openscad_cpp_evaluator/coverage.hpp"
#include "openscad_cpp_evaluator/eval_use.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_parser/api.hpp"

#include "test_helpers.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <set>

namespace oscadeval {
using namespace test;
namespace {

struct ScopedVm {
    explicit ScopedVm(bool on) { Evaluator::setBytecodeVmEnabledForTesting(on); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

CoverageResult cover(const std::string& code, std::vector<std::string>* log = nullptr) {
    auto ast = parseSrc(code);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    Evaluator ev([&](const std::string& m) { if (log) log->push_back(m); }, nullptr, nullptr, {}, false, /*coverage=*/true);
    ev.evaluate(ast, ctx, {}, /*generate=*/false);
    EXPECT_TRUE(ev.coverageResult.has_value());
    return ev.coverageResult.value_or(CoverageResult{});
}

// hits of the first span of `kind` on `line`, or -1 if the line has none.
long hits(const CoverageResult& r, int line, CoverageKind kind, bool arm = false) {
    for (const CoverageSpan& s : r.spans) {
        if (s.line == line && s.kind == kind && (!arm || s.arm)) return static_cast<long>(s.hits);
    }
    return -1;
}
long branchHits(const CoverageResult& r, int line, int column) {
    for (const CoverageSpan& s : r.spans) {
        if (s.line == line && s.column == column && s.kind == CoverageKind::Branch) return static_cast<long>(s.hits);
    }
    return -1;
}
size_t count(const CoverageResult& r, CoverageKind kind) {
    size_t n = 0;
    for (const CoverageSpan& s : r.spans) n += s.kind == kind;
    return n;
}

} // namespace

TEST(Coverage, OffByDefaultAndEmptyWhenOff) {
    Evaluated e = evalSrc("cube(1);");
    EXPECT_FALSE(e.ev.coverageEnabled());
    EXPECT_FALSE(e.ev.coverageResult.has_value());
}

TEST(Coverage, StatementsAndIfArms) {
    const CoverageResult r = cover(
        "a = 1;\n"                 // 1
        "if (a > 0) {\n"           // 2
        "    cube(1);\n"           // 3  taken arm
        "} else {\n"
        "    sphere(1);\n"         // 5  untaken arm
        "}\n"
        "for (i = [0:2]) cube(i + 1);\n"); // 7  body runs three times
    EXPECT_EQ(hits(r, 1, CoverageKind::Statement), 1);
    EXPECT_EQ(hits(r, 2, CoverageKind::Statement), 1);
    EXPECT_EQ(hits(r, 3, CoverageKind::Statement, /*arm=*/true), 1);
    EXPECT_EQ(hits(r, 5, CoverageKind::Statement, /*arm=*/true), 0);
    EXPECT_EQ(hits(r, 7, CoverageKind::Statement), 1) << "the for itself";
    // Two statements on line 7: the for, then its body. The body is the
    // second Statement on the line and ran once per iteration.
    long bodyHits = -1;
    int seen = 0;
    for (const CoverageSpan& s : r.spans) {
        if (s.line == 7 && s.kind == CoverageKind::Statement && ++seen == 2) bodyHits = static_cast<long>(s.hits);
    }
    EXPECT_EQ(bodyHits, 3);
}

TEST(Coverage, TernaryArmsAndShortCircuit) {
    const CoverageResult r = cover(
        "function f(x) = x > 0 ? 1 : 2;\n"       // 1  false arm never taken
        "a = f(1);\n"
        "b = false && f(2);\n"                    // 3  right operand skipped
        "c = true || f(3);\n"                     // 4  right operand skipped
        "d = true && f(4);\n");                   // 5  right operand runs
    EXPECT_EQ(hits(r, 1, CoverageKind::Body), 2) << "f entered by a and d";
    EXPECT_EQ(branchHits(r, 1, 25), 2) << "true arm `1`";
    EXPECT_EQ(branchHits(r, 1, 29), 0) << "false arm `2`";
    EXPECT_EQ(hits(r, 3, CoverageKind::Branch), 0);
    EXPECT_EQ(hits(r, 4, CoverageKind::Branch), 0);
    EXPECT_EQ(hits(r, 5, CoverageKind::Branch), 1);
}

TEST(Coverage, BodiesForModulesFunctionsAndLiterals) {
    const CoverageResult r = cover(
        "module used() { cube(1); }\n"            // 1
        "module unused() { sphere(1); }\n"        // 2
        "function g() = 3;\n"                     // 3  never called
        "h = function(x) x * 2;\n"                // 4  literal, called twice
        "used();\n"
        "k = [h(1), h(2)];\n");
    EXPECT_EQ(hits(r, 1, CoverageKind::Body), 1);
    EXPECT_EQ(hits(r, 1, CoverageKind::Statement), 1) << "the cube inside used()";
    EXPECT_EQ(hits(r, 2, CoverageKind::Body), 0);
    EXPECT_EQ(hits(r, 2, CoverageKind::Statement), 0) << "the sphere inside unused()";
    EXPECT_EQ(hits(r, 3, CoverageKind::Body), 0);
    EXPECT_EQ(hits(r, 4, CoverageKind::Body), 2);
}

TEST(Coverage, ComprehensionArms) {
    const CoverageResult r = cover(
        "a = [for (i = [0:3]) if (i % 2 == 0) i else -i];\n"   // both arms
        "b = [for (i = [0:3]) if (i > 10) i];\n");             // arm never taken
    EXPECT_EQ(count(r, CoverageKind::Branch), 3u);
    long taken = 0, skipped = 0;
    for (const CoverageSpan& s : r.spans) {
        if (s.kind != CoverageKind::Branch) continue;
        if (s.line == 1) taken += s.hits;
        if (s.line == 2) skipped += s.hits;
    }
    EXPECT_EQ(taken, 4) << "two even, two odd";
    EXPECT_EQ(skipped, 0);
}

TEST(Coverage, ModifiedStatementIsOneStatement) {
    const CoverageResult r = cover("%cube(1);\n#sphere(1);\n*cylinder(1);\n");
    EXPECT_EQ(count(r, CoverageKind::Statement), 3u);
    EXPECT_EQ(hits(r, 1, CoverageKind::Statement), 1);
    EXPECT_EQ(hits(r, 2, CoverageKind::Statement), 1);
    EXPECT_EQ(hits(r, 3, CoverageKind::Statement), 1) << "a disabled statement is still visited";
}

// The VM and the interpreter must agree on what ran. Compared as sets of
// (line, column, kind, ran?), not hit counts: the two paths may legitimately
// checkpoint a hot node a different number of times.
TEST(Coverage, VmAndInterpreterAgree) {
    const std::string code =
        "function f(x) = x > 0 ? (x > 5 ? \"big\" : \"small\") : \"neg\";\n"
        "function g(v) = is_list(v) && len(v) > 0 && v[0] == 1;\n"
        "function h(v) = is_undef(v) || v == 0;\n"
        "function sq(n) = [for (i = [0:n]) if (i % 2 == 0) i * i else let(j = i) -j];\n"
        "module m(k) { if (k > 1) { cube(k); } else { sphere(k); } for (i = [0:k]) cube(i + 1); }\n"
        "a = [f(1), f(9), g([1]), g([]), h(undef), sq(4)];\n"
        "m(2);\n"
        "echo(a);\n";
    std::set<std::tuple<int, int, int, bool>> vmSet, interpSet;
    {
        ScopedVm vm(true);
        for (const CoverageSpan& s : cover(code).spans) vmSet.insert({s.line, s.column, (int)s.kind, s.hits > 0});
    }
    {
        ScopedVm vm(false);
        for (const CoverageSpan& s : cover(code).spans) interpSet.insert({s.line, s.column, (int)s.kind, s.hits > 0});
    }
    EXPECT_EQ(vmSet.size(), interpSet.size());
    for (const auto& t : vmSet) {
        EXPECT_TRUE(interpSet.count(t)) << "VM covered (" << std::get<0>(t) << ":" << std::get<1>(t) << ", kind "
                                        << std::get<2>(t) << ", ran=" << std::get<3>(t) << ") but the interpreter disagrees";
    }
    for (const auto& t : interpSet) {
        EXPECT_TRUE(vmSet.count(t)) << "interpreter covered (" << std::get<0>(t) << ":" << std::get<1>(t) << ", kind "
                                    << std::get<2>(t) << ", ran=" << std::get<3>(t) << ") but the VM disagrees";
    }
}

// A tail call hops the frame instead of pushing one, so it never reaches
// enterUserCall -- where the body hit is recorded. A function reached ONLY
// in tail position therefore reported its body as never run while its own
// ternary arms reported hits, which reads on screen as "the condition is
// uncovered but both branches are covered". BOSL2's _str_split_recurse is
// the shape that shows it: recursion through a ternary arm is a tail call
// every time.
TEST(Coverage, TailCalledBodyIsCounted) {
    const std::string code =
        "function inner(i) = i == 0 ? \"done\" : inner(i-1);\n"
        "function tail_caller(i) = inner(i);\n"
        "echo(tail_caller(2));\n";
    for (bool useVm : {true, false}) {
        ScopedVm vm(useVm);
        const CoverageResult r = cover(code);
        std::uint32_t innerBody = 0, arms = 0;
        for (const CoverageSpan& s : r.spans) {
            if (s.kind == CoverageKind::Body && s.line == 1) innerBody = s.hits;
            if (s.kind == CoverageKind::Branch && s.line == 1) arms += s.hits;
        }
        EXPECT_GT(innerBody, 0u) << (useVm ? "VM" : "interpreter")
                                 << ": a body reached only by tail calls still ran";
        EXPECT_EQ(innerBody, arms) << (useVm ? "VM" : "interpreter")
                                   << ": every entry takes exactly one arm";
    }
}

TEST(Coverage, PerFilePercentages) {
    const CoverageResult r = cover(
        "function f(x) = x > 0 ? 1 : 2;\n"   // body hit; arms: 1 of 2
        "module unused() { cube(1); }\n"    // body 0, its statement 0
        "a = f(1);\n"                        // statement hit
        "if (a == 1) sphere(1); else cube(1);\n"); // if hit, arms: 1 of 2
    ASSERT_EQ(r.files.size(), 1u);
    const CoverageFileSummary& f = r.files[0];
    EXPECT_EQ(f.bodies, 2u);
    EXPECT_EQ(f.bodies_hit, 1u);
    EXPECT_EQ(f.statements, 5u) << "cube in unused, a, if, sphere arm, cube arm";
    EXPECT_EQ(f.statements_hit, 3u);
    EXPECT_EQ(f.branches, 4u) << "two ternary arms + two if arms";
    EXPECT_EQ(f.branches_hit, 2u);
    EXPECT_EQ(f.spans, 9u);
    EXPECT_EQ(f.spans_hit, 5u);
    EXPECT_NEAR(f.percent(), 100.0 * 5 / 9, 1e-9);
    EXPECT_NEAR(f.statement_percent(), 60.0, 1e-9);
    EXPECT_NEAR(f.branch_percent(), 50.0, 1e-9);
    EXPECT_NEAR(f.body_percent(), 50.0, 1e-9);
    EXPECT_EQ(r.total.spans, f.spans);
    EXPECT_EQ(r.total.spans_hit, f.spans_hit);
    CoverageFileSummary empty;
    EXPECT_EQ(empty.percent(), 100.0) << "nothing to cover is fully covered, not a divide by zero";
}

TEST(Coverage, IncludedAndUsedFilesCarryTheirOwnOrigin) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "oscad_cov_test";
    fs::create_directories(dir);
    const fs::path inc = dir / "inc.scad", used = dir / "used.scad", main = dir / "main.scad";
    std::ofstream(inc) << "function inc_fn(x) = x + 1;\nfunction inc_unused() = 0;\n";
    std::ofstream(used) << "used_global = 5;\nfunction used_fn() = used_global;\nfunction used_unused() = 1;\n";
    std::ofstream(main) << "include <" << inc.string() << ">\nuse <" << used.string() << ">\n"
                        << "echo(inc_fn(1), used_fn());\n";
    auto ast = oscad::getASTFromFile(main.string());
    ResolvedUseScopes uses = resolveUseScopes(ast, main.string(), {});
    Evaluator ev({}, nullptr, nullptr, {}, false, /*coverage=*/true);
    ev.setUsedFileGlobals(uses.usedFileGlobals);
    EvalContext ctx = EvalContext::makeRoot(uses.rootScope.get());
    ev.evaluate(uses.processedNodes, ctx, {}, false);
    ASSERT_TRUE(ev.coverageResult.has_value());
    std::map<std::pair<std::string, int>, std::uint32_t> bodies;
    std::map<std::string, std::uint32_t> statements;
    for (const CoverageSpan& s : ev.coverageResult->spans) {
        const std::string file = fs::path(s.origin).filename().string();
        if (s.kind == CoverageKind::Body) bodies[{file, s.line}] = s.hits;
        if (s.kind == CoverageKind::Statement) statements[file] += 1;
    }
    const auto body = [&](const char* file, int line) { return bodies[std::make_pair(std::string(file), line)]; };
    EXPECT_EQ(body("inc.scad", 1), 1u);
    EXPECT_EQ(body("inc.scad", 2), 0u);
    EXPECT_EQ(body("used.scad", 2), 1u);
    EXPECT_EQ(body("used.scad", 3), 0u);
    EXPECT_EQ(statements["used.scad"], 1u) << "the used file's own global is in the universe";
    for (const CoverageSpan& s : ev.coverageResult->spans) {
        if (s.kind == CoverageKind::Statement && fs::path(s.origin).filename() == "used.scad") {
            EXPECT_EQ(s.hits, 1u) << "and it ran, eagerly, when used_fn() first read it";
        }
    }
    std::map<std::string, double> pct;
    for (const CoverageFileSummary& f : ev.coverageResult->files) pct[fs::path(f.origin).filename().string()] = f.body_percent();
    EXPECT_EQ(pct.size(), 3u) << "main, include and used file each get a summary";
    EXPECT_NEAR(pct["inc.scad"], 50.0, 1e-9);
    EXPECT_NEAR(pct["used.scad"], 50.0, 1e-9);
    fs::remove_all(dir);
}

} // namespace oscadeval
