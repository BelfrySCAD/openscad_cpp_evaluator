// In-process tests for the CLI (tools/cli/cli_lib.hpp) and its --debug REPL
// (debug_repl.hpp) -- ported from the reference's own test_cli.py, using
// runCli() directly (no subprocess spawned) exactly the way that file uses
// cli.main() directly with monkeypatched input()/capsys.

#include "cli_lib.hpp"

#include "openscad_cpp_evaluator/debug_repl.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>

using namespace oscadeval;

namespace {

const char* kCubeScript = "cube([10, 10, 10]);\n";
const char* kModuleScript = "width = 10;\n"
                            "cube([width, width, width]);\n"
                            "echo(\"hi\");\n";
// A function and a module deliberately sharing the name "fib" -- for
// "list fib" ambiguity tests.
const char* kAmbiguousNameScript = "function fib(n) = n < 2 ? n : fib(n-1) + fib(n-2);\n"
                                    "module thing(x) {\n"
                                    "    cube(x);\n"
                                    "}\n"
                                    "module fib() {\n"
                                    "    sphere(1);\n"
                                    "}\n"
                                    "thing(1);\n";

std::filesystem::path writeScript(const std::string& name, const std::string& text) {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / ("oscad_cli_test_" + name);
    std::ofstream out(p);
    out << text;
    return p;
}

std::string readFile(const std::filesystem::path& p) {
    std::ifstream in(p);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Feeds `responses` into runCli() as canned REPL input lines, mirroring
// the reference's own _feed_input helper.
std::istringstream feedInput(const std::vector<std::string>& responses) {
    std::string joined;
    for (const auto& r : responses) joined += r + "\n";
    return std::istringstream(joined);
}

} // namespace

// -- export formats -------------------------------------------------------

TEST(CliExportFormats, StlExport) {
    auto src = writeScript("cube.scad", kCubeScript);
    auto out = src.parent_path() / "cube_out.stl";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 0);
    ASSERT_TRUE(std::filesystem::exists(out));
    EXPECT_GT(std::filesystem::file_size(out), 0u);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliExportFormats, ObjExport) {
    auto src = writeScript("cube.scad", kCubeScript);
    auto out = src.parent_path() / "cube_out.obj";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 0);
    EXPECT_TRUE(readFile(out).rfind("v ", 0) == 0);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliExportFormats, OffExport) {
    auto src = writeScript("cube.scad", kCubeScript);
    auto out = src.parent_path() / "cube_out.off";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 0);
    EXPECT_TRUE(readFile(out).rfind("OFF\n", 0) == 0);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliExportFormats, ThreeMfExport) {
    auto src = writeScript("cube.scad", kCubeScript);
    auto out = src.parent_path() / "cube_out.3mf";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 0);
    ASSERT_TRUE(std::filesystem::exists(out));
    EXPECT_GT(std::filesystem::file_size(out), 0u);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliExportFormats, UnrecognizedExtensionErrors) {
    auto src = writeScript("cube.scad", kCubeScript);
    auto out = src.parent_path() / "cube_out.xyz";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 1);
    EXPECT_FALSE(std::filesystem::exists(out));
    EXPECT_NE(stderr_.str().find("cannot infer output format"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliExportFormats, ExplicitFormatOverridesExtension) {
    auto src = writeScript("cube.scad", kCubeScript);
    auto out = src.parent_path() / "cube_out.mesh";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--format", "stl"}, std::cin, stdout_, stderr_), 0);
    ASSERT_TRUE(std::filesystem::exists(out));
    EXPECT_GT(std::filesystem::file_size(out), 0u);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

// -- --profile ------------------------------------------------------------

TEST(CliProfile, WritesReportWithCallSiteAndSummary) {
    // A recursive user function gives real call sites to report on --
    // fib(0)/fib(1) never recurse further, so the tree stays small.
    auto src = writeScript("profile.scad", "function fib(n) = n < 2 ? n : fib(n-1) + fib(n-2);\n"
                                            "cube([fib(4)+1, 1, 1]);\n");
    auto out = src.parent_path() / "profile_out.stl";
    auto report = src.parent_path() / "profile_out.txt";
    std::filesystem::remove(out);
    std::filesystem::remove(report);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--profile", report.string()}, std::cin, stdout_, stderr_), 0);
    ASSERT_TRUE(std::filesystem::exists(out));
    ASSERT_TRUE(std::filesystem::exists(report));
    const std::string text = readFile(report);
    EXPECT_NE(text.find("Profile report for"), std::string::npos);
    EXPECT_NE(text.find("Total time:"), std::string::npos);
    EXPECT_NE(text.find("fib"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
    std::filesystem::remove(report);
}

TEST(CliProfile, UnwritableProfilePathReturns1) {
    auto src = writeScript("profile2.scad", kCubeScript);
    auto out = src.parent_path() / "profile2_out.stl";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    // A path inside a nonexistent directory can never be opened for writing.
    const std::string badPath = (src.parent_path() / "no_such_dir_xyz" / "p.txt").string();
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--profile", badPath}, std::cin, stdout_, stderr_), 1);
    EXPECT_NE(stderr_.str().find("error:"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliProfile, SortCallsOrdersHigherCallCountFirst) {
    // fib(6): the recursive call site is "fib(n-1)"/"fib(n-2)" on line 1
    // (many calls); the toplevel call site is "fib(6)" on line 2 (one
    // call). --profile-sort calls must put the former's row first
    // regardless of self time. Keyed on the unique "file:line" location
    // column rather than exact column spacing, which is an implementation
    // detail this test shouldn't depend on.
    auto src = writeScript("profile3.scad", "function fib(n) = n < 2 ? n : fib(n-1) + fib(n-2);\n"
                                             "cube([fib(6)+1, 1, 1]);\n");
    auto out = src.parent_path() / "profile3_out.stl";
    auto report = src.parent_path() / "profile3_out.txt";
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--profile", report.string(), "--profile-sort", "calls"},
                      std::cin, stdout_, stderr_),
              0);
    const std::string text = readFile(report);
    const size_t recursiveRow = text.find("profile3.scad:1");
    const size_t toplevelRow = text.find("profile3.scad:2");
    ASSERT_NE(recursiveRow, std::string::npos);
    ASSERT_NE(toplevelRow, std::string::npos);
    EXPECT_LT(recursiveRow, toplevelRow);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
    std::filesystem::remove(report);
}

TEST(CliProfile, MinCallsFiltersOutLowVolumeCallSites) {
    // Same script/call sites as above -- the line-2 (toplevel, 1 call)
    // site is filtered out by a threshold the line-1 (recursive, dozens
    // of calls) site clears.
    auto src = writeScript("profile4.scad", "function fib(n) = n < 2 ? n : fib(n-1) + fib(n-2);\n"
                                             "cube([fib(6)+1, 1, 1]);\n");
    auto out = src.parent_path() / "profile4_out.stl";
    auto report = src.parent_path() / "profile4_out.txt";
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(
        runCli({src.string(), "-o", out.string(), "--profile", report.string(), "--profile-min-calls", "10"},
               std::cin, stdout_, stderr_),
        0);
    const std::string text = readFile(report);
    EXPECT_NE(text.find("profile4.scad:1"), std::string::npos);
    EXPECT_EQ(text.find("profile4.scad:2"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
    std::filesystem::remove(report);
}

TEST(CliProfile, CsvFormatWritesHeaderAndCommaSeparatedRows) {
    auto src = writeScript("profile5.scad", "function fib(n) = n < 2 ? n : fib(n-1) + fib(n-2);\n"
                                             "cube([fib(4)+1, 1, 1]);\n");
    auto out = src.parent_path() / "profile5_out.stl";
    auto report = src.parent_path() / "profile5_out.csv";
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--profile", report.string(), "--profile-format", "csv"},
                      std::cin, stdout_, stderr_),
              0);
    const std::string text = readFile(report);
    EXPECT_NE(text.find("# total_time,"), std::string::npos);
    EXPECT_NE(text.find("kind,name,caller,call_origin,call_line,call_column,call_count,self_time,cumulative_time\n"), std::string::npos);
    EXPECT_NE(text.find("function,fib,"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
    std::filesystem::remove(report);
}

TEST(CliProfile, InvalidProfileSortReturns1) {
    auto src = writeScript("profile6.scad", kCubeScript);
    auto out = src.parent_path() / "profile6_out.stl";
    auto report = src.parent_path() / "profile6_out.txt";
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--profile", report.string(), "--profile-sort", "bogus"},
                      std::cin, stdout_, stderr_),
              1);
    EXPECT_NE(stderr_.str().find("--profile-sort"), std::string::npos);
    EXPECT_FALSE(std::filesystem::exists(report));
    std::filesystem::remove(src);
}

TEST(CliProfile, InvalidProfileFormatReturns1) {
    auto src = writeScript("profile7.scad", kCubeScript);
    auto out = src.parent_path() / "profile7_out.stl";
    auto report = src.parent_path() / "profile7_out.txt";
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--profile", report.string(), "--profile-format", "bogus"},
                      std::cin, stdout_, stderr_),
              1);
    EXPECT_NE(stderr_.str().find("--profile-format"), std::string::npos);
    EXPECT_FALSE(std::filesystem::exists(report));
    std::filesystem::remove(src);
}

// -- use <file> ---------------------------------------------------------

TEST(CliUseStatement, InjectsModuleAndExports) {
    auto lib = writeScript("use_lib.scad", "module lib_cube(s) { cube(s); }\n");
    auto main = writeScript("use_main.scad", "use <" + lib.string() + ">\nlib_cube(5);\n");
    auto out = main.parent_path() / "use_main_out.stl";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({main.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 0);
    ASSERT_TRUE(std::filesystem::exists(out));
    EXPECT_GT(std::filesystem::file_size(out), 0u);
    std::filesystem::remove(lib);
    std::filesystem::remove(main);
    std::filesystem::remove(out);
}

// -- error handling ---------------------------------------------------

TEST(CliErrorHandling, SyntaxErrorReturns1) {
    auto src = writeScript("bad.scad", "cube([10,10,10]\n");
    auto out = src.parent_path() / "bad_out.stl";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 1);
    EXPECT_FALSE(std::filesystem::exists(out));
    std::filesystem::remove(src);
}

TEST(CliErrorHandling, EvalErrorReturns1AndPrintsToStderr) {
    auto src = writeScript("err.scad", "assert(false, \"boom\");\n");
    auto out = src.parent_path() / "err_out.stl";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 1);
    EXPECT_NE(stderr_.str().find("boom"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliErrorHandling, EchoGoesToStdout) {
    auto src = writeScript("echo.scad", kModuleScript);
    auto out = src.parent_path() / "echo_out.stl";
    std::filesystem::remove(out);
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string()}, std::cin, stdout_, stderr_), 0);
    EXPECT_NE(stdout_.str().find("ECHO: \"hi\""), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

// -- --debug REPL -----------------------------------------------------

TEST(CliDebugRepl, BreakpointThenContinueExports) {
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    // "run" itself pauses at line 1 first (break-on-first, gdb "start"
    // style); the first "continue" resumes to the line-2 breakpoint, the
    // second runs it to completion.
    std::istringstream stdin_ = feedInput({"break 2", "run", "continue", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    ASSERT_TRUE(std::filesystem::exists(out));
    EXPECT_GT(std::filesystem::file_size(out), 0u);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, PrintShowsVariableAfterAssignment) {
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    // break_on_first pauses at line 1 (before `width` is assigned), "next"
    // steps to line 2, where `width` is now visible.
    std::istringstream stdin_ = feedInput({"run", "next", "print width", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_NE(stdout_.str().find("$1 = 10"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, QuitMidDebugAbortsWithoutExporting) {
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 1);
    EXPECT_FALSE(std::filesystem::exists(out));
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, QuitBeforeRunExitsCleanlyWithoutExporting) {
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_FALSE(std::filesystem::exists(out));
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, SetOverridesVariableOnResume) {
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.off";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"break 2", "run", "continue", "set width=2", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);

    std::ifstream in(out);
    std::string line;
    std::getline(in, line); // "OFF"
    std::getline(in, line); // "$verts $tris 0"
    double maxCoord = 0.0;
    while (std::getline(in, line)) {
        if (line.rfind("3 ", 0) == 0) continue; // a face line, not a vertex
        std::istringstream ls(line);
        double v;
        while (ls >> v) maxCoord = std::max(maxCoord, std::fabs(v));
    }
    in.close(); // Windows can't remove() a file while it's still open.
    EXPECT_DOUBLE_EQ(maxCoord, 2.0); // would be 10.0 without the override
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, ErrorBreakLetsUserInspectThenAborts) {
    auto src = writeScript("err.scad", "assert(false, \"boom\");\n");
    auto out = src.parent_path() / "err_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 1);
    EXPECT_NE(stderr_.str().find("boom"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, ListShowsSourceFromUseInjectedFileWhenPausedThere) {
    // A breakpoint hit inside a `use <file>`-injected module's own body
    // lives in a *different* file than the main script -- `list` (both the
    // automatic display on the breakpoint hit and an explicit `list`
    // command) must show that file's lines, not the main script's.
    auto lib = writeScript("dbg_lib.scad", "module lib_cube(s) {\n    cube(s);\n}\n");
    auto src = writeScript("dbg_main.scad", "use <" + lib.string() + ">\nlib_cube(5);\n");
    auto out = src.parent_path() / "dbg_main_out.stl";
    std::filesystem::remove(out);
    // "run" pauses at main:1 first (break-on-first); "continue" resumes to
    // the lib.scad:2 breakpoint (whose hit auto-lists); "list" re-lists
    // explicitly from the same paused location.
    std::istringstream stdin_ = feedInput({"break " + lib.string() + ":2", "run", "continue", "list", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    // "cube(s);" only exists in lib.scad -- before the fix, listSource()
    // always read sourcePath_'s own lines regardless of which file the
    // debugger was actually paused in, so this string could never appear
    // (main's own line 2 is "lib_cube(5);", not "cube(s);").
    EXPECT_NE(stdout_.str().find("cube(s);"), std::string::npos);
    std::filesystem::remove(lib);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, PreRunEofExitsCleanlyWithoutExporting) {
    // Closing stdin (no "quit" command at all) before "run" must be
    // handled the same as an explicit quit -- runPrompt()'s own
    // std::getline-fails branch, distinct from QuitBeforeRunExitsCleanly's
    // explicit "quit" command.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_(""); // immediate EOF
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_FALSE(std::filesystem::exists(out));
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, PreRunCommandDispatchCoversAllBranches) {
    // Every pre-run command runPrompt() dispatches, beyond the plain
    // "break"/"run"/"quit" already exercised by other tests: an
    // unparsable "break"/"delete" location (parseLocation's own failure
    // path), "delete" of one breakpoint vs. "delete" with no args
    // (clear-all), "info breakpoints" both with and without any set,
    // "list" with a numeric arg and with an unparsable one, "help", and an
    // unrecognized command.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({
        "break bogus",
        "delete bogus",
        "break 2",
        "info breakpoints",
        "delete 2",
        "info breakpoints",
        "delete",
        "info bogus_sub_command",
        "list 2",
        "list abc",
        "help",
        "totally_unknown_command",
        "quit",
    });
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    const std::string log = stdout_.str();
    EXPECT_NE(log.find("Usage: break [file:]line"), std::string::npos);
    EXPECT_NE(log.find("Usage: delete [file:]line"), std::string::npos);
    EXPECT_NE(log.find("Breakpoint set at"), std::string::npos);
    EXPECT_NE(log.find("breakpoint at"), std::string::npos);
    EXPECT_NE(log.find("No breakpoints set."), std::string::npos);
    EXPECT_NE(log.find("All breakpoints deleted"), std::string::npos);
    EXPECT_NE(log.find("Undefined info command: \"bogus_sub_command\""), std::string::npos);
    EXPECT_NE(log.find("Commands (before"), std::string::npos);
    EXPECT_NE(log.find("Undefined command: \"totally_unknown_command\""), std::string::npos);
    EXPECT_FALSE(std::filesystem::exists(out));
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, PausedEofAbortsWithoutExporting) {
    // interact()'s own std::getline-fails branch (distinct from
    // QuitMidDebugAbortsWithoutExporting's explicit "quit"): closing
    // stdin while paused must abort exactly like "quit" would.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run"}); // pauses (break-on-first), then EOF
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 1);
    EXPECT_FALSE(std::filesystem::exists(out));
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, PausedCommandDispatchCoversAllBranches) {
    // Every paused-REPL command interact() dispatches, beyond the
    // print/set/continue already exercised elsewhere: "print" with no
    // argument and with an unknown symbol (both usage-error branches),
    // "set" with no "=" at all (usage error), a quoted-string "set" value
    // and a bare-token "set" value that parses as neither number nor
    // quoted string (parseValueForRepl's undef fallback), "delete" while
    // paused, "help", and an unrecognized command.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({
        "run",
        "print",
        "print does_not_exist",
        "set",
        "set width=\"hi\"",
        "set width=not_a_number",
        "delete",
        "info bogus_sub_command",
        "help",
        "totally_unknown_command",
        "continue",
    });
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    const std::string log = stdout_.str();
    EXPECT_NE(log.find("Usage: print <name>"), std::string::npos);
    EXPECT_NE(log.find("No symbol \"does_not_exist\" in current context."), std::string::npos);
    EXPECT_NE(log.find("Usage: set <name>=<value>"), std::string::npos);
    EXPECT_NE(log.find("width will be set to"), std::string::npos);
    EXPECT_NE(log.find("All breakpoints deleted"), std::string::npos);
    EXPECT_NE(log.find("Undefined info command: \"bogus_sub_command\""), std::string::npos);
    EXPECT_NE(log.find("Commands (while paused)"), std::string::npos);
    EXPECT_NE(log.find("Undefined command: \"totally_unknown_command\""), std::string::npos);
    ASSERT_TRUE(std::filesystem::exists(out));
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, StepEntersNextStatement) {
    // "step"'s own stepCmd_="into" branch in debugHook -- distinct from
    // PrintShowsVariableAfterAssignment's "next" ("over"), which is the
    // only step-family command any other existing test exercises.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "step", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    // Two separate pauses expected: break-on-first at line 1, then "step"
    // landing on line 2.
    const std::string log = stdout_.str();
    EXPECT_NE(log.find("Breakpoint hit at " + src.filename().string() + ":1"), std::string::npos);
    EXPECT_NE(log.find("Breakpoint hit at " + src.filename().string() + ":2"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, FinishInsideFunctionPrintsReturnedValueAndBacktraceShowsCallFrame) {
    // "finish" (returnHook's own print, only reachable from inside a user
    // *function* call -- modules never invoke returnHook) and backtrace's
    // "haveFrame" branch (a non-empty call stack), neither reachable from
    // any existing test (which only ever pause at toplevel or inside a
    // module).
    auto src = writeScript("fn.scad", "function f(x) = x + 1;\ny = f(2);\necho(y);\ncube(1);\n");
    auto out = src.parent_path() / "fn_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"break 1", "run", "continue", "backtrace", "finish", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    const std::string log = stdout_.str();
    EXPECT_NE(log.find("#0  f() at " + src.filename().string() + ":1"), std::string::npos);
    EXPECT_NE(log.find("#1  <toplevel> at " + src.filename().string() + ":2"), std::string::npos);
    EXPECT_NE(log.find("Value returned is $1 = 3"), std::string::npos);
    ASSERT_TRUE(std::filesystem::exists(out));
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, ChildStepsToChildrenCallForwardedStatement) {
    // Paused at the `wrapper() { cube(1); sphere(1); }` call itself
    // (line 4), "child" should run until wrapper's own `children();`
    // (line 2) forwards control to one of that call's own children --
    // here, cube(1) at line 5, its first child statement -- not stop at
    // line 2 itself (children() is not one of the snapshotted targets,
    // only the block's own top-level statements are). "run" itself
    // already pauses directly at line 4 (break-on-first and the explicit
    // breakpoint coincide there, since line 1 is a declaration and is
    // never checked) -- no preceding "continue" needed.
    auto src = writeScript("children.scad", "module wrapper() {\n"
                                             "    children();\n"
                                             "}\n"
                                             "wrapper() {\n"
                                             "    cube(1);\n"
                                             "    sphere(1);\n"
                                             "}\n");
    auto out = src.parent_path() / "children_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"break 4", "run", "child", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    const std::string log = stdout_.str();
    EXPECT_NE(log.find("Breakpoint hit at " + src.filename().string() + ":5"), std::string::npos);
    EXPECT_NE(log.find("cube(1);"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, ChildFallsBackToCallReturnWhenChildrenNeverInvoked) {
    // A module that never calls children() at all -- the target position
    // is never reached, so "child" relies purely on the depth-drop
    // fallback (or, issued at top level with nothing shallower to drop
    // to, simply lets the rest of the script run normally) -- either way
    // evaluation must still complete, not hang.
    auto src = writeScript("noop.scad", "module noop() {\n"
                                         "}\n"
                                         "noop() {\n"
                                         "    cube(1);\n"
                                         "}\n"
                                         "sphere(1);\n");
    auto out = src.parent_path() / "noop_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"break 3", "run", "child", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    ASSERT_TRUE(std::filesystem::exists(out));
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, RequestPauseCausesNextDebugHookCallToPauseLikeABreakpoint) {
    // requestPause() sets the exact same flag a real SIGINT handler would
    // (see its own doc comment -- the in-process test harness here, an
    // injected istream/ostream with no subprocess, can't deliver a real
    // OS signal). Constructs a DebugRepl directly (not through runCli())
    // and drives debugHook() by hand so this is isolated from
    // breakOnFirst_/breakpoints_/stepHit -- a different origin than the
    // constructed source path means break-on-first can't be what's
    // causing the pause, so this specifically proves requestPause()'s
    // own contribution to the shouldPause OR-chain.
    auto src = writeScript("m.scad", kModuleScript);
    std::ostringstream out;
    std::istringstream stdin_ = feedInput({"continue"});
    DebugRepl repl(src.string(), stdin_, out);
    repl.requestPause();
    std::vector<CallStackFrame> callStack;
    DebugAction action =
        repl.debugHook(5, 0, /*forced=*/false, /*exprLevel=*/false, "/some/other/file.scad", callStack,
                       [] { return std::vector<DebugFrame>{}; });
    EXPECT_FALSE(action.stop);
    EXPECT_NE(out.str().find("Interrupted at"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, StopReturnsToPreRunPromptThenRunExports) {
    // "stop" aborts the current evaluation but -- unlike "quit" -- returns
    // to the pre-run prompt instead of exiting the CLI; a plain "run" from
    // there starts a fresh evaluation that completes normally.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "stop", "run", "continue", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_NE(stdout_.str().find("Evaluation stopped."), std::string::npos);
    ASSERT_TRUE(std::filesystem::exists(out));
    EXPECT_GT(std::filesystem::file_size(out), 0u);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, StopThenQuitAbortsWithoutExporting) {
    // If the user never restarts after "stop", quitting from the pre-run
    // prompt behaves exactly like never having run at all -- no export,
    // but a clean exit (0), matching QuitBeforeRunExitsCleanlyWithoutExporting.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "stop", "quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_FALSE(std::filesystem::exists(out));
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, RestartWhilePausedAbortsAndRunsAgainFromStart) {
    // "restart" while paused aborts the current run and immediately
    // re-runs (no intervening pre-run prompt) -- breakpoints carry over,
    // so the fresh run pauses at the same breakpoint again before
    // finally being allowed to complete.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"break 2", "run", "continue", "restart", "continue", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    // Each full run pauses twice (break-on-first at line 1, then the
    // explicit breakpoint at line 2); running twice (once before "restart",
    // once after) means "Breakpoint hit" must appear exactly 4 times --
    // proving the second run genuinely started over from the top rather
    // than "restart" being a no-op or immediately finishing.
    const std::string text = stdout_.str();
    int hitCount = 0;
    for (size_t pos = text.find("Breakpoint hit"); pos != std::string::npos; pos = text.find("Breakpoint hit", pos + 1)) {
        ++hitCount;
    }
    EXPECT_EQ(hitCount, 4);
    ASSERT_TRUE(std::filesystem::exists(out));
    EXPECT_GT(std::filesystem::file_size(out), 0u);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, RestartAcceptedAtPreRunPromptAfterStop) {
    // A user who just typed "stop" naturally reaches for "restart" again
    // out of habit -- with nothing currently running, it must behave
    // exactly like "run" at the pre-run prompt, not "Undefined command".
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "stop", "restart", "continue", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_EQ(stdout_.str().find("Undefined command"), std::string::npos);
    ASSERT_TRUE(std::filesystem::exists(out));
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, ExitAliasWorksLikeQuitBeforeAndDuringRun) {
    auto src = writeScript("m.scad", kModuleScript);
    auto out1 = src.parent_path() / "m_out1.stl";
    auto out2 = src.parent_path() / "m_out2.stl";
    std::filesystem::remove(out1);
    std::filesystem::remove(out2);
    {
        std::istringstream stdin_ = feedInput({"exit"});
        std::ostringstream stdout_, stderr_;
        EXPECT_EQ(runCli({src.string(), "-o", out1.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
        EXPECT_FALSE(std::filesystem::exists(out1));
    }
    {
        std::istringstream stdin_ = feedInput({"run", "exit"});
        std::ostringstream stdout_, stderr_;
        EXPECT_EQ(runCli({src.string(), "-o", out2.string(), "--debug"}, stdin_, stdout_, stderr_), 1);
        EXPECT_FALSE(std::filesystem::exists(out2));
    }
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, InfoFunctionsAndModulesListUserDeclarationsBeforeAndDuringRun) {
    auto src = writeScript(
        "m.scad",
        "function fib(n) = n < 2 ? n : fib(n-1) + fib(n-2);\n"
        "module wrapper(s) {\n"
        "    children();\n"
        "}\n"
        "wrapper(1) {\n"
        "    cube(1);\n"
        "}\n");
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"info functions", "info modules", "run", "info functions", "info modules", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    // writeScript()'s actual filename is "oscad_cli_test_m.scad", not
    // literally "m.scad" -- "fib(n) at " and "m.scad:1" are checked as
    // separate substrings rather than one combined string spanning across
    // that filename, which a literal "at m.scad:1" would require (and
    // never find, since "at " isn't immediately followed by "m.scad").
    const std::string text = stdout_.str();
    EXPECT_NE(text.find("User-defined functions:"), std::string::npos);
    EXPECT_NE(text.find("fib(n) at "), std::string::npos);
    EXPECT_NE(text.find("m.scad:1"), std::string::npos);
    EXPECT_NE(text.find("User-defined modules:"), std::string::npos);
    EXPECT_NE(text.find("wrapper(s) at "), std::string::npos);
    EXPECT_NE(text.find("m.scad:2"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, InfoFunctionsAndModulesReportNoneWhenScriptDeclaresNeither) {
    auto src = writeScript("m.scad", kCubeScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"info functions", "info modules", "quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    const std::string text = stdout_.str();
    EXPECT_NE(text.find("No user-defined functions."), std::string::npos);
    EXPECT_NE(text.find("No user-defined modules."), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, InfoVariablesShowsCurrentlyVisibleVariablesOnlyWhilePaused) {
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    // Pre-run: no variables to show yet (nothing has executed). Paused
    // after "next" (past the `width` assignment): `width` is visible.
    std::istringstream stdin_ = feedInput({"info variables", "run", "next", "info variables", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    const std::string text = stdout_.str();
    EXPECT_NE(text.find("No variables to show before \"run\"."), std::string::npos);
    EXPECT_NE(text.find("width = 10"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, BlankLineRepeatsLastStepCommand) {
    // "next" once, then two blank lines -- mirrors gdb's own repeat-last-
    // command convention. Four statement lines means four total pauses
    // (break-on-first at line 1, then one "next" advance per subsequent
    // command) if the repeat genuinely re-issued "next" each time.
    auto src = writeScript("m.scad", "a = 1;\nb = 2;\nc = 3;\ncube(1);\n");
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "next", "", "", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    const std::string text = stdout_.str();
    EXPECT_NE(text.find("m.scad:1"), std::string::npos);
    EXPECT_NE(text.find("m.scad:2"), std::string::npos);
    EXPECT_NE(text.find("m.scad:3"), std::string::npos);
    EXPECT_NE(text.find("m.scad:4"), std::string::npos);
    ASSERT_TRUE(std::filesystem::exists(out));
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

TEST(CliDebugRepl, BlankLineBeforeAnyRepeatableCommandIsANoOp) {
    // A blank line before any of step/next/child/restart/continue/finish/
    // list has ever been issued has nothing to repeat -- must fall back
    // to the pre-feature behavior (silently re-prompt), not crash or
    // treat it as some other command.
    auto src = writeScript("m.scad", kModuleScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "", "print width", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_EQ(stdout_.str().find("Undefined command"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}

// -- list <name> ----------------------------------------------------------

TEST(CliDebugRepl, ListByNameJumpsToUnambiguousDeclaration) {
    auto src = writeScript("m.scad", kAmbiguousNameScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"list thing", "quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_NE(stdout_.str().find("module thing(x) {"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, ListByAmbiguousNameReportsBothNamespacesAndListsNothing) {
    auto src = writeScript("m.scad", kAmbiguousNameScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"list fib", "quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    const std::string text = stdout_.str();
    EXPECT_NE(text.find("Both a function and a module are named \"fib\""), std::string::npos);
    EXPECT_NE(text.find("list function:fib"), std::string::npos);
    EXPECT_NE(text.find("list module:fib"), std::string::npos);
    // Ambiguous means nothing gets listed -- neither declaration's own
    // source line should appear.
    EXPECT_EQ(text.find("function fib(n)"), std::string::npos);
    EXPECT_EQ(text.find("module fib() {"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, ListWithFunctionQualifierDisambiguatesFromModule) {
    auto src = writeScript("m.scad", kAmbiguousNameScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"list function:fib", "quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_NE(stdout_.str().find("function fib(n)"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, ListWithModuleQualifierDisambiguatesFromFunction) {
    auto src = writeScript("m.scad", kAmbiguousNameScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"list module:fib", "quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_NE(stdout_.str().find("module fib() {"), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, ListByUnknownNameReportsNoSymbol) {
    auto src = writeScript("m.scad", kAmbiguousNameScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"list bogus_name", "quit"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_NE(stdout_.str().find("No symbol \"bogus_name\" in current context."), std::string::npos);
    std::filesystem::remove(src);
}

TEST(CliDebugRepl, ListByNameWorksFromPausedPromptToo) {
    // Jumping to a named declaration's own source works while paused too,
    // and shows source from *that* declaration's location, not wherever
    // the debugger is currently paused.
    auto src = writeScript("m.scad", kAmbiguousNameScript);
    auto out = src.parent_path() / "m_out.stl";
    std::filesystem::remove(out);
    std::istringstream stdin_ = feedInput({"run", "list thing", "continue"});
    std::ostringstream stdout_, stderr_;
    EXPECT_EQ(runCli({src.string(), "-o", out.string(), "--debug"}, stdin_, stdout_, stderr_), 0);
    EXPECT_NE(stdout_.str().find("module thing(x) {"), std::string::npos);
    std::filesystem::remove(src);
    std::filesystem::remove(out);
}
