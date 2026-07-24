// In-process tests for the CLI (tools/cli/cli_lib.hpp) and its --debug REPL
// (debug_repl.hpp) -- ported from the reference's own test_cli.py, using
// runCli() directly (no subprocess spawned) exactly the way that file uses
// cli.main() directly with monkeypatched input()/capsys.

#include "cli_lib.hpp"

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
