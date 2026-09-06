// The shared include cache (oscad::getProgramFromFile) and the ScopeTable
// that makes sharing possible (oscad::ScopeTable).
//
// A parsed tree used to be owned by whoever asked for it, and each node
// carried its own Scope*. Both had to change together: sharing one tree
// between evaluations is what makes a re-render cheap (parsing
// BOSL2/std.scad is ~55ms and ~82% of evaluating a small script against
// it), and a shared node cannot hold a scope, because `include` puts it in
// the INCLUDER's scope -- a different one per script.

#include "openscad_cpp_evaluator/eval_use.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

using namespace oscadeval;

namespace {

std::filesystem::path writeFile(const std::string& name, const std::string& content) {
    const auto p = std::filesystem::temp_directory_path() / ("oscad_inccache_" + name);
    std::ofstream out(p);
    out << content;
    return p;
}

std::vector<std::string> echoesOf(const std::filesystem::path& path) {
    std::vector<std::string> echoes;
    auto log = [&echoes](const std::string& m) { echoes.push_back(m); };
    oscad::ParsedProgram program = oscad::getProgramFromFile(path.string());
    ResolvedUseScopes used = resolveUseScopes(program.nodes, path.string(), log);
    Evaluator ev(log);
    EvalContext ctx = EvalContext::makeRoot(used.rootScope.get());
    ev.evaluate(used.processedNodes, ctx, {}, /*generate=*/false);
    return echoes;
}

} // namespace

TEST(IncludeCache, SameFileIncludedTwiceContributesItsStatementsOnce) {
    // A file is spliced in at most once per resolution. This is why only
    // the per-FILE parses are cached and never a whole resolved program:
    // caching `mid` already-resolved would splice `base` twice here.
    writeFile("base.scad", "BASE = 1;\n");
    const auto mid = writeFile("mid.scad", "include <oscad_inccache_base.scad>\nMID = 2;\n");
    const auto top = writeFile("top.scad",
                                "include <oscad_inccache_base.scad>\n"
                                "include <oscad_inccache_mid.scad>\n"
                                "echo(BASE + MID);\n");
    (void)mid;
    oscad::ParsedProgram program = oscad::getProgramFromFile(top.string());
    int baseAssignments = 0;
    for (const oscad::ASTNode* n : program.nodes) {
        if (n->kind() == oscad::NodeKind::Assignment &&
            static_cast<const oscad::Assignment&>(*n).name->name == "BASE")
            ++baseAssignments;
    }
    EXPECT_EQ(baseAssignments, 1);
    EXPECT_EQ(echoesOf(top), std::vector<std::string>{"ECHO: 3"});
}

TEST(IncludeCache, TwoProgramsShareTheSameParsedNodes) {
    // The whole point: the second script must not re-parse the library.
    writeFile("lib.scad", "LIB = 7;\nfunction libf(x) = x * 2;\n");
    const auto a = writeFile("a.scad", "include <oscad_inccache_lib.scad>\necho(libf(LIB));\n");
    const auto b = writeFile("b.scad", "include <oscad_inccache_lib.scad>\necho(libf(LIB) + 1);\n");

    oscad::ParsedProgram pa = oscad::getProgramFromFile(a.string());
    oscad::ParsedProgram pb = oscad::getProgramFromFile(b.string());
    // Same node ADDRESSES, not merely equal trees.
    EXPECT_EQ(pa.nodes.front(), pb.nodes.front());

    // And both still evaluate correctly, each in its own scope.
    EXPECT_EQ(echoesOf(a), std::vector<std::string>{"ECHO: 14"});
    EXPECT_EQ(echoesOf(b), std::vector<std::string>{"ECHO: 15"});
}

TEST(IncludeCache, EditingAnIncludedFileReplacesItsEntryRatherThanAddingOne) {
    // Keyed on (path, mtime, size), so an edit invalidates on its own.
    const auto lib = writeFile("edited.scad", "V = 1;\n");
    const auto top = writeFile("edtop.scad", "include <oscad_inccache_edited.scad>\necho(V);\n");
    EXPECT_EQ(echoesOf(top), std::vector<std::string>{"ECHO: 1"});
    const size_t cacheEntriesBefore = oscad::astCacheSize();

    std::filesystem::last_write_time(lib, std::filesystem::last_write_time(lib) + std::chrono::seconds(2));
    { std::ofstream out(lib); out << "V = 99;\n"; }
    std::filesystem::last_write_time(lib, std::filesystem::last_write_time(lib) + std::chrono::seconds(4));
    EXPECT_EQ(echoesOf(top), std::vector<std::string>{"ECHO: 99"});

    // The old version must not still be held: an editor re-renders on every
    // save, so keying the cache by (path, stamp) instead of replacing in
    // place would keep a full AST copy of every version ever saved.
    oscad::ParsedProgram again = oscad::getProgramFromFile(top.string());
    const oscad::ASTNode* first = again.nodes.front();
    ASSERT_EQ(first->kind(), oscad::NodeKind::Assignment);
    EXPECT_EQ(oscad::astCacheSize(), cacheEntriesBefore) << "editing a file must replace its entry, not add one";
}

TEST(IncludeCache, ADerivedContextKeepsTheScopeTable) {
    // The bug this exists for: EvalContext::childCtx/callCtx build a FRESH
    // context field by field, and one that forgot to carry scopeTable made
    // every node's scope read back as null. Name resolution then fell back
    // to the enclosing scope, so a module body calling a builtin of its own
    // name found ITSELF -- BOSL2's `module _cube(...) cube(...);` wrapper
    // recursed until the depth limit. Nothing else in the suite noticed.
    const auto top = writeFile("derived.scad",
                                "module _mycube(s) cube(s);\n"
                                "module wrap(s) _mycube(s);\n"
                                "wrap(2);\n");
    std::vector<std::string> logs;
    auto log = [&logs](const std::string& m) { logs.push_back(m); };
    oscad::ParsedProgram program = oscad::getProgramFromFile(top.string());
    ResolvedUseScopes used = resolveUseScopes(program.nodes, top.string(), log);
    Evaluator ev(log);
    EvalContext ctx = EvalContext::makeRoot(used.rootScope.get());
    ASSERT_NE(ctx.scopeTable, nullptr);

    // Every derived shape must carry it -- checked directly, so a new
    // constructor that forgets fails here rather than as a recursion
    // blow-up in somebody's library.
    EXPECT_EQ(ctx.withScope(ctx.scope).scopeTable, ctx.scopeTable);
    EXPECT_EQ(ctx.childCtx().scopeTable, ctx.scopeTable);
    EXPECT_EQ(ctx.callCtx().scopeTable, ctx.scopeTable);
    EXPECT_EQ(ctx.letChildCtx().scopeTable, ctx.scopeTable);

    ev.evaluate(used.processedNodes, ctx, {}, /*generate=*/false);
    for (const std::string& m : logs) EXPECT_EQ(m.find("Recursion"), std::string::npos) << m;
}
