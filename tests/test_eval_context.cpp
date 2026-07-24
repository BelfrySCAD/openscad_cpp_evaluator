#include "openscad_cpp_evaluator/eval_context.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::unique_ptr<oscad::Scope> makeScope(std::vector<std::unique_ptr<oscad::ASTNode>>& ast) {
    ast = parseSrc("x = 1;");
    return oscad::buildScopes(ast);
}

} // namespace

TEST(EvalContextRoot, SeedsDefaultDollarVars) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    auto scope = makeScope(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EXPECT_DOUBLE_EQ(std::get<double>(ctx.dyn->at("$fn")), 0.0);
    EXPECT_DOUBLE_EQ(std::get<double>(ctx.dyn->at("$fa")), 12.0);
    EXPECT_DOUBLE_EQ(std::get<double>(ctx.dyn->at("$fs")), 2.0);
    EXPECT_DOUBLE_EQ(std::get<double>(ctx.dyn->at("$t")), 0.0);
    EXPECT_DOUBLE_EQ(std::get<double>(ctx.dyn->at("$parent_modules")), 0.0);
    EXPECT_TRUE(ctx.let_->empty());
}

TEST(EvalContextWithScope, AliasesLetForSiblingVisibility) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    auto scope = makeScope(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EvalContext sibling1 = ctx.withScope(scope.get());
    EvalContext sibling2 = ctx.withScope(scope.get());

    (*sibling1.let_)["a"] = Value{5.0};
    // sibling2 shares the SAME underlying map (not a copy), so it must see
    // sibling1's write immediately -- the mechanism that makes
    // `a = 1; cube(a); a = 2;`-style sequential assignment visibility work.
    ASSERT_TRUE(sibling2.let_->count("a"));
    EXPECT_DOUBLE_EQ(std::get<double>(sibling2.let_->at("a")), 5.0);
    // ...and the root ctx itself, since withScope() aliases with `this`.
    EXPECT_DOUBLE_EQ(std::get<double>(ctx.let_->at("a")), 5.0);
}

TEST(EvalContextChildCtx, CopiesLetBreakingAliasingWithParent) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    auto scope = makeScope(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    EvalContext child = ctx.childCtx();
    (*child.let_)["a"] = Value{1.0};
    EXPECT_FALSE(ctx.let_->count("a")); // parent unaffected -- childCtx() snapshot-copies
}

TEST(EvalContextChildCtx, ResetsDynPositionsButStillCopiesDyn) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    auto scope = makeScope(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    (*ctx.dynPositions)["a"] = nullptr; // placeholder entry; only presence/absence matters here
    EvalContext child = ctx.childCtx();
    EXPECT_TRUE(child.dynPositions->empty()); // fresh -- the reference's asymmetric child_ctx() rule
    EXPECT_DOUBLE_EQ(std::get<double>(child.dyn->at("$fn")), 0.0); // dyn itself still copied
}

TEST(EvalContextChildCtx, DefaultsToInheritingChildrenNodesNotResetting) {
    // Getting this backwards is a real shipped-bug class in the reference:
    // a children() forwarding chain silently swallowed under
    // `color(c) children();`-shaped user modules. See
    // openscad_evaluator/docs/evaluator.md's child_ctx() entry.
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    auto scope = makeScope(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    auto nodes = std::make_shared<const ChildrenNodeList>(ChildrenNodeList{ast[0].get()});
    ctx.childrenNodes = nodes;
    EvalContext child = ctx.childCtx(); // no explicit override
    EXPECT_EQ(child.childrenNodes, nodes); // same shared_ptr, inherited
}

TEST(EvalContextCallCtx, IsolatesLetAndResetsChildrenNodes) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    auto scope = makeScope(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    (*ctx.let_)["a"] = Value{1.0};
    ctx.childrenNodes = std::make_shared<const ChildrenNodeList>(ChildrenNodeList{ast[0].get()});

    EvalContext call = ctx.callCtx();
    EXPECT_TRUE(call.let_->empty());          // isolated variable scope
    EXPECT_TRUE(call.childrenNodes->empty()); // reset, NOT inherited (unlike childCtx())
    EXPECT_DOUBLE_EQ(std::get<double>(call.dyn->at("$fn")), 0.0); // $-vars stay dynamically scoped
}

TEST(EvalContextLetChildCtx, CopiesLetIndependentlyOfParent) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    auto scope = makeScope(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    (*ctx.let_)["a"] = Value{1.0};

    EvalContext letCtx = ctx.letChildCtx();
    (*letCtx.let_)["a"] = Value{2.0};
    EXPECT_DOUBLE_EQ(std::get<double>(ctx.let_->at("a")), 1.0); // parent's binding untouched
    EXPECT_DOUBLE_EQ(std::get<double>(letCtx.let_->at("a")), 2.0);
}
