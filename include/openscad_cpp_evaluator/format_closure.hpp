#pragma once

#include <string>

namespace oscad {
class ASTNode;
class FunctionLiteral;
}

namespace oscadeval {

// A function literal spelled the way the reference spells it when a script
// does `str(f)` or `echo(f)`: its own source, re-printed from the AST with
// every binary operator and ternary parenthesised.
//
// Deliberately not the parser's ASTNode::toString(), which is
// precedence-minimal and which 94 places inside the parser depend on. See
// format_closure.cpp for where each rule came from.
std::string formatFunctionLiteral(const oscad::FunctionLiteral& fn);
// Any expression, spelled the same way: what assert() quotes as its
// condition ("Assertion '(a > 1)' failed").
std::string formatExpression(const oscad::ASTNode& expr);

} // namespace oscadeval
