#pragma once

#include "openscad_cpp_evaluator/bytecode.hpp"

#include <optional>

namespace oscad {
class FunctionDeclaration;
} // namespace oscad

namespace oscadeval {

// Attempts to compile `decl`'s parameter defaults + body to bytecode.
// Returns nullopt if `decl` uses any construct Phase 1 doesn't compile yet
// (a call of any kind, a function-literal value, a list comprehension,
// echo()/assert(), or a $-prefixed parameter) -- the caller falls back to
// the ordinary AST interpreter for that whole function, unconditionally
// correct since nothing about this function's behavior changes based on
// whether it happened to compile.
std::optional<CompiledChunk> tryCompileFunction(const oscad::FunctionDeclaration& decl);

} // namespace oscadeval
