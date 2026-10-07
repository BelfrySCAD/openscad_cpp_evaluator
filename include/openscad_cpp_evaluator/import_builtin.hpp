#pragma once

#include "openscad_cpp_evaluator/call_args.hpp"

namespace oscad {
class ASTNode;
} // namespace oscad

namespace oscadeval {

class Evaluator;
struct EvalContext;

// import() used as an expression (not a geometry statement): returns a VNF
// ([[verts],[faces]]) for a mesh file (.stl/.obj/.off/.3mf), or the JSON
// file's content converted to native values (.json). DXF/SVG-as-Region
// import lands alongside their module-context counterparts in a later
// phase. Mirrors Evaluator._import_as_value (minus the dxf/svg branch).
// `ctx` supplies $fn/$fa/$fs/$fe for an SVG's curves.
Value importAsValue(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node, const EvalContext& ctx);

} // namespace oscadeval
