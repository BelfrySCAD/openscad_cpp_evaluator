#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

namespace oscadeval {

// dxf_dim() / dxf_cross() -- read a measurement straight out of a DXF file
// rather than out of the model. This file has its own DXF reading; it does
// not share src/import/dxf_import.cpp's.

// dxf_dim(file, name, layer, origin, scale).
// CLEAN-ROOM: reimplement from spec section F1.
Value builtinDxfDim(Evaluator&, const CallArgs&, const oscad::ASTNode&) {
    return Value{};
}

// dxf_cross(file, layer, origin, scale).
// CLEAN-ROOM: reimplement from spec section F1.
Value builtinDxfCross(Evaluator&, const CallArgs&, const oscad::ASTNode&) {
    return Value{};
}

} // namespace oscadeval
