#pragma once

#include "openscad_cpp_evaluator/value.hpp"

#include <manifold/cross_section.h>
#include <manifold/manifold.h>

#include <array>
#include <optional>
#include <vector>

namespace oscadeval {

enum class BodyRole { Normal, Highlight, Background, ShowOnly };

// A single piece of evaluated geometry: exactly one of `body` (3D) or
// `section` (2D) is set, never both -- mirrors the Python reference's
// ColoredBody (body: Optional[Manifold] XOR section: Optional[CrossSection]).
// `role`/`triColors` are later-phase fields (modifier tagging -- Phase 3;
// multi-color CSG merges -- Phase 3/8) included now so the struct shape
// doesn't need reworking when those land.
struct ColoredBody {
    std::optional<manifold::Manifold> body;
    std::optional<std::array<float, 4>> color; // RGBA; nullopt = "no explicit color() -- follow the live theme"
    std::optional<manifold::CrossSection> section;
    bool flatPreview = false;
    BodyRole role = BodyRole::Normal;
    std::optional<std::vector<std::array<float, 4>>> triColors; // per-triangle RGBA, multi-color CSG merges only
};

// `EvalContext::color`/CSGParams round-trip: a resolve function reads
// ctx.color (double-precision, matching Value's own numeric type) but a
// generate function only sees CSGParams (Value-typed, no ctx access) -- so
// color has to cross that boundary as a Value, same as every other
// resolved argument. Represented as either undef (no color) or a 4-element
// [r,g,b,a] list, exactly the shape a script's own `color([r,g,b,a])`
// argument already has.
Value colorToValue(const std::optional<std::array<double, 4>>& color);
std::optional<std::array<float, 4>> valueToColor(const Value& v);

// Converts every top-level 2D-only result (`body` unset, `section` set --
// e.g. a bare `circle();`) into a thin-extruded (1e-3 units tall) Manifold
// tagged `flatPreview = true`, so a mesh exporter/renderer that only
// understands 3D Manifolds (writeStl/writeObj/writeOff/writeThreeMf) can
// still show a 2D top-level result instead of silently dropping it. 3D
// bodies pass through unchanged. Mirrors to_renderable_bodies -- callers
// (the CLI, any other mesh-export caller) should apply this to
// Evaluator::evaluate()'s result before exporting, the same way the
// reference's own cli.py does right before export_bodies().
std::vector<ColoredBody> toRenderableBodies(const std::vector<ColoredBody>& bodies);

} // namespace oscadeval
