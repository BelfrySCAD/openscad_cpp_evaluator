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

    // What a 2D `section` has been transformed BY that a CrossSection
    // cannot itself hold: a translation along Z, a rotation out of the XY
    // plane, anything with a Z component. Applied when
    // toRenderableBodies() extrudes the section.
    //
    // A manifold::CrossSection is purely 2D, so `down(1) square(10)` had
    // nowhere to put the -1 and `rotate([55,0,25]) text("Foo")` had nowhere
    // to put the rotation -- both were silently dropped. Neither is
    // cosmetic: BOSL2 layers 2D overlays by pushing one down (isosurface's
    // contour() Example 1), and turns 2D labels to face the camera with
    // `rotate($vpr)` (skin's style figure), which came out lying flat and
    // read as a blob once the preview slab became a full unit tall.
    //
    // Identity while every transform so far has been 2D-representable --
    // those still go into the CrossSection itself, so 2D booleans and
    // offset() keep working on real 2D geometry. From the first transform
    // that is NOT representable, every later one composes here instead:
    // applying an outer transform to the section after an out-of-plane one
    // would put it in the wrong order.
    manifold::mat3x4 sectionXform{manifold::vec3(1, 0, 0), manifold::vec3(0, 1, 0),
                                   manifold::vec3(0, 0, 1), manifold::vec3(0, 0, 0)};
    BodyRole role = BodyRole::Normal;
    std::optional<std::vector<std::array<float, 4>>> triColors; // per-triangle RGBA, multi-color CSG merges only

    // Set ONLY when a mesh could not be built into a valid Manifold -- in
    // practice a polyhedron() whose faces don't close the surface (it has
    // boundary edges), which Manifold reports by returning an *empty* body
    // with Status() == NotManifold rather than by failing. Note the status
    // name is misleading: a closed mesh with reversed winding, non-manifold
    // vertices or self-intersections builds fine; only an OPEN one lands
    // here.
    //
    // Carrying the raw triangle soup alongside lets the renderer still draw
    // what the script described, which is far more use for spotting the
    // missing face than an empty viewport. `body` is deliberately left set
    // (and empty), so every existing `cb.body->...` path keeps working.
    //
    // Such a body is display-only: it can never take part in a CSG
    // operation, since there is no Manifold for Manifold to operate on.
    // splitByRole() pulls it aside much as it does a `%` background body,
    // and the CSG/hull/minkowski generators re-join it unchanged.
    std::optional<manifold::MeshGL> rawMesh;

    bool isDisplayOnly() const { return rawMesh.has_value(); }
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
// e.g. a bare `circle();`) into a 1-unit-tall extruded Manifold
// tagged `flatPreview = true`, so a mesh exporter/renderer that only
// understands 3D Manifolds (writeStl/writeObj/writeOff/writeThreeMf) can
// still show a 2D top-level result instead of silently dropping it. 3D
// bodies pass through unchanged. Mirrors to_renderable_bodies -- callers
// (the CLI, any other mesh-export caller) should apply this to
// Evaluator::evaluate()'s result before exporting, the same way the
// reference's own cli.py does right before export_bodies().
std::vector<ColoredBody> toRenderableBodies(const std::vector<ColoredBody>& bodies);

} // namespace oscadeval
