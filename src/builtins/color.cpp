#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/css_colors.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

namespace oscadeval {

// color(c = [1,1,1,1], alpha = 1) -- c is a CSS/hex color name, an [r,g,b]
// (alpha from the separate `alpha` argument), or an [r,g,b,a] (alpha taken
// from the list itself, the separate `alpha` argument is ignored in that
// case). Sets the color on ctx for every descendant statement. Mirrors
// _resolve_color/_generate_color.

BuiltinWrapParams computeColorParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    const Value defaultC =
        Value{std::make_shared<const ValueList>(ValueList{{Value{1.0}, Value{1.0}, Value{1.0}, Value{1.0}}})};
    Value cArg = getArg(args, 0, "c", defaultC);
    const double alpha = toDoubleLenient(getArg(args, 1, "alpha", Value{1.0}));

    std::array<double, 4> rgba{1.0, 1.0, 1.0, 1.0};
    if (const std::string* s = std::get_if<std::string>(&cArg)) {
        rgba = cssColor(*s, alpha);
    } else if (const ListPtr* l = std::get_if<ListPtr>(&cArg); l && *l) {
        const auto& items = (*l)->items;
        if (items.size() == 3) {
            rgba = {toDoubleLenient(items[0]), toDoubleLenient(items[1]), toDoubleLenient(items[2]), alpha};
        } else if (items.size() >= 4) {
            rgba = {toDoubleLenient(items[0]), toDoubleLenient(items[1]), toDoubleLenient(items[2]),
                     toDoubleLenient(items[3])};
        }
    }

    EvalContext childCtx = effCtx.childCtx(nullptr, std::optional<std::array<double, 4>>(rgba));
    CSGParams params;
    params["rgba"] = colorToValue(rgba);
    return BuiltinWrapParams{std::move(params), std::move(childCtx)};
}

CSGParams resolveColor(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    BuiltinWrapParams result = computeColorParams(ev, node, ctx);
    // Evaluated for the side effect of building the children's own CSGNodes
    // under the color-tagged context -- the returned bodies themselves are
    // unused here; generate reads them back via each child's own .bodies
    // and stamps `rgba` onto all of them. Mirrors _resolve_color.
    // The child block is its own scope -- see Evaluator::blockScope.
    EvalContext blockCtx = ev.blockScope(result.ctx);
    ev.evalChildren(node.children, blockCtx);
    return std::move(result.params);
}

namespace {

// Stamping cb.color is not enough: the colour has to be recorded against
// the body's RUN IDS too.
//
// attachTriColors (booleans.cpp) reads `idToColor` back AFTER a merge has
// thrown the individual bodies away -- run IDs are all it has left to go
// on -- and a run it cannot find looks uncoloured. tagGenerated() records
// the colour a body was BORN with, which covers `color("red") cube(10)`
// because the colour reaches the primitive through the context. It does
// not cover a body that already existed when colour was applied to it,
// which is every module whose geometry comes out of its own CSG:
// `union() { color("red") stroke(...); text(); }` had no red recorded for
// the stroke's runs at all, so every run looked identical, the merge kept
// the first child's colour, and the whole thing came out red.
void recordRunColors(Evaluator& ev, const ColoredBody& b, const std::optional<std::array<float, 4>>& rgba) {
    if (!b.body || b.body->IsEmpty()) return;
    // A body that is still one original knows its own ID without building
    // a mesh -- the common case, and the cheap one.
    const int original = b.body->OriginalID();
    if (original >= 0) {
        ev.idToColor[static_cast<uint32_t>(original)] = rgba;
        return;
    }
    for (uint32_t id : b.body->GetMeshGL().runOriginalID) ev.idToColor[id] = rgba;
}

} // namespace

std::vector<ColoredBody> generateColor(Evaluator& ev, const CSGParams& params,
                                        const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode&) {
    const auto rgba = valueToColor(params.at("rgba"));
    std::vector<ColoredBody> result;
    for (ColoredBody b : flattenCsgTree(children)) {
        b.color = rgba;
        if (!ev.measuring()) recordRunColors(ev, b, rgba);
        result.push_back(std::move(b));
    }
    return result;
}

} // namespace oscadeval
