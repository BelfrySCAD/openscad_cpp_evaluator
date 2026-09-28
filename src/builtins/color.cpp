#include "builtins.hpp"

#include <cstdio>

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/css_colors.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"

namespace oscadeval {

// color(c, alpha), as OpenSCAD's builtin_color (ColorNode.cc):
// - a string goes through parseColor (CSS / "xkcd:" names, hex with or
//   without alpha); one it cannot read warns "Unable to parse color" and
//   leaves the children's colour alone -- inherited from outside, not reset;
// - a list fills [r, g, b, a] in order, 1.0 for any it lacks, warning on a
//   component outside 0..1;
// - a numeric `alpha` then replaces the alpha of whatever `c` gave -- even a
//   4-element list's, and even "#rrggbbaa"'s -- warning when outside 0..1;
// - anything else for `c`, or no `c` at all, sets no colour (OpenSCAD's
//   unset Color4f is (-1,-1,-1,-1), which is not a colour).

namespace {
std::string oneDecimal(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.1f", v);
    return buf;
}
} // namespace

BuiltinWrapParams computeColorParams(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    const Value cArg = getArg(args, 0, "c");
    const Value alphaArg = getArg(args, 1, "alpha");

    std::optional<std::array<double, 4>> rgba;
    if (const std::string* s = std::get_if<std::string>(&cArg)) {
        rgba = parseColor(*s);
        if (!rgba) ev.warn("Unable to parse color \"" + *s + "\"", &node.position());
    } else if (const ListPtr* l = std::get_if<ListPtr>(&cArg); l && *l) {
        const auto& items = (*l)->items;
        std::array<double, 4> c{1.0, 1.0, 1.0, 1.0};
        for (size_t i = 0; i < 4; ++i) {
            if (i < items.size()) c[i] = toDoubleLenient(items[i]);
            if (c[i] > 1 || c[i] < 0)
                ev.warn("color() expects numbers between 0.0 and 1.0. Value of " + oneDecimal(c[i]) +
                            " is out of range",
                        &node.position());
        }
        rgba = c;
    }
    if (const double* a = std::get_if<double>(&alphaArg)) {
        if (rgba) (*rgba)[3] = *a;
        if (*a < 0.0 || *a > 1.0)
            ev.warn("color() expects alpha between 0.0 and 1.0. Value of " + oneDecimal(*a) + " is out of range",
                    &node.position());
    }

    // No colour: the child context inherits the outer one (childCtx's
    // nullopt), and generateColor passes the children through untouched.
    EvalContext childCtx = effCtx.childCtx(nullptr, rgba);
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

} // namespace

namespace {
std::shared_ptr<const std::vector<ColoredBody>> recolorParts(const std::vector<ColoredBody>& parts,
                                                              const std::optional<std::array<float, 4>>& rgba) {
    std::vector<ColoredBody> out;
    for (ColoredBody part : parts) {
        part.color = rgba;
        if (part.mergedFrom) part.mergedFrom = recolorParts(*part.mergedFrom, rgba);
        out.push_back(std::move(part));
    }
    return std::make_shared<const std::vector<ColoredBody>>(std::move(out));
}
} // namespace

std::vector<ColoredBody> generateColor(Evaluator& ev, const CSGParams& params,
                                        const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode&) {
    const auto rgba = valueToColor(params.at("rgba"));
    // An unreadable colour sets none: the children keep their own.
    if (!rgba) return flattenCsgTree(children);
    std::vector<ColoredBody> result;
    for (ColoredBody b : flattenCsgTree(children)) {
        b.color = rgba;
        if (!ev.measuring()) ev.recordRunColors(b, rgba);
        // color() over a union() colours every part it was merged from
        // (see ColoredBody::mergedFrom) -- the parts' runs are the body's
        // runs, so recording once above already covers them.
        if (b.mergedFrom) b.mergedFrom = recolorParts(*b.mergedFrom, rgba);
        result.push_back(std::move(b));
    }
    return result;
}

} // namespace oscadeval
