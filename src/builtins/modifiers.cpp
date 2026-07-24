#include "builtins.hpp"

namespace oscadeval {

// The 3 tag modifiers (#/%/!) each just flatten their one child's already-
// generated bodies and stamp a role onto every one of them -- role tagging
// is the entire behavior; the actual isolation (excluding background/
// show_only from CSG merges, filtering to show_only+highlight at the
// top level) happens elsewhere (booleans.cpp's splitByRole,
// Evaluator::evaluate()'s top-level filter). `*` (disable) needs no
// generate function at all: it never gets a CSGNode in the first place
// (see Evaluator::evalStatement).

std::vector<ColoredBody> generateHighlight(Evaluator&, const CSGParams&, const std::vector<std::unique_ptr<CSGNode>>& children,
                                            const oscad::ASTNode&) {
    std::vector<ColoredBody> result = flattenCsgTree(children);
    for (ColoredBody& b : result) b.role = BodyRole::Highlight;
    return result;
}

std::vector<ColoredBody> generateBackground(Evaluator&, const CSGParams&,
                                             const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode&) {
    std::vector<ColoredBody> result = flattenCsgTree(children);
    for (ColoredBody& b : result) b.role = BodyRole::Background;
    return result;
}

std::vector<ColoredBody> generateShowOnly(Evaluator&, const CSGParams&,
                                           const std::vector<std::unique_ptr<CSGNode>>& children, const oscad::ASTNode&) {
    std::vector<ColoredBody> result = flattenCsgTree(children);
    for (ColoredBody& b : result) b.role = BodyRole::ShowOnly;
    return result;
}

} // namespace oscadeval
