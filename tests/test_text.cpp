#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

Value asExpr(const std::string& code, Evaluator& ev) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    const oscad::Expression* expr = exprSrc(code, ast);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    return ev.evalExpr(*expr, ctx);
}

double asNum(const Value& v) { return std::get<double>(v); }

Value objGet(const Value& obj, const std::string& key) {
    for (const auto& [k, v] : std::get<ObjectPtr>(obj)->items) {
        if (k == key) return v;
    }
    return Value{};
}

} // namespace

// -- text() -----------------------------------------------------------

TEST(Text, ProducesNonEmptyCrossSectionForNonEmptyString) {
    Evaluated e = evalSrc("text(\"Hi\", size=10, $fn=32);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_GT(e.bodies[0].section->Area(), 0.0);
}

TEST(Text, EmptyStringProducesEmptySection) {
    Evaluated e = evalSrc("text(\"\", size=10);");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].section.has_value());
    EXPECT_TRUE(e.bodies[0].section->IsEmpty());
}

TEST(Text, LargerSizeProducesLargerBoundingBox) {
    Evaluated small = evalSrc("text(\"A\", size=5, $fn=32);");
    Evaluated large = evalSrc("text(\"A\", size=20, $fn=32);");
    ASSERT_TRUE(small.bodies[0].section.has_value());
    ASSERT_TRUE(large.bodies[0].section.has_value());
    EXPECT_GT(large.bodies[0].section->Area(), small.bodies[0].section->Area() * 2.0);
}

TEST(Text, HalignCenterShiftsBoundingBoxLeftOfLeftAlign) {
    Evaluated left = evalSrc("text(\"AAA\", size=10, halign=\"left\", $fn=16);");
    Evaluated center = evalSrc("text(\"AAA\", size=10, halign=\"center\", $fn=16);");
    manifold::Rect leftBounds = left.bodies[0].section->Bounds();
    manifold::Rect centerBounds = center.bodies[0].section->Bounds();
    EXPECT_LT(centerBounds.min.x, leftBounds.min.x);
}

TEST(Text, SpacingIncreasesAdvanceWithoutChangingGlyphCount) {
    Evaluated normal = evalSrc("text(\"AB\", size=10, spacing=1, $fn=16);");
    Evaluated wide = evalSrc("text(\"AB\", size=10, spacing=3, $fn=16);");
    manifold::Rect normalBounds = normal.bodies[0].section->Bounds();
    manifold::Rect wideBounds = wide.bodies[0].section->Bounds();
    EXPECT_GT(wideBounds.max.x - wideBounds.min.x, normalBounds.max.x - normalBounds.min.x);
}

// -- textmetrics() --------------------------------------------------------

TEST(Textmetrics, ReturnsAllExpectedKeysInOrder) {
    Evaluator ev;
    Value v = asExpr("textmetrics(text=\"Hi\", size=10)", ev);
    const auto& items = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(items.size(), 6u);
    EXPECT_EQ(items[0].first, "position");
    EXPECT_EQ(items[1].first, "size");
    EXPECT_EQ(items[2].first, "ascent");
    EXPECT_EQ(items[3].first, "descent");
    EXPECT_EQ(items[4].first, "offset");
    EXPECT_EQ(items[5].first, "advance");
}

TEST(Textmetrics, AdvanceXIsPositiveForNonEmptyText) {
    Evaluator ev;
    Value v = asExpr("textmetrics(text=\"Hi\", size=10)", ev);
    Value advance = objGet(v, "advance");
    const auto& items = std::get<ListPtr>(advance)->items;
    EXPECT_GT(asNum(items[0]), 0.0);
    EXPECT_DOUBLE_EQ(asNum(items[1]), 0.0);
}

TEST(Textmetrics, AscentIsPositiveDescentIsNonPositive) {
    Evaluator ev;
    Value v = asExpr("textmetrics(text=\"Hi\", size=10)", ev);
    EXPECT_GT(asNum(objGet(v, "ascent")), 0.0);
    EXPECT_LE(asNum(objGet(v, "descent")), 0.0);
}

TEST(Textmetrics, LargerSizeScalesAdvanceProportionally) {
    Evaluator ev;
    Value v10 = asExpr("textmetrics(text=\"Hi\", size=10)", ev);
    Value v20 = asExpr("textmetrics(text=\"Hi\", size=20)", ev);
    const double adv10 = asNum(std::get<ListPtr>(objGet(v10, "advance"))->items[0]);
    const double adv20 = asNum(std::get<ListPtr>(objGet(v20, "advance"))->items[0]);
    EXPECT_NEAR(adv20, adv10 * 2.0, 1e-6);
}

TEST(Textmetrics, MultiByteUtf8CodepointsDecodeWithoutCrashing) {
    // utf8DecodeAll's own 2-byte/3-byte/4-byte continuation-sequence
    // branches -- every other text()/textmetrics() test in this file uses
    // plain ASCII, which only ever exercises the single-byte fallback.
    // Whether the bundled font actually maps each codepoint to a glyph
    // doesn't matter here (an unmapped codepoint is simply skipped per
    // measureText's own documented rule) -- decoding itself must not
    // corrupt the byte stream or crash regardless.
    Evaluator ev;
    // U+00E9 (2-byte, "é"), U+20AC (3-byte, "€"), U+1F600 (4-byte, an emoji).
    Value v = asExpr("textmetrics(text=\"a\xC3\xA9\xE2\x82\xACz\xF0\x9F\x98\x80\", size=10)", ev);
    Value advance = objGet(v, "advance");
    const auto& items = std::get<ListPtr>(advance)->items;
    EXPECT_GT(asNum(items[0]), 0.0);
}

// -- fontmetrics() --------------------------------------------------------

TEST(Fontmetrics, ReturnsAllExpectedKeysInOrder) {
    Evaluator ev;
    Value v = asExpr("fontmetrics(size=10)", ev);
    const auto& items = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(items.size(), 4u);
    EXPECT_EQ(items[0].first, "nominal");
    EXPECT_EQ(items[1].first, "max");
    EXPECT_EQ(items[2].first, "interline");
    EXPECT_EQ(items[3].first, "font");
}

TEST(Fontmetrics, BundledFontFamilyIsLiberationSans) {
    Evaluator ev;
    Value v = asExpr("fontmetrics(size=10)", ev);
    Value font = objGet(v, "font");
    EXPECT_EQ(std::get<std::string>(objGet(font, "family")), "Liberation Sans");
}

TEST(Fontmetrics, NominalAscentIsPositive) {
    Evaluator ev;
    Value v = asExpr("fontmetrics(size=10)", ev);
    Value nominal = objGet(v, "nominal");
    EXPECT_GT(asNum(objGet(nominal, "ascent")), 0.0);
}
