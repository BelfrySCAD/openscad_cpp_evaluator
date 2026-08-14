#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/font_match.hpp"

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

TEST(Fontmetrics, UnmatchedFontFallsBackToTheBundledOne) {
    // fontconfig-style matching never fails -- an unavailable font
    // silently becomes the nearest available one, which for this provider
    // is always the bundled font. A model asking for a font the machine
    // doesn't have must still render, not error.
    Evaluator ev;
    Value v = asExpr("fontmetrics(size=10, font=\"No Such Font Exists Anywhere\")", ev);
    EXPECT_EQ(std::get<std::string>(objGet(objGet(v, "font"), "family")), "Liberation Sans");
}

// -- shaping ---------------------------------------------------------------

TEST(TextShaping, KerningPullsAPairTighterThanItsGlyphsMeasuredApart) {
    // The whole reason layout goes through a shaper: "AV" is a kern pair
    // in Liberation Sans, so shaping the run must produce a shorter
    // advance than adding up the two glyphs' own advance widths. Measuring
    // per character, as a naive layout does, cannot see this.
    Evaluator ev;
    const auto advanceOf = [&](const std::string& s) {
        Value v = asExpr("textmetrics(text=\"" + s + "\", size=10)", ev);
        return asNum(std::get<ListPtr>(objGet(v, "advance"))->items[0]);
    };
    const double apart = advanceOf("A") + advanceOf("V");
    const double shaped = advanceOf("AV");
    EXPECT_LT(shaped, apart);
    // A kern, not a collapse: the pair is still nearly as wide as its parts.
    EXPECT_GT(shaped, apart * 0.9);
}

TEST(TextShaping, DirectionAndScriptAreAcceptedAndMeasured) {
    // Explicit direction/script are passed to the shaper rather than
    // ignored. Whether the bundled font covers the script doesn't matter
    // here -- what matters is that the options reach it and a run comes
    // back measurable.
    Evaluator ev;
    Value v = asExpr("textmetrics(text=\"abc\", size=10, direction=\"rtl\", script=\"Latn\", language=\"en\")", ev);
    EXPECT_GT(asNum(std::get<ListPtr>(objGet(v, "advance"))->items[0]), 0.0);
}

TEST(TextShaping, TrailingSpaceAdvancesThePenWithoutGrowingTheInkBox) {
    Evaluator ev;
    Value bare = asExpr("textmetrics(text=\"Hi\", size=10)", ev);
    Value spaced = asExpr("textmetrics(text=\"Hi \", size=10)", ev);
    const auto width = [&](const Value& v) { return asNum(std::get<ListPtr>(objGet(v, "size"))->items[0]); };
    const auto advance = [&](const Value& v) { return asNum(std::get<ListPtr>(objGet(v, "advance"))->items[0]); };
    EXPECT_NEAR(width(spaced), width(bare), 1e-9);
    EXPECT_GT(advance(spaced), advance(bare));
}

TEST(Text, CounterOfAnOIsAHoleNotAFilledBlob) {
    // Glyph contours are wound so that inner contours run opposite the
    // outer one; building the CrossSection with the NonZero fill rule is
    // what turns that into a hole. If the rule or the winding were wrong,
    // the 'o' would come out solid and its area would match its bounds.
    Evaluated e = evalSrc("text(\"o\", size=20, $fn=32);");
    ASSERT_TRUE(e.bodies[0].section.has_value());
    const manifold::Rect b = e.bodies[0].section->Bounds();
    const double boxArea = (b.max.x - b.min.x) * (b.max.y - b.min.y);
    EXPECT_GT(e.bodies[0].section->Area(), 0.0);
    EXPECT_LT(e.bodies[0].section->Area(), boxArea * 0.75);
}

// -- font matching (pure, no system fonts involved) ------------------------

TEST(FontMatch, ParsesFamilyListAndStyle) {
    const FontSpec s = parseFontSpec("Helvetica, Arial ,sans-serif:style=Bold Italic");
    ASSERT_EQ(s.families.size(), 3u);
    EXPECT_EQ(s.families[0], "Helvetica");
    EXPECT_EQ(s.families[1], "Arial");
    EXPECT_EQ(s.families[2], "sans-serif");
    EXPECT_EQ(s.style, "Bold Italic");
}

TEST(FontMatch, ParsesBareStyleAndIgnoresOtherProperties) {
    EXPECT_EQ(parseFontSpec("Courier New:Bold").style, "Bold");
    EXPECT_EQ(parseFontSpec("Courier New:size=12:style=Oblique").style, "Oblique");
    EXPECT_TRUE(parseFontSpec("Courier New:weight=200").style.empty());
}

TEST(FontMatch, EmptySpecHasNoFamilyToMatch) {
    EXPECT_TRUE(parseFontSpec("").families.empty());
    EXPECT_TRUE(parseFontSpec(":style=Bold").families.empty());
}

TEST(FontMatch, GenericFamiliesExpandToConcreteOnes) {
    EXPECT_FALSE(expandGenericFamily("sans-serif").empty());
    EXPECT_EQ(expandGenericFamily("monospace")[0], "Liberation Mono");
    EXPECT_TRUE(expandGenericFamily("Liberation Sans").empty());
}

TEST(FontMatch, PrefersExactStyleThenSubstringThenRegular) {
    const std::vector<FontFace> faces = {
        {"/f/sans.ttf", 0, "Test Sans", "Regular"},
        {"/f/sans.ttf", 1, "Test Sans", "Bold"},
        {"/f/sans.ttf", 2, "Test Sans", "Bold Italic"},
    };
    EXPECT_EQ(matchFace(parseFontSpec("Test Sans:style=Bold"), faces)->faceIndex, 1);
    EXPECT_EQ(matchFace(parseFontSpec("Test Sans:style=Italic"), faces)->faceIndex, 2);
    EXPECT_EQ(matchFace(parseFontSpec("Test Sans"), faces)->faceIndex, 0);
}

TEST(FontMatch, FamilyMatchIsCaseInsensitiveAndFallsThroughTheList) {
    const std::vector<FontFace> faces = {{"/f/a.ttf", 0, "Test Sans", "Regular"}};
    EXPECT_TRUE(matchFace(parseFontSpec("TEST SANS"), faces).has_value());
    EXPECT_TRUE(matchFace(parseFontSpec("Missing One,Test Sans"), faces).has_value());
    EXPECT_FALSE(matchFace(parseFontSpec("Missing One"), faces).has_value());
}

TEST(FontMatch, StyleIsAPreferenceNotAFilter) {
    // A family that exists but not in the requested style still matches --
    // dropping to Regular beats falling through to a different family
    // entirely, which is how fontconfig behaves too.
    const std::vector<FontFace> faces = {{"/f/a.ttf", 0, "Test Sans", "Regular"}};
    const std::optional<FontFace> hit = matchFace(parseFontSpec("Test Sans:style=Black"), faces);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->style, "Regular");
}
