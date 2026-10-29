#include "openscad_cpp_evaluator/export.hpp"

#include "test_helpers.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::vector<std::string> namesOf(const std::vector<ExportObject>& objs) {
    std::vector<std::string> out;
    for (const ExportObject& o : objs) out.push_back(o.name);
    return out;
}

std::vector<std::string> split(const std::string& code, bool components = false, bool colors = true) {
    Evaluated e = evaluateSrc(code);
    return namesOf(splitBodiesForExport(e.bodies, nullptr, components, colors));
}

std::vector<std::string> warningsOf(const std::string& code) {
    std::vector<std::string> log;
    evaluateSrc(code, [&](const std::string& m) { log.push_back(m); });
    return log;
}

std::string slurp(const std::filesystem::path& p) {
    std::ifstream in(p);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

TEST(ExportAs, TouchingNamedPartsStaySeparate) {
    // Same colour, sharing a face: without names these weld into one object.
    EXPECT_EQ(split(R"(export_as("jar") cube(10); export_as("lid") translate([10,0,0]) cube(10);)"),
              (std::vector<std::string>{"jar", "lid"}));
}

TEST(ExportAs, NameSurvivesTransformsAndColor) {
    EXPECT_EQ(split(R"(translate([5,0,0]) color("red") export_as("jar") rotate(10) cube(10);)"),
              (std::vector<std::string>{"jar"}));
}

TEST(ExportAs, UnnamedGeometryKeepsAutomaticNaming) {
    EXPECT_EQ(split(R"(export_as("jar") cube(10); translate([20,0,0]) cube(10);)"),
              (std::vector<std::string>{"jar", ""}));
}

TEST(ExportAs, NamedPartIsOneObjectEvenWithColorsNotSplit) {
    EXPECT_EQ(split(R"(export_as("jar") cube(10); export_as("lid") translate([10,0,0]) cube(10);)", false, false),
              (std::vector<std::string>{"jar", "lid"}));
}

TEST(ExportAs, PiecesOfOnePartAreNumbered) {
    const std::string code = R"(export_as("leg") { cube(1); translate([5,0,0]) cube(1); })";
    EXPECT_EQ(split(code), (std::vector<std::string>{"leg"}));
    EXPECT_EQ(split(code, /*components=*/true), (std::vector<std::string>{"leg_1", "leg_2"}));
}

TEST(ExportAs, InnerNameWins) {
    EXPECT_EQ(split(R"(export_as("kit") { export_as("jar") cube(1); translate([5,0,0]) cube(1); })"),
              (std::vector<std::string>{"jar", "kit"}));
}

TEST(ExportAs, NameInsideAMergingOperationWarns) {
    const auto log = warningsOf(R"(union() { export_as("jar") cube(1); cube(2); })");
    EXPECT_TRUE(anyContains(log, "export_as(\"jar\") is inside union()"));
    // Through a user module too: the module is not the merger, union() is.
    EXPECT_TRUE(anyContains(warningsOf(R"(module m() export_as("a") cube(1); difference() { m(); cube(0.5); })"),
                            "export_as(\"a\") is inside difference()"));
}

TEST(ExportAs, DuplicateNameWarnsAndMerges) {
    const std::string code = R"(export_as("jar") cube(1); export_as("jar") translate([5,0,0]) cube(1);)";
    EXPECT_TRUE(anyContains(warningsOf(code), "export_as(\"jar\") is used more than once"));
    EXPECT_EQ(split(code), (std::vector<std::string>{"jar"}));
}

TEST(ExportAs, MissingNameWarns) {
    EXPECT_TRUE(anyContains(warningsOf("export_as() cube(1);"), "export_as() needs a non-empty name string"));
    EXPECT_EQ(split("export_as() cube(1);"), (std::vector<std::string>{""}));
}

TEST(ExportAs, WritersCarryTheName) {
    Evaluated e = evaluateSrc(R"(export_as("big jar") cube(10); export_as("a&b") translate([20,0,0]) cube(1);)");
    const auto dir = std::filesystem::temp_directory_path();
    exportModel((dir / "oscad_eval_test_export_as.obj").string(), e.bodies);
    const std::string obj = slurp(dir / "oscad_eval_test_export_as.obj");
    EXPECT_NE(obj.find("o big_jar\n"), std::string::npos);
    exportModel((dir / "oscad_eval_test_export_as.amf").string(), e.bodies);
    const std::string amf = slurp(dir / "oscad_eval_test_export_as.amf");
    EXPECT_NE(amf.find("<metadata type=\"name\">a&amp;b</metadata>"), std::string::npos);
}

TEST(ExportAs, FeatureIsAdvertised) {
    EXPECT_TRUE(anyContains(warningsOf(R"(echo(supported_feature("export-as"));)"), "ECHO: 1"));
}
