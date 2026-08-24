// $_BELFRYSCAD and supported_feature() -- how a script asks which evaluator
// it is running under, and what that evaluator can actually do.
//
// The hazard they exist for: OpenSCAD does not reject arguments or names it
// has never heard of. `children(separate=true)` is silently ignored there,
// so a script using an extension runs and quietly renders something else.
// Both of these are readable from OpenSCAD too -- an unknown $-variable and
// an unknown function are undef (with a warning), not errors -- so a guard
// written against them works in both.

#include "openscad_cpp_evaluator/evaluator.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

class ScopedVm {
public:
    explicit ScopedVm(bool enabled) { Evaluator::setBytecodeVmEnabledForTesting(enabled); }
    ~ScopedVm() { Evaluator::setBytecodeVmEnabledForTesting(std::nullopt); }
};

std::vector<std::string> echoesFrom(const std::string& src) {
    std::vector<std::string> out;
    evalSrc(src, [&](const std::string& m) { out.push_back(m); });
    return out;
}

// Every feature name the docs promise. A typo in either direction -- here or
// in featureLevels() -- shows up as a 0.
const char* kFeatures[] = {"separate-children", "minkowski-diff", "sphere-styles", "export-name",
                            "simplify-op",       "expr-import",    "object-function", "roof-op"};

} // namespace

TEST(FeatureDetection, BelfryscadVariableIsTheEvaluatorVersion) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        // Shaped like OpenSCAD's own version(): [major, minor, patch], so a
        // comparison needs no string splitting. Read from pyproject.toml at
        // configure time, so this cannot drift from the released version.
        const std::vector<std::string> e = echoesFrom("echo(is_list($_BELFRYSCAD), len($_BELFRYSCAD));");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm;
        EXPECT_NE(e[0].find("true"), std::string::npos) << "vm=" << vm << ", got " << e[0];
        EXPECT_NE(e[0].find("3"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}

TEST(FeatureDetection, BelfryscadVariableHoldsThreeNumbers) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e = echoesFrom(
            "echo(is_num($_BELFRYSCAD[0]), is_num($_BELFRYSCAD[1]), is_num($_BELFRYSCAD[2]));");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm;
        EXPECT_EQ(e[0].find("false"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}

TEST(FeatureDetection, EveryDocumentedFeatureReportsALevel) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        for (const char* f : kFeatures) {
            const std::vector<std::string> e =
                echoesFrom(std::string("echo(supported_feature(\"") + f + "\"));");
            ASSERT_EQ(e.size(), 1u) << f << ", vm=" << vm;
            EXPECT_NE(e[0].find("1"), std::string::npos) << f << ", vm=" << vm << ", got " << e[0];
            EXPECT_EQ(e[0].find("0"), std::string::npos) << f << " reported unsupported, vm=" << vm;
        }
    }
}

TEST(FeatureDetection, AnUnknownFeatureIsZeroAndSilent) {
    // Probing for a feature from a future release has to be safe -- that is
    // the whole point of asking. No warning, no error, just 0.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e = echoesFrom("echo(supported_feature(\"time-travel\"));");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm << " (unexpected warning?)";
        EXPECT_NE(e[0].find("0"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}

TEST(FeatureDetection, ANonStringArgumentIsZeroRatherThanAnError) {
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        for (const char* arg : {"42", "undef", "[1,2]"}) {
            const std::vector<std::string> e =
                echoesFrom(std::string("echo(supported_feature(") + arg + "));");
            ASSERT_EQ(e.size(), 1u) << arg << ", vm=" << vm;
            EXPECT_NE(e[0].find("0"), std::string::npos) << arg << ", vm=" << vm << ", got " << e[0];
        }
    }
}

TEST(FeatureDetection, TheGuardIdiomReadsTrueHere) {
    // What a script actually writes. In OpenSCAD $_BELFRYSCAD is undef and
    // supported_feature() is an unknown function returning undef -- both
    // falsy -- so the same source takes the other branch there.
    for (bool vm : {false, true}) {
        ScopedVm guard(vm);
        const std::vector<std::string> e = echoesFrom(
            "if ($_BELFRYSCAD == undef) echo(\"portable\");\n"
            "else if (supported_feature(\"separate-children\")) echo(\"extended\");\n");
        ASSERT_EQ(e.size(), 1u) << "vm=" << vm;
        EXPECT_NE(e[0].find("extended"), std::string::npos) << "vm=" << vm << ", got " << e[0];
    }
}
