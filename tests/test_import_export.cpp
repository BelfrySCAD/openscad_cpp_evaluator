#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/export.hpp"
#include "openscad_cpp_evaluator/zip_stored.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <filesystem>
#include <map>
#include <fstream>
#include <gtest/gtest.h>
#include <manifold/manifold.h>

using namespace oscadeval;
using namespace oscadeval::test;

namespace {

std::filesystem::path tempPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("oscad_eval_test_" + name);
}

// A unit cube (size 2, centered -> volume 8) exported to `path` in every
// format under test, via the real evaluator pipeline (not a hand-built
// mesh) so export.cpp itself is exercised too.
void writeCubeAs(const std::filesystem::path& path, const std::string& format = "") {
    // Through exportModel rather than a writer directly: that is the entry
    // point every front end uses, so it is the one worth exercising.
    Evaluated e = evalSrc("cube(2, center=true);");
    ExportOptions opts;
    opts.format = format;
    exportModel(path.string(), e.bodies, opts);
}

Value asExpr(const std::string& code, Evaluator& ev) {
    std::vector<std::unique_ptr<oscad::ASTNode>> ast;
    const oscad::Expression* expr = exprSrc(code, ast);
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    return ev.evalExpr(*expr, ctx);
}

} // namespace

// -- Module-context import() (geometry statement) --------------------------

TEST(ImportModuleContext, StlRoundTripPreservesVolume) {
    const auto path = tempPath("cube.stl");
    writeCubeAs(path);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, ObjRoundTripPreservesVolume) {
    const auto path = tempPath("cube.obj");
    writeCubeAs(path);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, OffRoundTripPreservesVolume) {
    const auto path = tempPath("cube.off");
    writeCubeAs(path);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, ThreeMfRoundTripPreservesVolume) {
    const auto path = tempPath("cube.3mf");
    writeCubeAs(path);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body.has_value());
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

// The .3mf writeThreeMf() produces is DEFLATE-compressed, not stored. The
// round-trip test above passes either way -- the reader accepts both
// methods -- so without this a regression to STORED would show up only as
// files several times bigger than they need to be.
TEST(ImportModuleContext, ThreeMfIsDeflateCompressed) {
    const auto path = tempPath("cube_compressed.3mf");
    writeCubeAs(path);

    std::ifstream in(path, std::ios::binary);
    ASSERT_TRUE(in);
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();

    bool sawModel = false;
    for (size_t i = 0; i + 46 <= buf.size(); ++i) {
        if (!(buf[i] == 0x50 && buf[i + 1] == 0x4B && buf[i + 2] == 0x01 && buf[i + 3] == 0x02)) continue;
        const uint16_t method = static_cast<uint16_t>(buf[i + 10] | (buf[i + 11] << 8));
        const uint32_t compressedSize =
            static_cast<uint32_t>(buf[i + 20] | (buf[i + 21] << 8) | (buf[i + 22] << 16) | (buf[i + 23] << 24));
        const uint32_t rawSize =
            static_cast<uint32_t>(buf[i + 24] | (buf[i + 25] << 8) | (buf[i + 26] << 16) | (buf[i + 27] << 24));
        const uint16_t nameLen = static_cast<uint16_t>(buf[i + 28] | (buf[i + 29] << 8));
        if (i + 46 + nameLen > buf.size()) continue;
        const std::string name(reinterpret_cast<const char*>(&buf[i + 46]), nameLen);
        if (name.find("3dmodel.model") == std::string::npos) continue;
        sawModel = true;
        EXPECT_EQ(method, 8) << "3dmodel.model stored uncompressed";
        EXPECT_LT(compressedSize, rawSize) << "compressed " << compressedSize << " vs raw " << rawSize;
    }
    EXPECT_TRUE(sawModel) << "no 3dmodel.model entry in the archive";

    std::filesystem::remove(path);
}

// -- AMF, X3D, VRML: the multi-object formats export writes ------------------

TEST(ImportModuleContext, AmfRoundTripPreservesVolume) {
    const auto path = tempPath("cube.amf");
    writeCubeAs(path);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, X3dRoundTripPreservesVolume) {
    const auto path = tempPath("cube.x3d");
    writeCubeAs(path);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, VrmlRoundTripPreservesVolume) {
    const auto path = tempPath("cube.wrl");
    writeCubeAs(path);
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-6);
    std::filesystem::remove(path);
}

namespace {

// A unit cube as six quads; the last polygon deliberately has no closing -1,
// which both specs allow.
const char* kCubePoints = "0 0 0, 1 0 0, 1 1 0, 0 1 0, 0 0 1, 1 0 1, 1 1 1, 0 1 1";
const char* kCubeQuads = "0 3 2 1 -1 4 5 6 7 -1 0 1 5 4 -1 1 2 6 5 -1 2 3 7 6 -1 3 0 4 7";

struct Imported {
    double volume = 0;
    manifold::Box box;
    std::vector<std::string> messages;
};

Imported importText(const std::string& name, const std::string& text) {
    const auto path = tempPath(name);
    std::ofstream(path, std::ios::binary) << text;
    Imported out;
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");",
                          [&](const std::string& m) { out.messages.push_back(m); });
    std::filesystem::remove(path);
    if (!e.bodies.empty() && e.bodies[0].body) {
        out.volume = e.bodies[0].body->Volume();
        out.box = e.bodies[0].body->BoundingBox();
    }
    return out;
}

} // namespace

// A cube scaled x2 and moved, the same cube USEd again under a 90-degree
// rotation, a tetrahedron written clockwise with ccw="false", and a Box
// primitive that is skipped with a warning. 8 + 1 + 4.5 = 13.5.
TEST(ImportModuleContext, X3dAppliesTransformsDefUseAndCcw) {
    const std::string x3d = std::string(R"(<?xml version="1.0"?>
<!DOCTYPE X3D PUBLIC "ISO//Web3D//DTD X3D 3.3//EN" "x">
<X3D><Scene>
  <!-- <Shape> in a comment is not a shape -->
  <Transform translation="10 0 0" scale="2 2 2">
    <Shape DEF="CUBE"><IndexedFaceSet coordIndex=")") + kCubeQuads + R"("><Coordinate point=")" + kCubePoints + R"("/></IndexedFaceSet></Shape>
  </Transform>
  <Transform translation="30 0 0" rotation="0 0 1 1.5707963267948966"><Shape USE="CUBE"/></Transform>
  <Shape><IndexedTriangleSet ccw="false" index="0 1 2 0 3 1 0 2 3 1 3 2"><Coordinate point="50 0 0, 53 0 0, 50 3 0, 50 0 3"/></IndexedTriangleSet></Shape>
  <Shape><Box size="4 4 4"/></Shape>
</Scene></X3D>)";
    const Imported r = importText("hand.x3d", x3d);
    EXPECT_NEAR(r.volume, 13.5, 1e-9);
    EXPECT_NEAR(r.box.min.x, 10, 1e-9);   // the scaled cube
    EXPECT_NEAR(r.box.max.x, 53, 1e-9);   // the tetrahedron
    EXPECT_TRUE(anyContains(r.messages, "skipped what is not a mesh: 1 Box")) << ::testing::PrintToString(r.messages);
}

// The same scene in VRML97, plus a PROTO, a ROUTE and comments the parser
// has to step over. The rotated cube spans x 29..30 -- 31 would mean the
// rotation was ignored.
TEST(ImportModuleContext, VrmlAppliesTransformsAndSkipsProtoAndRoute) {
    const std::string wrl = std::string("#VRML V2.0 utf8\n# a comment\n"
        "PROTO Unused [ field SFFloat x 1 ] { Group { } }\n"
        "Transform { translation 10 0 0 scale 2 2 2 children [\n"
        "  DEF CUBE Shape { geometry IndexedFaceSet { coord Coordinate { point [ ") + kCubePoints +
        " ] } coordIndex [ " + kCubeQuads + " ] } }\n] }\n"
        "Transform { translation 30 0 0 rotation 0 0 1 1.5707963267948966 children [ USE CUBE ] }\n"
        "Group { children [ Shape { geometry Sphere { radius 2 } } ] }\n"
        "DEF T TimeSensor { }\nROUTE T.fraction_changed TO T.set_startTime\n";
    const Imported r = importText("hand.wrl", wrl);
    EXPECT_NEAR(r.volume, 9.0, 1e-9);
    EXPECT_NEAR(r.box.max.x, 30, 1e-9);
    EXPECT_TRUE(anyContains(r.messages, "skipped what is not a mesh: 1 Sphere")) << ::testing::PrintToString(r.messages);
}

TEST(ImportModuleContext, VrmlIndexPastItsPointsErrors) {
    const auto path = tempPath("bad.wrl");
    std::ofstream(path) << "#VRML V2.0 utf8\nShape { geometry IndexedFaceSet { coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }"
                           " coordIndex [ 0 1 7 -1 ] } }\n";
    const std::string err = importFailure("import(\"" + path.generic_string() + "\");");
    EXPECT_EQ(err.rfind("ERROR: ", 0), 0u) << err;
    std::filesystem::remove(path);
}

namespace {

// The unit cube translated by `x` as one X3D / VRML node.
std::string x3dCubeAt(int x) {
    return "<Transform translation=\"" + std::to_string(x) + " 0 0\"><Shape><IndexedFaceSet coordIndex=\"" +
           kCubeQuads + "\"><Coordinate point=\"" + kCubePoints + "\"/></IndexedFaceSet></Shape></Transform>";
}
std::string wrlCubeAt(int x) {
    return "Transform { translation " + std::to_string(x) +
           " 0 0 children [ Shape { geometry IndexedFaceSet { coord Coordinate { point [ " + kCubePoints +
           " ] } coordIndex [ " + kCubeQuads + " ] } } ] }\n";
}

} // namespace

// Switch draws only its whichChoice child, and nothing at -1 (the default);
// LOD draws only its first level. Every child was imported before: 7 cubes.
// Here: Switch 1 -> the cube at 3; Switch -1 and a bare Switch -> nothing;
// LOD -> the cube at 9. Two cubes, x 3..10.
TEST(ImportModuleContext, X3dSwitchAndLodImportOnlyTheSelectedChild) {
    const std::string x3d = "<X3D><Scene><Switch whichChoice=\"1\">" + x3dCubeAt(0) + x3dCubeAt(3) +
                            "</Switch><Switch whichChoice=\"-1\">" + x3dCubeAt(6) + "</Switch><Switch>" +
                            x3dCubeAt(6) + "</Switch><Switch whichChoice=\"5\">" + x3dCubeAt(6) + "</Switch><LOD>" +
                            x3dCubeAt(9) + x3dCubeAt(12) + "</LOD></Scene></X3D>";
    const Imported r = importText("switch.x3d", x3d);
    EXPECT_NEAR(r.volume, 2.0, 1e-9);
    EXPECT_NEAR(r.box.min.x, 3, 1e-9);
    EXPECT_NEAR(r.box.max.x, 10, 1e-9);
}

TEST(ImportModuleContext, VrmlSwitchAndLodImportOnlyTheSelectedChild) {
    const std::string wrl = "#VRML V2.0 utf8\nSwitch { whichChoice 1 choice [ " + wrlCubeAt(0) + wrlCubeAt(3) +
                            " ] }\nSwitch { whichChoice -1 choice [ " + wrlCubeAt(6) + " ] }\nSwitch { choice [ " +
                            wrlCubeAt(6) + " ] }\nSwitch { whichChoice 5 choice [ " + wrlCubeAt(6) +
                            " ] }\nLOD { level [ " + wrlCubeAt(9) + wrlCubeAt(12) + " ] }\n";
    const Imported r = importText("switch.wrl", wrl);
    EXPECT_NEAR(r.volume, 2.0, 1e-9);
    EXPECT_NEAR(r.box.min.x, 3, 1e-9);
    EXPECT_NEAR(r.box.max.x, 10, 1e-9);
}

// Prototypes are not expanded. An X3D ProtoDeclare's body is a template, so
// it is no longer imported as if it were geometry (it was: a cube at the
// origin), and the declaration and the instance are both reported.
TEST(ImportModuleContext, X3dPrototypesAreSkippedWithAWarning) {
    const std::string x3d = "<X3D><Scene><ProtoDeclare name=\"Box1\"><ProtoInterface><field name=\"t\" "
                            "type=\"SFVec3f\" accessType=\"initializeOnly\" value=\"0 0 0\"/></ProtoInterface><ProtoBody>" +
                            x3dCubeAt(0) + "</ProtoBody></ProtoDeclare><ProtoInstance name=\"Box1\"><fieldValue name=\"t\" "
                            "value=\"5 0 0\"/></ProtoInstance>" + x3dCubeAt(20) + "</Scene></X3D>";
    const Imported r = importText("proto.x3d", x3d);
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
    EXPECT_NEAR(r.box.min.x, 20, 1e-9);
    EXPECT_TRUE(anyContains(r.messages, "skipped what is not a mesh: 1 ProtoDeclare, 1 ProtoInstance"))
        << ::testing::PrintToString(r.messages);
}

// A VRML PROTO instance's geometry is lost; it used to be lost silently.
TEST(ImportModuleContext, VrmlProtoInstanceIsReported) {
    const std::string wrl = "#VRML V2.0 utf8\nPROTO Box1 [ field SFVec3f t 0 0 0 ] { " + wrlCubeAt(0) +
                            " }\nEXTERNPROTO Far [ ] \"far.wrl\"\nBox1 { t 5 0 0 }\nFar { }\n" + wrlCubeAt(20);
    const Imported r = importText("proto.wrl", wrl);
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
    EXPECT_TRUE(anyContains(r.messages, "skipped what is not a mesh: 2 PROTO instance"))
        << ::testing::PrintToString(r.messages);
}

namespace {

std::string inchCubeAmf() {
    const int tris[12][3] = {{0, 3, 2}, {0, 2, 1}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                             {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    const int pts[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    std::string amf = "<?xml version=\"1.0\"?>\n<amf unit=\"inch\"><object id=\"1\"><mesh><vertices>";
    for (const auto& p : pts)
        amf += "<vertex><coordinates><x>" + std::to_string(p[0]) + "</x><y>" + std::to_string(p[1]) + "</y><z>" +
               std::to_string(p[2]) + "</z></coordinates></vertex>";
    amf += "</vertices><volume>";
    for (const auto& t : tris)
        amf += "<triangle><v1>" + std::to_string(t[0]) + "</v1><v2>" + std::to_string(t[1]) + "</v2><v3>" +
               std::to_string(t[2]) + "</v3></triangle>";
    amf += "</volume></mesh></object><constellation id=\"2\"><instance objectid=\"1\"/></constellation></amf>";
    return amf;
}

} // namespace

// unit="inch" is ignored; a constellation is not applied, and says so.
TEST(ImportModuleContext, AmfUnitIsIgnoredAsInOpenSCAD) {
    // OpenSCAD reads an AMF's numbers as millimetres whatever its unit says
    // (checked, 2026.02.01: a unit="inch" unit cube imports as a unit cube),
    // and says AMF import is deprecated.
    const Imported r = importText("inch.amf", inchCubeAmf());
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
    EXPECT_NEAR(r.box.max.x, 1.0, 1e-9);
    EXPECT_TRUE(anyContains(r.messages, "AMF constellations are not applied"));
    EXPECT_TRUE(anyContains(r.messages, "DEPRECATED: AMF import is deprecated. Please use 3MF instead."));
}

// Compressed AMF: a zip holding one .amf, deflated.
TEST(ImportModuleContext, ZippedAmfImports) {
    const std::string amf = inchCubeAmf();
    const auto path = tempPath("zipped.amf");
    writeDeflateZip(path.string(), {ZipEntry{"model.amf", std::vector<uint8_t>(amf.begin(), amf.end())}});
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");", [](const std::string&) {});
    std::filesystem::remove(path);
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0, 1e-9);
}

namespace {

using Tri = std::array<std::array<double, 3>, 3>;

// The unit cube at (x, y, z) as 12 outward triangles, two per face, the
// faces in -z, +z, -y, +x, +y, -x order.
std::vector<Tri> cubeTris(double x, double y, double z) {
    const int idx[12][3] = {{0, 3, 2}, {0, 2, 1}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                            {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    const double c[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    std::vector<Tri> out;
    for (const auto& t : idx) {
        Tri tri;
        for (int k = 0; k < 3; ++k) tri[k] = {c[t[k]][0] + x, c[t[k]][1] + y, c[t[k]][2] + z};
        out.push_back(tri);
    }
    return out;
}

// One <object> per entry, every triangle carrying its own three vertex
// copies: a triangle soup, as an exporter that never shares vertices writes.
std::string soupAmf(const std::vector<std::vector<Tri>>& objects) {
    std::string amf = "<?xml version=\"1.0\"?>\n<amf unit=\"millimeter\">";
    for (const auto& tris : objects) {
        amf += "<object id=\"1\"><mesh><vertices>";
        for (const Tri& t : tris)
            for (const auto& p : t)
                amf += "<vertex><coordinates><x>" + std::to_string(p[0]) + "</x><y>" + std::to_string(p[1]) +
                       "</y><z>" + std::to_string(p[2]) + "</z></coordinates></vertex>";
        amf += "</vertices><volume>";
        for (size_t i = 0; i < tris.size(); ++i)
            amf += "<triangle><v1>" + std::to_string(3 * i) + "</v1><v2>" + std::to_string(3 * i + 1) + "</v2><v3>" +
                   std::to_string(3 * i + 2) + "</v3></triangle>";
        amf += "</volume></mesh></object>";
    }
    return amf + "</amf>";
}

} // namespace

// OpenSCAD treats an AMF object's vertices at exactly the same position as
// one vertex (checked, 2026.02.01), so a closed object written as a soup
// imports as a closed solid. Without the weld it was an open surface with
// 36 boundary edges, drawn but unable to take part in CSG.
TEST(ImportModuleContext, AmfTriangleSoupWeldsIntoAClosedSolid) {
    const Imported r = importText("soup.amf", soupAmf({cubeTris(0, 0, 0)}));
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
    EXPECT_FALSE(anyContains(r.messages, "not a closed solid")) << ::testing::PrintToString(r.messages);
}

// A 3x3x1 slab with a 1x1 hole through it (eight unit cubes around the
// middle, the faces between neighbours left out), as a soup: closed, genus 1.
TEST(ImportModuleContext, AmfSoupWithAThroughHoleWeldsClosed) {
    std::map<std::vector<std::array<double, 3>>, std::vector<Tri>> faces;
    std::map<std::vector<std::array<double, 3>>, int> uses;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            if (i == 1 && j == 1) continue;
            const std::vector<Tri> c = cubeTris(i, j, 0);
            for (size_t f = 0; f < c.size(); f += 2) {
                std::vector<std::array<double, 3>> key{c[f][0], c[f][1], c[f][2], c[f + 1][0], c[f + 1][1], c[f + 1][2]};
                std::sort(key.begin(), key.end());
                key.erase(std::unique(key.begin(), key.end()), key.end());
                faces[key] = {c[f], c[f + 1]};
                ++uses[key];
            }
        }
    std::vector<Tri> ring;
    for (const auto& [key, tris] : faces)
        if (uses[key] == 1) ring.insert(ring.end(), tris.begin(), tris.end());
    const auto path = tempPath("ring.amf");
    std::ofstream(path, std::ios::binary) << soupAmf({ring});
    std::vector<std::string> messages;
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");",
                          [&](const std::string& m) { messages.push_back(m); });
    std::filesystem::remove(path);
    ASSERT_EQ(e.bodies.size(), 1u);
    ASSERT_TRUE(e.bodies[0].body);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 8.0, 1e-9);
    EXPECT_EQ(e.bodies[0].body->Genus(), 1);
    EXPECT_FALSE(anyContains(messages, "not a closed solid")) << ::testing::PrintToString(messages);
}

// The weld is per object, as in OpenSCAD: a cube split into two objects,
// each half its triangles, is two open surfaces, not one closed cube.
TEST(ImportModuleContext, AmfWeldsWithinAnObjectOnly) {
    const std::vector<Tri> c = cubeTris(0, 0, 0);
    const Imported r = importText("halves.amf", soupAmf({{c.begin(), c.begin() + 6}, {c.begin() + 6, c.end()}}));
    EXPECT_TRUE(anyContains(r.messages, "not a closed solid")) << ::testing::PrintToString(r.messages);
}

namespace {

// `objects` as a triangle soup (every triangle with its own three vertex
// copies) in `ext`'s own notion of an object: an `o` group for OBJ, an
// <object> for 3MF, a Shape for X3D/VRML. OFF has no objects, so they are
// simply concatenated.
Imported importSoup(const std::string& ext, const std::vector<std::vector<Tri>>& objects) {
    const auto num = [](double d) { return std::to_string(d); };
    std::string text;
    if (ext == "off") {
        size_t n = 0;
        for (const auto& o : objects) n += o.size();
        text = "OFF\n" + std::to_string(3 * n) + " " + std::to_string(n) + " 0\n";
        for (const auto& o : objects)
            for (const Tri& t : o)
                for (const auto& p : t) text += num(p[0]) + " " + num(p[1]) + " " + num(p[2]) + "\n";
        for (size_t i = 0; i < n; ++i)
            text += "3 " + std::to_string(3 * i) + " " + std::to_string(3 * i + 1) + " " + std::to_string(3 * i + 2) + "\n";
    } else if (ext == "obj") {
        size_t base = 1;
        for (size_t k = 0; k < objects.size(); ++k) {
            text += "o part" + std::to_string(k) + "\n";
            for (const Tri& t : objects[k])
                for (const auto& p : t) text += "v " + num(p[0]) + " " + num(p[1]) + " " + num(p[2]) + "\n";
            for (size_t i = 0; i < objects[k].size(); ++i, base += 3)
                text += "f " + std::to_string(base) + " " + std::to_string(base + 1) + " " + std::to_string(base + 2) + "\n";
        }
    } else if (ext == "3mf") {
        text = "<?xml version=\"1.0\"?><model unit=\"millimeter\"><resources>";
        for (size_t k = 0; k < objects.size(); ++k) {
            text += "<object id=\"" + std::to_string(k + 1) + "\" type=\"model\"><mesh><vertices>";
            for (const Tri& t : objects[k])
                for (const auto& p : t) text += "<vertex x=\"" + num(p[0]) + "\" y=\"" + num(p[1]) + "\" z=\"" + num(p[2]) + "\"/>";
            text += "</vertices><triangles>";
            for (size_t i = 0; i < objects[k].size(); ++i)
                text += "<triangle v1=\"" + std::to_string(3 * i) + "\" v2=\"" + std::to_string(3 * i + 1) + "\" v3=\"" +
                        std::to_string(3 * i + 2) + "\"/>";
            text += "</triangles></mesh></object>";
        }
        text += "</resources><build/></model>";
        const auto path = tempPath("soup.3mf");
        writeDeflateZip(path.string(), {ZipEntry{"3D/3dmodel.model", std::vector<uint8_t>(text.begin(), text.end())}});
        Imported out;
        Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");",
                              [&](const std::string& m) { out.messages.push_back(m); });
        std::filesystem::remove(path);
        if (!e.bodies.empty() && e.bodies[0].body) out.volume = e.bodies[0].body->Volume();
        return out;
    } else {
        const bool x3d = ext == "x3d";
        text = x3d ? "<?xml version=\"1.0\"?><X3D><Scene>" : "#VRML V2.0 utf8\n";
        for (const auto& o : objects) {
            std::string pts, idx;
            for (size_t i = 0; i < o.size(); ++i) {
                for (const auto& p : o[i]) pts += num(p[0]) + " " + num(p[1]) + " " + num(p[2]) + ", ";
                idx += std::to_string(3 * i) + " " + std::to_string(3 * i + 1) + " " + std::to_string(3 * i + 2) + " -1 ";
            }
            text += x3d ? "<Shape><IndexedFaceSet coordIndex=\"" + idx + "\"><Coordinate point=\"" + pts +
                              "\"/></IndexedFaceSet></Shape>"
                        : "Shape { geometry IndexedFaceSet { coord Coordinate { point [ " + pts + "] } coordIndex [ " +
                              idx + "] } }\n";
        }
        if (x3d) text += "</Scene></X3D>";
    }
    return importText("soup." + ext, text);
}

std::vector<std::vector<Tri>> cubeHalves() {
    const std::vector<Tri> c = cubeTris(0, 0, 0);
    return {{c.begin(), c.begin() + 6}, {c.begin() + 6, c.end()}};
}

} // namespace

// Every mesh format welds exactly-equal vertices, as AMF and STL do, so a
// closed solid written as a triangle soup imports closed. OpenSCAD
// 2026.02.01 imports such an OFF, OBJ or 3MF as a solid that takes part in
// CSG; X3D and VRML (which it cannot import) follow AMF. Without the weld
// each was an open surface with 36 boundary edges.
TEST(ImportModuleContext, EveryMeshFormatWeldsATriangleSoupClosed) {
    for (const char* ext : {"off", "obj", "3mf", "x3d", "wrl"}) {
        const Imported r = importSoup(ext, {cubeTris(0, 0, 0)});
        EXPECT_NEAR(r.volume, 1.0, 1e-9) << ext;
        EXPECT_FALSE(anyContains(r.messages, "not a closed solid")) << ext << ::testing::PrintToString(r.messages);
    }
}

// Where the weld stops. OFF is one mesh, and OBJ welds across its `o`
// groups (checked, 2026.02.01: two cube halves in separate `o` groups, each
// with its own vertex copies, import as one closed cube) -- so the halves
// close. 3MF welds each object on its own (OpenSCAD reports both halves
// not closed), and X3D/VRML each Shape, as AMF does -- so they stay open.
TEST(ImportModuleContext, MeshWeldObjectBoundaryRule) {
    for (const char* ext : {"off", "obj"}) {
        const Imported r = importSoup(ext, cubeHalves());
        EXPECT_NEAR(r.volume, 1.0, 1e-9) << ext;
        EXPECT_FALSE(anyContains(r.messages, "not a closed solid")) << ext << ::testing::PrintToString(r.messages);
    }
    for (const char* ext : {"3mf", "x3d", "wrl"})
        EXPECT_TRUE(anyContains(importSoup(ext, cubeHalves()).messages, "not a closed solid")) << ext;
}

// A cube whose eight vertices are listed twice, plus a triangle naming a
// vertex and its copy: welded, that triangle has two equal corners and
// drops out, leaving the closed cube (as OpenSCAD has it).
TEST(ImportModuleContext, OffDuplicatedVerticesAndATriangleDegenerateAfterWelding) {
    const Imported r = importText("dup.off",
                                  "OFF\n16 13 0\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n0 0 1\n1 0 1\n1 1 1\n0 1 1\n"
                                  "0 0 0\n1 0 0\n1 1 0\n0 1 0\n0 0 1\n1 0 1\n1 1 1\n0 1 1\n"
                                  "3 0 11 2\n3 8 2 1\n3 4 13 6\n3 12 6 7\n3 0 9 5\n3 8 5 4\n"
                                  "3 1 10 6\n3 9 6 5\n3 2 11 7\n3 10 7 6\n3 3 8 4\n3 11 4 7\n3 0 8 1\n");
    EXPECT_NEAR(r.volume, 1.0, 1e-9);
    EXPECT_FALSE(anyContains(r.messages, "not a closed solid")) << ::testing::PrintToString(r.messages);
}

// A face index past the vertex list is an error, now that the weld reads
// every index (it was handed to Manifold unchecked before).
TEST(ImportModuleContext, ObjFaceIndexPastItsVerticesErrors) {
    const auto path = tempPath("bad.obj");
    std::ofstream(path) << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n";
    const std::string err = importFailure("import(\"" + path.generic_string() + "\");");
    EXPECT_EQ(err.rfind("ERROR: ", 0), 0u) << err;
    EXPECT_NE(err.find("face index 8 is past its 3 vertices"), std::string::npos) << err;
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, UnsupportedExtensionErrors) {
    const auto path = tempPath("unsupported.xyz");
    std::ofstream(path) << "x";
    const std::string err = importFailure("import(\"" + path.generic_string() + "\");");
    EXPECT_NE(err.find("Unsupported file format while trying to import file '\"" + path.generic_string() +
                       "\"', import() at line 1"),
              std::string::npos)
        << err;
}

TEST(ImportModuleContext, MissingFileArgumentErrors) {
    EXPECT_EQ(importFailure("import();"),
              "ERROR: Unsupported file format while trying to import file '\"\"', import() at line 1");
}

// filename= still works, with OpenSCAD's deprecation notice.
TEST(ImportModuleContext, FilenameIsADeprecatedSpellingOfFile) {
    const auto path = tempPath("legacy.off");
    std::ofstream(path) << "OFF\n4 4 0\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n3 0 2 1\n3 0 1 3\n3 1 2 3\n3 0 3 2\n";
    std::vector<std::string> log;
    Evaluated e = evalSrc("import(filename=\"" + path.generic_string() + "\");", [&](const std::string& m) { log.push_back(m); });
    std::filesystem::remove(path);
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0 / 6, 1e-9);
    EXPECT_TRUE(anyContains(log, "DEPRECATED: filename= is deprecated. Please use file=")) << ::testing::PrintToString(log);
}

// COFF (per-vertex colours) and counts on the header line both import.
TEST(ImportModuleContext, OffVariantsImport) {
    for (const std::string off : {"COFF\n4 4 0\n0 0 0 1 0 0 1\n1 0 0 1 0 0 1\n0 1 0 1 0 0 1\n0 0 1 1 0 0 1\n",
                                  "OFF 4 4 0\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n"}) {
        const auto path = tempPath("variant.off");
        std::ofstream(path) << off << "3 0 2 1\n3 0 1 3\n3 1 2 3\n3 0 3 2\n";
        Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");");
        std::filesystem::remove(path);
        ASSERT_EQ(e.bodies.size(), 1u) << off;
        EXPECT_NEAR(e.bodies[0].body->Volume(), 1.0 / 6, 1e-9) << off;
    }
}

// A truncated OFF read past the end of its lines.
TEST(ImportModuleContext, TruncatedOffFailsCleanly) {
    const auto path = tempPath("trunc.off");
    std::ofstream(path) << "OFF\n4 4 0\n0 0 0\n";
    const std::string err = importFailure("import(\"" + path.generic_string() + "\");");
    std::filesystem::remove(path);
    EXPECT_NE(err.find("file is truncated"), std::string::npos) << err;
}

TEST(ImportModuleContext, JsonExtensionErrorsAsGeometryStatement) {
    const auto path = tempPath("data_as_module.json");
    std::ofstream(path) << R"({"a": 1})";
    const std::string err = importFailure("import(\"" + path.generic_string() + "\");");
    EXPECT_NE(err.find("Unsupported file format"), std::string::npos) << err;
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, MalformedMeshFileErrors) {
    // OpenSCAD's wording, and the render goes on.
    const auto path = tempPath("malformed.stl");
    std::ofstream(path) << "this is not a valid STL file at all";
    const std::string err = importFailure("import(\"" + path.generic_string() + "\");");
    EXPECT_NE(err.find("STL format not recognized in '"), std::string::npos) << err;
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, NonManifoldMeshWarns) {
    // generateImport's own Manifold::Status() != NoError branch -- a
    // single free-floating triangle (not welded to anything, non-manifold
    // boundary) via a hand-written OFF file, distinct from every other
    // mesh test here which round-trips a valid, closed, watertight cube.
    const auto path = tempPath("nonmanifold.off");
    {
        std::ofstream out(path);
        out << "OFF\n3 1 0\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n";
    }
    std::string lastWarning;
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");", [&](const std::string& msg) { lastWarning = msg; });
    EXPECT_NE(lastWarning.find("import: mesh is not a closed solid"), std::string::npos);
    // The triangle is now handed back for display rather than dropped: a
    // file that warns once and then shows nothing gives no way to see what
    // is actually wrong with it. It carries no Manifold, so it still can't
    // take part in a CSG operation.
    ASSERT_EQ(e.bodies.size(), 1u);
    EXPECT_TRUE(e.bodies[0].isDisplayOnly());
    EXPECT_EQ(e.bodies[0].rawMesh->triVerts.size(), 3u);
    std::filesystem::remove(path);
}

TEST(ImportModuleContext, EmptyMeshDrawsNothing) {
    const auto path = tempPath("empty.off");
    std::ofstream(path) << "OFF\n0 0 0\n";
    std::vector<std::string> log;
    Evaluated e = evalSrc("import(\"" + path.generic_string() + "\");", [&](const std::string& m) { log.push_back(m); });
    EXPECT_TRUE(e.bodies.empty());
    EXPECT_TRUE(log.empty()) << ::testing::PrintToString(log);
    std::filesystem::remove(path);
}

// -- Expression-context import() -------------------------------------------

TEST(ImportExpressionContext, StlReturnsVnfShape) {
    const auto path = tempPath("cube_vnf.stl");
    writeCubeAs(path);
    Evaluator ev;
    Value v = asExpr("import(\"" + path.generic_string() + "\")", ev);
    const auto& outer = std::get<ListPtr>(v)->items;
    ASSERT_EQ(outer.size(), 2u);
    const auto& verts = std::get<ListPtr>(outer[0])->items;
    const auto& faces = std::get<ListPtr>(outer[1])->items;
    EXPECT_EQ(verts.size(), 8u); // a welded cube has 8 corners
    EXPECT_GT(faces.size(), 0u);
    std::filesystem::remove(path);
}

TEST(ImportExpressionContext, JsonReturnsNativeValues) {
    const auto path = tempPath("data.json");
    {
        std::ofstream out(path);
        out << R"({"name": "x", "n": 3, "nested": {"a": 1, "b": 2}, "list": [1, 2, 3]})";
    }
    Evaluator ev;
    Value v = asExpr("import(\"" + path.generic_string() + "\")", ev);
    const auto& obj = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(obj.size(), 4u);
    EXPECT_EQ(obj[0].first, "name");
    EXPECT_EQ(std::get<std::string>(obj[0].second), "x");
    EXPECT_EQ(obj[1].first, "n");
    EXPECT_DOUBLE_EQ(std::get<double>(obj[1].second), 3.0);
    const auto& nested = std::get<ObjectPtr>(obj[2].second)->items;
    ASSERT_EQ(nested.size(), 2u);
    EXPECT_EQ(nested[0].first, "a");
    const auto& list = std::get<ListPtr>(obj[3].second)->items;
    ASSERT_EQ(list.size(), 3u);
    EXPECT_DOUBLE_EQ(std::get<double>(list[2]), 3.0);
    std::filesystem::remove(path);
}

TEST(ImportExpressionContext, JsonNullValueBecomesUndef) {
    // jsonToValue's own final fallback (null, or any other unhandled JSON
    // node type) -- every other JsonReturnsNativeValues field above is a
    // string/number/object/list.
    const auto path = tempPath("data_null.json");
    {
        std::ofstream out(path);
        out << R"({"x": null})";
    }
    Evaluator ev;
    Value v = asExpr("import(\"" + path.generic_string() + "\")", ev);
    const auto& obj = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(obj.size(), 1u);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(obj[0].second));
    std::filesystem::remove(path);
}

TEST(ImportExpressionContext, DxfReturnsRegionContours) {
    const auto path = tempPath("square_expr_ie.dxf");
    {
        std::ofstream out(path);
        out << "0\nSECTION\n2\nENTITIES\n0\nLWPOLYLINE\n8\n0\n90\n4\n70\n1\n"
               "10\n0.0\n20\n0.0\n10\n1.0\n20\n0.0\n10\n1.0\n20\n1.0\n10\n0.0\n20\n1.0\n"
               "0\nENDSEC\n0\nEOF\n";
    }
    Evaluator ev;
    Value v = asExpr("import(\"" + path.generic_string() + "\")", ev);
    const auto& contours = std::get<ListPtr>(v)->items;
    ASSERT_EQ(contours.size(), 1u);
    std::filesystem::remove(path);
}

TEST(ImportExpressionContext, MissingFileArgumentErrors) {
    Evaluator ev;
    EXPECT_THROW(asExpr("import()", ev), EvalError);
}

TEST(ImportExpressionContext, UnsupportedExtensionErrors) {
    const auto path = tempPath("unsupported_expr.xyz");
    std::ofstream(path) << "x";
    Evaluator ev;
    EXPECT_THROW(asExpr("import(\"" + path.generic_string() + "\")", ev), EvalError);
}

// A missing file warns and carries on, in OpenSCAD 2026.02.01's words; it
// aborted the render. The module form warns at generate, after the echoes.
TEST(ImportMissingFile, ModuleFormWarnsAndImportsNothing) {
    const auto dir = tempPath("missing_dir");
    std::filesystem::create_directories(dir);
    std::vector<std::string> log;
    Evaluator ev([&](const std::string& m) { log.push_back(m); });
    const std::string stl = (dir / "nope.stl").generic_string(), dxf = (dir / "nope.dxf").generic_string();
    auto ast = parseSrc("import(\"" + stl + "\");\nimport(\"" + dxf + "\");\necho(1);\ncube(1);\n");
    auto scope = oscad::buildScopes(ast);
    EvalContext ctx = EvalContext::makeRoot(scope.get());
    const std::vector<ColoredBody> bodies = ev.evaluate(ast, ctx);
    EXPECT_EQ(bodies.size(), 1u);
    ASSERT_EQ(log.size(), 3u);
    EXPECT_EQ(log[0], "ECHO: 1");
    EXPECT_EQ(log[1], "WARNING: Can't open import file '" + stl + "', import() at line 1");
    EXPECT_EQ(log[2], "WARNING: Can't open DXF file '" + dxf + "'.");
}

TEST(ImportMissingFile, ExpressionFormWarnsAndIsUndef) {
    std::string last;
    Evaluator ev([&](const std::string& m) { last = m; });
    const std::string path = tempPath("nope_expr.json").generic_string();
    EXPECT_TRUE(std::holds_alternative<std::monostate>(asExpr("import(\"" + path + "\")", ev)));
    EXPECT_EQ(last.rfind("WARNING: Could not read file '" + path + "'", 0), 0u) << last;
}

TEST(ImportExpressionContext, MalformedMeshFileErrors) {
    const auto path = tempPath("malformed_expr.stl");
    {
        std::ofstream out(path);
        out << "not a valid STL file";
    }
    Evaluator ev;
    EXPECT_THROW(asExpr("import(\"" + path.generic_string() + "\")", ev), EvalError);
    std::filesystem::remove(path);
}

// -- object()/is_object()/has_key() ----------------------------------------

TEST(ObjectBuiltin, ConstructsOrderedMapFromNamedArgs) {
    Evaluator ev;
    Value v = asExpr("object(b=2, a=1)", ev);
    const auto& items = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(items.size(), 2u);
    EXPECT_TRUE(std::get<bool>(asExpr("is_object(object(a=1))", ev)));
}

TEST(ObjectBuiltin, PositionalMergesExistingObject) {
    Evaluator ev;
    Value v = asExpr("object(object(a=1,b=2), c=3)", ev);
    const auto& items = std::get<ObjectPtr>(v)->items;
    ASSERT_EQ(items.size(), 3u);
}

// -- mesh-built geometry keeps full precision ----------------------------
//
// polyhedron()/sphere()/roof()/surface()/import() build their Manifold from
// a mesh, and MeshGL is MeshGLP<float>. Going through it truncated script
// doubles to ~7 significant digits, which does not merely lose precision:
// it snaps nearly-distinct coordinates onto exactly-equal ones and
// manufactures degenerate coincidences that make a later boolean leave a
// zero-thickness membrane behind, sealing a hole that should go through.
//
// Found via a real user model (a coin-cell dispenser using BOSL2's
// rounded/teardrop cyl(), which builds through polyhedron()): the bore came
// out capped, the export was a valid closed solid a slicer would happily
// print solid, and every mesh integrity check passed -- watertight,
// manifold, orientable, no duplicate or degenerate faces. Only the genus
// gave it away. A plain cylinder() never showed it, because that is a
// Manifold primitive built in double precision all along.
// A whole-model reproduction needs BOSL2 (its rounded/teardrop cyl() is what
// builds through polyhedron() in the wild), which this suite cannot depend
// on -- and a hand-written box does NOT reproduce it: Manifold handles
// exactly-coincident PLANAR faces fine, as checked directly. The end-to-end
// case lives in BelfrySCAD's scratch/coplanar_cut_repro.scad. What is
// guarded here is the cause rather than one of its symptoms: that
// mesh-built geometry keeps the precision it was given.
TEST(MeshPrecision, PolyhedronKeepsCoordinatesFloatWouldRound) {
    // 0.1 + 0.2 style values a float cannot hold: at float32 these two
    // vertices would land on the same coordinate and the solid would
    // degenerate. In doubles they stay apart.
    Evaluated e = evalSrc(R"(
        polyhedron(
            points = [[0,0,0],[10.000000123,0,0],[10.000000123,10,0],[0,10,0],
                      [0,0,10],[10.000000123,0,10],[10.000000123,10,10],[0,10,10]],
            faces  = [[1,2,3,0],[7,6,5,4],[4,5,1,0],
                      [5,6,2,1],[6,7,3,2],[7,4,0,3]]);
    )");
    ASSERT_FALSE(e.bodies.empty());
    const manifold::MeshGL64 mesh = e.bodies.front().body->GetMeshGL64();
    double maxX = 0.0;
    for (size_t i = 0; i < mesh.vertProperties.size(); i += mesh.numProp) {
        maxX = std::max(maxX, mesh.vertProperties[i]);
    }
    // float32 would round this to 10.0 exactly; double keeps the tail.
    EXPECT_GT(maxX, 10.0);
    EXPECT_NEAR(maxX, 10.000000123, 1e-9);
}
