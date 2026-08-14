#include "builtins.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/dxf_svg_import.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/import_builtin.hpp"
#include "openscad_cpp_evaluator/mesh_import.hpp"

#include <manifold/manifold.h>

#include "openscad_cpp_evaluator/mesh_check.hpp"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>

namespace oscadeval {

// Mirrors _resolve_import_path: relative to the *source .scad file's*
// directory, not the process CWD. Shared by every file-reading builtin
// (import/surface/DXF/SVG), declared in builtins.hpp.
std::string resolveFilePath(const Value& fileArg, const oscad::ASTNode& node) {
    const std::string path = std::holds_alternative<std::string>(fileArg) ? std::get<std::string>(fileArg) : fmtValue(fileArg);
    const std::string& origin = node.position().origin;
    std::filesystem::path p(path);
    if (!origin.empty() && p.is_relative()) {
        const std::filesystem::path base = std::filesystem::path(origin).parent_path();
        if (!base.empty()) p = base / p;
    }
    return p.string();
}

namespace {

std::string lowerExt(const std::string& path) {
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

LoadedMesh loadMeshByExt(const std::string& path, const std::string& ext) {
    if (ext == ".stl") return loadStl(path);
    if (ext == ".obj") return loadObj(path);
    if (ext == ".off") return loadOff(path);
    return loadThreeMf(path);
}

bool isMeshExt(const std::string& ext) { return ext == ".stl" || ext == ".obj" || ext == ".off" || ext == ".3mf"; }


Value jsonToValue(const nlohmann::ordered_json& j) {
    if (j.is_boolean()) return Value{j.get<bool>()};
    if (j.is_number()) return Value{j.get<double>()};
    if (j.is_string()) return Value{j.get<std::string>()};
    if (j.is_array()) {
        std::vector<Value> items;
        items.reserve(j.size());
        for (const auto& el : j) items.push_back(jsonToValue(el));
        return Value{std::make_shared<const ValueList>(ValueList{std::move(items)})};
    }
    if (j.is_object()) {
        std::vector<std::pair<std::string, Value>> items;
        items.reserve(j.size());
        for (auto it = j.begin(); it != j.end(); ++it) items.emplace_back(it.key(), jsonToValue(it.value()));
        return Value{std::make_shared<const ValueObject>(ValueObject{std::move(items)})};
    }
    return Value{}; // null (or any other JSON edge case) -> undef
}

Value loadJsonAsValue(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open '" + path + "'");
    nlohmann::ordered_json j;
    in >> j;
    return jsonToValue(j);
}

Value meshToVnf(const LoadedMesh& mesh) {
    // Re-dedups by exact [x,y,z] value regardless of the source format's own
    // indexing -- mirrors _import_as_vnf's vert_map exactly.
    std::map<std::array<double, 3>, int> vertMap;
    std::vector<Value> vertsOut;
    std::vector<Value> facesOut;
    for (const auto& tri : mesh.tris) {
        std::vector<Value> face;
        face.reserve(3);
        for (int vi : tri) {
            if (vi < 0 || static_cast<size_t>(vi) >= mesh.verts.size()) continue;
            const std::array<double, 3>& v = mesh.verts[static_cast<size_t>(vi)];
            auto it = vertMap.find(v);
            int idx;
            if (it == vertMap.end()) {
                idx = static_cast<int>(vertsOut.size());
                vertMap.emplace(v, idx);
                vertsOut.push_back(Value{std::make_shared<const ValueList>(
                    ValueList{{Value{v[0]}, Value{v[1]}, Value{v[2]}}})});
            } else {
                idx = it->second;
            }
            face.push_back(Value{static_cast<double>(idx)});
        }
        facesOut.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(face)})});
    }
    std::vector<Value> outer = {
        Value{std::make_shared<const ValueList>(ValueList{std::move(vertsOut)})},
        Value{std::make_shared<const ValueList>(ValueList{std::move(facesOut)})},
    };
    return Value{std::make_shared<const ValueList>(ValueList{std::move(outer)})};
}

} // namespace

// manifold::ToString(Manifold::Error) only exists under MANIFOLD_DEBUG --
// not enabled in this project's build -- so this mirrors it locally for the
// not-manifold warning message.
std::string manifoldErrorName(manifold::Manifold::Error e) {
    switch (e) {
        case manifold::Manifold::Error::NoError: return "NoError";
        case manifold::Manifold::Error::NonFiniteVertex: return "NonFiniteVertex";
        case manifold::Manifold::Error::NotManifold: return "NotManifold";
        case manifold::Manifold::Error::VertexOutOfBounds: return "VertexOutOfBounds";
        case manifold::Manifold::Error::PropertiesWrongLength: return "PropertiesWrongLength";
        case manifold::Manifold::Error::MissingPositionProperties: return "MissingPositionProperties";
        case manifold::Manifold::Error::MergeVectorsDifferentLengths: return "MergeVectorsDifferentLengths";
        case manifold::Manifold::Error::MergeIndexOutOfBounds: return "MergeIndexOutOfBounds";
        case manifold::Manifold::Error::TransformWrongLength: return "TransformWrongLength";
        case manifold::Manifold::Error::RunIndexWrongLength: return "RunIndexWrongLength";
        case manifold::Manifold::Error::FaceIDWrongLength: return "FaceIDWrongLength";
        case manifold::Manifold::Error::InvalidConstruction: return "InvalidConstruction";
        case manifold::Manifold::Error::ResultTooLarge: return "ResultTooLarge";
        case manifold::Manifold::Error::InvalidTangents: return "InvalidTangents";
        case manifold::Manifold::Error::Cancelled: return "Cancelled";
    }
    return "Unknown";
}

namespace {

Value contoursToValue(const std::vector<Contour2d>& contours) {
    std::vector<Value> outer;
    outer.reserve(contours.size());
    for (const auto& c : contours) {
        std::vector<Value> pts;
        pts.reserve(c.size());
        for (const auto& p : c) pts.push_back(Value{std::make_shared<const ValueList>(ValueList{{Value{p[0]}, Value{p[1]}}})});
        outer.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(pts)})});
    }
    return Value{std::make_shared<const ValueList>(ValueList{std::move(outer)})};
}

std::vector<Contour2d> valueToContours(const Value& v) {
    std::vector<Contour2d> out;
    const ListPtr* outer = std::get_if<ListPtr>(&v);
    if (!outer || !*outer) return out;
    for (const Value& cVal : (*outer)->items) {
        const ListPtr* c = std::get_if<ListPtr>(&cVal);
        if (!c || !*c) continue;
        Contour2d contour;
        for (const Value& pVal : (*c)->items) {
            const ListPtr* p = std::get_if<ListPtr>(&pVal);
            if (!p || !*p || (*p)->items.size() < 2) continue;
            contour.push_back({toDoubleLenient((*p)->items[0]), toDoubleLenient((*p)->items[1])});
        }
        out.push_back(std::move(contour));
    }
    return out;
}

} // namespace

CSGParams resolveImport(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    const Value fileArg = getArg(args, 0, "file", Value{});
    const Value layerArg = getArg(args, std::nullopt, "layer", Value{});
    // Not an OpenSCAD parameter. A script using it will not run upstream,
    // which is why it has to be asked for rather than being the default.
    const bool repair = truthy(getArg(args, std::nullopt, "repair", Value{false}));

    CSGParams params;
    params["color"] = colorToValue(effCtx.color);
    if (std::holds_alternative<std::monostate>(fileArg)) {
        ev.error("import: 'file' parameter is required", node);
    }
    const std::string path = resolveFilePath(fileArg, node);
    const std::string ext = lowerExt(path);

    if (ext == ".dxf" || ext == ".svg" || ext == ".pdf") {
        std::vector<Contour2d> contours;
        try {
            if (ext == ".dxf") {
                std::optional<std::string> layer;
                if (const std::string* s = std::get_if<std::string>(&layerArg)) layer = *s;
                contours = loadDxfContours(path, layer);
            } else {
                contours = loadSvgContours(path);
            }
        } catch (const std::exception& e) {
            ev.error(std::string("import: ") + e.what(), node);
        }
        if (contours.empty()) {
            ev.error(ext == ".dxf" ? "import: no closed contours found in DXF file" : "import: no shapes found in SVG file", node);
        }
        params["kind"] = Value{std::string("region")};
        params["contours"] = contoursToValue(contours);
        return params;
    }
    if (isMeshExt(ext)) {
        LoadedMesh mesh;
        try {
            mesh = loadMeshByExt(path, ext);
        } catch (const std::exception& e) {
            ev.error(std::string("import: ") + e.what(), node);
        }
        std::vector<Value> vertsFlat;
        vertsFlat.reserve(mesh.verts.size() * 3);
        for (const auto& v : mesh.verts) {
            vertsFlat.push_back(Value{v[0]});
            vertsFlat.push_back(Value{v[1]});
            vertsFlat.push_back(Value{v[2]});
        }
        std::vector<Value> trisFlat;
        trisFlat.reserve(mesh.tris.size() * 3);
        for (const auto& t : mesh.tris) {
            trisFlat.push_back(Value{static_cast<double>(t[0])});
            trisFlat.push_back(Value{static_cast<double>(t[1])});
            trisFlat.push_back(Value{static_cast<double>(t[2])});
        }
        params["kind"] = Value{std::string("mesh")};
        params["repair"] = Value{repair};
        params["verts"] = Value{std::make_shared<const ValueList>(ValueList{std::move(vertsFlat)})};
        params["tris"] = Value{std::make_shared<const ValueList>(ValueList{std::move(trisFlat)})};
        return params;
    }
    if (ext == ".json") {
        ev.error("import: .json returns data, not geometry -- use as an expression", node);
    }
    ev.error("import: unsupported file type '" + ext + "'", node);
    return params;
}

std::vector<ColoredBody> generateImport(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                         const oscad::ASTNode& node) {
    const auto kindIt = params.find("kind");
    if (kindIt == params.end()) return {};
    const std::string& kind = std::get<std::string>(kindIt->second);
    if (kind == "region") {
        const std::vector<Contour2d> contours = valueToContours(params.at("contours"));
        manifold::Polygons polys;
        polys.reserve(contours.size());
        for (const auto& c : contours) {
            manifold::SimplePolygon poly;
            poly.reserve(c.size());
            for (const auto& p : c) poly.push_back(manifold::vec2(p[0], p[1]));
            polys.push_back(std::move(poly));
        }
        ColoredBody result;
        result.section = manifold::CrossSection(polys, manifold::CrossSection::FillRule::EvenOdd);
        result.color = valueToColor(params.at("color"));
        return {result};
    }
    if (kind != "mesh") return {};

    const auto& tris = std::get<ListPtr>(params.at("tris"))->items;
    if (tris.empty()) ev.error("import: mesh has no triangles", node);

    // Kept at full precision for the Manifold that CSG will actually use --
    // MeshGL is MeshGLP<float>, and truncating there does not merely lose
    // digits, it snaps nearly-distinct coordinates onto exactly-equal ones
    // and manufactures the degenerate coincidences that make a later
    // boolean leave a zero-thickness membrane behind (see
    // generatePolyhedron). STL carries only float32 to begin with, but
    // OBJ/OFF/3MF are text and can hold more, and this is the same
    // import() either way.
    manifold::MeshGL64 mesh64;
    mesh64.numProp = 3;
    for (const Value& v : std::get<ListPtr>(params.at("verts"))->items) {
        mesh64.vertProperties.push_back(std::get<double>(v));
    }
    for (const Value& t : tris) mesh64.triVerts.push_back(static_cast<uint64_t>(std::get<double>(t)));

    // checkMesh/repairMesh/tagDisplayOnly all work on MeshGL, and the
    // repair path rewrites vertices, so it stays on floats: repair only
    // runs on a mesh already known to be broken, where the topology surgery
    // matters far more than the last few digits. Duplicating the repair
    // code for both precisions would be a poor trade for that.
    manifold::MeshGL mesh;
    mesh.numProp = 3;
    mesh.vertProperties.assign(mesh64.vertProperties.begin(), mesh64.vertProperties.end());
    mesh.triVerts.assign(mesh64.triVerts.begin(), mesh64.triVerts.end());

    const auto repairIt = params.find("repair");
    const bool repair = repairIt != params.end() && truthy(repairIt->second);
    if (repair) {
        const MeshDiagnosis before = checkMesh(mesh);
        MeshRepairReport rep;
        manifold::MeshGL fixed = repairMesh(mesh, rep);
        const MeshDiagnosis after = checkMesh(fixed);
        if (rep.didAnything()) {
            ev.warn("import: repaired the mesh -- " + rep.summary(), &node.position());
        }
        if (!after.ok() && !before.ok()) {
            // Say what is left rather than only what was done: a repair
            // that helped but did not finish is the case where the user
            // most needs to know the difference.
            ev.warn("import: still not manifold after repair -- " + after.summary(),
                    &node.position());
        }
        mesh = std::move(fixed);
    }

    // Only the untouched mesh still has its full precision; a repaired one
    // has been through the float path, so build from whichever is current.
    manifold::Manifold body = repair ? manifold::Manifold(mesh) : manifold::Manifold(mesh64);
    if (body.Status() != manifold::Manifold::Error::NoError) {
        // Same treatment as an open polyhedron() (see generatePolyhedron):
        // keep the triangles so the file can still be LOOKED at. Previously
        // this warned and then handed back an empty body, which was dropped
        // downstream -- so a broken STL warned once and showed nothing,
        // giving no way to see what was actually wrong with it.
        const MeshDiagnosis d = checkMesh(mesh);
        std::string why = d.summary();
        if (why.empty()) why = manifoldErrorName(body.Status());
        ev.warn("import: mesh is not a closed solid (" + why +
                    "); drawing it as a surface, but it cannot take part in any CSG "
                    "operation" + (repair ? "" : ". Try import(..., repair=true)"),
                &node.position());
        return {ev.tagDisplayOnly(std::move(mesh), node, params.at("color"))};
    }
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

Value importAsValue(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node) {
    const Value fileArg = getArg(args, 0, "file", Value{});
    const Value layerArg = getArg(args, std::nullopt, "layer", Value{});
    if (std::holds_alternative<std::monostate>(fileArg)) {
        ev.error("import: 'file' parameter is required", node);
    }
    const std::string path = resolveFilePath(fileArg, node);
    const std::string ext = lowerExt(path);

    try {
        if (ext == ".json") return loadJsonAsValue(path);
        if (isMeshExt(ext)) return meshToVnf(loadMeshByExt(path, ext));
        if (ext == ".dxf" || ext == ".svg" || ext == ".pdf") {
            std::vector<Contour2d> contours;
            if (ext == ".dxf") {
                std::optional<std::string> layer;
                if (const std::string* s = std::get_if<std::string>(&layerArg)) layer = *s;
                contours = loadDxfContours(path, layer);
            } else {
                contours = loadSvgContours(path);
            }
            return contoursToValue(contours);
        }
    } catch (const std::exception& e) {
        ev.error(std::string("import: ") + e.what(), node);
    }
    ev.error("import: unsupported file type '" + ext + "'", node);
    return Value{};
}

} // namespace oscadeval
