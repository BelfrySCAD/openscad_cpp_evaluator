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
#include <cmath>
#include <array>
#include <cctype>
#include <filesystem>
#include <iterator>
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
    if (ext == ".amf") return loadAmf(path);
    if (ext == ".x3d") return loadX3d(path);
    if (ext == ".wrl") return loadVrml(path);
    return loadThreeMf(path);
}

bool isMeshExt(const std::string& ext) {
    return ext == ".stl" || ext == ".obj" || ext == ".off" || ext == ".3mf" || ext == ".amf" || ext == ".x3d" ||
           ext == ".wrl";
}


Value jsonToValue(const nlohmann::ordered_json& j) {
    if (j.is_boolean()) return Value{j.get<bool>()};
    if (j.is_number()) return Value{j.get<double>()};
    if (j.is_string()) return Value{j.get<std::string>()};
    if (j.is_array()) {
        std::vector<Value> items;
        items.reserve(j.size());
        for (const auto& el : j) items.push_back(jsonToValue(el));
        return Value{makeList(std::move(items))};
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
                vertsOut.push_back(Value{makeList({Value{v[0]}, Value{v[1]}, Value{v[2]}})});
            } else {
                idx = it->second;
            }
            face.push_back(Value{static_cast<double>(idx)});
        }
        facesOut.push_back(Value{makeList(std::move(face))});
    }
    std::vector<Value> outer = {
        Value{makeList(std::move(vertsOut))},
        Value{makeList(std::move(facesOut))},
    };
    return Value{makeList(std::move(outer))};
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
        for (const auto& p : c) pts.push_back(Value{makeList({Value{p[0]}, Value{p[1]}})});
        outer.push_back(Value{makeList(std::move(pts))});
    }
    return Value{makeList(std::move(outer))};
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

// The SVG half of import(), shared by the statement and the expression
// form: the id/class/layer filter, dpi, and OpenSCAD's warnings for a bad
// dpi and for a filter that matched nothing.
std::vector<Contour2d> importSvg(Evaluator& ev, const CallArgs& args, const Value& layerArg, const std::string& path,
                                 bool center, const Discretizer& disc, const oscad::ASTNode& node) {
    // A number names "5"; undef (or absent) is no filter.
    const auto filterText = [](const Value& v) -> std::optional<std::string> {
        if (std::holds_alternative<std::monostate>(v)) return std::nullopt;
        const std::string* s = std::get_if<std::string>(&v);
        return s ? *s : fmtValue(v);
    };
    SvgFilter filter;
    filter.id = filterText(getArg(args, std::nullopt, "id", Value{}));
    filter.cls = filterText(getArg(args, std::nullopt, "class", Value{}));
    filter.layer = filterText(layerArg);
    // Any number below 0.001 is refused with a warning; anything that is
    // not a number is silently the default. (OpenSCAD's own wording reads
    // "giving, using default of undef dpi" -- its default fails to print.)
    double dpi = 72.0;
    const Value dpiArg = getArg(args, std::nullopt, "dpi", Value{});
    if (const double* d = std::get_if<double>(&dpiArg)) {
        if (*d >= 0.001) {
            dpi = *d;
        } else {
            ev.warn("Invalid dpi value given, using default of 72 dpi. Value must be positive and >= 0.001",
                    &node.position());
        }
    }
    bool matched = true;
    std::vector<Contour2d> contours = loadSvgContours(path, filter, &matched, dpi, center, disc);
    if (!matched) {
        // Nothing imported, rather than silently falling back to the whole
        // drawing -- a cut layer that quietly became every layer is the
        // worst answer available. OpenSCAD's words, which name no file.
        std::string what;
        const auto add = [&](const char* name, const std::optional<std::string>& v) {
            if (!v) return;
            if (!what.empty()) what += ", ";
            what += std::string(name) + " = \"" + *v + "\"";
        };
        add("id", filter.id);
        add("layer", filter.layer);
        add("class", filter.cls);
        ev.warn("import() filter " + what + " did not match anything", &node.position());
    }
    return contours;
}

} // namespace

CSGParams resolveImport(Evaluator& ev, const oscad::ModularCall& node, EvalContext& ctx) {
    auto [args, effCtx] = resolveCallArgs(ev, node.arguments, ctx);
    Value fileArg = getArg(args, 0, "file", Value{});
    if (std::holds_alternative<std::monostate>(fileArg)) {
        fileArg = getArg(args, std::nullopt, "filename", Value{});
        if (!std::holds_alternative<std::monostate>(fileArg))
            ev.emitWarning("DEPRECATED: filename= is deprecated. Please use file=");
    }
    Value layerArg = getArg(args, std::nullopt, "layer", Value{});
    if (std::holds_alternative<std::monostate>(layerArg)) {
        layerArg = getArg(args, std::nullopt, "layername", Value{});
        if (!std::holds_alternative<std::monostate>(layerArg))
            ev.emitWarning("DEPRECATED: layername= is deprecated. Please use layer=");
    }
    // Not an OpenSCAD parameter. A script using it will not run upstream,
    // which is why it has to be asked for rather than being the default.
    const bool repair = truthy(getArg(args, std::nullopt, "repair", Value{false}));
    // How far apart two vertices may be and still be welded into one.
    // Named rather than hidden so import() and mesh_repair() answer the
    // same question the same way (issue #190); undef keeps the default.
    const Value toleranceArg = getArg(args, std::nullopt, "tolerance", Value{});
    // ImportNode: center counts only as a real bool, and centres every type
    // on its bounding box (optionally_center). Only SVG used to honour it.
    const Value centerArg = getArg(args, std::nullopt, "center", Value{false});
    const bool center = std::holds_alternative<bool>(centerArg) && std::get<bool>(centerArg);

    CSGParams params;
    params["color"] = colorToValue(effCtx.color);
    // An import that fails draws nothing and the render goes on, as in
    // OpenSCAD; the message is printed at generate, where OpenSCAD prints
    // it, after the script's echoes. These used to abort the whole render.
    const auto failed = [&](const std::string& message) {
        params["kind"] = Value{std::string("failed")};
        params["message"] = Value{message};
        return params;
    };
    const auto unsupported = [&](const std::string& given) {
        return failed("Unsupported file format while trying to import file '\"" + given +
                      "\"', import() at line " + std::to_string(node.position().line));
    };
    if (std::holds_alternative<std::monostate>(fileArg)) return unsupported("");
    const std::string path = resolveFilePath(fileArg, node);
    const std::string ext = lowerExt(path);
    if (ext != ".dxf" && ext != ".svg" && !isMeshExt(ext)) {
        const std::string* given = std::get_if<std::string>(&fileArg);
        return unsupported(given ? *given : fmtValue(fileArg));
    }

    if (!std::filesystem::exists(path)) {
        // A missing file warns and imports nothing, as in OpenSCAD; it
        // aborted the whole render. Warned at generate, where OpenSCAD does,
        // so it follows the script's echoes.
        params["kind"] = Value{std::string("missing")};
        params["path"] = Value{path};
        params["dxf"] = Value{ext == ".dxf"};
        return params;
    }
    // .pdf is not here: it used to be, handed to the SVG parser, which hung on
    // a PDF's bytes. Nothing reads PDF (OpenSCAD does not either), so it takes
    // the ordinary "unsupported file type" error below.
    if (ext == ".dxf" || ext == ".svg") {
        std::vector<Contour2d> contours;
        try {
            if (ext == ".dxf") {
                std::optional<std::string> layer;
                if (const std::string* s = std::get_if<std::string>(&layerArg)) layer = *s;
                contours = loadDxfContours(path, layer);
            } else {
                contours = importSvg(ev, args, layerArg, path, center, Discretizer::fromCtx(effCtx), node);
            }
        } catch (const std::exception& e) {
            return failed(std::string(e.what()) + locSuffix(&node.position()));
        }
        if (center && ext == ".dxf" && !contours.empty()) {
            double lo[2] = {INFINITY, INFINITY}, hi[2] = {-INFINITY, -INFINITY};
            for (const auto& c : contours)
                for (const auto& p : c)
                    for (int k = 0; k < 2; ++k) {
                        lo[k] = std::min(lo[k], p[k]);
                        hi[k] = std::max(hi[k], p[k]);
                    }
            for (auto& c : contours)
                for (auto& p : c)
                    for (int k = 0; k < 2; ++k) p[k] -= (lo[k] + hi[k]) / 2;
        }
        // A drawing with nothing to fill imports nothing, silently, as in
        // OpenSCAD -- except a file that is not SVG at all, which OpenSCAD
        // reports as a parse error.
        if (contours.empty() && ext == ".svg") {
            std::ifstream in(path, std::ios::binary);
            const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            if (text.find("<svg") == std::string::npos)
                return failed("Error parsing file '" + path + "', import() at line " + std::to_string(node.position().line));
        }
        params["kind"] = Value{std::string("region")};
        params["contours"] = contoursToValue(contours);
        return params;
    }
    if (isMeshExt(ext)) {
        // OpenSCAD 2026.02.01's notice, without a location, as it prints it.
        if (ext == ".amf") ev.emitWarning("DEPRECATED: AMF import is deprecated. Please use 3MF instead.");
        LoadedMesh mesh;
        try {
            mesh = loadMeshByExt(path, ext);
        } catch (const std::exception& e) {
            return failed(std::string(e.what()) + locSuffix(&node.position()));
        }
        for (const std::string& w : mesh.warnings) ev.warn("import: '" + path + "': " + w, &node.position());
        if (center && !mesh.verts.empty()) {
            double lo[3] = {INFINITY, INFINITY, INFINITY}, hi[3] = {-INFINITY, -INFINITY, -INFINITY};
            for (const auto& v : mesh.verts)
                for (int k = 0; k < 3; ++k) {
                    lo[k] = std::min(lo[k], v[k]);
                    hi[k] = std::max(hi[k], v[k]);
                }
            for (auto& v : mesh.verts)
                for (int k = 0; k < 3; ++k) v[k] -= (lo[k] + hi[k]) / 2;
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
        params["tolerance"] = toleranceArg;
        params["verts"] = Value{makeList(std::move(vertsFlat))};
        params["tris"] = Value{makeList(std::move(trisFlat))};
        return params;
    }
    return params;  // unreachable: every supported extension returned above
}

std::vector<ColoredBody> generateImport(Evaluator& ev, const CSGParams& params, const std::vector<std::unique_ptr<CSGNode>>&,
                                         const oscad::ASTNode& node) {
    const auto kindIt = params.find("kind");
    if (kindIt == params.end()) return {};
    const std::string& kind = std::get<std::string>(kindIt->second);
    if (kind == "failed") {
        ev.emitWarning("ERROR: " + std::get<std::string>(params.at("message")));
        return {};
    }
    if (kind == "missing") {
        // OpenSCAD's own wording, which names a line rather than a file.
        const std::string& path = std::get<std::string>(params.at("path"));
        ev.warn(std::get<bool>(params.at("dxf"))
                    ? "Can't open DXF file '" + path + "'."
                    : "Can't open import file '" + path + "', import() at line " +
                          std::to_string(node.position().line),
                nullptr);
        return {};
    }
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
    if (tris.empty()) return {};  // nothing to draw, as OpenSCAD draws nothing

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

    // checkMesh/repairMesh work at this precision too, which matters here:
    // they weld on a fixed 1e-6 grid, and float32's step is coarser than
    // that beyond coordinate magnitude ~17, so a float mesh would have them
    // merging vertices that are genuinely distinct on any real model.
    manifold::MeshGL64& mesh = mesh64;

    const auto repairIt = params.find("repair");
    const bool repair = repairIt != params.end() && truthy(repairIt->second);
    if (repair) {
        double tolerance = kDefaultWeldTolerance;
        const auto tolIt = params.find("tolerance");
        if (tolIt != params.end()) {
            if (const double* t = std::get_if<double>(&tolIt->second)) {
                if (*t < 0.0) {
                    ev.warn("import: tolerance must not be negative; using the default",
                            &node.position());
                } else {
                    tolerance = *t;
                }
            } else if (!std::holds_alternative<std::monostate>(tolIt->second)) {
                ev.warn("import: tolerance must be a number", &node.position());
            }
        }
        const MeshDiagnosis before = checkMesh(mesh);
        MeshRepairReport rep;
        manifold::MeshGL64 fixed = repairMesh(mesh, rep, tolerance);
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

    manifold::Manifold body(mesh);
    if (body.Status() != manifold::Manifold::Error::NoError) {
        // Same treatment as an open polyhedron() (see generatePolyhedron):
        // keep the triangles so the file can still be LOOKED at. Previously
        // this warned and then handed back an empty body, which was dropped
        // downstream -- so a broken STL warned once and showed nothing,
        // giving no way to see what was actually wrong with it.
        const MeshDiagnosis d = checkMesh(mesh);
        std::string why = d.summary();
        if (why.empty()) why = manifoldErrorName(body.Status());
        if (!ev.insideHull) {
            ev.warn("import: mesh is not a closed solid (" + why +
                        // Same rewording as polyhedron's: see the comment there.
                        "); drawing the object as an open surface rather than a solid "
                        "-- nothing is patched. hull() can still use its points, but "
                        "it cannot take part in union/difference/intersection" +
                        std::string(repair ? "" : ". Try import(..., repair=true)"),
                    &node.position());
        }
        // tagDisplayOnly carries a raw triangle soup for the renderer to
        // draw, and the renderer is float either way, so narrowing here
        // costs nothing this mesh will ever be measured on.
        manifold::MeshGL soup;
        soup.numProp = 3;
        soup.vertProperties.assign(mesh.vertProperties.begin(), mesh.vertProperties.end());
        soup.triVerts.assign(mesh.triVerts.begin(), mesh.triVerts.end());
        return {ev.tagDisplayOnly(std::move(soup), node, params.at("color"))};
    }
    return {ev.tagGenerated(std::move(body), node, params.at("color"))};
}

Value importAsValue(Evaluator& ev, const CallArgs& args, const oscad::ASTNode& node, const EvalContext& ctx) {
    const Value fileArg = getArg(args, 0, "file", Value{});
    const Value layerArg = getArg(args, std::nullopt, "layer", Value{});
    if (std::holds_alternative<std::monostate>(fileArg)) {
        ev.error("import: 'file' parameter is required", node);
    }
    const std::string path = resolveFilePath(fileArg, node);
    const std::string ext = lowerExt(path);
    if (!std::filesystem::exists(path)) {
        ev.warn("Could not read file '" + path + "'", &node.position());
        return Value{};
    }

    try {
        if (ext == ".json") return loadJsonAsValue(path);
        if (isMeshExt(ext)) {
            const LoadedMesh mesh = loadMeshByExt(path, ext);
            for (const std::string& w : mesh.warnings) ev.warn("import: '" + path + "': " + w, &node.position());
            return meshToVnf(mesh);
        }
        if (ext == ".dxf" || ext == ".svg") {
            std::vector<Contour2d> contours;
            if (ext == ".dxf") {
                std::optional<std::string> layer;
                if (const std::string* s = std::get_if<std::string>(&layerArg)) layer = *s;
                contours = loadDxfContours(path, layer);
            } else {
                // $fn and friends from the call's own arguments, else the scope's.
                Discretizer disc = Discretizer::fromCtx(ctx);
                for (auto [name, field] : {std::pair{"$fn", &disc.fn}, {"$fa", &disc.fa}, {"$fs", &disc.fs}, {"$fe", &disc.fe}}) {
                    if (const Value v = getArg(args, std::nullopt, name, Value{}); std::holds_alternative<double>(v))
                        *field = std::get<double>(v);
                }
                const Value centerArg = getArg(args, std::nullopt, "center", Value{false});
                contours = importSvg(ev, args, layerArg, path,
                                     std::holds_alternative<bool>(centerArg) && std::get<bool>(centerArg), disc, node);
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
