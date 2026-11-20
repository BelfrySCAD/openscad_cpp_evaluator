#include "openscad_cpp_evaluator/font_match.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <tuple>
#include <cstdlib>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <system_error>

namespace fs = std::filesystem;

namespace oscadeval {

namespace {

std::string trim(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t");
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(" \t");
    return s.substr(b, e - b + 1);
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::vector<std::string> splitOn(const std::string& s, char sep) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        const size_t at = s.find(sep, start);
        out.push_back(s.substr(start, at == std::string::npos ? std::string::npos : at - start));
        if (at == std::string::npos) break;
        start = at + 1;
    }
    return out;
}

// Not fs::exists: a broken symlink or an unreadable mount throws through
// the throwing overload, and a missing font directory is entirely normal.
bool isDir(const fs::path& p) {
    std::error_code ec;
    return fs::is_directory(p, ec);
}

void addDir(std::vector<std::string>& out, const fs::path& p) {
    if (isDir(p)) out.push_back(p.string());
}

std::string envOr(const char* name, const std::string& fallback) {
    const char* v = std::getenv(name);
    return v ? std::string(v) : fallback;
}

} // namespace

namespace {

// fontconfig's constants for the two properties matched here.
const std::unordered_map<std::string, std::pair<char, double>>& fcConstants() {
    static const std::unordered_map<std::string, std::pair<char, double>> table = {
        {"thin", {'w', 0}},       {"extralight", {'w', 40}}, {"ultralight", {'w', 40}},
        {"light", {'w', 50}},     {"demilight", {'w', 55}},  {"semilight", {'w', 55}},
        {"book", {'w', 75}},      {"regular", {'w', 80}},    {"normal", {'w', 80}},
        {"medium", {'w', 100}},   {"demibold", {'w', 180}},  {"semibold", {'w', 180}},
        {"bold", {'w', 200}},     {"extrabold", {'w', 205}}, {"ultrabold", {'w', 205}},
        {"black", {'w', 210}},    {"heavy", {'w', 210}},
        {"roman", {'s', 0}},      {"italic", {'s', 100}},    {"oblique", {'s', 110}},
    };
    return table;
}

// fontconfig's object names: any other name fails its parse. Case matters
// there (`SLANT=` is refused), so it does here.
bool knownProperty(const std::string& name) {
    static const std::unordered_set<std::string> names = {
        "family", "familylang", "style", "stylelang", "fullname", "fullnamelang", "slant", "weight",
        "width", "size", "aspect", "pixelsize", "spacing", "foundry", "antialias", "hintstyle",
        "hinting", "verticallayout", "autohint", "globaladvance", "file", "index", "ftface",
        "rasterizer", "outline", "scalable", "dpi", "rgba", "scale", "minspace", "charwidth",
        "charheight", "matrix", "charset", "lang", "fontversion", "capability", "fontformat",
        "embolden", "embeddedbitmap", "decorative", "lcdfilter", "namelang", "fontfeatures",
        "prgname", "hash", "postscriptname", "color", "symbol", "fontvariations", "variable",
        "fonthashint", "order", "desktop", "namedinstance", "fontwrapper"};
    return names.count(name) != 0;
}

std::optional<double> number(const std::string& s) {
    if (s.empty()) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') return std::nullopt;
    return v;
}

} // namespace

FontSpec parseFontSpec(const std::string& spec) {
    FontSpec out;
    const std::vector<std::string> parts = splitOn(spec, ':');

    for (const std::string& fam : splitOn(parts.empty() ? "" : parts[0], ',')) {
        const std::string f = trim(fam);
        if (!f.empty()) out.families.push_back(f);
    }

    // The first value given for a property wins, as fontconfig prefers the
    // first of a list: "weight=bold:weight=thin" is bold.
    for (size_t i = 1; i < parts.size(); ++i) {
        const std::string prop = trim(parts[i]);
        if (prop.empty()) continue;
        const size_t eq = prop.find('=');
        if (eq == std::string::npos) {
            // Bare `:Bold`: the constant's own property. Any other bare
            // word is ignored by fontconfig, so here too.
            const auto c = fcConstants().find(lower(prop));
            if (c == fcConstants().end()) continue;
            auto& slot = c->second.first == 'w' ? out.weight : out.slant;
            if (!slot) slot = c->second.second;
            continue;
        }
        const std::string name = trim(prop.substr(0, eq));
        const std::string value = trim(prop.substr(eq + 1));
        if (name.empty() || value.empty()) continue;
        if (!knownProperty(name)) {
            out.valid = false;
            continue;
        }
        if (name == "style") {
            if (out.style.empty()) out.style = value;
        } else if (name == "weight" || name == "slant") {
            const auto c = fcConstants().find(lower(value));
            std::optional<double> v = c != fcConstants().end() ? std::optional<double>(c->second.second)
                                                               : number(value);
            if (!v && name == "weight" && value.front() == '[') v = 80;  // a range: accepted, not matched
            if (!v && name == "weight") {
                out.valid = false;                 // fontconfig: "Could not parse font"
                continue;
            }
            if (!v) v = 0;                         // slant is an integer: strtol of junk is 0
            auto& slot = name == "weight" ? out.weight : out.slant;
            if (!slot) slot = v;
        }
        // Every other known property (size=, lang=, ...) is ignored.
    }
    return out;
}

double fontconfigWeight(int ot) {
    // FcWeightFromOpenType's table, interpolated between its points.
    static const double map[][2] = {{100, 0},   {200, 40},  {300, 50},  {350, 55},  {380, 75}, {400, 80},
                                    {500, 100}, {600, 180}, {700, 200}, {800, 205}, {900, 210}, {1000, 215}};
    if (ot <= 100) return 0;
    for (size_t i = 1; i < sizeof map / sizeof map[0]; ++i) {
        if (ot <= map[i][0]) {
            const double t = (ot - map[i - 1][0]) / (map[i][0] - map[i - 1][0]);
            return map[i - 1][1] + t * (map[i][1] - map[i - 1][1]);
        }
    }
    return 215;
}

std::vector<std::string> expandGenericFamily(const std::string& family) {
    const std::string f = lower(family);
    if (f == "sans-serif" || f == "sans" || f == "helvetica" || f == "arial") {
        return {"Liberation Sans", "DejaVu Sans", "Arial", "Helvetica", "Helvetica Neue", "Segoe UI"};
    }
    if (f == "serif" || f == "times" || f == "times new roman") {
        return {"Liberation Serif", "DejaVu Serif", "Times New Roman", "Times", "Georgia"};
    }
    if (f == "monospace" || f == "mono" || f == "courier" || f == "courier new") {
        return {"Liberation Mono", "DejaVu Sans Mono", "Courier New", "Menlo", "Consolas"};
    }
    return {};
}

std::vector<std::string> systemFontDirs() {
    std::vector<std::string> dirs;
    const std::string home = envOr("HOME", envOr("USERPROFILE", ""));

#if defined(__APPLE__)
    addDir(dirs, "/System/Library/Fonts");
    addDir(dirs, "/System/Library/Fonts/Supplemental");
    addDir(dirs, "/Library/Fonts");
    addDir(dirs, "/Network/Library/Fonts");
    if (!home.empty()) addDir(dirs, fs::path(home) / "Library" / "Fonts");
#elif defined(_WIN32)
    addDir(dirs, fs::path(envOr("SystemRoot", "C:\\Windows")) / "Fonts");
    const std::string localApp = envOr("LOCALAPPDATA", "");
    if (!localApp.empty()) addDir(dirs, fs::path(localApp) / "Microsoft" / "Windows" / "Fonts");
#else
    addDir(dirs, "/usr/share/fonts");
    addDir(dirs, "/usr/local/share/fonts");
    // Flatpak exposes the host's fonts here; harmless to look elsewhere.
    addDir(dirs, "/run/host/fonts");
    if (!home.empty()) {
        addDir(dirs, fs::path(home) / ".fonts");
        addDir(dirs, fs::path(home) / ".local" / "share" / "fonts");
    }
#endif

    // Same variable real OpenSCAD reads, same separator convention.
#if defined(_WIN32)
    const char sep = ';';
#else
    const char sep = ':';
#endif
    for (const std::string& p : splitOn(envOr("OPENSCAD_FONT_PATH", ""), sep)) {
        const std::string t = trim(p);
        if (!t.empty()) addDir(dirs, t);
    }
    return dirs;
}

std::vector<std::string> findFontFiles() {
    std::vector<std::string> files;
    for (const std::string& dir : systemFontDirs()) {
        std::error_code ec;
        // skip_permission_denied keeps one unreadable subdirectory from
        // aborting the whole walk; the error_code overloads keep a
        // vanishing file (a font installed or removed mid-scan) from
        // throwing out of a render.
        fs::recursive_directory_iterator it(dir, fs::directory_options::skip_permission_denied, ec);
        if (ec) continue;
        for (fs::recursive_directory_iterator end; it != end; it.increment(ec)) {
            if (ec) break;
            if (!it->is_regular_file(ec) || ec) continue;
            const std::string ext = lower(it->path().extension().string());
            if (ext == ".ttf" || ext == ".otf" || ext == ".ttc" || ext == ".otc") {
                files.push_back(it->path().string());
            }
        }
    }
    return files;
}

std::optional<FontFace> matchFace(const FontSpec& spec, const std::vector<FontFace>& faces) {
    std::vector<std::string> wanted;
    for (const std::string& fam : spec.families) {
        wanted.push_back(fam);
        for (const std::string& alias : expandGenericFamily(fam)) wanted.push_back(alias);
    }

    const std::string wantStyle = lower(spec.style);

    for (const std::string& fam : wanted) {
        const std::string want = lower(fam);
        std::vector<const FontFace*> inFamily;
        for (const FontFace& f : faces) {
            if (lower(f.family) == want) inFamily.push_back(&f);
        }
        if (inFamily.empty()) continue;

        if (!wantStyle.empty()) {
            for (const FontFace* f : inFamily) {
                if (lower(f->style) == wantStyle) return *f;
            }
            for (const FontFace* f : inFamily) {
                if (lower(f->style).find(wantStyle) != std::string::npos) return *f;
            }
        }
        if (wantStyle.empty() && (spec.weight || spec.slant)) {
            const double w = spec.weight.value_or(80), sl = spec.slant.value_or(0);
            const FontFace* best = inFamily.front();
            auto key = [&](const FontFace* f) {
                return std::make_tuple(std::abs(f->slant - sl), -f->slant, std::abs(f->weight - w), -f->weight);
            };
            for (const FontFace* f : inFamily) {
                if (key(f) < key(best)) best = f;
            }
            return *best;
        }
        for (const FontFace* f : inFamily) {
            if (lower(f->style) == "regular") return *f;
        }
        return *inFamily.front();
    }
    return std::nullopt;
}

} // namespace oscadeval
