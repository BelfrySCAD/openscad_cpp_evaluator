#include "openscad_cpp_evaluator/font_match.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
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

FontSpec parseFontSpec(const std::string& spec) {
    FontSpec out;
    const std::vector<std::string> parts = splitOn(spec, ':');

    for (const std::string& fam : splitOn(parts.empty() ? "" : parts[0], ',')) {
        const std::string f = trim(fam);
        if (!f.empty()) out.families.push_back(f);
    }

    for (size_t i = 1; i < parts.size(); ++i) {
        const std::string prop = trim(parts[i]);
        if (prop.empty()) continue;
        const size_t eq = prop.find('=');
        if (eq == std::string::npos) {
            // Bare `:Bold`. Only taken as a style if nothing already set
            // one, so an explicit style= always wins.
            if (out.style.empty()) out.style = prop;
        } else if (lower(trim(prop.substr(0, eq))) == "style") {
            out.style = trim(prop.substr(eq + 1));
        }
        // Any other property (size=, weight=, lang=) is ignored on purpose.
    }
    return out;
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
        for (const FontFace* f : inFamily) {
            if (lower(f->style) == "regular") return *f;
        }
        return *inFamily.front();
    }
    return std::nullopt;
}

} // namespace oscadeval
