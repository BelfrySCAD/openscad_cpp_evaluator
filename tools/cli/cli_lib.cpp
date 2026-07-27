#include "cli_lib.hpp"

#include "openscad_cpp_evaluator/debug_repl.hpp"
#include "openscad_cpp_evaluator/eval_error.hpp"
#include "openscad_cpp_evaluator/eval_use.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/export.hpp"

#include "openscad_cpp_parser/api.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>

// Mirrors just enough of openscad_evaluator's own cli.py to prove the
// pipeline end to end: format-from-extension dispatch, and --debug
// dropping into a gdb-style REPL (debug_repl.hpp) instead of running
// straight through.
namespace oscadeval {

namespace {

std::string formatForPath(const std::string& explicitFormat, const std::string& outputPath) {
    if (!explicitFormat.empty()) return explicitFormat;
    std::string ext = std::filesystem::path(outputPath).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".stl") return "stl";
    if (ext == ".obj") return "obj";
    if (ext == ".off") return "off";
    if (ext == ".3mf") return "3mf";
    return "";
}

// --profile's tunables: which rows to keep (minSelf/minCalls thresholds)
// and how to order/render them. Defaults reproduce the original
// (pre-sorting/filtering) behavior exactly: every call site, sorted by
// self time descending, as plain text.
struct ProfileOptions {
    std::string sortKey = "self"; // "self" | "cumulative" | "calls" | "name"
    double minSelf = 0.0;
    int minCalls = 0;
    std::string format = "text"; // "text" | "csv"
};

// Filters profile.callSites to selfTime >= minSelf and callCount >=
// minCalls, then sorts by opts.sortKey. Every non-"name" order is
// tie-broken by (callOrigin, callLine, name) -- CallSiteProfile's own
// storage order (a std::map's key order) isn't sorted by any of these, so
// this needs an explicit, deterministic tie-break regardless of sort key.
// Ported identically to the Python reference's own
// _select_and_sort_call_sites in cli.py.
std::vector<CallSiteProfile> selectAndSortCallSites(const ProfileResult& profile, const ProfileOptions& opts) {
    std::vector<CallSiteProfile> sites;
    for (const CallSiteProfile& site : profile.callSites) {
        if (site.selfTime < opts.minSelf || site.callCount < opts.minCalls) continue;
        sites.push_back(site);
    }
    auto tieBreak = [](const CallSiteProfile& a, const CallSiteProfile& b) {
        if (a.callOrigin != b.callOrigin) return a.callOrigin < b.callOrigin;
        if (a.callLine != b.callLine) return a.callLine < b.callLine;
        return a.name < b.name;
    };
    if (opts.sortKey == "cumulative") {
        std::sort(sites.begin(), sites.end(), [&](const CallSiteProfile& a, const CallSiteProfile& b) {
            return a.cumulativeTime != b.cumulativeTime ? a.cumulativeTime > b.cumulativeTime : tieBreak(a, b);
        });
    } else if (opts.sortKey == "calls") {
        std::sort(sites.begin(), sites.end(), [&](const CallSiteProfile& a, const CallSiteProfile& b) {
            return a.callCount != b.callCount ? a.callCount > b.callCount : tieBreak(a, b);
        });
    } else if (opts.sortKey == "name") {
        std::sort(sites.begin(), sites.end(), [](const CallSiteProfile& a, const CallSiteProfile& b) {
            if (a.name != b.name) return a.name < b.name;
            if (a.callOrigin != b.callOrigin) return a.callOrigin < b.callOrigin;
            return a.callLine < b.callLine;
        });
    } else { // "self" (default)
        std::sort(sites.begin(), sites.end(), [&](const CallSiteProfile& a, const CallSiteProfile& b) {
            return a.selfTime != b.selfTime ? a.selfTime > b.selfTime : tieBreak(a, b);
        });
    }
    return sites;
}

std::string renderProfileReportText(const std::string& sourcePath, const ProfileResult& profile,
                                     const std::vector<CallSiteProfile>& sites) {
    std::ostringstream out;
    out << "Profile report for " << sourcePath << "\n\n";
    out << std::fixed << std::setprecision(6);
    out << "Total time:      " << profile.totalTime << "s\n";
    out << "  resolve:       " << profile.resolveTime << "s\n";
    out << "  generate:      " << profile.generateTime << "s\n";
    out << "  unattributed:  " << profile.unattributedTime << "s\n\n";

    out << std::left << std::setw(8) << "kind" << " " << std::setw(24) << "name" << " " << std::setw(24) << "caller"
        << " " << std::setw(28) << "location" << " " << std::right << std::setw(6) << "calls" << " " << std::setw(12)
        << "self(s)" << " " << std::setw(14) << "cumulative(s)" << "\n";

    for (const CallSiteProfile& site : sites) {
        const std::string& origin = site.callOrigin.empty() ? sourcePath : site.callOrigin;
        const std::string location = std::filesystem::path(origin).filename().string() + ":" + std::to_string(site.callLine);
        out << std::left << std::setw(8) << site.kind << " " << std::setw(24) << site.name << " " << std::setw(24)
            << site.callerName << " " << std::setw(28) << location << " " << std::right << std::setw(6) << site.callCount
            << " " << std::setw(12) << std::fixed << std::setprecision(6) << site.selfTime << " " << std::setw(14)
            << std::fixed << std::setprecision(6) << site.cumulativeTime << "\n";
    }
    return out.str();
}

// Quotes a CSV field per RFC 4180 (wrap in double quotes, double any
// embedded quotes) only when it actually needs it -- a plain identifier
// (every kind/name/caller value in practice) round-trips unchanged.
std::string csvField(const std::string& s) {
    if (s.find_first_of(",\"\n\r") == std::string::npos) return s;
    std::string escaped = "\"";
    for (char c : s) {
        if (c == '"') escaped += "\"\"";
        else escaped += c;
    }
    escaped += "\"";
    return escaped;
}

std::string renderProfileReportCsv(const std::string& sourcePath, const ProfileResult& profile,
                                    const std::vector<CallSiteProfile>& sites) {
    // The summary lives in "#"-prefixed comment lines ahead of the real
    // CSV header/rows -- readers that want just the tabular data can skip
    // them (e.g. pandas.read_csv(..., comment="#")) or a plain `grep -v
    // '^#'`; these aren't real CSV fields so aren't comma-escaped.
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "# source," << sourcePath << "\n";
    out << "# total_time," << profile.totalTime << "\n";
    out << "# resolve_time," << profile.resolveTime << "\n";
    out << "# generate_time," << profile.generateTime << "\n";
    out << "# unattributed_time," << profile.unattributedTime << "\n";
    out << "kind,name,caller,call_origin,call_line,call_count,self_time,cumulative_time\n";
    for (const CallSiteProfile& site : sites) {
        const std::string origin = site.callOrigin.empty() ? sourcePath : site.callOrigin;
        out << csvField(site.kind) << "," << csvField(site.name) << "," << csvField(site.callerName) << ","
            << csvField(origin) << "," << site.callLine << "," << site.callCount << "," << std::fixed
            << std::setprecision(6) << site.selfTime << "," << std::fixed << std::setprecision(6) << site.cumulativeTime
            << "\n";
    }
    return out.str();
}

// --profile report: a summary of resolve/generate/total time, then one
// row per call site (optionally filtered/sorted per `opts`), as plain
// text (default) or CSV. Ported identically (same column layout, same
// tie-break rule, same CSV columns) to the Python reference's own
// _format_profile_report in cli.py, so a report generated by either CLI
// on the same script looks the same.
std::string formatProfileReport(const std::string& sourcePath, const ProfileResult& profile, const ProfileOptions& opts) {
    std::vector<CallSiteProfile> sites = selectAndSortCallSites(profile, opts);
    if (opts.format == "csv") return renderProfileReportCsv(sourcePath, profile, sites);
    return renderProfileReportText(sourcePath, profile, sites);
}

} // namespace

int runCli(const std::vector<std::string>& args, std::istream& in, std::ostream& out, std::ostream& err) {
    std::string inputPath;
    std::string outputPath;
    std::string format;
    std::string profilePath;
    std::string profileFormat = "text";
    std::string profileSort = "self";
    std::string profileMinSelfStr;
    std::string profileMinCallsStr;
    bool debug = false;
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "-o" && i + 1 < args.size()) {
            outputPath = args[++i];
        } else if (arg == "--format" && i + 1 < args.size()) {
            format = args[++i];
        } else if (arg == "--profile" && i + 1 < args.size()) {
            profilePath = args[++i];
        } else if (arg == "--profile-format" && i + 1 < args.size()) {
            profileFormat = args[++i];
        } else if (arg == "--profile-sort" && i + 1 < args.size()) {
            profileSort = args[++i];
        } else if (arg == "--profile-min-self" && i + 1 < args.size()) {
            profileMinSelfStr = args[++i];
        } else if (arg == "--profile-min-calls" && i + 1 < args.size()) {
            profileMinCallsStr = args[++i];
        } else if (arg == "--debug") {
            debug = true;
        } else if (inputPath.empty()) {
            inputPath = arg;
        }
    }
    if (inputPath.empty() || outputPath.empty()) {
        err << "usage: openscad-cpp-evaluator <input.scad> -o <output.{stl,obj,off,3mf}> [--format stl|obj|off|3mf] "
               "[--profile FILENAME [--profile-format text|csv] [--profile-sort self|cumulative|calls|name] "
               "[--profile-min-self SECONDS] [--profile-min-calls N]] [--debug]\n";
        return 1;
    }
    const std::string fmt = formatForPath(format, outputPath);
    if (fmt.empty()) {
        err << "error: cannot infer output format from '" << outputPath << "' -- pass --format\n";
        return 1;
    }

    ProfileOptions profileOpts;
    if (!profilePath.empty()) {
        if (profileFormat != "text" && profileFormat != "csv") {
            err << "error: --profile-format must be 'text' or 'csv' (got '" << profileFormat << "')\n";
            return 1;
        }
        if (profileSort != "self" && profileSort != "cumulative" && profileSort != "calls" && profileSort != "name") {
            err << "error: --profile-sort must be one of self|cumulative|calls|name (got '" << profileSort << "')\n";
            return 1;
        }
        profileOpts.format = profileFormat;
        profileOpts.sortKey = profileSort;
        if (!profileMinSelfStr.empty()) {
            try {
                size_t consumed = 0;
                profileOpts.minSelf = std::stod(profileMinSelfStr, &consumed);
                if (consumed != profileMinSelfStr.size()) throw std::invalid_argument("trailing characters");
            } catch (...) {
                err << "error: --profile-min-self must be a number (got '" << profileMinSelfStr << "')\n";
                return 1;
            }
        }
        if (!profileMinCallsStr.empty()) {
            try {
                size_t consumed = 0;
                profileOpts.minCalls = std::stoi(profileMinCallsStr, &consumed);
                if (consumed != profileMinCallsStr.size()) throw std::invalid_argument("trailing characters");
            } catch (...) {
                err << "error: --profile-min-calls must be an integer (got '" << profileMinCallsStr << "')\n";
                return 1;
            }
        }
    }

    try {
        std::vector<std::unique_ptr<oscad::ASTNode>> ast = oscad::getASTFromFile(inputPath);
        ResolvedUseScopes used = resolveUseScopes(ast, inputPath, [&out](const std::string& msg) { out << msg << "\n"; });

        std::optional<DebugRepl> repl;
        DebugHooks hooks;
        if (debug) {
            repl.emplace(inputPath, in, out);
            repl->installInterruptHandler(); // Ctrl+C during evaluate() pauses like a breakpoint
            // linenoise reads/writes the real stdin/stdout file descriptors
            // directly, not `in`/`out` themselves -- only turn it on when
            // those really are std::cin/std::cout (genuine interactive use),
            // never for the test suite's injected istringstream/ostringstream.
            if (&in == &std::cin && &out == &std::cout) repl->enableLineEditing();
            if (!repl->runPrompt()) return 0; // user quit before "run"
            hooks.debugHook = [&](int line, int depth, bool forced, const std::string& origin,
                                   const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame) {
                return repl->debugHook(line, depth, forced, origin, callStack, getFrame);
            };
            hooks.errorBreak = [&](int line, const std::string& header, const std::string& origin,
                                    const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame) {
                repl->errorBreak(line, header, origin, callStack, getFrame);
            };
            hooks.returnHook = [&](const std::string& name, const Value& result, int depth) {
                repl->returnHook(name, result, depth);
            };
        }

        Evaluator evaluator([&out](const std::string& msg) { out << msg << "\n"; }, nullptr, nullptr, hooks,
                             /*profiling=*/!profilePath.empty());
        if (repl) repl->attachEvaluator(evaluator); // lets "child" read Evaluator::lastChildrenPositions()
        EvalContext ctx = EvalContext::makeRoot(used.rootScope.get());

        std::vector<ColoredBody> bodies = toRenderableBodies(evaluator.evaluate(used.processedNodes, ctx));

        if (!profilePath.empty()) {
            std::ofstream profileFile(profilePath);
            if (!profileFile) {
                err << "error: cannot open '" << profilePath << "' for writing\n";
                return 1;
            }
            profileFile << formatProfileReport(inputPath, *evaluator.profileResult, profileOpts);
        }

        if (fmt == "stl") {
            writeStl(outputPath, bodies);
        } else if (fmt == "obj") {
            writeObj(outputPath, bodies);
        } else if (fmt == "off") {
            writeOff(outputPath, bodies);
        } else {
            writeThreeMf(outputPath, bodies);
        }
        out << "Exported to " << outputPath << "\n";
        return 0;
    } catch (const oscad::ParseError& e) {
        err << e.what() << "\n";
        return 1;
    } catch (const EvalError& e) {
        // e.what() is already the fully formatted "ERROR: ...\nTRACE: ..."
        // message (see eval_error.hpp) -- no extra prefix here, unlike the
        // generic catch below.
        err << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        err << "ERROR: " << e.what() << "\n";
        return 1;
    }
}

} // namespace oscadeval
