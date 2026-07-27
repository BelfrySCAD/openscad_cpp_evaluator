#include "openscad_cpp_evaluator/debug_repl.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/value.hpp"

#include <linenoise.hpp>

#include <algorithm>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace oscadeval {

namespace {

// A plain C function pointer is the only thing std::signal() accepts --
// it can't capture `this`, so this bridges to whichever DebugRepl
// instance last called installInterruptHandler(). Only one DebugRepl is
// ever active at a time in this CLI's own usage; a future embedder
// running more than one concurrently would need a different design, but
// nothing here does that today.
DebugRepl* g_activeDebugRepl = nullptr;

extern "C" void handleSigint(int) {
    if (g_activeDebugRepl) g_activeDebugRepl->requestPause();
}

std::string realpath(const std::string& path) {
    std::error_code ec;
    std::filesystem::path resolved = std::filesystem::weakly_canonical(path, ec);
    return ec ? path : resolved.string();
}

std::string basename(const std::string& path) { return std::filesystem::path(path).filename().string(); }

const DeclInfo* findDecl(const std::vector<DeclInfo>& decls, const std::string& name) {
    for (const DeclInfo& d : decls) {
        if (d.name == name) return &d;
    }
    return nullptr;
}

void printDecls(std::ostream& out, const std::vector<DeclInfo>& decls, const char* kindLabel) {
    if (decls.empty()) {
        out << "No user-defined " << kindLabel << ".\n";
        return;
    }
    out << "User-defined " << kindLabel << ":\n";
    for (const DeclInfo& d : decls) {
        out << "  " << d.name << "(" << d.params << ") at " << basename(d.origin) << ":" << d.line << "\n";
    }
}

std::vector<std::string> readLines(const std::string& path) {
    std::vector<std::string> lines;
    std::ifstream file(path);
    if (file) {
        std::string line;
        while (std::getline(file, line)) lines.push_back(line);
    }
    return lines;
}

std::string trim(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::pair<std::string, std::string> splitFirstWord(const std::string& raw) {
    const std::string t = trim(raw);
    const size_t sp = t.find(' ');
    if (sp == std::string::npos) return {t, ""};
    return {t.substr(0, sp), trim(t.substr(sp + 1))};
}

Value parseValueForRepl(const std::string& s) {
    if (s == "undef") return Value{};
    if (s == "true") return Value{true};
    if (s == "false") return Value{false};
    try {
        size_t consumed = 0;
        const double d = std::stod(s, &consumed);
        if (consumed == s.size()) return Value{d};
    } catch (...) {
    }
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"') return Value{s.substr(1, s.size() - 2)};
    return Value{};
}

constexpr const char* kPreRunHelp =
    "Commands (before \"run\"):\n"
    "  run, r, restart        Start evaluating the script\n"
    "  break [file:]line, b   Set a breakpoint\n"
    "  delete [file:]line, d  Delete a breakpoint (no args: delete all)\n"
    "  info breakpoints       List breakpoints\n"
    "  info modules           List user-defined modules\n"
    "  info functions         List user-defined functions\n"
    "  list [line|name], l    Show source around a line, or a function/module\n"
    "                          (\"foo\" if unambiguous, else \"function:foo\"/\"module:foo\")\n"
    "  quit, q, exit          Exit without running\n"
    "  help, h                Show this text\n"
    "(Enter on a blank line repeats the last restart/list)";

constexpr const char* kPausedHelp =
    "Commands (while paused):\n"
    "  continue, c             Resume until the next breakpoint\n"
    "  step, s                 Step into the next statement/call\n"
    "  next, n                 Step over the next statement (don't descend into calls)\n"
    "  finish, fin             Run until the current call returns\n"
    "  child, sc               Step to child: run until children()/children(N) forwards\n"
    "                          control to one of this call's own { ... } children\n"
    "                          (or it returns, if it never calls children() at all)\n"
    "  stop                    Abort the current evaluation, return to the pre-run prompt\n"
    "  restart, r              Abort the current evaluation and run again from the start\n"
    "  print <name>, p         Print a variable's value\n"
    "  backtrace, bt, where    Show the call stack (innermost first)\n"
    "  up                      Select the caller frame (view its variables)\n"
    "  down                    Select the callee frame\n"
    "  frame [n], f            Select frame #n (no arg: show the current frame)\n"
    "  info breakpoints        List breakpoints\n"
    "  info variables          List the selected frame's variables\n"
    "  info modules            List user-defined modules\n"
    "  info functions          List user-defined functions\n"
    "  list [line|name], l     Show source around a line, or a function/module\n"
    "                          (\"foo\" if unambiguous, else \"function:foo\"/\"module:foo\")\n"
    "  break [file:]line, b    Set a breakpoint\n"
    "  delete [file:]line, d   Delete a breakpoint (no args: delete all)\n"
    "  set <name>=<value>      Override a variable's value on resume\n"
    "  quit, q, exit           Abort evaluation\n"
    "  help, h                 Show this text\n"
    "(Enter on a blank line repeats the last step/next/child/restart/\n"
    " continue/finish/list)";

} // namespace

DebugRepl::DebugRepl(const std::string& sourcePath, std::istream& in, std::ostream& out)
    : in_(in), out_(out), sourcePath_(realpath(sourcePath)) {
    sourceLinesByOrigin_[sourcePath_] = readLines(sourcePath);
}

void DebugRepl::installInterruptHandler() {
    g_activeDebugRepl = this;
    std::signal(SIGINT, handleSigint);
}

void DebugRepl::setDeclaredNames(std::vector<DeclInfo> functions, std::vector<DeclInfo> modules) {
    // The caller (cli_lib.cpp) doesn't realpath() these -- it has no
    // access to this class's own private realpath() helper -- so every
    // origin gets normalized here, the same way every other origin this
    // class stores/compares (sourcePath_, breakpoints_' keys, stepOrigin_,
    // ...) already is.
    for (DeclInfo& d : functions) d.origin = realpath(d.origin);
    for (DeclInfo& d : modules) d.origin = realpath(d.origin);
    declaredFunctions_ = std::move(functions);
    declaredModules_ = std::move(modules);
}

bool DebugRepl::readCommandLine(const std::string& prompt, std::string& raw) {
    if (lineEditingEnabled_) {
        // linenoise::Readline returns true when the user wants to quit
        // (Ctrl-C, Ctrl-D on an empty line, or EOF) -- inverted from
        // std::getline's own "true means got a line" convention, so this
        // flips it right here rather than leaking that surprise to callers.
        const bool quit = linenoise::Readline(prompt.c_str(), raw);
        if (quit) return false;
        if (!raw.empty()) linenoise::AddHistory(raw.c_str());
        return true;
    }
    out_ << prompt << std::flush;
    return static_cast<bool>(std::getline(in_, raw));
}

std::string DebugRepl::resolveOrigin(const std::string& origin) const { return origin.empty() ? sourcePath_ : realpath(origin); }

const std::vector<std::string>& DebugRepl::linesFor(const std::string& origin) const {
    auto it = sourceLinesByOrigin_.find(origin);
    if (it == sourceLinesByOrigin_.end()) it = sourceLinesByOrigin_.emplace(origin, readLines(origin)).first;
    return it->second;
}

std::pair<std::string, std::optional<int>> DebugRepl::parseLocation(const std::string& arg) const {
    const std::string t = trim(arg);
    const size_t colon = t.rfind(':');
    std::string filePart, linePart;
    std::string origin;
    if (colon != std::string::npos) {
        origin = realpath(t.substr(0, colon));
        linePart = t.substr(colon + 1);
    } else {
        origin = sourcePath_;
        linePart = t;
    }
    try {
        size_t consumed = 0;
        const int line = std::stoi(linePart, &consumed);
        if (consumed == linePart.size()) return {origin, line};
    } catch (...) {
    }
    return {origin, std::nullopt};
}

void DebugRepl::addBreakpoint(const std::string& arg) {
    auto [origin, line] = parseLocation(arg);
    if (!line) {
        out_ << "Usage: break [file:]line\n";
        return;
    }
    breakpoints_[origin].insert(*line);
    out_ << "Breakpoint set at " << basename(origin) << ":" << *line << "\n";
}

void DebugRepl::deleteBreakpoint(const std::string& arg) {
    if (trim(arg).empty()) {
        breakpoints_.clear();
        out_ << "All breakpoints deleted\n";
        return;
    }
    auto [origin, line] = parseLocation(arg);
    if (!line) {
        out_ << "Usage: delete [file:]line\n";
        return;
    }
    auto it = breakpoints_.find(origin);
    if (it != breakpoints_.end()) it->second.erase(*line);
}

void DebugRepl::printBreakpoints() const {
    bool any = false;
    for (const auto& [origin, lines] : breakpoints_) {
        for (int line : lines) {
            out_ << "breakpoint at " << basename(origin) << ":" << line << "\n";
            any = true;
        }
    }
    if (!any) out_ << "No breakpoints set.\n";
}

void DebugRepl::listSource(const std::string& arg, std::optional<int> currentLine, const std::string& origin) const {
    std::string listOrigin = origin.empty() ? sourcePath_ : origin;
    int target = currentLine.value_or(1);
    const std::string t = trim(arg);

    if (!t.empty()) {
        size_t consumed = 0;
        bool isNumber = false;
        int parsed = 0;
        try {
            parsed = std::stoi(t, &consumed);
            isNumber = consumed == t.size();
        } catch (...) {
        }
        if (isNumber) {
            target = parsed;
        } else {
            // "function:name" / "module:name" qualifies which namespace to
            // search -- reuses this REPL's existing "prefix:rest" colon
            // convention (break/delete's own [file:]line parsing).
            // Unqualified: search both, erroring if the name exists in
            // both (a function and a module CAN share a name in OpenSCAD).
            std::string qualifier, name = t;
            const size_t colon = t.find(':');
            if (colon != std::string::npos) {
                qualifier = t.substr(0, colon);
                name = t.substr(colon + 1);
            }
            const DeclInfo* fn = (qualifier.empty() || qualifier == "function") ? findDecl(declaredFunctions_, name) : nullptr;
            const DeclInfo* mod = (qualifier.empty() || qualifier == "module") ? findDecl(declaredModules_, name) : nullptr;
            if (fn && mod) {
                out_ << "Both a function and a module are named \"" << name << "\" -- use \"list function:" << name
                     << "\" or \"list module:" << name << "\".\n";
                return;
            }
            const DeclInfo* decl = fn ? fn : mod;
            if (!decl) {
                out_ << "No symbol \"" << name << "\" in current context.\n";
                return;
            }
            listOrigin = decl->origin;
            target = decl->line;
            currentLine = std::nullopt; // jumping to a declaration, not the paused line -- no "->" marker
        }
    }

    const std::vector<std::string>& lines = linesFor(listOrigin);
    if (lines.empty()) {
        out_ << "No source available.\n";
        return;
    }
    const int lo = std::max(1, target - 5);
    const int hi = std::min(static_cast<int>(lines.size()), target + 4);
    for (int n = lo; n <= hi; ++n) {
        const char* marker = (currentLine && n == *currentLine) ? "->" : "  ";
        out_ << marker << std::setw(4) << n << "\t" << lines[static_cast<size_t>(n - 1)] << "\n";
    }
}

void DebugRepl::setVar(const std::string& arg) {
    const size_t eq = arg.find('=');
    if (eq == std::string::npos) {
        out_ << "Usage: set <name>=<value>\n";
        return;
    }
    const std::string name = trim(arg.substr(0, eq));
    const Value parsed = parseValueForRepl(trim(arg.substr(eq + 1)));
    pendingMods_[name] = parsed;
    out_ << name << " will be set to " << fmtValue(parsed) << " on resume\n";
}

void DebugRepl::printVar(const std::string& arg, const std::unordered_map<std::string, Value>& visibleVars) {
    const std::string name = trim(arg);
    if (name.empty()) {
        out_ << "Usage: print <name>\n";
        return;
    }
    auto it = visibleVars.find(name);
    if (it == visibleVars.end()) {
        out_ << "No symbol \"" << name << "\" in current context.\n";
        return;
    }
    ++printCount_;
    out_ << "$" << printCount_ << " = " << fmtValue(it->second) << "\n";
}

void DebugRepl::printVariables(const std::unordered_map<std::string, Value>& visibleVars) const {
    if (visibleVars.empty()) {
        out_ << "No variables in current context.\n";
        return;
    }
    std::vector<std::string> names;
    names.reserve(visibleVars.size());
    for (const auto& [name, value] : visibleVars) names.push_back(name);
    std::sort(names.begin(), names.end()); // unordered_map iteration order isn't deterministic
    for (const std::string& name : names) out_ << name << " = " << fmtValue(visibleVars.at(name)) << "\n";
}

void DebugRepl::printDeclaredFunctions() const { printDecls(out_, declaredFunctions_, "functions"); }

void DebugRepl::printDeclaredModules() const { printDecls(out_, declaredModules_, "modules"); }

std::pair<std::string, int> DebugRepl::frameLocation(int k, const std::vector<CallStackFrame>& callStack,
                                                      const std::string& pauseOrigin, int pauseLine) const {
    const int n = static_cast<int>(callStack.size());
    std::string curOrigin = pauseOrigin;
    int curLine = pauseLine;
    for (int i = 0; i < k && i < n; ++i) {
        const oscad::Position* callPos = callStack[static_cast<size_t>(n - 1 - i)].callPosition;
        curOrigin = callPos ? callPos->origin : sourcePath_;
        curLine = callPos ? callPos->line : 0;
    }
    return {curOrigin, curLine};
}

void DebugRepl::printBacktrace(const std::vector<CallStackFrame>& callStack, const std::string& origin, int line) const {
    const int n = static_cast<int>(callStack.size());
    for (int k = 0; k <= n; ++k) {
        const bool haveFrame = k < n;
        const std::string label = haveFrame ? callStack[static_cast<size_t>(n - 1 - k)].name + "()" : "<toplevel>";
        auto [o, l] = frameLocation(k, callStack, origin, line);
        out_ << "#" << k << "  " << label << " at " << (o.empty() ? "?" : basename(o)) << ":" << l << "\n";
    }
}

bool DebugRepl::runPrompt() {
    out_ << "Reading symbols from " << sourcePath_ << "...\n";
    for (;;) {
        std::string raw;
        if (!readCommandLine("(scad-dbg) ", raw)) {
            out_ << "\n";
            return false;
        }
        auto [cmd, arg] = splitFirstWord(raw);
        if (cmd.empty()) {
            // Hitting Enter on a blank line repeats the last "restart"/
            // "list" (the only two of the repeatable commands valid at
            // this prompt) -- mirrors gdb's own repeat-last-command
            // convention. No prior repeatable command yet: unchanged
            // no-op behavior.
            if (lastRepeatableCmd_.empty()) continue;
            cmd = lastRepeatableCmd_;
            arg = lastRepeatableArg_;
        }
        // "restart" is also accepted here (not just "run"/"r") so a user
        // who just typed "stop" can reflexively type "restart" again --
        // with nothing currently running, the two commands mean the same
        // thing at this prompt.
        if (cmd == "run" || cmd == "r" || cmd == "restart") {
            lastRepeatableCmd_ = "restart";
            lastRepeatableArg_.clear();
            return true;
        }
        if (cmd == "break" || cmd == "b") {
            addBreakpoint(arg);
        } else if (cmd == "delete" || cmd == "d") {
            deleteBreakpoint(arg);
        } else if (cmd == "info") {
            const std::string sub = trim(arg);
            if (sub.rfind("break", 0) == 0) {
                printBreakpoints();
            } else if (sub == "modules") {
                printDeclaredModules();
            } else if (sub == "functions") {
                printDeclaredFunctions();
            } else if (sub == "variables") {
                out_ << "No variables to show before \"run\".\n";
            } else {
                out_ << "Undefined info command: \"" << sub << "\". Try \"help\".\n";
            }
        } else if (cmd == "list" || cmd == "l") {
            lastRepeatableCmd_ = "list";
            lastRepeatableArg_ = arg;
            listSource(arg);
        } else if (cmd == "quit" || cmd == "q" || cmd == "exit") {
            return false;
        } else if (cmd == "help" || cmd == "h") {
            out_ << kPreRunHelp << "\n";
        } else {
            out_ << "Undefined command: \"" << cmd << "\". Try \"help\".\n";
        }
    }
}

DebugAction DebugRepl::debugHook(int line, int depth, bool forced, const std::string& origin,
                                  const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame) {
    if (quit_) return DebugAction{true, {}};

    const std::string resolved = resolveOrigin(origin);
    bool stepHit = false;
    if (stepCmd_ == "over") {
        stepHit = depth <= stepDepth_ && resolved == stepOrigin_ && line != stepLine_;
    } else if (stepCmd_ == "into") {
        stepHit = line != stepLine_ || resolved != stepOrigin_;
    } else if (stepCmd_ == "out") {
        stepHit = depth < stepDepth_;
    } else if (stepCmd_ == "to_child") {
        // Pause the first time control reaches one of the paused call's
        // own children (wherever children()/children(N) forwards to
        // them) -- or, if the call never invokes children() at all, fall
        // back to the same "call returned" safety net step-out uses, so
        // this can never hang. Mirrors BelfrySCAD's DebugSession::
        // _make_hook exactly.
        stepHit = stepToChildTargets_.count({resolved, line}) > 0 || depth < stepDepth_;
    }

    // Read-and-clear, same as BelfrySCAD's own DebugSession.pause()/
    // pause_now -- a stray SIGINT that arrives while already blocked on
    // stdin at a prompt (not mid-evaluate()) just gets consumed here on
    // the next statement check once the user resumes, causing an
    // immediate re-pause; a harmless quirk, not a hang or crash.
    const bool pauseRequested = pauseRequested_.exchange(false, std::memory_order_relaxed);

    const bool shouldPause = forced || pauseRequested || (breakOnFirst_ && resolved == sourcePath_) ||
                              breakpoints_[resolved].count(line) > 0 || stepHit;
    if (!shouldPause) return DebugAction{};

    breakOnFirst_ = false;
    stepCmd_.reset();

    std::vector<DebugFrame> frames = getFrame();
    if (pauseRequested) {
        out_ << "\nInterrupted at " << basename(resolved) << ":" << line << "\n";
    } else {
        out_ << "\nBreakpoint hit at " << basename(resolved) << ":" << line << "\n";
    }
    listSource("", line, resolved);
    return interact(line, depth, resolved, frames, callStack);
}

void DebugRepl::errorBreak(int line, const std::string& header, const std::string& origin,
                            const std::vector<CallStackFrame>& callStack, const DebugFramesFn& getFrame) {
    if (quit_) return;
    const std::string resolved = resolveOrigin(origin);
    out_ << "\n" << header << "\n";
    listSource("", line, resolved);
    std::vector<DebugFrame> frames = getFrame();
    out_ << "(evaluation will abort once you resume; inspect state, then continue/quit)\n";
    interact(line, static_cast<int>(callStack.size()), resolved, frames, callStack);
}

void DebugRepl::returnHook(const std::string&, const Value& result, int depth) {
    if (stepCmd_ == "out" && depth == stepDepth_) {
        ++printCount_;
        out_ << "Value returned is $" << printCount_ << " = " << fmtValue(result) << "\n";
    }
}

DebugAction DebugRepl::interact(int line, int depth, const std::string& origin,
                                 const std::vector<DebugFrame>& frames,
                                 const std::vector<CallStackFrame>& callStack) {
    // Which frame `print`/`info variables` inspect. 0 = innermost (the paused
    // statement); `up`/`down`/`frame N` move it. Aligns with backtrace #k.
    size_t curFrame = 0;
    static const std::unordered_map<std::string, Value> kEmptyVars;
    auto visibleVars = [&]() -> const std::unordered_map<std::string, Value>& {
        return curFrame < frames.size() ? frames[curFrame].locals : kEmptyVars;
    };
    auto printFrameHeader = [&]() {
        const int n = static_cast<int>(callStack.size());
        const int k = static_cast<int>(curFrame);
        const std::string label =
            (k < n) ? callStack[static_cast<size_t>(n - 1 - k)].name + "()" : "<toplevel>";
        auto [o, l] = frameLocation(k, callStack, origin, line);
        out_ << "#" << curFrame << "  " << label << " at " << (o.empty() ? "?" : basename(o)) << ":" << l << "\n";
    };
    for (;;) {
        std::string raw;
        if (!readCommandLine("(scad-dbg) ", raw)) {
            out_ << "\n";
            quit_ = true;
            postRunAction_ = PostRunAction::Quit;
            return DebugAction{true, {}};
        }
        auto [cmd, arg] = splitFirstWord(raw);
        if (cmd.empty()) {
            // Hitting Enter on a blank line repeats the last step/next/
            // child/restart/continue/finish/list -- mirrors gdb's own
            // repeat-last-command convention. No prior repeatable command
            // yet: unchanged no-op behavior.
            if (lastRepeatableCmd_.empty()) continue;
            cmd = lastRepeatableCmd_;
            arg = lastRepeatableArg_;
        }

        if (cmd == "continue" || cmd == "c") {
            lastRepeatableCmd_ = "continue";
            lastRepeatableArg_.clear();
            return resume(std::nullopt);
        }
        if (cmd == "step" || cmd == "s") {
            lastRepeatableCmd_ = "step";
            lastRepeatableArg_.clear();
            return resume(std::string("into"), line, depth, origin);
        }
        if (cmd == "next" || cmd == "n") {
            lastRepeatableCmd_ = "next";
            lastRepeatableArg_.clear();
            return resume(std::string("over"), line, depth, origin);
        }
        if (cmd == "finish" || cmd == "fin") {
            lastRepeatableCmd_ = "finish";
            lastRepeatableArg_.clear();
            return resume(std::string("out"), line, depth, origin);
        }
        if (cmd == "child" || cmd == "sc") {
            lastRepeatableCmd_ = "child";
            lastRepeatableArg_.clear();
            return resume(std::string("to_child"), line, depth, origin);
        }
        if (cmd == "stop") {
            quit_ = true;
            postRunAction_ = PostRunAction::Stopped;
            return DebugAction{true, {}};
        }
        if (cmd == "restart" || cmd == "r") {
            lastRepeatableCmd_ = "restart";
            lastRepeatableArg_.clear();
            quit_ = true;
            postRunAction_ = PostRunAction::Restart;
            return DebugAction{true, {}};
        }
        if (cmd == "print" || cmd == "p") {
            printVar(arg, visibleVars());
        } else if (cmd == "up") {
            if (curFrame + 1 < frames.size()) {
                ++curFrame;
                printFrameHeader();
            } else {
                out_ << "Already at the outermost frame.\n";
            }
        } else if (cmd == "down") {
            if (curFrame > 0) {
                --curFrame;
                printFrameHeader();
            } else {
                out_ << "Already at the innermost frame.\n";
            }
        } else if (cmd == "frame" || cmd == "f") {
            const std::string a = trim(arg);
            if (a.empty()) {
                printFrameHeader();
            } else {
                try {
                    const size_t idx = static_cast<size_t>(std::stoul(a));
                    if (idx < frames.size()) {
                        curFrame = idx;
                        printFrameHeader();
                    } else {
                        out_ << "No frame #" << a << " (have 0.." << (frames.empty() ? 0 : frames.size() - 1) << ").\n";
                    }
                } catch (...) {
                    out_ << "Usage: frame <n>\n";
                }
            }
        } else if (cmd == "backtrace" || cmd == "bt" || cmd == "where") {
            printBacktrace(callStack, origin, line);
        } else if (cmd == "info") {
            const std::string sub = trim(arg);
            if (sub.rfind("break", 0) == 0) {
                printBreakpoints();
            } else if (sub == "variables") {
                printVariables(visibleVars());
            } else if (sub == "modules") {
                printDeclaredModules();
            } else if (sub == "functions") {
                printDeclaredFunctions();
            } else {
                out_ << "Undefined info command: \"" << sub << "\". Try \"help\".\n";
            }
        } else if (cmd == "list" || cmd == "l") {
            lastRepeatableCmd_ = "list";
            lastRepeatableArg_ = arg;
            listSource(arg, line, origin);
        } else if (cmd == "break" || cmd == "b") {
            addBreakpoint(arg);
        } else if (cmd == "delete" || cmd == "d") {
            deleteBreakpoint(arg);
        } else if (cmd == "set") {
            setVar(arg);
        } else if (cmd == "quit" || cmd == "q" || cmd == "exit") {
            quit_ = true;
            postRunAction_ = PostRunAction::Quit;
            return DebugAction{true, {}};
        } else if (cmd == "help" || cmd == "h") {
            out_ << kPausedHelp << "\n";
        } else {
            out_ << "Undefined command: \"" << cmd << "\". Try \"help\".\n";
        }
    }
}

void DebugRepl::prepareForRun() {
    quit_ = false;
    postRunAction_ = PostRunAction::None;
    breakOnFirst_ = true;
    stepCmd_.reset();
    stepToChildTargets_.clear();
    pendingMods_.clear();
}

PostRunAction DebugRepl::takePostRunAction() {
    const PostRunAction action = postRunAction_;
    postRunAction_ = PostRunAction::None;
    return action;
}

DebugAction DebugRepl::resume(std::optional<std::string> stepCmd, int line, int depth, const std::string& origin) {
    if (stepCmd) {
        stepCmd_ = stepCmd;
        stepLine_ = line;
        stepDepth_ = depth;
        stepOrigin_ = origin;
    }
    if (stepCmd == "to_child") {
        stepToChildTargets_.clear();
        if (evaluator_) {
            const auto& targets = evaluator_->lastChildrenPositions();
            if (targets) {
                for (const auto& [childOrigin, childLine] : *targets) {
                    stepToChildTargets_.emplace(resolveOrigin(childOrigin), childLine);
                }
            }
        }
    }
    DebugAction action;
    action.mods = std::move(pendingMods_);
    pendingMods_.clear();
    return action;
}

} // namespace oscadeval
