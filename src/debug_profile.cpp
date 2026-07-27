#include "openscad_cpp_evaluator/evaluator.hpp"

namespace oscadeval {

namespace {

// (origin, line) for each top-level, non-declaration child of `node` (a
// ModularCall's own `.children` -- the `{ ... }` block passed to a module
// call), if any. See Evaluator::lastChildrenPositions()'s own doc comment
// for what this feeds. Mirrors the reference's own
// Evaluator._child_statement_positions.
std::optional<std::vector<std::pair<std::string, int>>> childStatementPositions(const oscad::ASTNode& node) {
    if (node.kind() != oscad::NodeKind::ModularCall) return std::nullopt;
    const auto& call = static_cast<const oscad::ModularCall&>(node);
    std::vector<std::pair<std::string, int>> positions;
    for (const auto& c : call.children) {
        if (c->kind() == oscad::NodeKind::Assignment || c->kind() == oscad::NodeKind::ModuleDeclaration ||
            c->kind() == oscad::NodeKind::FunctionDeclaration) {
            continue;
        }
        positions.emplace_back(c->position().origin, c->position().line);
    }
    if (positions.empty()) return std::nullopt;
    return positions;
}

} // namespace

DebugFrame Evaluator::buildDebugFrame(const EvalContext* ctx) const {
    DebugFrame frame;
    if (!ctx) return frame;
    for (const auto& [k, v] : ctx->let_->items()) frame.locals[k] = v;
    for (const auto& [k, v] : ctx->dyn->items()) {
        if (!k.empty() && k[0] == '$') frame.locals[k] = v;
    }
    // Top-level script variables not shadowed by a nested call's own
    // locals -- lets `print` reach a script-level variable while paused
    // deep inside a module/function call, matching the reference's own
    // outer_scope merge for the innermost frame.
    if (!callStack_.empty() && rootCtx_ != nullptr) {
        for (const auto& [k, v] : rootCtx_->let_->items()) {
            if (!frame.locals.count(k)) frame.locals[k] = v;
        }
    }
    return frame;
}

void Evaluator::checkDebug(const oscad::ASTNode& node, EvalContext& ctx, bool forced) {
    if (!debugHooks_.debugHook) return;
    const oscad::Position& pos = node.position();
    const int depth = static_cast<int>(callStack_.size());
    const DebugFramesFn getFrame = [this, &ctx]() { return buildDebugFrame(&ctx); };
    lastChildrenPositions_ = childStatementPositions(node);

    DebugAction action = debugHooks_.debugHook(pos.line, depth, forced, pos.origin, callStack_, getFrame);
    for (auto& [k, v] : action.mods) ctx.let_->set(k, v);
    if (action.stop) throw EvalError(kDebuggingStoppedMessage);
}

std::optional<Evaluator::ProfileHandle> Evaluator::profileEnter(const std::string& kind, const std::string& name,
                                                                  const oscad::Position* callPos, const oscad::Position* declPos) {
    if (!profiling_) return std::nullopt;
    const std::string callOrigin = callPos ? callPos->origin : "";
    const int callLine = callPos ? callPos->line : 0;
    const ProfileSiteKey key{kind, name, callOrigin, callLine};

    auto it = profileSites_.find(key);
    if (it == profileSites_.end()) {
        // The caller's frame is still on top of callStack_ -- this call's
        // own frame isn't pushed until after profileEnter() returns. A
        // given call expression is always lexically inside the same
        // enclosing body regardless of which invocation this is, so
        // callerName is a one-time-computed structural property of the
        // site, same as declOrigin/declLine.
        CallSiteProfile site;
        site.kind = kind;
        site.name = name;
        site.callerName = callStack_.empty() ? "<toplevel>" : callStack_.back().name;
        site.callOrigin = callOrigin;
        site.callLine = callLine;
        site.declOrigin = declPos ? declPos->origin : "";
        site.declLine = declPos ? declPos->line : 0;
        it = profileSites_.emplace(key, std::move(site)).first;
    }
    it->second.callCount += 1;

    const bool recursiveReentry = profileActive_.count(key) > 0;
    if (!recursiveReentry) profileActive_.insert(key);
    profileChildTime_.push_back(0.0);
    return ProfileHandle{key, recursiveReentry, std::chrono::steady_clock::now()};
}

void Evaluator::profileRecordTailHop(const std::string& kind, const std::string& name, const oscad::Position* callPos,
                                      const oscad::Position* declPos) {
    if (!profiling_) return;
    const std::string callOrigin = callPos ? callPos->origin : "";
    const int callLine = callPos ? callPos->line : 0;
    const ProfileSiteKey key{kind, name, callOrigin, callLine};

    auto it = profileSites_.find(key);
    if (it == profileSites_.end()) {
        CallSiteProfile site;
        site.kind = kind;
        site.name = name;
        // The trampoline has already mutated callStack_.back() to this
        // hop's own identity by the time this runs -- its PREVIOUS
        // contents (the caller of this hop, from the trampoline chain's
        // point of view) aren't recoverable here, so callerName falls back
        // to the frame one level further out (the real, non-trampolined
        // caller) -- matches profileEnter's own "callStack_.back() is the
        // caller" rule, since that's genuinely one level too shallow only
        // in the middle of a tail chain, a case this port's profiler
        // doesn't try to distinguish further (see this function's own doc
        // comment on the wall-time lumping simplification).
        site.callerName = callStack_.empty() ? "<toplevel>" : callStack_.back().name;
        site.callOrigin = callOrigin;
        site.callLine = callLine;
        site.declOrigin = declPos ? declPos->origin : "";
        site.declLine = declPos ? declPos->line : 0;
        it = profileSites_.emplace(key, std::move(site)).first;
    }
    it->second.callCount += 1;
}

void Evaluator::profileExit(const ProfileHandle& handle) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - handle.start).count();
    const double childTime = profileChildTime_.back();
    profileChildTime_.pop_back();

    CallSiteProfile& site = profileSites_.at(handle.key);
    // Self time is unconditional (disjoint wall-clock slices via the
    // child-time stack, so nothing can double-count). Cumulative time
    // skips a recursive re-entry -- the outermost invocation's own
    // elapsed already includes every nested invocation's time via child-
    // time propagation to the parent frame below; without this guard a
    // self-recursive call site's cumulative_time would balloon past total
    // wall time.
    site.selfTime += elapsed - childTime;
    if (!handle.recursiveReentry) {
        site.cumulativeTime += elapsed;
        profileActive_.erase(handle.key);
    }
    if (!profileChildTime_.empty()) profileChildTime_.back() += elapsed;
}

} // namespace oscadeval
