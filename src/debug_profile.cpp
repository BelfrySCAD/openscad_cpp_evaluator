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

DebugFrame Evaluator::buildDebugFrame(const EvalContext* ctx, bool includeOuter) const {
    DebugFrame frame;
    if (!ctx) return frame;
    // localScope: this frame's own let-bound locals (also the editable
    // dynNames) plus its $-dynamic vars.
    for (const auto& [k, v] : ctx->let_->items()) {
        frame.localScope[k] = v;
        frame.dynNames.push_back(k);
    }
    for (const auto& [k, v] : ctx->dyn->items()) {
        if (!k.empty() && k[0] == '$') frame.localScope[k] = v;
    }
    // outerScope: top-level script variables not shadowed by this frame's
    // locals -- only for the innermost (paused) frame, matching the
    // reference's own outer_scope merge. Lets `print` / the vars pane reach
    // a script-level variable while paused deep inside a call.
    if (includeOuter && !callStack_.empty() && rootCtx_ != nullptr) {
        for (const auto& [k, v] : rootCtx_->let_->items()) {
            if (!frame.localScope.count(k)) frame.outerScope[k] = v;
        }
    }
    // Merged view for the CLI REPL (localScope wins on collision).
    frame.locals = frame.localScope;
    for (const auto& [k, v] : frame.outerScope) frame.locals.emplace(k, v);
    return frame;
}

std::vector<DebugFrame> Evaluator::buildDebugFrames(const EvalContext* ctx) const {
    std::vector<DebugFrame> frames;
    // Frame 0: the paused statement's own scope (with outerScope merged in).
    DebugFrame current = buildDebugFrame(ctx, /*includeOuter=*/true);
    // Enclosing calls, inner-to-outer -- every active call except the
    // innermost (frame 0 already represents that call's body). Mirrors the
    // reference's reversed(_frame_ctxs[:-1]).
    frames.push_back(current);
    for (int i = static_cast<int>(callStack_.size()) - 2; i >= 0; --i) {
        frames.push_back(buildDebugFrame(callStack_[i].bodyCtx, /*includeOuter=*/false));
    }
    // A final top-level frame (the script's own globals) when inside a call.
    if (!callStack_.empty()) {
        DebugFrame top;
        top.localScope = current.outerScope;
        top.locals = current.outerScope;
        frames.push_back(std::move(top));
    }
    return frames;
}

void Evaluator::checkDebug(const oscad::ASTNode& node, EvalContext& ctx, bool forced, bool exprLevel) {
    if (!debugHooks_.debugHook) return;
    const oscad::Position& pos = node.position();
    // Fast-continue's hook-skippable mode (setFastContinueBreakpoints' own
    // doc comment): a plain "Continue" with no step pending needs the debug
    // hook called ONLY for a line that actually has a breakpoint -- every
    // other checkpoint is guaranteed to do nothing (no forced/breakpoint/
    // step_hit condition on the Python side can possibly fire), so skip the
    // call (and the childStatementPositions/getFrame setup below, all
    // wasted work otherwise) entirely rather than crossing into Python just
    // to be told "continue". `forced` (the explicit breakpoint() builtin)
    // always bypasses this, matching its own "bypasses nothing" contract.
    // Never applies to step_over/step_out (hookSkippable is false for those
    // even though they also set a real breakpoints set, see the setter's
    // own doc comment) or step_into/step_to_child (fastContinueBreakpoints_
    // itself is nullopt then) -- both need every statement inspected.
    if (!forced && fastContinueHookSkippable_ && fastContinueBreakpoints_) {
        // Test-and-clear: if the main thread requested an interrupt (Pause,
        // or a breakpoint edit -- see setFastContinueInterruptFlag's own doc
        // comment) since the last checkDebug() call, this call falls
        // through and actually invokes the hook below instead of skipping,
        // regardless of whether THIS specific line has a breakpoint --
        // that's what lets the hook's own logic (pause_now, or a freshly
        // updated breakpoints dict) run at all in hook-skippable mode.
        const bool interrupted =
            fastContinueInterrupt_ && fastContinueInterrupt_->exchange(false, std::memory_order_acq_rel);
        if (!interrupted) {
            auto originIt = fastContinueBreakpoints_->find(pos.origin);
            if (originIt == fastContinueBreakpoints_->end() || !originIt->second.count(pos.line)) {
                return;
            }
        }
    }
    const int depth = static_cast<int>(callStack_.size());
    const DebugFramesFn getFrame = [this, &ctx]() { return buildDebugFrames(&ctx); };
    lastChildrenPositions_ = childStatementPositions(node);

    DebugAction action = debugHooks_.debugHook(pos.line, depth, forced, exprLevel, pos.origin, callStack_, getFrame);
    for (auto& [k, v] : action.mods) ctx.let_->set(k, v);
    if (action.stop) throw EvalError(kDebuggingStoppedMessage);
}

int Evaluator::profilePathEnter(const std::string& kind, const std::string& name, const std::string& callOrigin,
                                 int callLine, int callColumn, const oscad::Position* declPos, bool& folded) {
    folded = false;
    if (profilePaths_.empty()) {
        ProfilePathNode root;
        root.name = "<toplevel>";
        profilePaths_.push_back(std::move(root));
        profileCurrentPath_ = 0;
    }
    const int parent = profileCurrentPath_ < 0 ? 0 : profileCurrentPath_;

    // Already on this path? Fold onto that node rather than growing a chain
    // per recursive level (see ProfilePathNode).
    for (int walk = parent; walk > 0; walk = profilePaths_[static_cast<size_t>(walk)].parent) {
        const ProfilePathNode& n = profilePaths_[static_cast<size_t>(walk)];
        if (n.name == name && n.callOrigin == callOrigin && n.callLine == callLine &&
            n.callColumn == callColumn && n.kind == kind) {
            folded = true;   // recursion: this site is already open on this path
            return walk;
        }
    }
    for (int childIdx : profilePaths_[static_cast<size_t>(parent)].children) {
        const ProfilePathNode& n = profilePaths_[static_cast<size_t>(childIdx)];
        if (n.name == name && n.callOrigin == callOrigin && n.callLine == callLine &&
            n.callColumn == callColumn && n.kind == kind) {
            return childIdx;
        }
    }
    if (profilePaths_.size() >= kMaxProfilePathNodes) {
        folded = true;       // cap reached: stop subdividing, keep totals honest
        return parent;
    }

    ProfilePathNode node;
    node.parent = parent;
    node.kind = kind;
    node.name = name;
    node.callOrigin = callOrigin;
    node.callLine = callLine;
    node.callColumn = callColumn;
    node.declOrigin = declPos ? declPos->origin : "";
    node.declLine = declPos ? declPos->line : 0;
    const int idx = static_cast<int>(profilePaths_.size());
    profilePaths_.push_back(std::move(node));
    profilePaths_[static_cast<size_t>(parent)].children.push_back(idx);
    return idx;
}

void Evaluator::finalizeProfilePaths() {
    // Children always have a higher index than their parent (a node is
    // appended when first reached, and folding never creates an edge back
    // to an ancestor), so one reverse sweep resolves the whole tree with
    // no recursion and no risk of an unbounded native stack.
    for (size_t i = profilePaths_.size(); i-- > 0;) {
        ProfilePathNode& n = profilePaths_[i];
        double total = n.selfTime;
        for (int c : n.children) total += profilePaths_[static_cast<size_t>(c)].cumulativeTime;
        n.cumulativeTime = total;
    }
}

std::optional<Evaluator::ProfileHandle> Evaluator::profileEnter(const std::string& kind, const std::string& name,
                                                                  const oscad::Position* callPos, const oscad::Position* declPos) {
    if (!profiling_) return std::nullopt;
    const std::string callOrigin = callPos ? callPos->origin : "";
    const int callLine = callPos ? callPos->line : 0;
    const int callColumn = callPos ? callPos->column : 0;
    const ProfileSiteKey key{kind, name, callOrigin, callLine, callColumn};

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
        site.callColumn = callColumn;
        site.declOrigin = declPos ? declPos->origin : "";
        site.declLine = declPos ? declPos->line : 0;
        it = profileSites_.emplace(key, std::move(site)).first;
    }
    it->second.callCount += 1;

    const bool recursiveReentry = profileActive_.count(key) > 0;
    if (!recursiveReentry) profileActive_.insert(key);
    profileChildTime_.push_back(0.0);

    const int prevPath = profileCurrentPath_;
    bool folded = false;   // only affects node reuse now, not accounting
    const int pathNode = profilePathEnter(kind, name, callOrigin, callLine, callColumn, declPos, folded);
    profilePaths_[static_cast<size_t>(pathNode)].callCount += 1;
    profileCurrentPath_ = pathNode;

    ProfileHandle handle{key, recursiveReentry, std::chrono::steady_clock::now()};
    handle.pathNode = pathNode;
    handle.pathPrev = prevPath;
    return handle;
}

void Evaluator::profileRecordTailHop(const std::string& kind, const std::string& name, const oscad::Position* callPos,
                                      const oscad::Position* declPos) {
    if (!profiling_) return;
    const std::string callOrigin = callPos ? callPos->origin : "";
    const int callLine = callPos ? callPos->line : 0;
    const int callColumn = callPos ? callPos->column : 0;
    const ProfileSiteKey key{kind, name, callOrigin, callLine, callColumn};

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
        site.callColumn = callColumn;
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

    // Same accounting, but attributed to this call's own node in the
    // calling-context tree rather than to the site aggregated over every
    // path that reached it.
    if (handle.pathNode >= 0 && static_cast<size_t>(handle.pathNode) < profilePaths_.size()) {
        // Self time only. Cumulative is DERIVED from the subtree once the
        // run finishes (finalizeProfilePaths), not measured here.
        //
        // Measuring it per entry is wrong at a fold point: when a site
        // recurses, every level lands on the same node, but only the
        // outermost entry may add its elapsed (or nested time double-
        // counts) -- while the calls made from every level still attach as
        // that node's children. The node then reads smaller than the
        // children under it. Found on a real model: a `_translate` entered
        // 7 times via recursion showed 22.85ms with 90.09ms of children.
        //
        // Self time has no such problem: it is disjoint by construction
        // (the child-time stack subtracts nested work), so summing it
        // across every entry is exactly this node's own cost on this path,
        // and cumulative built from it can never contradict the subtree.
        ProfilePathNode& node = profilePaths_[static_cast<size_t>(handle.pathNode)];
        node.selfTime += elapsed - childTime;
        profileCurrentPath_ = handle.pathPrev;
    }

    if (!profileChildTime_.empty()) profileChildTime_.back() += elapsed;
}

} // namespace oscadeval
