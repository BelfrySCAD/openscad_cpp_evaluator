#pragma once

#include "openscad_cpp_parser/api.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace oscadeval {

// Owns everything resolveUseScopes() pulls in from `use <file>` statements
// (every used file's own AST, plus its own combined root Scope, flattened
// into pools rather than kept nested per recursion level) -- the caller
// must keep this struct alive for as long as evaluation runs against
// .rootScope/.processedNodes, since declarations injected from a used file
// are raw (non-owning) pointers into it.
// One used file's own top-level assignments, in source order, keyed by
// that file's root Scope (the one its injected declarations were
// re-anchored onto). Evaluator::setUsedFileGlobals takes the list so a
// used file's globals can be evaluated ONCE per run, eagerly, the first
// time anything in that file reads one -- see Evaluator::fileGlobal.
// Raw pointers into usedFileAsts, same lifetime contract as processedNodes.
struct UsedFileGlobals {
    const oscad::Scope* root = nullptr;
    std::vector<const oscad::ASTNode*> assignments;
};

struct ResolvedUseScopes {
    std::vector<std::vector<std::unique_ptr<oscad::ASTNode>>> usedFileAsts;
    std::vector<std::unique_ptr<oscad::Scope>> usedFileScopes;
    std::vector<UsedFileGlobals> usedFileGlobals;
    // What currentFile should actually be evaluated as: declarations
    // injected via `use` (raw pointers into usedFileAsts, above), followed
    // by currentFile's own nodes (raw pointers into the caller-owned
    // `ownNodes` passed into resolveUseScopes -- not owned here; the
    // caller must keep that vector alive too).
    std::vector<const oscad::ASTNode*> processedNodes;
    // currentFile's own nodes minus UseStatements -- exposed so a
    // further-up caller that itself `use`s currentFile can inject exactly
    // these (not anything currentFile itself pulled in via `use`; "nested
    // use has no effect on the base file's environment").
    std::vector<const oscad::ASTNode*> ownNodesFiltered;
    // Owns the ScopeTable holding every node's lexical scope -- see
    // oscad::ScopeTable for why that cannot live in the nodes themselves.
    std::unique_ptr<oscad::Scope> rootScope;
};

// Resolves `use <file>` statements per OpenSCAD semantics: each top-level
// UseStatement is replaced by the used file's own module and function
// declarations -- its top-level geometry and variable assignments are not
// injected, so currentFile's own variable namespace stays isolated from
// (and invisible to) the used file's globals. Declarations that the used
// file itself pulled in via a nested `use` are not re-exported. Injected
// declarations are re-anchored to their own file's root scope after the
// combined scope is built, so their bodies resolve names against their own
// file's globals, not currentFile's -- mirrors the reference's
// resolve_use_scopes exactly, including the silent skip for a missing
// `use` target (a file that can't be found is not reported at all,
// matching real OpenSCAD's own tolerant behavior here) vs. logging any
// other failure (e.g. a syntax error in the used file) via `logFn`.
ResolvedUseScopes resolveUseScopes(const std::vector<std::unique_ptr<oscad::ASTNode>>& ownNodes,
                                    const std::string& currentFile, const std::function<void(const std::string&)>& logFn);

// Same, for a statement list that is BORROWED rather than owned -- what
// oscad::ParsedProgram hands back when includes come from the shared AST
// cache. The caller must keep that ParsedProgram alive alongside the
// result, exactly as it must keep an owned AST alive.
ResolvedUseScopes resolveUseScopes(const std::vector<const oscad::ASTNode*>& ownNodes, const std::string& currentFile,
                                    const std::function<void(const std::string&)>& logFn);

} // namespace oscadeval
