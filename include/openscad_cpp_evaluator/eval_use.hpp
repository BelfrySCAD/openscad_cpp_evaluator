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
struct ResolvedUseScopes {
    std::vector<std::vector<std::unique_ptr<oscad::ASTNode>>> usedFileAsts;
    std::vector<std::unique_ptr<oscad::Scope>> usedFileScopes;
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

} // namespace oscadeval
