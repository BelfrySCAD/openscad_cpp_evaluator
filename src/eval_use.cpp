#include "openscad_cpp_evaluator/eval_use.hpp"

#include "openscad_cpp_parser/ast/declarations.hpp"

#include <utility>

namespace oscadeval {

ResolvedUseScopes resolveUseScopes(const std::vector<std::unique_ptr<oscad::ASTNode>>& ownNodes,
                                    const std::string& currentFile, const std::function<void(const std::string&)>& logFn) {
    ResolvedUseScopes result;

    std::vector<const oscad::ASTNode*> injected;
    // Declarations pulled in from one used file, paired with that file's
    // own root scope to reanchor them onto once the combined scope below
    // is built -- mirrors the reference's `reanchor` list exactly.
    std::vector<std::pair<std::vector<const oscad::ASTNode*>, oscad::Scope*>> reanchor;

    for (const auto& nodePtr : ownNodes) {
        if (nodePtr->kind() != oscad::NodeKind::UseStatement) continue;
        const auto& useNode = static_cast<const oscad::UseStatement&>(*nodePtr);

        // include<>'d files are flattened into ownNodes at parse time, so a
        // use statement may have originated from a different file than
        // currentFile -- resolve relative paths against where it was
        // actually written.
        const std::string& origin = useNode.position().origin;

        oscad::LibraryFileResult lib;
        try {
            lib = oscad::getASTFromLibraryFile(origin.empty() ? currentFile : origin, useNode.filepath->val,
                                                /*includeComments=*/false);
        } catch (const std::exception& e) {
            const std::string msg = e.what();
            if (msg.find("not found") == std::string::npos && msg.find("No such file") == std::string::npos) {
                logFn(std::string("use error: ") + msg);
            }
            continue;
        }
        if (lib.ast.empty()) continue;

        // Move the library's AST into a stable slot in usedFileAsts before
        // recursing, so the recursive call's own raw pointers (into this
        // same vector) stay valid once stored there.
        result.usedFileAsts.push_back(std::move(lib.ast));
        const std::vector<std::unique_ptr<oscad::ASTNode>>& libAst = result.usedFileAsts.back();

        ResolvedUseScopes nested = resolveUseScopes(libAst, lib.resolvedPath, logFn);

        std::vector<const oscad::ASTNode*> libInjected;
        for (const oscad::ASTNode* n : nested.ownNodesFiltered) {
            if (n->kind() == oscad::NodeKind::ModuleDeclaration || n->kind() == oscad::NodeKind::FunctionDeclaration) {
                libInjected.push_back(n);
            }
        }
        injected.insert(injected.end(), libInjected.begin(), libInjected.end());

        // Absorb the nested resolution's own owned pools so everything
        // transitively pulled in (a used file that itself uses another
        // file) stays alive as long as `result` does.
        for (auto& ast : nested.usedFileAsts) result.usedFileAsts.push_back(std::move(ast));
        for (auto& sc : nested.usedFileScopes) result.usedFileScopes.push_back(std::move(sc));
        result.usedFileScopes.push_back(std::move(nested.rootScope));
        if (!libInjected.empty()) reanchor.emplace_back(std::move(libInjected), result.usedFileScopes.back().get());
    }

    for (const auto& nodePtr : ownNodes) {
        if (nodePtr->kind() != oscad::NodeKind::UseStatement) result.ownNodesFiltered.push_back(nodePtr.get());
    }

    result.processedNodes = injected;
    result.processedNodes.insert(result.processedNodes.end(), result.ownNodesFiltered.begin(), result.ownNodesFiltered.end());

    // oscad::buildScopes()'s raw-pointer overload takes mutable ASTNode*
    // (matching Scope's own internal storage, e.g. defineVariable's
    // ASTNode* parameter -- scope-building always needs to write each
    // node's own .scope_) -- processedNodes stays const ASTNode* to match
    // evaluate()'s/evalChildren's own read-only-traversal convention
    // everywhere else in this port, so build one throwaway mutable view
    // just for this call.
    std::vector<oscad::ASTNode*> mutableProcessed;
    mutableProcessed.reserve(result.processedNodes.size());
    for (const oscad::ASTNode* n : result.processedNodes) mutableProcessed.push_back(const_cast<oscad::ASTNode*>(n));
    result.rootScope = oscad::buildScopes(mutableProcessed);

    for (const auto& [libInjected, libRootScope] : reanchor) {
        for (const oscad::ASTNode* n : libInjected) const_cast<oscad::ASTNode*>(n)->buildScope(*libRootScope);
    }

    return result;
}

} // namespace oscadeval
