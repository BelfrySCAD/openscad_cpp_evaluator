// Snapshots the parser's AST into plain Python dicts.
//
// Deliberately a COPY rather than wrappers around live ASTNode pointers.
// The tree is owned by a `std::vector<std::unique_ptr<ASTNode>>` local to
// whichever binding parsed it, and dies when that call returns; handing
// Python raw pointers into it would be a use-after-free the moment anything
// outlived the call, and this project has already shipped one dangling
// pointer of exactly that shape (the bodyCtx bug fixed in v0.13.1). It also
// sidesteps the threading problem entirely -- a render worker thread can
// parse, hand the snapshot to the GUI thread, and exit, with nothing shared.
//
// The cost is one walk plus allocation per parse. That is affordable
// because callers ask for this explicitly (parse_ast()), not on the render
// path, which never needs it.
//
// Every node dict carries at least:
//   kind      -- str, the NodeKind name ("NumberLiteral", "ModularCall", ...)
//   position  -- dict(origin, line, column, start_offset, end_offset)
// plus that kind's own fields, named in snake_case. Child nodes are nested
// dicts; child lists are lists of dicts; an absent optional child is None.
//
// `position.start_offset`/`end_offset` are the point of the whole exercise:
// they let a caller recover the ORIGINAL source text of any subexpression
// and reuse it verbatim, rather than re-serialising a parsed value and
// losing how the author wrote it (1.500 -> 1.5, 1e3 -> 1000).

#include "openscad_cpp_parser/api.hpp"
#include "openscad_cpp_parser/ast/ast_node.hpp"
#include "openscad_cpp_parser/ast/comments.hpp"
#include "openscad_cpp_parser/ast/declarations.hpp"
#include "openscad_cpp_parser/ast/expression.hpp"
#include "openscad_cpp_parser/ast/module_instantiation.hpp"
#include "openscad_cpp_parser/ast/vector_element.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <memory>
#include <vector>

namespace nb = nanobind;

namespace oscadbind {

namespace {

nb::dict positionToPy(const oscad::Position& p) {
    nb::dict d;
    // origin is "" for the main file (matching how the evaluator reports
    // it); left as-is rather than mapped to None, so a caller can compare
    // it against a path without a null check.
    d["origin"] = p.origin;
    d["line"] = p.line;
    d["column"] = p.column;
    d["start_offset"] = p.start_offset;
    d["end_offset"] = p.end_offset;
    return d;
}

nb::object nodeToPy(const oscad::ASTNode* n);

// A null unique_ptr child (e.g. a parameter with no default) becomes None
// rather than being omitted, so a consumer can index the key unconditionally.
template <typename T>
nb::object childToPy(const std::unique_ptr<T>& p) {
    return p ? nodeToPy(p.get()) : nb::none();
}

template <typename T>
nb::list listToPy(const std::vector<std::unique_ptr<T>>& v) {
    nb::list out;
    for (const std::unique_ptr<T>& item : v) out.append(nodeToPy(item.get()));
    return out;
}

nb::dict baseDict(const oscad::ASTNode* n) {
    nb::dict d;
    d["kind"] = oscad::nodeKindName(n->kind());
    d["position"] = positionToPy(n->position());
    return d;
}

nb::object nodeToPy(const oscad::ASTNode* n) {
    if (n == nullptr) return nb::none();
    nb::dict d = baseDict(n);

    // kind() is authoritative about the concrete type (every leaf class sets
    // its own), so static_cast is safe here and avoids a dynamic_cast per
    // node on what is already an allocation-heavy walk.
#define OSC_NODE(KindName, Type) case oscad::NodeKind::KindName: { const auto* x = static_cast<const oscad::Type*>(n); (void)x;
#define OSC_END break; }

    switch (n->kind()) {
        // -- comments ---------------------------------------------------
        OSC_NODE(CommentLine, CommentLine) d["text"] = x->text; OSC_END
        OSC_NODE(BlankLine, BlankLine) OSC_END
        OSC_NODE(CommentSpan, CommentSpan) d["text"] = x->text; OSC_END
        OSC_NODE(CommentedExpr, CommentedExpr)
            d["leading_comments"] = listToPy(x->leadingComments);
            d["trailing_comments"] = listToPy(x->trailingComments);
            d["expr"] = childToPy(x->expr);
        OSC_END

        // -- primaries / literals ---------------------------------------
        OSC_NODE(Identifier, Identifier) d["name"] = x->name; OSC_END
        // `val` is the DECODED string minus its quotes; escape sequences
        // are kept raw (a source \n stays two characters). Use the span to
        // recover exactly what was written.
        OSC_NODE(StringLiteral, StringLiteral) d["val"] = x->val; OSC_END
        // Parsed to double, so the source spelling (1.500, 1e3, 0x10) is
        // NOT recoverable from here -- that is what the span is for.
        OSC_NODE(NumberLiteral, NumberLiteral) d["val"] = x->val; OSC_END
        OSC_NODE(BooleanLiteral, BooleanLiteral) d["val"] = x->val; OSC_END
        OSC_NODE(UndefinedLiteral, UndefinedLiteral) OSC_END
        OSC_NODE(RangeLiteral, RangeLiteral)
            d["start"] = childToPy(x->start);
            d["end"] = childToPy(x->end);
            // Never null: the parser defaults an omitted step to 1.
            d["step"] = childToPy(x->step);
        OSC_END

        // -- declarations / arguments -----------------------------------
        OSC_NODE(ParameterDeclaration, ParameterDeclaration)
            d["name"] = childToPy(x->name);
            d["default_value"] = childToPy(x->defaultValue); // nullable
            d["leading_comments"] = listToPy(x->leadingComments);
            d["trailing_comments"] = listToPy(x->trailingComments);
        OSC_END
        OSC_NODE(PositionalArgument, PositionalArgument) d["expr"] = childToPy(x->expr); OSC_END
        OSC_NODE(NamedArgument, NamedArgument)
            d["name"] = childToPy(x->name);
            d["expr"] = childToPy(x->expr);
        OSC_END
        OSC_NODE(Assignment, Assignment)
            d["name"] = childToPy(x->name);
            d["expr"] = childToPy(x->expr);
        OSC_END

        // -- prefix expression forms ------------------------------------
        OSC_NODE(LetOp, LetOp)
            d["assignments"] = listToPy(x->assignments);
            d["body"] = childToPy(x->body);
        OSC_END
        OSC_NODE(EchoOp, EchoOp)
            d["arguments"] = listToPy(x->arguments);
            d["body"] = childToPy(x->body);
        OSC_END
        OSC_NODE(AssertOp, AssertOp)
            d["arguments"] = listToPy(x->arguments);
            d["body"] = childToPy(x->body);
        OSC_END
        OSC_NODE(FunctionLiteral, FunctionLiteral)
            d["parameters"] = listToPy(x->parameters);
            d["body"] = childToPy(x->body);
        OSC_END

        // -- unary operators --------------------------------------------
#define OSC_UNARY(KindName) OSC_NODE(KindName, KindName) d["expr"] = childToPy(x->expr); OSC_END
        OSC_UNARY(UnaryMinusOp)
        OSC_UNARY(LogicalNotOp)
        OSC_UNARY(BitwiseNotOp)
#undef OSC_UNARY

        // -- binary operators -------------------------------------------
#define OSC_BINARY(KindName)                                                                                          \
    OSC_NODE(KindName, KindName) d["left"] = childToPy(x->left); d["right"] = childToPy(x->right); OSC_END
        OSC_BINARY(AdditionOp)
        OSC_BINARY(SubtractionOp)
        OSC_BINARY(MultiplicationOp)
        OSC_BINARY(DivisionOp)
        OSC_BINARY(ModuloOp)
        OSC_BINARY(ExponentOp)
        OSC_BINARY(BitwiseAndOp)
        OSC_BINARY(BitwiseOrOp)
        OSC_BINARY(BitwiseShiftLeftOp)
        OSC_BINARY(BitwiseShiftRightOp)
        OSC_BINARY(LogicalAndOp)
        OSC_BINARY(LogicalOrOp)
        OSC_BINARY(EqualityOp)
        OSC_BINARY(InequalityOp)
        OSC_BINARY(GreaterThanOp)
        OSC_BINARY(GreaterThanOrEqualOp)
        OSC_BINARY(LessThanOp)
        OSC_BINARY(LessThanOrEqualOp)
#undef OSC_BINARY

        OSC_NODE(TernaryOp, TernaryOp)
            d["condition"] = childToPy(x->condition);
            d["true_expr"] = childToPy(x->trueExpr);
            d["false_expr"] = childToPy(x->falseExpr);
        OSC_END

        // -- postfix ----------------------------------------------------
        OSC_NODE(PrimaryCall, PrimaryCall)
            d["left"] = childToPy(x->left);
            d["arguments"] = listToPy(x->arguments);
        OSC_END
        OSC_NODE(PrimaryIndex, PrimaryIndex)
            d["left"] = childToPy(x->left);
            d["index"] = childToPy(x->index);
        OSC_END
        OSC_NODE(PrimaryMember, PrimaryMember)
            d["left"] = childToPy(x->left);
            d["member"] = childToPy(x->member);
        OSC_END

        // -- list comprehension clauses ---------------------------------
        OSC_NODE(ListCompLet, ListCompLet)
            d["assignments"] = listToPy(x->assignments);
            d["body"] = childToPy(x->body);
        OSC_END
        OSC_NODE(ListCompEach, ListCompEach) d["body"] = childToPy(x->body); OSC_END
        OSC_NODE(ListCompFor, ListCompFor)
            d["assignments"] = listToPy(x->assignments);
            d["body"] = childToPy(x->body);
        OSC_END
        OSC_NODE(ListCompCFor, ListCompCFor)
            d["inits"] = listToPy(x->inits);
            d["condition"] = childToPy(x->condition);
            d["incrs"] = listToPy(x->incrs);
            d["body"] = childToPy(x->body);
        OSC_END
        OSC_NODE(ListCompIf, ListCompIf)
            d["condition"] = childToPy(x->condition);
            d["true_expr"] = childToPy(x->trueExpr);
        OSC_END
        OSC_NODE(ListCompIfElse, ListCompIfElse)
            d["condition"] = childToPy(x->condition);
            d["true_expr"] = childToPy(x->trueExpr);
            d["false_expr"] = childToPy(x->falseExpr);
        OSC_END
        // Also how a plain vector literal ([1,2,3]) is represented --
        // elements mix comprehension clauses and bare expressions.
        OSC_NODE(ListComprehension, ListComprehension) d["elements"] = listToPy(x->elements); OSC_END

        // -- module instantiation ---------------------------------------
        OSC_NODE(ModularCall, ModularCall)
            d["name"] = childToPy(x->name);
            d["arguments"] = listToPy(x->arguments);
            d["children"] = listToPy(x->children);
        OSC_END
        OSC_NODE(ModularFor, ModularFor)
            d["assignments"] = listToPy(x->assignments);
            d["body"] = listToPy(x->body);
        OSC_END
        OSC_NODE(ModularIntersectionFor, ModularIntersectionFor)
            d["assignments"] = listToPy(x->assignments);
            d["body"] = listToPy(x->body);
        OSC_END
        OSC_NODE(ModularLet, ModularLet)
            d["assignments"] = listToPy(x->assignments);
            d["children"] = listToPy(x->children);
        OSC_END
        OSC_NODE(ModularEcho, ModularEcho)
            d["arguments"] = listToPy(x->arguments);
            d["children"] = listToPy(x->children);
        OSC_END
        OSC_NODE(ModularAssert, ModularAssert)
            d["arguments"] = listToPy(x->arguments);
            d["children"] = listToPy(x->children);
        OSC_END
        OSC_NODE(ModularIf, ModularIf)
            d["condition"] = childToPy(x->condition);
            d["true_branch"] = listToPy(x->trueBranch);
        OSC_END
        OSC_NODE(ModularIfElse, ModularIfElse)
            d["condition"] = childToPy(x->condition);
            d["true_branch"] = listToPy(x->trueBranch);
            d["false_branch"] = listToPy(x->falseBranch);
        OSC_END

        // -- modifier wrappers (! # % *) --------------------------------
#define OSC_MODIFIER(KindName) OSC_NODE(KindName, KindName) d["child"] = childToPy(x->child); OSC_END
        OSC_MODIFIER(ModularModifierShowOnly)
        OSC_MODIFIER(ModularModifierHighlight)
        OSC_MODIFIER(ModularModifierBackground)
        OSC_MODIFIER(ModularModifierDisable)
#undef OSC_MODIFIER

        // -- top-level declarations -------------------------------------
        OSC_NODE(ModuleDeclaration, ModuleDeclaration)
            d["name"] = childToPy(x->name);
            d["parameters"] = listToPy(x->parameters);
            d["children"] = listToPy(x->children);
            d["pre_name_comments"] = listToPy(x->preNameComments);
            d["post_name_comments"] = listToPy(x->postNameComments);
            d["post_params_comments"] = listToPy(x->postParamsComments);
        OSC_END
        OSC_NODE(FunctionDeclaration, FunctionDeclaration)
            d["name"] = childToPy(x->name);
            d["parameters"] = listToPy(x->parameters);
            d["expr"] = childToPy(x->expr);
            d["pre_name_comments"] = listToPy(x->preNameComments);
            d["post_name_comments"] = listToPy(x->postNameComments);
            d["post_params_comments"] = listToPy(x->postParamsComments);
        OSC_END
        OSC_NODE(UseStatement, UseStatement) d["filepath"] = childToPy(x->filepath); OSC_END
        OSC_NODE(IncludeStatement, IncludeStatement) d["filepath"] = childToPy(x->filepath); OSC_END
    }

#undef OSC_NODE
#undef OSC_END

    // No default: above -- the switch is exhaustive over NodeKind, so
    // adding a node kind to the parser without handling it here is a
    // compiler warning rather than a silently field-less dict.
    return d;
}

} // namespace

nb::list astToPy(const std::vector<std::unique_ptr<oscad::ASTNode>>& ast) {
    nb::list out;
    for (const std::unique_ptr<oscad::ASTNode>& n : ast) out.append(nodeToPy(n.get()));
    return out;
}

nb::list parseAstFromFile(const std::string& path, bool includeComments) {
    return astToPy(oscad::getASTFromFile(path, includeComments));
}

nb::list parseAstFromString(const std::string& code, bool includeComments) {
    return astToPy(oscad::getASTFromString(code, includeComments));
}

} // namespace oscadbind
