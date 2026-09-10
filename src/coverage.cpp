#include "openscad_cpp_evaluator/coverage.hpp"

#include "openscad_cpp_parser/ast/declarations.hpp"
#include "openscad_cpp_parser/ast/expression.hpp"
#include "openscad_cpp_parser/ast/module_instantiation.hpp"
#include "openscad_cpp_parser/ast/vector_element.hpp"

#include <algorithm>
#include <map>
#include <unordered_set>

namespace oscadeval {

const char* coverageKindName(CoverageKind kind) {
    switch (kind) {
        case CoverageKind::Statement: return "statement";
        case CoverageKind::Branch: return "branch";
        case CoverageKind::Body: return "body";
    }
    return "?";
}

namespace {

using oscad::ASTNode;
using oscad::NodeKind;

// One walk, one vocabulary. Which node a checkpoint lands on is decided by
// the evaluator (an if-arm is checkpointed on its first statement, a
// ternary arm on the arm expression, a body on its declaration), and this
// walk emits exactly those nodes, so the recorder's hits and this universe
// line up without either side knowing about the other's tables.
class Walker {
public:
    explicit Walker(std::vector<Coverable>& out) : out_(out) {}

    void statements(const std::vector<std::unique_ptr<ASTNode>>& list, bool firstIsArm = false) {
        bool first = true;
        for (const auto& s : list) {
            statement(*s, firstIsArm && first, true);
            first = false;
        }
    }

    void statement(const ASTNode& n, bool arm, bool emitSelf) {
        switch (n.kind()) {
            case NodeKind::Assignment: {
                auto& a = static_cast<const oscad::Assignment&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                expr(*a.expr);
                break;
            }
            case NodeKind::ModularCall: {
                auto& c = static_cast<const oscad::ModularCall&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                args(c.arguments);
                statements(c.children);
                break;
            }
            case NodeKind::ModularFor: {
                auto& f = static_cast<const oscad::ModularFor&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                for (const auto& a : f.assignments) expr(*a->expr);
                statements(f.body);
                break;
            }
            case NodeKind::ModularIntersectionFor: {
                auto& f = static_cast<const oscad::ModularIntersectionFor&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                for (const auto& a : f.assignments) expr(*a->expr);
                statements(f.body);
                break;
            }
            case NodeKind::ModularLet: {
                auto& l = static_cast<const oscad::ModularLet&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                for (const auto& a : l.assignments) expr(*a->expr);
                statements(l.children);
                break;
            }
            case NodeKind::ModularEcho: {
                auto& e = static_cast<const oscad::ModularEcho&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                args(e.arguments);
                statements(e.children);
                break;
            }
            case NodeKind::ModularAssert: {
                auto& e = static_cast<const oscad::ModularAssert&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                args(e.arguments);
                statements(e.children);
                break;
            }
            case NodeKind::ModularIf: {
                auto& i = static_cast<const oscad::ModularIf&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                expr(*i.condition);
                statements(i.trueBranch, /*firstIsArm=*/true);
                break;
            }
            case NodeKind::ModularIfElse: {
                auto& i = static_cast<const oscad::ModularIfElse&>(n);
                if (emitSelf) emit(&n, CoverageKind::Statement, arm);
                expr(*i.condition);
                statements(i.trueBranch, /*firstIsArm=*/true);
                statements(i.falseBranch, /*firstIsArm=*/true);
                break;
            }
            // A modifier is the statement the evaluator checkpoints (it is
            // the node in the statement list); the wrapped call is walked
            // for what it contains but is not a statement of its own.
            case NodeKind::ModularModifierShowOnly:
                modifier(static_cast<const oscad::ModularModifierShowOnly&>(n).child.get(), n, arm, emitSelf);
                break;
            case NodeKind::ModularModifierHighlight:
                modifier(static_cast<const oscad::ModularModifierHighlight&>(n).child.get(), n, arm, emitSelf);
                break;
            case NodeKind::ModularModifierBackground:
                modifier(static_cast<const oscad::ModularModifierBackground&>(n).child.get(), n, arm, emitSelf);
                break;
            case NodeKind::ModularModifierDisable:
                modifier(static_cast<const oscad::ModularModifierDisable&>(n).child.get(), n, arm, emitSelf);
                break;
            case NodeKind::ModuleDeclaration: {
                auto& d = static_cast<const oscad::ModuleDeclaration&>(n);
                emit(&n, CoverageKind::Body);
                params(d.parameters);
                statements(d.children);
                break;
            }
            case NodeKind::FunctionDeclaration: {
                auto& d = static_cast<const oscad::FunctionDeclaration&>(n);
                emit(&n, CoverageKind::Body);
                params(d.parameters);
                expr(*d.expr);
                break;
            }
            default:
                break; // use/include, comments, blank lines
        }
    }

private:
    void modifier(const ASTNode* child, const ASTNode& self, bool arm, bool emitSelf) {
        if (emitSelf) emit(&self, CoverageKind::Statement, arm);
        if (child) statement(*child, false, /*emitSelf=*/false);
    }

    void emit(const ASTNode* n, CoverageKind kind, bool arm = false) {
        if (n && seen_.insert(n).second) out_.push_back(Coverable{n, kind, arm});
    }

    void args(const std::vector<std::unique_ptr<oscad::Argument>>& list) {
        for (const auto& a : list) {
            if (a->kind() == NodeKind::NamedArgument) {
                expr(*static_cast<const oscad::NamedArgument&>(*a).expr);
            } else if (a->kind() == NodeKind::PositionalArgument) {
                expr(*static_cast<const oscad::PositionalArgument&>(*a).expr);
            }
        }
    }

    void params(const std::vector<std::unique_ptr<oscad::ParameterDeclaration>>& list) {
        for (const auto& p : list) {
            if (p->defaultValue) expr(*p->defaultValue);
        }
    }

    template <typename Binary>
    void binary(const ASTNode& n) {
        auto& b = static_cast<const Binary&>(n);
        expr(*b.left);
        expr(*b.right);
    }
    template <typename Unary>
    void unary(const ASTNode& n) {
        expr(*static_cast<const Unary&>(n).expr);
    }

    void expr(const ASTNode& e) {
        switch (e.kind()) {
            case NodeKind::RangeLiteral: {
                auto& r = static_cast<const oscad::RangeLiteral&>(e);
                expr(*r.start);
                expr(*r.end);
                if (r.step) expr(*r.step);
                break;
            }
            case NodeKind::UnaryMinusOp: unary<oscad::UnaryMinusOp>(e); break;
            case NodeKind::BitwiseNotOp: unary<oscad::BitwiseNotOp>(e); break;
            case NodeKind::LogicalNotOp: unary<oscad::LogicalNotOp>(e); break;
            case NodeKind::AdditionOp: binary<oscad::AdditionOp>(e); break;
            case NodeKind::SubtractionOp: binary<oscad::SubtractionOp>(e); break;
            case NodeKind::MultiplicationOp: binary<oscad::MultiplicationOp>(e); break;
            case NodeKind::DivisionOp: binary<oscad::DivisionOp>(e); break;
            case NodeKind::ModuloOp: binary<oscad::ModuloOp>(e); break;
            case NodeKind::ExponentOp: binary<oscad::ExponentOp>(e); break;
            case NodeKind::BitwiseAndOp: binary<oscad::BitwiseAndOp>(e); break;
            case NodeKind::BitwiseOrOp: binary<oscad::BitwiseOrOp>(e); break;
            case NodeKind::BitwiseShiftLeftOp: binary<oscad::BitwiseShiftLeftOp>(e); break;
            case NodeKind::BitwiseShiftRightOp: binary<oscad::BitwiseShiftRightOp>(e); break;
            case NodeKind::EqualityOp: binary<oscad::EqualityOp>(e); break;
            case NodeKind::InequalityOp: binary<oscad::InequalityOp>(e); break;
            case NodeKind::GreaterThanOp: binary<oscad::GreaterThanOp>(e); break;
            case NodeKind::GreaterThanOrEqualOp: binary<oscad::GreaterThanOrEqualOp>(e); break;
            case NodeKind::LessThanOp: binary<oscad::LessThanOp>(e); break;
            case NodeKind::LessThanOrEqualOp: binary<oscad::LessThanOrEqualOp>(e); break;
            case NodeKind::LogicalAndOp: {
                auto& b = static_cast<const oscad::LogicalAndOp&>(e);
                expr(*b.left);
                emit(b.right.get(), CoverageKind::Branch);
                expr(*b.right);
                break;
            }
            case NodeKind::LogicalOrOp: {
                auto& b = static_cast<const oscad::LogicalOrOp&>(e);
                expr(*b.left);
                emit(b.right.get(), CoverageKind::Branch);
                expr(*b.right);
                break;
            }
            case NodeKind::TernaryOp: {
                auto& t = static_cast<const oscad::TernaryOp&>(e);
                expr(*t.condition);
                emit(t.trueExpr.get(), CoverageKind::Branch);
                expr(*t.trueExpr);
                emit(t.falseExpr.get(), CoverageKind::Branch);
                expr(*t.falseExpr);
                break;
            }
            case NodeKind::PrimaryCall: {
                auto& c = static_cast<const oscad::PrimaryCall&>(e);
                expr(*c.left);
                args(c.arguments);
                break;
            }
            case NodeKind::PrimaryIndex: {
                auto& i = static_cast<const oscad::PrimaryIndex&>(e);
                expr(*i.left);
                expr(*i.index);
                break;
            }
            case NodeKind::PrimaryMember:
                expr(*static_cast<const oscad::PrimaryMember&>(e).left);
                break;
            case NodeKind::LetOp: {
                auto& l = static_cast<const oscad::LetOp&>(e);
                for (const auto& a : l.assignments) expr(*a->expr);
                expr(*l.body);
                break;
            }
            case NodeKind::EchoOp: {
                auto& o = static_cast<const oscad::EchoOp&>(e);
                args(o.arguments);
                expr(*o.body);
                break;
            }
            case NodeKind::AssertOp: {
                auto& o = static_cast<const oscad::AssertOp&>(e);
                args(o.arguments);
                expr(*o.body);
                break;
            }
            case NodeKind::FunctionLiteral: {
                auto& f = static_cast<const oscad::FunctionLiteral&>(e);
                emit(&e, CoverageKind::Body);
                params(f.parameters);
                expr(*f.body);
                break;
            }
            case NodeKind::RenderExpression: {
                auto& r = static_cast<const oscad::RenderExpression&>(e);
                args(r.arguments);
                statements(r.children);
                break;
            }
            case NodeKind::CommentedExpr:
                expr(*static_cast<const oscad::CommentedExpr&>(e).expr);
                break;
            case NodeKind::ListComprehension: {
                auto& l = static_cast<const oscad::ListComprehension&>(e);
                for (const auto& el : l.elements) element(*el);
                break;
            }
            default:
                break; // literals, identifiers
        }
    }

    void element(const ASTNode& el) {
        switch (el.kind()) {
            case NodeKind::ListCompLet: {
                auto& n = static_cast<const oscad::ListCompLet&>(el);
                for (const auto& a : n.assignments) expr(*a->expr);
                element(*n.body);
                break;
            }
            case NodeKind::ListCompEach:
                element(*static_cast<const oscad::ListCompEach&>(el).body);
                break;
            case NodeKind::ListCompFor: {
                auto& n = static_cast<const oscad::ListCompFor&>(el);
                for (const auto& a : n.assignments) expr(*a->expr);
                element(*n.body);
                break;
            }
            case NodeKind::ListCompCFor: {
                auto& n = static_cast<const oscad::ListCompCFor&>(el);
                for (const auto& a : n.inits) expr(*a->expr);
                expr(*n.condition);
                for (const auto& a : n.incrs) expr(*a->expr);
                element(*n.body);
                break;
            }
            case NodeKind::ListCompIf: {
                auto& n = static_cast<const oscad::ListCompIf&>(el);
                expr(*n.condition);
                emit(n.trueExpr.get(), CoverageKind::Branch);
                element(*n.trueExpr);
                break;
            }
            case NodeKind::ListCompIfElse: {
                auto& n = static_cast<const oscad::ListCompIfElse&>(el);
                expr(*n.condition);
                emit(n.trueExpr.get(), CoverageKind::Branch);
                element(*n.trueExpr);
                emit(n.falseExpr.get(), CoverageKind::Branch);
                element(*n.falseExpr);
                break;
            }
            default:
                expr(el); // a plain expression element
                break;
        }
    }

    std::vector<Coverable>& out_;
    std::unordered_set<const ASTNode*> seen_;
};

} // namespace

std::vector<Coverable> collectCoverable(const std::vector<const ASTNode*>& roots,
                                        const std::vector<const ASTNode*>& extraStatements) {
    std::vector<Coverable> out;
    Walker w(out);
    for (const ASTNode* n : roots) w.statement(*n, false, true);
    for (const ASTNode* n : extraStatements) w.statement(*n, false, true);
    return out;
}

CoverageResult buildCoverageResult(const std::vector<const ASTNode*>& roots,
                                   const std::vector<const ASTNode*>& extraStatements,
                                   const CoverageRecorder& recorder) {
    CoverageResult result;
    for (const Coverable& c : collectCoverable(roots, extraStatements)) {
        const oscad::Position& p = c.node->position();
        result.spans.push_back(CoverageSpan{p.origin, p.line, p.column, p.start_offset, p.end_offset, c.kind, c.arm,
                                            recorder.hitsFor(*c.node)});
    }
    summarizeCoverage(result);
    return result;
}

void summarizeCoverage(CoverageResult& result) {
    std::map<std::string, CoverageFileSummary> files;
    CoverageFileSummary total;
    const auto add = [](CoverageFileSummary& f, const CoverageSpan& s) {
        const bool hit = s.hits > 0;
        ++f.spans;
        f.spans_hit += hit;
        if (s.kind == CoverageKind::Statement) {
            ++f.statements;
            f.statements_hit += hit;
        }
        if (s.kind == CoverageKind::Branch || s.arm) {
            ++f.branches;
            f.branches_hit += hit;
        }
        if (s.kind == CoverageKind::Body) {
            ++f.bodies;
            f.bodies_hit += hit;
        }
    };
    for (const CoverageSpan& s : result.spans) {
        CoverageFileSummary& f = files[s.origin];
        f.origin = s.origin;
        add(f, s);
        add(total, s);
    }
    result.files.clear();
    for (auto& [origin, f] : files) result.files.push_back(std::move(f));
    result.total = total;
}

} // namespace oscadeval
