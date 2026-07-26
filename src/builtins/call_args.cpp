#include "openscad_cpp_evaluator/call_args.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include <algorithm>

namespace oscadeval {

void CallArgs::setPositional(int pos, Value v) {
    for (auto& [k, existing] : positional) {
        if (k == pos) {
            existing = std::move(v);
            return;
        }
    }
    positional.emplace_back(pos, std::move(v));
}

void CallArgs::setNamed(const std::string& name, Value v) {
    for (auto& [k, existing] : named) {
        if (k == name) {
            existing = std::move(v);
            return;
        }
    }
    named.emplace_back(name, std::move(v));
}

const Value* CallArgs::findPositional(int pos) const {
    for (const auto& [k, v] : positional) {
        if (k == pos) return &v;
    }
    return nullptr;
}

const Value* CallArgs::findNamed(const std::string& name) const {
    for (const auto& [k, v] : named) {
        if (k == name) return &v;
    }
    return nullptr;
}

CallArgs resolveArgs(Evaluator& ev, const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx) {
    CallArgs result;
    result.positional.reserve(arguments.size());
    int pos = 0;
    for (const auto& argPtr : arguments) {
        if (argPtr->kind() == oscad::NodeKind::PositionalArgument) {
            auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
            result.positional.emplace_back(pos++, ev.evalExpr(*a.expr, ctx));
        } else if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
            result.named.emplace_back(a.name->name, ev.evalExpr(*a.expr, ctx));
        }
    }
    return result;
}

Value getArg(const CallArgs& args, std::optional<int> pos, const std::string& name, Value defaultValue) {
    if (const Value* named = args.findNamed(name)) return *named;
    if (pos.has_value()) {
        if (const Value* positional = args.findPositional(*pos)) return *positional;
    }
    return defaultValue;
}

ResolvedCallArgs resolveCallArgs(Evaluator& ev, const std::vector<std::unique_ptr<oscad::Argument>>& arguments,
                                  EvalContext& ctx) {
    CallArgs args = resolveArgs(ev, arguments, ctx);

    bool anyDollar = false;
    for (const auto& [key, value] : args.named) {
        if (!key.empty() && key[0] == '$') {
            anyDollar = true;
            break;
        }
    }
    if (!anyDollar) return ResolvedCallArgs{std::move(args), ctx};

    EvalContext effCtx = ctx.childCtx();
    for (const auto& [key, value] : args.named) {
        if (!key.empty() && key[0] == '$') effCtx.dyn->set(key, value);
    }
    return ResolvedCallArgs{std::move(args), std::move(effCtx)};
}

Value callArgsToValue(const CallArgs& args) {
    int maxPos = -1;
    for (const auto& [idx, v] : args.positional) maxPos = std::max(maxPos, idx);
    std::vector<Value> posVec(static_cast<size_t>(maxPos + 1));
    for (const auto& [idx, v] : args.positional) posVec[static_cast<size_t>(idx)] = v;

    std::vector<std::pair<std::string, Value>> namedVec(args.named.begin(), args.named.end());

    std::vector<Value> outer = {
        Value{std::make_shared<const ValueList>(ValueList{std::move(posVec)})},
        Value{std::make_shared<const ValueObject>(ValueObject{std::move(namedVec)})},
    };
    return Value{std::make_shared<const ValueList>(ValueList{std::move(outer)})};
}

const oscad::Expression* argExpr(const oscad::Argument& arg) {
    if (arg.kind() == oscad::NodeKind::NamedArgument) {
        return static_cast<const oscad::NamedArgument&>(arg).expr.get();
    }
    return static_cast<const oscad::PositionalArgument&>(arg).expr.get();
}

std::vector<Value> allPositional(const CallArgs& args) {
    int maxPos = -1;
    for (const auto& [idx, v] : args.positional) maxPos = std::max(maxPos, idx);
    std::vector<Value> out(static_cast<size_t>(maxPos + 1));
    for (const auto& [idx, v] : args.positional) out[static_cast<size_t>(idx)] = v;
    return out;
}

CallArgs valueToCallArgs(const Value& v) {
    CallArgs args;
    const ListPtr* outer = std::get_if<ListPtr>(&v);
    if (!outer || !*outer || (*outer)->items.size() != 2) return args;

    if (const ListPtr* posList = std::get_if<ListPtr>(&(*outer)->items[0]); posList && *posList) {
        const auto& items = (*posList)->items;
        args.positional.reserve(items.size());
        for (size_t i = 0; i < items.size(); ++i) args.positional.emplace_back(static_cast<int>(i), items[i]);
    }
    if (const ObjectPtr* namedObj = std::get_if<ObjectPtr>(&(*outer)->items[1]); namedObj && *namedObj) {
        args.named.reserve((*namedObj)->items.size());
        for (const auto& [key, value] : (*namedObj)->items) args.named.emplace_back(key, value);
    }
    return args;
}

} // namespace oscadeval
