#include "openscad_cpp_evaluator/call_args.hpp"

#include "openscad_cpp_evaluator/evaluator.hpp"

#include <algorithm>

namespace oscadeval {

CallArgs resolveArgs(Evaluator& ev, const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& ctx) {
    CallArgs result;
    int pos = 0;
    for (const auto& argPtr : arguments) {
        if (argPtr->kind() == oscad::NodeKind::PositionalArgument) {
            auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
            result.positional[pos++] = ev.evalExpr(*a.expr, ctx);
        } else if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
            result.named[a.name->name] = ev.evalExpr(*a.expr, ctx);
        }
    }
    return result;
}

Value getArg(const CallArgs& args, std::optional<int> pos, const std::string& name, Value defaultValue) {
    auto namedIt = args.named.find(name);
    if (namedIt != args.named.end()) return namedIt->second;
    if (pos.has_value()) {
        auto posIt = args.positional.find(*pos);
        if (posIt != args.positional.end()) return posIt->second;
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
    std::vector<Value> posVec;
    posVec.reserve(static_cast<size_t>(maxPos + 1));
    for (int i = 0; i <= maxPos; ++i) {
        auto it = args.positional.find(i);
        posVec.push_back(it != args.positional.end() ? it->second : Value{});
    }

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
        for (size_t i = 0; i < items.size(); ++i) args.positional[static_cast<int>(i)] = items[i];
    }
    if (const ObjectPtr* namedObj = std::get_if<ObjectPtr>(&(*outer)->items[1]); namedObj && *namedObj) {
        for (const auto& [key, value] : (*namedObj)->items) args.named[key] = value;
    }
    return args;
}

} // namespace oscadeval
