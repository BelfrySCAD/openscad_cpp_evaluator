#include "openscad_cpp_evaluator/bytecode_vm.hpp"

#include "openscad_cpp_evaluator/call_args.hpp"
#include "openscad_cpp_evaluator/evaluator.hpp"
#include "openscad_cpp_evaluator/function_builtins.hpp"
#include "openscad_cpp_evaluator/import_builtin.hpp"
#include "openscad_cpp_evaluator/scope_trail.hpp"

#include "openscad_cpp_parser/ast/declarations.hpp"

namespace oscadeval {

namespace {

// RAII pool checkout -- see VmFrame's own doc comment (bytecode_vm.hpp) for
// why pooling exists. Always returns its frame to the pool on destruction,
// including via exception unwind (a nested compiled call throwing through
// CallFn's own evalUserFunctionFromBound/evalBuiltinFunction call).
class VmFrameGuard {
public:
    explicit VmFrameGuard(Evaluator& ev) : ev_(ev), frame_(ev.acquireVmFrame()) {}
    ~VmFrameGuard() { ev_.releaseVmFrame(std::move(frame_)); }
    VmFrameGuard(const VmFrameGuard&) = delete;
    VmFrameGuard& operator=(const VmFrameGuard&) = delete;

    VmFrame& get() { return *frame_; }

private:
    Evaluator& ev_;
    std::unique_ptr<VmFrame> frame_;
};

// `stack` is a caller-owned, pooled scratch buffer (see VmFrameGuard) --
// cleared here at entry rather than declared fresh, since one compiled
// call's own VmFrame is reused across each of its unbound-default
// evaluations AND its body (sequential, never concurrent, so sharing one
// buffer across those sub-runs is safe and avoids allocating a new stack
// vector for each).
// One ListCompFor assignment's own materialized iteration state -- see
// IterMaterialize/IterReset/IterNext's own doc comments (bytecode.hpp).
struct IterList {
    IterableValues values;
    size_t index = 0;
    // Cached once at IterMaterialize time -- IterableValues::size() is O(1)
    // for a list/owned vector, but walks a RANGE's whole sequence from
    // scratch every call (its own doc comment: "no caller in this codebase
    // actually calls size() on a range today" -- Op::IterNext, below, used
    // to be exactly that stale caller). Recomputing it once per IterNext
    // call turned a single range-based for()/list-comp `for` into an O(n^2)
    // walk (a 100,000-element range: interpreter ~0.03s, VM ~9.5s) -- caught
    // via a BOSL2 corpus sweep (test_math.scadtest's own gaussian_rands(),
    // which builds a 100,000-element list via `count()`).
    size_t total = 0;
};

// Shared by Op::CallFn's isBuiltin (evalBuiltinFunction) and isImport
// (importAsValue) branches -- both take the same pre-resolved CallArgs
// shape resolveArgs would build for the interpreter path. `args` is
// consumed (each element moved out).
CallArgs buildCallArgs(const CompiledChunk::CallSite& site, std::vector<Value>& args, size_t argCount) {
    CallArgs callArgs;
    int positionalIdx = 0;
    for (size_t i = 0; i < argCount; ++i) {
        if (site.argNames[i]) {
            // Each call site's argNames are distinct source arguments, never
            // a repeated key, so a direct emplace_back (no overwrite check)
            // is safe and avoids a pointless linear scan on every insert.
            callArgs.named.emplace_back(*site.argNames[i], std::move(args[i]));
        } else {
            callArgs.positional.emplace_back(positionalIdx++, std::move(args[i]));
        }
    }
    return callArgs;
}

Value runChunk(Evaluator& ev, const CompiledChunk& chunk, const std::vector<Instruction>& code,
                std::vector<Value>& slots, EvalContext& ctx, std::vector<Value>& stack,
                TailCallRequest* tailOut = nullptr) {
    stack.clear();
    // Native scratch state for list-comprehension clauses (Phase 3) -- not
    // pooled (unlike stack/slots/bound): only chunks containing a real
    // comprehension clause ever touch these, and both start genuinely
    // empty per runChunk call regardless, so there's no steady-state
    // capacity to preserve across calls the way stack/slots benefit from.
    std::vector<std::vector<Value>> accumStack;
    std::vector<IterList> iterLists(static_cast<size_t>(chunk.numIterLists));
    size_t pc = 0;
    while (pc < code.size()) {
        const Instruction& ins = code[pc];
        switch (ins.op) {
            case Op::PushConst:
                stack.push_back(chunk.constants[static_cast<size_t>(ins.a)]);
                ++pc;
                break;
            case Op::PushBool:
                stack.push_back(Value{ins.a != 0});
                ++pc;
                break;
            case Op::LoadLocal:
                stack.push_back(slots[static_cast<size_t>(ins.a)]);
                ++pc;
                break;
            case Op::StoreLocal: {
                Value v = std::move(stack.back());
                stack.pop_back();
                slots[static_cast<size_t>(ins.a)] = std::move(v);
                ++pc;
                break;
            }
            case Op::LoadDyn: {
                const Value* v = ctx.dyn->find(chunk.names[static_cast<size_t>(ins.a)]);
                stack.push_back(v ? *v : Value{});
                ++pc;
                break;
            }
            case Op::StoreDyn: {
                Value v = std::move(stack.back());
                stack.pop_back();
                const std::string& name = chunk.names[static_cast<size_t>(ins.a)];
                ctx.dyn->set(name, v);
                ctx.dynExplicit->set(name, true);
                ++pc;
                break;
            }
            case Op::LoadFree:
                stack.push_back(ev.evalIdentifier(chunk.names[static_cast<size_t>(ins.a)], ins.pos, ctx, true));
                ++pc;
                break;
            case Op::LoadUpvalue: {
                // findUpvalue walks the LIVE call stack -- correct as long
                // as the declaring call is still active, however many
                // upvalue levels out `uv.targetDecl` sits (an ordinary,
                // not-yet-escaped nested closure, e.g. reduce()'s own
                // self-recursive accumulator). If that frame is gone
                // (this chunk is running as an ESCAPED closure's own body
                // -- see the FunctionLiteral case's own doc comment,
                // bytecode_compiler.cpp), fall back to `ctx.let_`: for any
                // closure invocation with a non-null capturedLet,
                // callCtxFor roots `ctx.let_` at that exact snapshot, which
                // is guaranteed (by construction -- effectiveCaptures is a
                // superset of this chunk's own upvalues) to contain this
                // name.
                const CompiledChunk::UpvalueRef& uv = chunk.upvalues[static_cast<size_t>(ins.a)];
                const Value* v = ev.findUpvalue(uv.targetDecl, uv.slot);
                if (!v) v = ctx.let_->find(uv.name);
                stack.push_back(v ? *v : Value{});
                ++pc;
                break;
            }
            case Op::MakeClosure: {
                // Builds a FRESH captured environment every time this
                // instruction actually runs (never a compile-time constant
                // -- a loop creating one closure per iteration must capture
                // THAT iteration's own values, not share one snapshot).
                // Each capture is genuinely local to the CURRENTLY-RUNNING
                // chunk only when its own `targetDecl` matches this chunk's
                // `selfDecl` (this literal's direct free-variable
                // references) -- read straight out of `slots` in that case.
                // Anything else is a capture bubbled up from a literal
                // nested even deeper (see ClosureSite's own doc comment,
                // bytecode.hpp): resolved the same way Op::LoadUpvalue
                // resolves any upvalue reference, above -- the live call
                // stack first (still live if THIS closure hasn't itself
                // escaped yet), `ctx.let_` otherwise (this chunk's own
                // capturedLet, when it's running as an escaped closure's
                // body). The resulting Closure::capturedLet is a real,
                // standalone TrailView (isolated root -- nothing else
                // shares it, and it owes nothing to `ctx.let_` itself,
                // which a compiled call never writes to directly -- see
                // bindCompiledArgs' own doc comment), so invoking this
                // closure later works through the EXACT same
                // callCtxFor/capturedLetTrail machinery every interpreter-
                // created escaping closure already uses, unchanged.
                const CompiledChunk::ClosureSite& site = chunk.closureSites[static_cast<size_t>(ins.a)];
                auto capturedTrail = TrailView<Value>::makeRoot();
                // A letrec-style self-reference (UpvalueRef::isSelfReference,
                // see its own doc comment) is skipped here, not read -- the
                // closure it names is THIS instruction's own result, which
                // doesn't exist until the shared_ptr below is constructed.
                // Patched in immediately afterward via the exact same
                // capturedTrail (a live, shared TrailView -- mutating it
                // after `closure` wraps it is exactly as visible to that
                // closure's own later invocations as any other entry).
                std::vector<const std::string*> selfNames;
                for (const auto& cap : site.captures) {
                    if (cap.isSelfReference) {
                        selfNames.push_back(&cap.name);
                        continue;
                    }
                    if (cap.targetDecl == chunk.selfDecl) {
                        capturedTrail->set(cap.name, slots[static_cast<size_t>(cap.slot)]);
                        continue;
                    }
                    const Value* v = ev.findUpvalue(cap.targetDecl, cap.slot);
                    if (!v) v = ctx.let_->find(cap.name);
                    capturedTrail->set(cap.name, v ? *v : Value{});
                }
                auto closure = std::make_shared<const Closure>(Closure{site.node, capturedTrail});
                for (const std::string* selfName : selfNames) {
                    capturedTrail->set(*selfName, Value{closure});
                }
                stack.push_back(Value{std::move(closure)});
                ++pc;
                break;
            }
            case Op::PatchClosureCapture: {
                // Mutual-recursion support (see the LetOp compile case's
                // own doc comment, bytecode_compiler.cpp): `slots[ins.a]`
                // (the consumer, e.g. isEven) already holds a real
                // MakeClosure-created closure whose own capturedTrail is
                // missing this ONE entry -- it was left out entirely at
                // compile time because the sibling it names (e.g. isOdd)
                // hadn't been constructed yet. `slots[ins.c]` (that
                // sibling's own slot) does now, since this instruction only
                // ever runs immediately after that sibling's own
                // Op::StoreLocal. Mutates the SAME shared TrailView the
                // consumer's own capturedLet already points at -- visible
                // to it exactly like every other capture, no different
                // from Op::MakeClosure's own self-reference patch, above.
                const Value& consumerVal = slots[static_cast<size_t>(ins.a)];
                if (const auto* closurePtr = std::get_if<ClosurePtr>(&consumerVal); closurePtr && *closurePtr) {
                    if (auto trail = capturedLetTrail(**closurePtr)) {
                        trail->set(chunk.names[static_cast<size_t>(ins.b)], slots[static_cast<size_t>(ins.c)]);
                    }
                }
                ++pc;
                break;
            }
            case Op::Range: {
                Value step = std::move(stack.back());
                stack.pop_back();
                Value end = std::move(stack.back());
                stack.pop_back();
                Value start = std::move(stack.back());
                stack.pop_back();
                stack.push_back(ev.applyRange(start, end, step));
                ++pc;
                break;
            }
            case Op::Index: {
                Value idx = std::move(stack.back());
                stack.pop_back();
                Value obj = std::move(stack.back());
                stack.pop_back();
                stack.push_back(ev.applyIndexAccess(obj, idx));
                ++pc;
                break;
            }
            case Op::Member: {
                Value obj = std::move(stack.back());
                stack.pop_back();
                stack.push_back(ev.applyMemberAccess(obj, chunk.names[static_cast<size_t>(ins.a)]));
                ++pc;
                break;
            }
            case Op::UnaryOp: {
                Value v = std::move(stack.back());
                stack.pop_back();
                stack.push_back(ev.applyUnaryOp(static_cast<oscad::NodeKind>(ins.a), v, *ins.pos));
                ++pc;
                break;
            }
            case Op::BinaryOp: {
                Value b = std::move(stack.back());
                stack.pop_back();
                Value a = std::move(stack.back());
                stack.pop_back();
                stack.push_back(ev.applyBinaryOp(static_cast<oscad::NodeKind>(ins.a), a, b, *ins.pos));
                ++pc;
                break;
            }
            case Op::Jump:
                pc = static_cast<size_t>(ins.a);
                break;
            case Op::JumpIfFalse: {
                Value v = std::move(stack.back());
                stack.pop_back();
                if (!truthy(v)) {
                    pc = static_cast<size_t>(ins.a);
                } else {
                    ++pc;
                }
                break;
            }
            case Op::JumpIfTrue: {
                Value v = std::move(stack.back());
                stack.pop_back();
                if (truthy(v)) {
                    pc = static_cast<size_t>(ins.a);
                } else {
                    ++pc;
                }
                break;
            }
            case Op::OpenLocalScope: {
                for (int i = 0; i < ins.b; ++i) slots[static_cast<size_t>(ins.a + i)] = Value{};
                ++pc;
                break;
            }
            case Op::BuildList: {
                const size_t count = static_cast<size_t>(ins.a);
                std::vector<Value> items(count);
                for (size_t i = 0; i < count; ++i) {
                    items[count - 1 - i] = std::move(stack.back());
                    stack.pop_back();
                }
                stack.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(items)})});
                ++pc;
                break;
            }
            case Op::AccumOpen:
                accumStack.emplace_back();
                ++pc;
                break;
            case Op::AccumAppendOne: {
                Value v = std::move(stack.back());
                stack.pop_back();
                accumStack.back().push_back(std::move(v));
                ++pc;
                break;
            }
            case Op::AccumAppendEach: {
                Value v = std::move(stack.back());
                stack.pop_back();
                appendEachInto(accumStack.back(), v);
                ++pc;
                break;
            }
            case Op::AccumClose: {
                std::vector<Value> items = std::move(accumStack.back());
                accumStack.pop_back();
                stack.push_back(Value{std::make_shared<const ValueList>(ValueList{std::move(items)})});
                ++pc;
                break;
            }
            case Op::AccumMergeEach: {
                std::vector<Value> inner = std::move(accumStack.back());
                accumStack.pop_back();
                for (const Value& item : inner) appendEachInto(accumStack.back(), item);
                ++pc;
                break;
            }
            case Op::IterMaterialize: {
                Value v = std::move(stack.back());
                stack.pop_back();
                IterList& il = iterLists[static_cast<size_t>(ins.a)];
                il.values = expandIterable(v, [&](size_t count) {
                    ev.warn("Bad range parameter in for statement: too many elements (" + std::to_string(count) + ")", ins.pos);
                });
                il.index = 0;
                il.total = il.values.size(); // once here, never per-iteration -- see IterList's own doc comment
                ++pc;
                break;
            }
            case Op::IterReset: {
                iterLists[static_cast<size_t>(ins.a)].index = 0;
                ++pc;
                break;
            }
            case Op::IterNext: {
                IterList& il = iterLists[static_cast<size_t>(ins.b)];
                if (il.index < il.total) {
                    slots[static_cast<size_t>(ins.a)] = il.values[il.index];
                    ++il.index;
                    ++pc;
                } else {
                    pc = static_cast<size_t>(ins.c);
                }
                break;
            }
            case Op::CheckIterLimit: {
                double count = std::get<double>(slots[static_cast<size_t>(ins.a)]) + 1.0;
                slots[static_cast<size_t>(ins.a)] = Value{count};
                if (count > static_cast<double>(ins.b)) {
                    ev.error("C-style for loop exceeded maximum iteration count", *ins.node);
                }
                ++pc;
                break;
            }
            case Op::CallFn: {
                const CompiledChunk::CallSite& site = chunk.callSites[static_cast<size_t>(ins.a)];
                const size_t argCount = static_cast<size_t>(ins.b);
                std::vector<Value> args(argCount);
                for (size_t i = 0; i < argCount; ++i) {
                    args[argCount - 1 - i] = std::move(stack.back());
                    stack.pop_back();
                }
                Value result;
                if (site.isImport) {
                    // import(): not in isBuiltinFunctionName's table (special-
                    // cased earlier in evalFunctionCall), but importAsValue
                    // takes the exact same pre-resolved CallArgs shape
                    // evalBuiltinFunction does.
                    CallArgs callArgs = buildCallArgs(site, args, argCount);
                    result = importAsValue(ev, callArgs, *site.callNode);
                } else if (site.isBuiltin && site.calleeName == "object") {
                    // object() needs its arguments merged in exact call-site
                    // interleaved order, which CallArgs' split positional/
                    // named maps can't represent -- but args/site.argNames
                    // are already in that exact source order (compiled and
                    // popped the same way every other call's arguments are),
                    // so no raw AST access is needed here at all.
                    std::vector<std::pair<std::optional<std::string>, Value>> pairs;
                    pairs.reserve(argCount);
                    for (size_t i = 0; i < argCount; ++i) pairs.emplace_back(site.argNames[i], std::move(args[i]));
                    result = mergeObjectArgs(pairs);
                } else if (site.isBuiltin) {
                    // evalBuiltinFunction takes a pre-resolved CallArgs, no
                    // live ctx needed at all -- builtin functions are
                    // already value-based, no interpreter bridge required.
                    CallArgs callArgs = buildCallArgs(site, args, argCount);
                    result = evalBuiltinFunction(ev, site.calleeName, callArgs, *site.callNode);
                } else {
                    // Replays bindArgs' own positional/named matching rule
                    // (positional index counted only among positional
                    // arguments, later write for a repeated name wins)
                    // against already-evaluated values.
                    BoundArgs bound;
                    bound.reserve(argCount);
                    size_t positionalIdx = 0;
                    const size_t nparams = site.decl->parameters.size();
                    for (size_t i = 0; i < argCount; ++i) {
                        if (site.argNames[i]) {
                            bound.set(*site.argNames[i], std::move(args[i]));
                        } else {
                            if (positionalIdx < nparams) {
                                bound.set(site.decl->parameters[positionalIdx]->name->name, std::move(args[i]));
                            }
                            ++positionalIdx;
                        }
                    }
                    result = ev.evalUserFunctionFromBound(site.calleeName, *site.decl, std::move(bound), ctx,
                                                           &site.callNode->position());
                }
                stack.push_back(std::move(result));
                ++pc;
                break;
            }
            case Op::CallDynamic: {
                const CompiledChunk::CallSite& site = chunk.callSites[static_cast<size_t>(ins.a)];
                const size_t argCount = static_cast<size_t>(ins.b);
                std::vector<Value> args(argCount);
                for (size_t i = 0; i < argCount; ++i) {
                    args[argCount - 1 - i] = std::move(stack.back());
                    stack.pop_back();
                }
                Value callee = std::move(stack.back());
                stack.pop_back();
                Value result;
                if (const auto* closurePtr = std::get_if<ClosurePtr>(&callee); closurePtr && *closurePtr) {
                    const Closure& closure = **closurePtr;
                    BoundArgs bound;
                    bound.reserve(argCount);
                    size_t positionalIdx = 0;
                    const size_t nparams = closure.node->parameters.size();
                    for (size_t i = 0; i < argCount; ++i) {
                        if (site.argNames[i]) {
                            bound.set(*site.argNames[i], std::move(args[i]));
                        } else {
                            if (positionalIdx < nparams) {
                                bound.set(closure.node->parameters[positionalIdx]->name->name, std::move(args[i]));
                            }
                            ++positionalIdx;
                        }
                    }
                    result = ev.evalFunctionLiteralFromBound(closure, std::move(bound), ctx, ins.pos);
                } else if (!site.calleeName.empty()) {
                    ev.warn("Ignoring unknown function '" + site.calleeName + "'", ins.pos);
                }
                stack.push_back(std::move(result));
                ++pc;
                break;
            }
            case Op::CallFnTail: {
                const CompiledChunk::CallSite& site = chunk.callSites[static_cast<size_t>(ins.a)];
                const size_t argCount = static_cast<size_t>(ins.b);
                std::vector<Value> args(argCount);
                for (size_t i = 0; i < argCount; ++i) {
                    args[argCount - 1 - i] = std::move(stack.back());
                    stack.pop_back();
                }
                BoundArgs bound;
                bound.reserve(argCount);
                size_t positionalIdx = 0;
                const size_t nparams = site.decl->parameters.size();
                for (size_t i = 0; i < argCount; ++i) {
                    if (site.argNames[i]) {
                        bound.set(*site.argNames[i], std::move(args[i]));
                    } else {
                        if (positionalIdx < nparams) {
                            bound.set(site.decl->parameters[positionalIdx]->name->name, std::move(args[i]));
                        }
                        ++positionalIdx;
                    }
                }
                // Eligible for trampolining only when isolated (not
                // closure-nested -- see Evaluator::isolatedCallCtxFor's own
                // doc comment) AND the callee itself has a compiled chunk
                // (a tail hop into a callee that doesn't compile pays one
                // real C++ frame at that boundary instead, same as today).
                // Ineligible falls through to exactly what CallFn already
                // does -- no new behavior for that case.
                if (tailOut != nullptr) {
                    if (auto hopCtx = ev.isolatedCallCtxFor(*site.decl, ctx)) {
                        if (ev.lookupOrCompileChunk(*site.decl) != nullptr) {
                            tailOut->decl = site.decl;
                            tailOut->literal = nullptr;
                            tailOut->bound = std::move(bound);
                            tailOut->ctx = std::move(*hopCtx);
                            tailOut->name = site.calleeName;
                            tailOut->callPos = &site.callNode->position();
                            return Value{};
                        }
                    }
                }
                Value result = ev.evalUserFunctionFromBound(site.calleeName, *site.decl, std::move(bound), ctx,
                                                              &site.callNode->position());
                stack.push_back(std::move(result));
                ++pc;
                break;
            }
            case Op::CallDynamicTail: {
                const CompiledChunk::CallSite& site = chunk.callSites[static_cast<size_t>(ins.a)];
                const size_t argCount = static_cast<size_t>(ins.b);
                std::vector<Value> args(argCount);
                for (size_t i = 0; i < argCount; ++i) {
                    args[argCount - 1 - i] = std::move(stack.back());
                    stack.pop_back();
                }
                Value callee = std::move(stack.back());
                stack.pop_back();
                Value result;
                if (const auto* closurePtr = std::get_if<ClosurePtr>(&callee); closurePtr && *closurePtr) {
                    const Closure& closure = **closurePtr;
                    const oscad::FunctionLiteral& funcNode = *closure.node;
                    BoundArgs bound;
                    bound.reserve(argCount);
                    size_t positionalIdx = 0;
                    const size_t nparams = funcNode.parameters.size();
                    for (size_t i = 0; i < argCount; ++i) {
                        if (site.argNames[i]) {
                            bound.set(*site.argNames[i], std::move(args[i]));
                        } else {
                            if (positionalIdx < nparams) {
                                bound.set(funcNode.parameters[positionalIdx]->name->name, std::move(args[i]));
                            }
                            ++positionalIdx;
                        }
                    }
                    if (tailOut != nullptr) {
                        if (auto hopCtx = ev.isolatedCallCtxFor(funcNode, ctx, capturedLetTrail(closure))) {
                            if (ev.lookupCompiledLiteralChunk(funcNode) != nullptr) {
                                tailOut->decl = nullptr;
                                tailOut->literal = &funcNode;
                                tailOut->bound = std::move(bound);
                                tailOut->ctx = std::move(*hopCtx);
                                tailOut->name = "<function literal>";
                                tailOut->callPos = ins.pos;
                                return Value{};
                            }
                        }
                    }
                    result = ev.evalFunctionLiteralFromBound(closure, std::move(bound), ctx, ins.pos);
                } else if (!site.calleeName.empty()) {
                    ev.warn("Ignoring unknown function '" + site.calleeName + "'", ins.pos);
                }
                stack.push_back(std::move(result));
                ++pc;
                break;
            }
            case Op::Echo: {
                const CompiledChunk::EchoSite& site = chunk.echoSites[static_cast<size_t>(ins.a)];
                const size_t argCount = static_cast<size_t>(ins.b);
                std::vector<std::pair<std::optional<std::string>, Value>> pairs(argCount);
                for (size_t i = 0; i < argCount; ++i) {
                    pairs[argCount - 1 - i] = {site.argNames[argCount - 1 - i], std::move(stack.back())};
                    stack.pop_back();
                }
                ev.emitEcho(pairs);
                ++pc;
                break;
            }
            case Op::AssertFail: {
                // Only ever reached on the condition-false path (guarded by
                // a JumpIfTrue the compiler emits around it) -- always
                // throws, matching evalAssertExpr's own error() call
                // exactly (same message format, same "assert" innermostFrame).
                const bool hasMessage = ins.a == 1;
                std::string err = "Assertion '" + std::get<std::string>(chunk.constants[static_cast<size_t>(ins.b)]) +
                                   "' failed";
                if (hasMessage) {
                    Value msg = std::move(stack.back());
                    stack.pop_back();
                    const std::string* s = std::get_if<std::string>(&msg);
                    err += ": \"" + (s ? *s : fmtValue(msg)) + "\"";
                }
                ev.error(err, *ins.node, "assert");
            }
        }
    }
    return stack.empty() ? Value{} : std::move(stack.back());
}

// Mirrors Evaluator::bindArgs' own positional/named matching exactly (same
// "later write for a repeated name wins," same "evaluate every argument
// expression even if its name doesn't match a declared parameter" side-
// effect rule), but writes matched-plain-parameter values directly into
// `slots` instead of a fresh unordered_map, and routes a matched-$-name
// straight to `childCtx.dyn` (childCtx.dyn->set) since $-parameters never
// reach compilation (see tryCompileFunction) -- so ANY $-named argument here
// is necessarily an undeclared override, exactly like today's interpreter
// path. An undeclared plain (non-$) name has no slot and nothing in this
// chunk can ever reference it by name, so its already-evaluated value is
// simply discarded -- ponytail: ctx.let_ isn't written for it either
// (unlike the interpreter path), which changes nothing observable, since
// nothing downstream of a compiled call ever reads ctx.let_ for a name a
// compiled body didn't itself resolve to a slot.
void bindCompiledArgs(Evaluator& ev, const CompiledChunk& chunk,
                       const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& callerCtx,
                       EvalContext& childCtx, std::vector<Value>& slots, std::vector<bool>& bound) {
    size_t positionalIdx = 0;
    const size_t nparams = chunk.params.size();
    for (const auto& argPtr : arguments) {
        if (argPtr->kind() == oscad::NodeKind::NamedArgument) {
            auto& a = static_cast<const oscad::NamedArgument&>(*argPtr);
            Value v = ev.evalExpr(*a.expr, callerCtx);
            const std::string& name = a.name->name;
            bool matched = false;
            for (size_t i = 0; i < nparams; ++i) {
                if (chunk.params[i].name == name) {
                    slots[static_cast<size_t>(chunk.params[i].slot)] = std::move(v);
                    bound[i] = true;
                    matched = true;
                    break;
                }
            }
            if (!matched && !name.empty() && name[0] == '$') {
                childCtx.dyn->set(name, v);
            }
        } else {
            auto& a = static_cast<const oscad::PositionalArgument&>(*argPtr);
            Value v = ev.evalExpr(*a.expr, callerCtx);
            if (positionalIdx < nparams) {
                slots[static_cast<size_t>(chunk.params[positionalIdx].slot)] = std::move(v);
                bound[positionalIdx] = true;
            }
            ++positionalIdx;
        }
    }
}

} // namespace

Value runCompiledFunction(Evaluator& ev, const CompiledChunk& chunk,
                          const std::vector<std::unique_ptr<oscad::Argument>>& arguments, EvalContext& callerCtx,
                          EvalContext& childCtx, TailCallRequest* tailOut) {
    VmFrameGuard guard(ev);
    VmFrame& frame = guard.get();
    frame.slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame.bound.assign(chunk.params.size(), false);
    // Stamped onto the current (this call's own) CallStackFrame only once
    // `frame.slots` is a well-defined (if still all-undef) array -- a
    // nested compiled closure's own LOAD_UPVALUE could in principle find
    // this frame from here on; see Evaluator::findUpvalue's own doc
    // comment for why that's only reachable once this call's own body
    // (which alone could construct/expose such a closure) has actually
    // started running, never during argument binding itself.
    ev.setCurrentCallVmFrame(&frame);
    bindCompiledArgs(ev, chunk, arguments, callerCtx, childCtx, frame.slots, frame.bound);
    for (size_t i = 0; i < chunk.params.size(); ++i) {
        if (!frame.bound[i] && !chunk.defaultCode[i].empty()) {
            frame.slots[static_cast<size_t>(chunk.params[i].slot)] =
                runChunk(ev, chunk, chunk.defaultCode[i], frame.slots, childCtx, frame.stack);
        }
    }
    return runChunk(ev, chunk, chunk.bodyCode, frame.slots, childCtx, frame.stack, tailOut);
}

Value runCompiledFunctionFromBound(Evaluator& ev, const CompiledChunk& chunk, const BoundArgs& bound,
                                    EvalContext& childCtx, TailCallRequest* tailOut) {
    VmFrameGuard guard(ev);
    VmFrame& frame = guard.get();
    frame.slots.assign(static_cast<size_t>(chunk.numSlots), Value{});
    frame.bound.assign(chunk.params.size(), false);
    ev.setCurrentCallVmFrame(&frame);
    for (size_t i = 0; i < chunk.params.size(); ++i) {
        if (const Value* v = bound.find(chunk.params[i].name)) {
            frame.slots[static_cast<size_t>(chunk.params[i].slot)] = *v;
            frame.bound[i] = true;
        }
    }
    for (const auto& [k, v] : bound) {
        bool matched = false;
        for (const auto& p : chunk.params) {
            if (p.name == k) {
                matched = true;
                break;
            }
        }
        if (!matched && !k.empty() && k[0] == '$') childCtx.dyn->set(k, v);
    }
    for (size_t i = 0; i < chunk.params.size(); ++i) {
        if (!frame.bound[i] && !chunk.defaultCode[i].empty()) {
            frame.slots[static_cast<size_t>(chunk.params[i].slot)] =
                runChunk(ev, chunk, chunk.defaultCode[i], frame.slots, childCtx, frame.stack);
        }
    }
    return runChunk(ev, chunk, chunk.bodyCode, frame.slots, childCtx, frame.stack, tailOut);
}

// The trampoline entry points -- see bytecode_vm.hpp's own doc comment for
// the overview. Both share the identical loop shape: run the first
// (real) call via the ordinary entry function (with tailOut populated),
// then keep reusing runCompiledFunctionFromBound directly for every
// subsequent hop (a TailCallRequest's own bound-args shape already
// matches that function's own parameter shape exactly -- no new binding
// logic needed).
//
// Every hop's own EvalContext (TailCallRequest::ctx, already derived by
// isolatedCallCtxFor -- a fresh $-var/dyn trail level) is kept alive for
// the WHOLE trampoline run via `chain`, mirroring evalFunctionBodyTrampoline's
// identical fix on the interpreter side (user_calls.cpp) exactly: $-vars
// stay dynamically scoped THROUGH even an isolated call (callCtx()'s own
// dyn uses isolate=false), so a later hop may still need to walk back
// through a much earlier one's dyn level to resolve a name. Reusing a
// single EvalContext variable via plain assignment would drop an earlier
// level's shared_ptr refcount to zero and pop it (ScopeTrailStorage::
// popLevel, scope_trail.hpp's custom-deleter mechanism) -- permanently
// erasing a parent-chain link a later hop still needs. Trades native C++
// *stack* growth (what this trampoline exists to eliminate) for heap
// growth instead -- O(iteration count) EvalContext objects, bounded by
// available RAM rather than a ~few-MB thread stack.
namespace {
// Tears a trampoline's own `chain` down back-to-front on EVERY exit path --
// normal return OR exception unwind (recordTailCallHop's own recursion-guard
// error is the exact case that matters: an infinite tail-recursive function
// with no base case throws out of the loop entirely, bypassing any teardown
// code placed after it). Relying on `chain` simply falling out of scope
// instead is NOT equivalent: std::vector<T>'s own element-destruction order
// is unspecified by the standard -- libc++ destroys back-to-front (matching
// ScopeTrailStorage::popLevel's own "scan from the back" doc comment, which
// assumes the level being popped is usually the most recently pushed one --
// O(1) per pop that way), but libstdc++ destroys front-to-back, the
// adversarial order for that same scan -- each pop then walks almost the
// entire remaining vector, turning an intended O(N) teardown into a real,
// measured O(N^2) (see issue #50: an N-scaling experiment plus a minimal
// repro proving libc++/libstdc++ disagree on std::vector<T>::clear()'s
// destruction order for identical source). Popping back-to-front explicitly,
// via a destructor that always runs before `chain`'s own, is correct AND
// O(N) on every standard library and every exit path, not just the ones
// that happen to agree with libc++ and return normally.
struct ChainTeardown {
    std::vector<EvalContext>& chain;
    ~ChainTeardown() {
        while (!chain.empty()) chain.pop_back();
    }
};
} // namespace

Value runCompiledFunctionTrampoline(Evaluator& ev, const CompiledChunk& chunk,
                                     const std::vector<std::unique_ptr<oscad::Argument>>& arguments,
                                     EvalContext& callerCtx, EvalContext& childCtx) {
    std::vector<EvalContext> chain{childCtx};
    ChainTeardown teardown{chain};
    TailCallRequest req;
    Value result = runCompiledFunction(ev, chunk, arguments, callerCtx, chain.back(), &req);
    unsigned recursionGuard = 0;
    while (req.decl != nullptr || req.literal != nullptr) {
        const oscad::ASTNode& calleeDecl =
            req.decl ? static_cast<const oscad::ASTNode&>(*req.decl) : static_cast<const oscad::ASTNode&>(*req.literal);
        ev.recordTailCallHop(req.name, calleeDecl, req.callPos, recursionGuard);
        const CompiledChunk* curChunk =
            req.decl ? ev.lookupOrCompileChunk(*req.decl) : ev.lookupCompiledLiteralChunk(*req.literal);
        BoundArgs bound = std::move(req.bound);
        chain.push_back(std::move(req.ctx));
        TailCallRequest next;
        result = runCompiledFunctionFromBound(ev, *curChunk, bound, chain.back(), &next);
        req = std::move(next);
    }
    return result;
}

Value runCompiledFunctionFromBoundTrampoline(Evaluator& ev, const CompiledChunk& chunk, const BoundArgs& bound,
                                              EvalContext& childCtx) {
    std::vector<EvalContext> chain{childCtx};
    ChainTeardown teardown{chain};
    TailCallRequest req;
    Value result = runCompiledFunctionFromBound(ev, chunk, bound, chain.back(), &req);
    unsigned recursionGuard = 0;
    while (req.decl != nullptr || req.literal != nullptr) {
        const oscad::ASTNode& calleeDecl =
            req.decl ? static_cast<const oscad::ASTNode&>(*req.decl) : static_cast<const oscad::ASTNode&>(*req.literal);
        ev.recordTailCallHop(req.name, calleeDecl, req.callPos, recursionGuard);
        const CompiledChunk* curChunk =
            req.decl ? ev.lookupOrCompileChunk(*req.decl) : ev.lookupCompiledLiteralChunk(*req.literal);
        BoundArgs nextBound = std::move(req.bound);
        chain.push_back(std::move(req.ctx));
        TailCallRequest next;
        result = runCompiledFunctionFromBound(ev, *curChunk, nextBound, chain.back(), &next);
        req = std::move(next);
    }
    return result;
}

} // namespace oscadeval
