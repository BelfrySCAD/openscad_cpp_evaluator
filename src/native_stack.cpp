#include "openscad_cpp_evaluator/native_stack.hpp"

#include <cstdint>

#if defined(__APPLE__)
#include <pthread.h>
#elif defined(__linux__)
#include <pthread.h>
#elif defined(_WIN32)
// windows.h must come first -- it defines the target-architecture macros
// (_AMD64_/_X86_/_ARM64_) that processthreadsapi.h's own transitive
// winnt.h include needs; including processthreadsapi.h standalone first
// hits winnt.h before those are set, MSVC error C1189 "No Target
// Architecture" (confirmed for real: broke Windows CI, PR #63).
#include <windows.h>
#include <processthreadsapi.h>
#endif

namespace oscadeval {

namespace {

// This thread's own [low, high) native stack bounds (low < high; the
// stack grows from high DOWN toward low, the near-universal x86/ARM
// convention). {0, 0} means "not yet queried" (checked lazily on the
// first call on this thread); {kUnsupported, kUnsupported} means
// "queried once, couldn't determine it here, don't keep retrying."
struct StackBounds {
    uintptr_t low = 0;
    uintptr_t high = 0;
};

constexpr uintptr_t kUndetermined = 0;
// An otherwise-impossible bound (a real stack can never span the whole
// address space), reused as the "give up, this platform/thread doesn't
// support it" sentinel so a failed query is only ever attempted once.
constexpr uintptr_t kUnsupported = static_cast<uintptr_t>(-1);

StackBounds queryStackBounds() {
#if defined(__APPLE__)
    pthread_t self = pthread_self();
    // macOS convention: pthread_get_stackaddr_np returns the BASE (the
    // high address the stack grows DOWN from), unlike glibc's own
    // pthread_attr_getstack below.
    void* stackAddr = pthread_get_stackaddr_np(self);
    size_t stackSize = pthread_get_stacksize_np(self);
    if (stackAddr == nullptr || stackSize == 0) return StackBounds{kUnsupported, kUnsupported};
    const uintptr_t high = reinterpret_cast<uintptr_t>(stackAddr);
    const uintptr_t low = high - static_cast<uintptr_t>(stackSize);
    return StackBounds{low, high};
#elif defined(__linux__)
    pthread_attr_t attr;
    if (pthread_getattr_np(pthread_self(), &attr) != 0) return StackBounds{kUnsupported, kUnsupported};
    void* stackAddr = nullptr;
    size_t stackSize = 0;
    // glibc convention: stackAddr is the LOW address directly (opposite
    // of macOS's pthread_get_stackaddr_np, above).
    const int rc = pthread_attr_getstack(&attr, &stackAddr, &stackSize);
    pthread_attr_destroy(&attr);
    if (rc != 0 || stackAddr == nullptr || stackSize == 0) return StackBounds{kUnsupported, kUnsupported};
    const uintptr_t low = reinterpret_cast<uintptr_t>(stackAddr);
    const uintptr_t high = low + static_cast<uintptr_t>(stackSize);
    return StackBounds{low, high};
#elif defined(_WIN32)
    ULONG_PTR lowLimit = 0;
    ULONG_PTR highLimit = 0;
    GetCurrentThreadStackLimits(&lowLimit, &highLimit); // Windows 8+/Server 2012+, no failure return
    if (lowLimit == 0 || highLimit == 0) return StackBounds{kUnsupported, kUnsupported};
    return StackBounds{static_cast<uintptr_t>(lowLimit), static_cast<uintptr_t>(highLimit)};
#else
    return StackBounds{kUnsupported, kUnsupported};
#endif
}

StackBounds& cachedBounds() {
    static thread_local StackBounds bounds; // {0,0} (kUndetermined) until this thread's first call
    if (bounds.low == kUndetermined && bounds.high == kUndetermined) {
        bounds = queryStackBounds();
    }
    return bounds;
}

} // namespace

bool nativeStackBoundsKnown() { return cachedBounds().low != kUnsupported; }

bool nativeStackMarginLow(size_t marginBytes) {
    const StackBounds& bounds = cachedBounds();
    if (bounds.low == kUnsupported) return false; // callers keep their own fixed-count backstop for this case
    // Any local variable's own address is a reasonable proxy for the
    // current stack pointer -- doesn't need to be exact, only close
    // enough to compare against a margin measured in tens/hundreds of KB.
    volatile char marker = 0;
    const uintptr_t currentSp = reinterpret_cast<uintptr_t>(const_cast<char*>(&marker));
    if (currentSp < bounds.low) return true; // already past the known low bound -- no margin left, by definition
    const uintptr_t remaining = currentSp - bounds.low;
    return remaining < static_cast<uintptr_t>(marginBytes);
}

} // namespace oscadeval
