#pragma once

#include <cstddef>

namespace oscadeval {

// Real native-stack-overflow safety check, replacing "count native re-
// entries and hope the guess is right" (see driveVmNativeDepth_/
// kMaxDriveVmNativeDepth and kMaxUserCallDepth's own doc comments,
// evaluator.hpp, for the two fixed-count guards this backs up). A fixed
// frame count is a PROXY for actual stack usage, and a bad one: real
// BOSL2 usage (attachable()'s own multmatrix/_show_highlight/_show_ghost/
// _color wrapper chain, applied to every primitive) was found to need
// well past 100 levels of legitimate, bounded native reentry -- but 100
// and even 50 segfaulted for real on Windows CI when tried as the fixed
// ceiling (see this project's own PR #61 history). No single fixed count
// is both safe on every platform/thread AND generous enough for real
// deeply-wrapped BOSL2 scripts, because the actual per-level native
// stack cost isn't a constant the compiler/platform holds still --
// checking the REAL remaining stack space generalizes correctly
// regardless of call shape or platform, where a guessed count can't.
//
// Returns true if fewer than `marginBytes` remain on the CURRENT
// thread's own native call stack before it would genuinely overflow --
// i.e. it is NOT safe to recurse (natively) any further right now.
// Returns false both when there's still enough headroom AND when this
// thread's own stack bounds couldn't be determined at all (unknown
// platform, or the underlying OS call failed) -- a deliberately
// conservative-toward-"don't block" default, since every call site
// using this ALSO keeps its own much-higher fixed-count backstop for
// exactly that case (see e.g. kMaxDriveVmNativeDepth/kMaxUserCallDepth's
// own doc comments) -- this function is never the ONLY line of defense
// against a genuinely infinite/runaway recursion, only the PRIMARY,
// more accurate one when it's available.
//
// This thread's own stack bounds (base + size) are queried ONCE and
// cached in thread-local storage -- cheap to call as often as needed
// after the first call on a given thread (just an address-of-a-local
// comparison against the cached bounds), important since a debug
// session or a render worker each run on their OWN thread, with their
// OWN (possibly much smaller than the process main thread's) stack.
bool nativeStackMarginLow(size_t marginBytes);

// Whether this thread's own native stack bounds were successfully
// determined -- i.e. whether nativeStackMarginLow's answer for this
// thread is authoritative rather than the deliberate always-false
// fallback. A call site combining this check with its own fixed-count
// backstop should pick ONE of the two per call, not OR them together:
// trust nativeStackMarginLow alone when this is true, and fall back to
// the fixed count only when it's false. OR-ing both unconditionally
// defeats the whole point -- confirmed for real (openscad_cpp_evaluator,
// Anklet.scad investigation): a legitimate, deeply-wrapped BOSL2 call
// chain tripped a fixed native-reentry-count guard while genuinely over
// 8 MiB of an 8 MiB stack still remained free, because the old count was
// calibrated as a blunt worst-case guess, not a measurement -- OR-ing it
// in kept firing regardless of what the real, accurate check reported.
bool nativeStackBoundsKnown();

} // namespace oscadeval
