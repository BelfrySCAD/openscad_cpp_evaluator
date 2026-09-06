#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace oscadeval {

// A string is a sequence of CHARACTERS to a script, not of bytes. The
// reference reports len("aé—z") as 4, indexes s[1] to "é", and iterates the
// same four characters -- while a std::string holds 8 bytes for that. These
// three are what keeps the two views apart.
//
// Storage stays UTF-8. Indexing therefore scans, which is O(n) per index and
// so O(n^2) over a loop -- BOSL2's _str_count_leading is exactly that loop.
// The saving grace is that a pure-ASCII string needs no scan at all: byte i
// IS character i, and every function here takes that path first. Only a
// string that actually holds non-ASCII pays, and those are short in
// practice.

// One code point as UTF-8. The caller decides which code points are
// legal -- chr() drops an invalid one, a \uXXXX escape substitutes a space
// -- so this encodes whatever it is given.
std::string utf8Encode(uint32_t cp);

// True when every byte is < 0x80, so byte offsets and character offsets are
// the same thing.
bool utf8IsAscii(const std::string& s);

// Characters, not bytes. Counts lead bytes, so a malformed sequence counts
// the same way the reference's own decoder would rather than throwing.
size_t utf8Length(const std::string& s);

// The i-th character as its own string; empty if i is past the end. Empty is
// distinguishable from a real result because a character is never zero-length.
std::string utf8CharAt(const std::string& s, size_t i);

// Every character as its own string.
std::vector<std::string> utf8Chars(const std::string& s);

} // namespace oscadeval
