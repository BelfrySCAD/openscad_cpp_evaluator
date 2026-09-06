#include "openscad_cpp_evaluator/utf8.hpp"

namespace oscadeval {
namespace {

// A continuation byte is 0b10xxxxxx. Every other byte starts a character,
// which is what makes counting and stepping possible without decoding.
inline bool isContinuation(unsigned char c) { return (c & 0xC0) == 0x80; }

// Past the end of the character starting at `i`. Advances at least one byte
// even for a malformed sequence, so no caller can loop forever on bad input.
size_t nextCharStart(const std::string& s, size_t i) {
    ++i;
    while (i < s.size() && isContinuation(static_cast<unsigned char>(s[i]))) ++i;
    return i;
}

} // namespace

bool utf8IsAscii(const std::string& s) {
    for (char c : s) {
        if (static_cast<unsigned char>(c) >= 0x80) return false;
    }
    return true;
}

size_t utf8Length(const std::string& s) {
    if (utf8IsAscii(s)) return s.size();
    size_t n = 0;
    for (size_t i = 0; i < s.size(); i = nextCharStart(s, i)) ++n;
    return n;
}

std::string utf8CharAt(const std::string& s, size_t i) {
    if (utf8IsAscii(s)) return i < s.size() ? std::string(1, s[i]) : std::string{};
    size_t at = 0;
    for (size_t b = 0; b < s.size(); b = nextCharStart(s, b)) {
        if (at++ == i) {
            const size_t end = nextCharStart(s, b);
            return s.substr(b, end - b);
        }
    }
    return {};
}

std::vector<std::string> utf8Chars(const std::string& s) {
    std::vector<std::string> out;
    if (utf8IsAscii(s)) {
        out.reserve(s.size());
        for (char c : s) out.emplace_back(1, c);
        return out;
    }
    for (size_t b = 0; b < s.size();) {
        const size_t end = nextCharStart(s, b);
        out.push_back(s.substr(b, end - b));
        b = end;
    }
    return out;
}

} // namespace oscadeval
