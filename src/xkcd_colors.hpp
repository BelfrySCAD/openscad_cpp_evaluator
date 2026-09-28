#pragma once

#include <array>
#include <string>
#include <unordered_map>

namespace oscadeval {

// The xkcd colour survey's names -> 8-bit RGB, lowercase keys, no "xkcd:"
// prefix. See xkcd_colors.cpp.
const std::unordered_map<std::string, std::array<unsigned char, 3>>& xkcdColorTable();

} // namespace oscadeval
