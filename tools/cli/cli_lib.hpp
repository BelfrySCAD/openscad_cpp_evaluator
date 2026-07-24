#pragma once

#include <iostream>
#include <string>
#include <vector>

namespace oscadeval {

// The CLI's actual logic, extracted from main() into an in-process
// testable function -- mirrors the Python reference's own cli.main(argv)
// shape (a plain function taking argv-equivalent input and returning an
// exit code, with stdout/stderr/stdin all overridable) rather than
// leaving everything inline in main() where only a subprocess-spawning
// test could reach it. `args` excludes argv[0] (the program name), same
// convention as Python's sys.argv[1:]. `in`/`out`/`err` default to
// std::cin/std::cout/std::cerr for real use; a test passes an
// istringstream/ostringstream pair instead to drive a --debug session (or
// assert on plain-run stdout/stderr) without spawning a subprocess --
// mirrors the reference's own `_feed_input`-monkeypatched-`input()`
// pytest fixtures and `capsys`.
int runCli(const std::vector<std::string>& args, std::istream& in = std::cin, std::ostream& out = std::cout,
           std::ostream& err = std::cerr);

} // namespace oscadeval
