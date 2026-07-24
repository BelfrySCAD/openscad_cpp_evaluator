#include "cli_lib.hpp"

#include <string>
#include <vector>

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    return oscadeval::runCli(args);
}
