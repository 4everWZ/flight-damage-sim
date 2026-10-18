#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>

#ifndef BUILD_CONFIG_CPP
#define BUILD_CONFIG_CPP 0
#endif
#ifndef BUILD_CONFIG_CXX
#define BUILD_CONFIG_CXX 0
#endif
#ifndef BUILD_CONFIG_LABEL
#define BUILD_CONFIG_LABEL "default"
#endif

// Exercise real container-bearing core data across the translation-unit ABI,
// and report which settings the existing consumer executable actually uses.
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto d = flight::Definition::load(argv[1]);
    if (d.cells.empty() || d.chains.empty())
      throw std::runtime_error("core data did not cross the ABI intact");
#ifdef _GLIBCXX_DEBUG
    constexpr bool debug = true;
#else
    constexpr bool debug = false;
#endif
    std::cout << "{\"debug_stl\":" << debug
              << ",\"cpp_tag\":" << BUILD_CONFIG_CPP
              << ",\"cxx_tag\":" << BUILD_CONFIG_CXX
              << ",\"label\":" << std::quoted(BUILD_CONFIG_LABEL)
              << ",\"cells\":" << d.cells.size()
              << ",\"chains\":" << d.chains.size() << "}\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
