#include "xacro_cpp/processor.hpp"

#include <iostream>
#include <string>
#include <unordered_map>

using xacro_cpp::Options;
using xacro_cpp::Processor;

static void print_usage() {
  std::cerr << "Usage: xacro_cpp input.xacro [-o output.xml] [name:=value ...]\n";
}

int main(int argc, char** argv) {
  if (argc < 2) { print_usage(); return 1; }

  Options opts;
  opts.input_path = argv[1];
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-o" && i + 1 < argc) { opts.output_path = argv[++i]; continue; }
    auto pos = a.find(":=");
    if (pos != std::string::npos) {
      std::string name = a.substr(0, pos);
      std::string value = a.substr(pos + 2);
      opts.cli_args[name] = value;
    }
  }

  Processor p;
  std::string err;
  if (!p.run(opts, &err)) {
    std::cerr << "xacro_cpp error: " << err << "\n";
    return 2;
  }
  return 0;
}

