/// NowTechnologies Zrt. All rights reserved.
/// xacro_cpp CLI entry point.
/// Author: nilseuropa <marton@nowtech.hu>
/// Created: 2026.01.20
#include <iostream>
#include <string>
#include <unordered_map>

#include "xacro_cpp/processor.hpp"

using xacro_cpp::Options;
using xacro_cpp::Processor;

static void printUsage() {
  std::cerr << "Usage: xacro_cpp input.xacro [-o output.xml] [name:=value ...]\n";
}

int main(int argc, char** argv) {
  if (argc < 2) {
    printUsage();
    return 1;
  }

  Options opts;
  opts.mInputPath = argv[1];
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-o" && i + 1 < argc) {
      opts.mOutputPath = argv[++i];
      continue;
    }
    auto pos = a.find(":=");
    if (pos != std::string::npos) {
      std::string name = a.substr(0, pos);
      std::string value = a.substr(pos + 2);
      opts.mCliArgs[name] = value;
    }
  }

  Processor p;
  std::string err;
  try {
    p.run(opts, &err);
    return 0;
  } catch (const xacro_cpp::ProcessingError& e) {
    std::cerr << "xacro_cpp error: " << e.what() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "xacro_cpp unexpected error: " << e.what() << "\n";
  }
  return 2;
}
