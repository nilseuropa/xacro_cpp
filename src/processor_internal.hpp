#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "xacro_cpp/processor.hpp"

namespace xacro_cpp {

constexpr char kDollarMarker = '\x1D';

std::string evalStringWithVars(const std::string& expr,
                               const std::unordered_map<std::string, std::string>& vars,
                               bool* resolved = nullptr);
void collectIdentifiers(const std::string& expr, std::vector<std::string>* out);
bool splitListLiteral(const std::string& expr, std::vector<std::string>* items);

std::string resolveFind(const std::string& pkg);
std::string stripQuotes(const std::string& s);
std::string getDefaultName(const std::string& name);

YamlValue parseYamlFile(const std::string& path);
std::string yamlValueToString(const YamlValue& value);

} // namespace xacro_cpp
