/// NowTechnologies Zrt. All rights reserved.
/// YAML parsing helpers for xacro_cpp.
/// Author: nilseuropa <marton@nowtech.hu>
/// Created: 2026.01.20

#include "xacro_cpp/processor.hpp"

#include <cctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "processor_internal.hpp"

namespace xacro_cpp {

namespace {

static bool isNullScalar(const std::string& raw) {
  return raw == "~" || raw == "null" || raw == "cNull" || raw == "NULL";
}

static bool isBoolScalar(const std::string& raw) {
  std::string lower;
  lower.reserve(raw.size());
  for (char c : raw) {
    lower.push_back((char)std::tolower(static_cast<unsigned char>(c)));
  }
  return lower == "true" || lower == "false" || lower == "yes" || lower == "no" || lower == "on" || lower == "off";
}

static bool scalarAllowsInt(const std::string& raw) {
  return raw.find('.') == std::string::npos && raw.find('e') == std::string::npos && raw.find('E') == std::string::npos;
}

static YamlValue yamlNodeToValue(const YAML::Node& node) {
  YamlValue out;
  if (!node || node.IsNull()) {
    out.mType = YamlValue::Type::cNull;
    return out;
  }
  if (node.IsScalar()) {
    std::string raw = node.Scalar();
    if (isNullScalar(raw)) {
      out.mType = YamlValue::Type::cNull;
      return out;
    }
    if (isBoolScalar(raw)) {
      try {
        out.mType = YamlValue::Type::cBool;
        out.mBoolValue = node.as<bool>();
        return out;
      } catch (...) {
      }
    }
    if (scalarAllowsInt(raw)) {
      try {
        out.mType = YamlValue::Type::cInt;
        out.mIntValue = node.as<int64_t>();
        return out;
      } catch (...) {
      }
    }
    try {
      out.mType = YamlValue::Type::cDouble;
      out.mDoubleValue = node.as<double>();
      return out;
    } catch (...) {
    }
    out.mType = YamlValue::Type::cString;
    try {
      out.mStringValue = node.as<std::string>();
    } catch (...) {
      out.mStringValue = raw;
    }
    return out;
  }
  if (node.IsSequence()) {
    out.mType = YamlValue::Type::cList;
    out.mListValue.reserve(node.size());
    for (const auto& item : node) {
      out.mListValue.push_back(yamlNodeToValue(item));
    }
    return out;
  }
  if (node.IsMap()) {
    out.mType = YamlValue::Type::cMap;
    for (const auto& kv : node) {
      try {
        auto key = kv.first.as<std::string>();
        out.mMapValue.emplace(key, yamlNodeToValue(kv.second));
      } catch (...) {
      }
    }
    return out;
  }
  out.mType = YamlValue::Type::cNull;
  return out;
}

} // namespace

YamlValue parseYamlFile(const std::string& path) {
  try {
    YAML::Node node = YAML::LoadFile(path);
    return yamlNodeToValue(node);
  } catch (...) {
    YamlValue out;
    out.mType = YamlValue::Type::cNull;
    return out;
  }
}

std::string yamlValueToString(const YamlValue& value) {
  switch (value.mType) {
    case YamlValue::Type::cNull:
      return "None";
    case YamlValue::Type::cBool:
      return value.mBoolValue ? "True" : "False";
    case YamlValue::Type::cInt:
      return std::to_string(value.mIntValue);
    case YamlValue::Type::cDouble: {
      std::ostringstream oss;
      oss << std::setprecision(std::numeric_limits<double>::max_digits10) << value.mDoubleValue;
      std::string s = oss.str();
      if (s.find('.') == std::string::npos && s.find('e') == std::string::npos && s.find('E') == std::string::npos) {
        s += ".0";
      }
      return s;
    }
    case YamlValue::Type::cString:
      return value.mStringValue;
    case YamlValue::Type::cList: {
      std::ostringstream oss;
      oss << "[";
      for (size_t i = 0; i < value.mListValue.size(); ++i) {
        if (i != 0U) {
          oss << ", ";
        }
        oss << yamlValueToString(value.mListValue[i]);
      }
      oss << "]";
      return oss.str();
    }
    case YamlValue::Type::cMap:
      return std::string();
  }
  return std::string();
}

} // namespace xacro_cpp
