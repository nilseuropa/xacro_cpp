/// NowTechnologies Zrt. All rights reserved.
/// Public API for xacro_cpp processing.
/// Author: nilseuropa <marton@nowtech.hu>
/// Created: 2026.01.20
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace tinyxml2 {
class XMLDocument;
class XMLElement;
class XMLNode;
} // namespace tinyxml2

namespace xacro_cpp {

class ProcessingError : public std::runtime_error {
public:
  explicit ProcessingError(const std::string& what_arg) : std::runtime_error(what_arg) {}
};

struct Options {
  std::string mInputPath;
  /// empty -> stdout
  std::string mOutputPath;
  /// name:=value
  std::unordered_map<std::string, std::string> mCliArgs;
};

/// Minimal expression evaluator facade (wraps tinyexpr)
double evalNumber(const std::string& expr,
                  const std::unordered_map<std::string, std::string>& vars,
                  bool* ok = nullptr);
bool evalBool(const std::string& expr, const std::unordered_map<std::string, std::string>& vars);
std::string evalStringTemplate(const std::string& text, const std::unordered_map<std::string, std::string>& vars);

struct YamlValue {
  enum class Type { cNull, cBool, cInt, cDouble, cString, cList, cMap };
  Type mType = Type::cNull;
  bool mBoolValue = false;
  int64_t mIntValue = 0;
  double mDoubleValue = 0.0;
  std::string mStringValue;
  std::vector<YamlValue> mListValue;
  std::unordered_map<std::string, YamlValue> mMapValue;
};

std::string evalStringTemplate(const std::string& text,
                               const std::unordered_map<std::string, std::string>& vars,
                               const std::unordered_map<std::string, YamlValue>* yamlDocs);

class Processor {
public:
  /// Core xacro expansion pipeline: load, collect, expand, and serialize.
  Processor();
  ~Processor();
  Processor(const Processor&) = delete;
  Processor& operator=(const Processor&) = delete;
  Processor(Processor&&) = delete;
  Processor& operator=(Processor&&) = delete;

  struct MacroParam {
    std::string mName;
    /// optional
    std::string mDefaultValue;
    /// xacro block parameter (params prefixed with '*')
    bool mIsBlock = false;
    /// distinguishes explicit empty default from missing default
    bool mHasDefault = false;
  };
  struct MacroDef {
    std::string mName;
    /// points into mDoc
    tinyxml2::XMLNode* mContent = nullptr;
    std::vector<MacroParam> mParams;
    std::string mBaseDir;
  };

  /// Runs expansion and returns true on success. Result is serialized to mOutputPath or stdout.
  bool run(const Options& opts, std::string* errorMsg);

  /// Runs expansion and returns the expanded XML in-memory.
  /// Does not write to stdout. Returns true on success and fills urdfXml.
  bool runToString(const Options& opts, std::string* urdfXml, std::string* errorMsg);

  /// Parses just enough to collect xacro:arg and xacro:property defaults without full expansion.
  /// Returns true and fills argsOut with argument names and their default/effective values.
  /// CLI args in opts.mCliArgs override defaults.
  bool collectArgs(const Options& opts, std::map<std::string, std::string>* argsOut, std::string* errorMsg);

private:
  /// owned
  std::unique_ptr<tinyxml2::XMLDocument> mDoc;
  std::unordered_map<std::string, std::string> mVars;
  std::unordered_map<std::string, MacroDef> mMacros;
  std::unordered_set<std::string> mArgNames;
  std::string mBaseDir;
  bool mModified = false;
  std::unordered_map<std::string, std::unordered_set<std::string>> mPropDeps;
  std::unordered_map<std::string, std::vector<tinyxml2::XMLNode*>> mPropertyBlocks;
  std::unordered_map<const tinyxml2::XMLNode*, std::string> mNodeBaseDirs;
  /// YAML documents loaded via xacro.load_yaml()
  std::unordered_map<std::string, YamlValue> mYamlDocs;

  bool loadDocument(const std::string& path, std::string* errorMsg);
  bool processDocument(std::string* errorMsg);

  /// Passes
  bool passCollectArgsAndProps(std::string* errorMsg);
  bool passCollectMacros(std::string* errorMsg);
  bool passExpandIncludes(std::string* errorMsg);
  bool passExpand(std::string* errorMsg);

  /// Helpers
  static bool isXacroElement(const tinyxml2::XMLElement* el, const char* local);
  static std::string getAttr(const tinyxml2::XMLElement* el, const char* name);
  static std::string trim(const std::string& s);

  bool handleIfUnless(tinyxml2::XMLElement* el, std::vector<tinyxml2::XMLNode*>* insertedOut = nullptr);
  bool expandXacroElement(tinyxml2::XMLElement* el, const std::unordered_map<std::string, std::string>& scope);
  bool defineProperty(const tinyxml2::XMLElement* el);
  bool defineArg(const tinyxml2::XMLElement* el);
  bool defineMacro(tinyxml2::XMLElement* el, const std::string& baseDir);
  bool expandIncludesInNode(tinyxml2::XMLNode* node, const std::string& baseDir, std::string* errorMsg);
  bool expandNode(tinyxml2::XMLNode* node);
  void collectGlobalsInIncludedDoc(tinyxml2::XMLDocument& inc, const std::string& baseDir);
  void collectArgsAndProps(tinyxml2::XMLElement* root, bool collectProps, bool removeProps, bool removeArgs);
  bool loadIncludeFile(const std::string& full,
                       bool collectGlobals,
                       std::vector<tinyxml2::XMLNode*>* cloned,
                       bool* rootPresent,
                       std::string* baseDirOut,
                       std::string* errorMsg);
  void restoreDollarMarkers(tinyxml2::XMLNode* node);
  bool applyXacroAttribute(tinyxml2::XMLElement* el, const std::unordered_map<std::string, std::string>& scope);
  const MacroDef* findMacro(const std::vector<std::string>& names) const;
  void removeComments(tinyxml2::XMLNode* node);
  /// Returns true if 'el' still exists and its children should be traversed;
  /// returns false if 'el' was deleted/replaced and traversal should not use 'el'.
  bool expandElement(tinyxml2::XMLElement* el, std::vector<tinyxml2::XMLNode*>* inserted);
  bool expandMacroCall(tinyxml2::XMLElement* el,
                       const std::unordered_map<std::string, std::string>& parentScope,
                       const std::unordered_map<std::string, std::vector<tinyxml2::XMLNode*>>* parentBlocks = nullptr);
  void substituteAttributes(tinyxml2::XMLElement* el, const std::unordered_map<std::string, std::string>& scope);
};

} // namespace xacro_cpp
