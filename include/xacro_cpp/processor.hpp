#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <map>
#include <unordered_set>

namespace tinyxml2 { class XMLDocument; class XMLElement; class XMLNode; }

namespace xacro_cpp {

struct Options {
  std::string input_path;
  std::string output_path; // empty -> stdout
  std::unordered_map<std::string, std::string> cli_args; // name:=value
};

// Minimal expression evaluator facade (wraps tinyexpr)
double eval_number(const std::string& expr,
                   const std::unordered_map<std::string, std::string>& vars,
                   bool* ok = nullptr);
bool eval_bool(const std::string& expr,
               const std::unordered_map<std::string, std::string>& vars);
std::string eval_string_template(const std::string& text,
                                 const std::unordered_map<std::string, std::string>& vars);

class Processor {
public:
  Processor();
  ~Processor();

  struct MacroParam {
    std::string name;
    std::string default_value; // optional
    bool is_block = false;     // xacro block parameter (params prefixed with '*')
  };
  struct MacroDef {
    std::string name;
    tinyxml2::XMLNode* content; // points into doc_
    std::vector<MacroParam> params;
  };

  // Runs expansion and returns true on success. Result is serialized to output_path or stdout.
  bool run(const Options& opts, std::string* error_msg);

  // Runs expansion and returns the expanded XML in-memory.
  // Does not write to stdout. Returns true on success and fills urdf_xml.
  bool runToString(const Options& opts, std::string* urdf_xml, std::string* error_msg);

  // Parses just enough to collect xacro:arg and xacro:property defaults without full expansion.
  // Returns true and fills args_out with argument names and their default/effective values.
  // CLI args in opts.cli_args override defaults.
  bool collectArgs(const Options& opts,
                   std::map<std::string, std::string>* args_out,
                   std::string* error_msg);

private:
  tinyxml2::XMLDocument* doc_ = nullptr; // owned
  std::unordered_map<std::string, std::string> vars_;
  std::unordered_map<std::string, MacroDef> macros_;
  std::unordered_set<std::string> arg_names_;
  std::string base_dir_;
  bool modified_ = false;
  // Minimal YAML store for xacro.load_yaml(name) assigned to properties
  // maps property-name -> { key -> vector of doubles }
  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<double>>> yaml_maps_;

  bool loadDocument(const std::string& path, std::string* error_msg);
  bool processDocument(std::string* error_msg);

  // Passes
  bool passCollectArgsAndProps(std::string* error_msg);
  bool passCollectMacros(std::string* error_msg);
  bool passExpandIncludes(std::string* error_msg);
  bool passExpand(std::string* error_msg);

  // Helpers
  static bool isXacroElement(const tinyxml2::XMLElement* el, const char* local);
  static std::string getAttr(const tinyxml2::XMLElement* el, const char* name);
  static std::string trim(const std::string& s);

  bool handleIfUnless(tinyxml2::XMLElement* el, std::vector<tinyxml2::XMLNode*>* inserted_out = nullptr);
  bool defineProperty(const tinyxml2::XMLElement* el);
  bool defineArg(const tinyxml2::XMLElement* el);
  bool defineMacro(tinyxml2::XMLElement* el);
  bool expandIncludesInNode(tinyxml2::XMLNode* node, const std::string& base_dir, std::string* error_msg);
  bool expandNode(tinyxml2::XMLNode* node);
  // Returns true if 'el' still exists and its children should be traversed;
  // returns false if 'el' was deleted/replaced and traversal should not use 'el'.
  bool expandElement(tinyxml2::XMLElement* el);
  bool expandMacroCall(tinyxml2::XMLElement* el);
  void substituteAttributes(tinyxml2::XMLElement* el);
};

} // namespace xacro_cpp
