#pragma once

#include <string>
#include <unordered_map>
#include <vector>

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
  };
  struct MacroDef {
    std::string name;
    tinyxml2::XMLNode* content; // points into doc_
    std::vector<MacroParam> params;
  };

  // Runs expansion and returns true on success. Result is serialized to output_path or stdout.
  bool run(const Options& opts, std::string* error_msg);

private:
  tinyxml2::XMLDocument* doc_ = nullptr; // owned
  std::unordered_map<std::string, std::string> vars_;
  std::unordered_map<std::string, MacroDef> macros_;
  std::string base_dir_;

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

  bool handleIfUnless(tinyxml2::XMLElement* el);
  bool defineProperty(const tinyxml2::XMLElement* el);
  bool defineArg(const tinyxml2::XMLElement* el);
  bool defineMacro(tinyxml2::XMLElement* el);
  bool expandIncludesInNode(tinyxml2::XMLNode* node, std::string* error_msg);
  bool expandNode(tinyxml2::XMLNode* node);
  bool expandElement(tinyxml2::XMLElement* el);
  bool expandMacroCall(tinyxml2::XMLElement* el);
  void substituteAttributes(tinyxml2::XMLElement* el);
};

} // namespace xacro_cpp
