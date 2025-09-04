#include "xacro_cpp/processor.hpp"

#include <tinyxml2.h>

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <cctype>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>
#include <system_error>
#if __has_include(<filesystem>)
  #include <filesystem>
  namespace fs = std::filesystem;
#endif

#if __has_include(<ament_index_cpp/get_package_share_directory.hpp>)
  #include <ament_index_cpp/get_package_share_directory.hpp>
  #define XACRO_HAVE_AMENT_INDEX 1
#endif

#include "xacro_cpp/tinyexpr.h"

namespace xacro_cpp {

static std::string dirname(const std::string& path) {
  auto pos = path.find_last_of("/\\");
  if (pos == std::string::npos) return std::string(".");
  return path.substr(0, pos);
}

// Current-package fallback for ${find('pkg')} when ament index is unavailable.
static std::string g_current_pkg_name;
static std::string g_current_pkg_root;

static void detect_current_package_from(const std::string& start_dir) {
  g_current_pkg_name.clear();
  g_current_pkg_root.clear();
#if defined(__cpp_lib_filesystem)
  fs::path p = start_dir;
  std::error_code ec;
  while (!p.empty()) {
    fs::path pkgxml = p / "package.xml";
    if (fs::exists(pkgxml, ec)) {
      tinyxml2::XMLDocument d;
      if (d.LoadFile(pkgxml.string().c_str()) == tinyxml2::XML_SUCCESS) {
        if (auto* root = d.RootElement()) {
          // Expect <package> then <name>
          auto* name_el = root->FirstChildElement("name");
          if (!name_el) {
            // Some package.xml may have <package format="3"> then <name>
            for (auto* c = root->FirstChildElement(); c; c = c->NextSiblingElement()) {
              if (std::string(c->Name()) == "name") { name_el = c; break; }
            }
          }
          if (name_el && name_el->GetText()) {
            g_current_pkg_name = name_el->GetText();
            g_current_pkg_root = p.string();
            return;
          }
        }
      }
    }
    p = p.parent_path();
  }
#endif
}

double eval_number(const std::string& expr,
                   const std::unordered_map<std::string, std::string>& vars,
                   bool* ok) {
  // Prepare variables as doubles when possible
  std::vector<te_variable> v;
  v.reserve(vars.size());
  std::vector<double> storage; storage.reserve(vars.size());
  for (auto& kv : vars) {
    char* end = nullptr;
    double val = std::strtod(kv.second.c_str(), &end);
    if (end && *end == '\0') {
      storage.push_back(val);
      te_variable t{kv.first.c_str(), &storage.back(), TE_VARIABLE, nullptr};
      v.push_back(t);
    }
  }
  int err = 0;
  te_expr* comp = te_compile(expr.c_str(), v.empty()?nullptr: v.data(), (int)v.size(), &err);
  if (err) {
    if (ok) *ok = false;
    if (comp) te_free(comp);
    return 0.0;
  }
  double res = te_eval(comp);
  te_free(comp);
  if (ok) *ok = true;
  return res;
}

bool eval_bool(const std::string& expr,
               const std::unordered_map<std::string, std::string>& vars) {
  bool ok = false;
  double v = eval_number(expr, vars, &ok);
  if (ok) return v != 0.0;
  // Fallback: non-empty string that is not "0" or "false"
  std::string t = expr;
  for (auto& c : t) c = (char)tolower(c);
  if (t == "false" || t == "0" || t.empty()) return false;
  return true;
}

static std::string replace_all(std::string s, const std::string& from, const std::string& to) {
  if (from.empty()) return s;
  size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.size(), to);
    pos += to.size();
  }
  return s;
}

static std::string strip_quotes(const std::string& s) {
  if (s.size() >= 2) {
    if ((s.front()=='"' && s.back()=='"') || (s.front()=='\'' && s.back()=='\'')) return s.substr(1, s.size()-2);
  }
  return s;
}

static std::string resolve_find(const std::string& pkg) {
#ifdef XACRO_HAVE_AMENT_INDEX
  try {
    return ament_index_cpp::get_package_share_directory(pkg);
  } catch (const std::exception&) {
    // fall through to env scan
  }
#endif
  // Fallback: scan AMENT_PREFIX_PATH or COLCON_PREFIX_PATH for share/pkg
  auto get_env = [](const char* name) -> std::string {
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
  };
  std::string paths = get_env("AMENT_PREFIX_PATH");
  if (paths.empty()) paths = get_env("COLCON_PREFIX_PATH");
  if (paths.empty()) return std::string();
  char sep = ':';
  size_t start = 0;
  while (start <= paths.size()) {
    size_t end = paths.find(sep, start);
    std::string prefix = (end==std::string::npos) ? paths.substr(start) : paths.substr(start, end-start);
    if (!prefix.empty()) {
#if defined(__cpp_lib_filesystem)
      fs::path p = fs::path(prefix) / "share" / pkg;
      std::error_code ec; if (fs::exists(p, ec)) return p.string();
#else
      // Lightweight existence check via trying to open a known file fails; skip
#endif
    }
    if (end==std::string::npos) break; else start = end+1;
  }
  // Last fallback: if requested package equals current package, use source root
  if (!g_current_pkg_name.empty() && pkg == g_current_pkg_name && !g_current_pkg_root.empty()) {
    return g_current_pkg_root;
  }
  // Fallback: search source workspace near current package for a matching package.xml
#if defined(__cpp_lib_filesystem)
  if (!g_current_pkg_root.empty()) {
    std::error_code ec;
    fs::path cur = g_current_pkg_root;
    // climb up to find a 'src' dir
    fs::path root = cur;
    while (!root.empty() && root.filename() != "src") root = root.parent_path();
    if (!root.empty() && fs::exists(root, ec) && fs::is_directory(root, ec)) {
      for (auto it = fs::recursive_directory_iterator(root, ec); it != fs::recursive_directory_iterator(); ++it) {
        if (ec) break;
        if (!it->is_regular_file()) continue;
        if (it->path().filename() != "package.xml") continue;
        // read package.xml name
        tinyxml2::XMLDocument d;
        if (d.LoadFile(it->path().string().c_str()) == tinyxml2::XML_SUCCESS) {
          auto* r = d.RootElement();
          if (!r) continue;
          tinyxml2::XMLElement* name_el = r->FirstChildElement("name");
          if (!name_el) {
            for (auto* c = r->FirstChildElement(); c; c = c->NextSiblingElement()) {
              if (std::string(c->Name()) == "name") { name_el = c; break; }
            }
          }
          if (name_el && name_el->GetText() && pkg == std::string(name_el->GetText())) {
            return it->path().parent_path().string();
          }
        }
      }
    }
  }
#endif
  return std::string();
}

std::string Processor::trim(const std::string& s) {
  size_t i = 0, j = s.size();
  while (i < j && isspace((unsigned char)s[i])) ++i;
  while (j > i && isspace((unsigned char)s[j - 1])) --j;
  return s.substr(i, j - i);
}

std::string Processor::getAttr(const tinyxml2::XMLElement* el, const char* name) {
  const char* v = el->Attribute(name);
  return v ? std::string(v) : std::string();
}

bool Processor::isXacroElement(const tinyxml2::XMLElement* el, const char* local) {
  const char* name = el->Name();
  std::string n = name ? name : "";
  if (n == local) return true;
  const std::string p = std::string("xacro:") + local;
  return n == p;
}

std::string eval_string_with_vars(const std::string& expr,
                                  const std::unordered_map<std::string, std::string>& vars) {
  // If numeric expression, evaluate
  bool ok = false;
  double num = eval_number(expr, vars, &ok);
  if (ok) {
    std::ostringstream oss; oss << num; return oss.str();
  }
  // Else, try exact variable replacement
  auto it = vars.find(expr);
  if (it != vars.end()) return it->second;
  return expr; // leave as-is
}

std::string eval_string_template(const std::string& text,
                                 const std::unordered_map<std::string, std::string>& vars) {
  auto trim_str = [](const std::string& s) -> std::string {
    size_t i = 0, j = s.size();
    while (i < j && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1]))) --j;
    return s.substr(i, j - i);
  };
  // Replace ${...} occurrences
  std::string out; out.reserve(text.size());
  for (size_t i = 0; i < text.size();) {
    if (text[i] == '$' && i + 1 < text.size() && text[i + 1] == '{') {
      size_t j = i + 2;
      int depth = 1;
      while (j < text.size() && depth > 0) {
        if (text[j] == '{') depth++;
        else if (text[j] == '}') depth--;
        if (depth == 0) break;
        ++j;
      }
      if (j < text.size() && text[j] == '}') {
        std::string expr = trim_str(text.substr(i + 2, j - (i + 2)));
        // Handle find('pkg') or find("pkg") or find(pkg)
        if (expr.rfind("find", 0) == 0) {
          std::string inside;
          size_t lp = expr.find('(');
          size_t rp = expr.find_last_of(')');
          if (lp != std::string::npos && rp != std::string::npos && rp > lp) {
            inside = trim_str(expr.substr(lp + 1, rp - lp - 1));
            inside = strip_quotes(inside);
          }
          std::string found = inside.empty() ? std::string() : resolve_find(inside);
          out += found;
          i = j + 1;
          continue;
        }
        std::string val = eval_string_with_vars(expr, vars);
        out += val;
        i = j + 1;
        continue;
      }
    }
    out.push_back(text[i]);
    ++i;
  }
  // First handle $(arg name) outside of ${...}
  // Do this before $(find ...) so nested patterns like $(find $(arg pkg)) work.
  try {
    std::regex re_arg("\\$\\(\\s*arg\\s+([^\\)]+)\\)");
    std::smatch m;
    std::string s = out;
    std::string result; result.reserve(s.size());
    std::string::const_iterator searchStart(s.cbegin());
    while (std::regex_search(searchStart, s.cend(), m, re_arg)) {
      result.append(m.prefix().first, m.prefix().second);
      std::string name = trim_str(std::string(m[1]));
      name = strip_quotes(name);
      auto it = vars.find(name);
      result += (it != vars.end() ? it->second : std::string());
      searchStart = m.suffix().first;
    }
    result.append(searchStart, s.cend());
    out = std::move(result);
  } catch (...) {
    // ignore and keep out as-is
  }

  // Also handle $(find pkg) style outside of ${...}
  try {
    std::regex re("\\$\\(\\s*find\\s+([^\\)]+)\\)");
    std::smatch m;
    std::string s = out;
    std::string result; result.reserve(s.size());
    std::string::const_iterator searchStart(s.cbegin());
    while (std::regex_search(searchStart, s.cend(), m, re)) {
      result.append(m.prefix().first, m.prefix().second);
      std::string pkg = trim_str(std::string(m[1]));
      pkg = strip_quotes(pkg);
      result += resolve_find(pkg);
      searchStart = m.suffix().first;
    }
    result.append(searchStart, s.cend());
    return result;
  } catch (...) {
    return out;
  }
}

Processor::Processor() {}
Processor::~Processor() { delete doc_; }

bool Processor::run(const Options& opts, std::string* error_msg) {
  base_dir_ = dirname(opts.input_path);
  vars_.clear(); macros_.clear();
  for (const auto& kv : opts.cli_args) vars_[kv.first] = kv.second;

  // Detect current package for ${find('<this_pkg>')} fallback in source trees.
  detect_current_package_from(base_dir_);

  if (!loadDocument(opts.input_path, error_msg)) return false;
  if (!processDocument(error_msg)) return false;

  tinyxml2::XMLPrinter printer;
  doc_->Print(&printer);
  std::string result = printer.CStr();

  if (opts.output_path.empty()) {
    std::cout << result;
  } else {
    std::ofstream ofs(opts.output_path);
    if (!ofs) { if (error_msg) *error_msg = "Failed to write output"; return false; }
    ofs << result;
  }
  return true;
}

bool Processor::loadDocument(const std::string& path, std::string* error_msg) {
  doc_ = new tinyxml2::XMLDocument();
  auto rc = doc_->LoadFile(path.c_str());
  if (rc != tinyxml2::XML_SUCCESS) {
    if (error_msg) *error_msg = std::string("Failed to load XML: ") + doc_->ErrorStr();
    return false;
  }
  return true;
}

bool Processor::processDocument(std::string* error_msg) {
  if (!passCollectArgsAndProps(error_msg)) return false;
  if (!passCollectMacros(error_msg)) return false;
  if (!passExpandIncludes(error_msg)) return false;
  if (!passExpand(error_msg)) return false;
  return true;
}

bool Processor::defineProperty(const tinyxml2::XMLElement* el) {
  std::string name = getAttr(el, "name");
  std::string value = getAttr(el, "value");
  if (value.empty()) {
    const char* text = el->GetText();
    if (text) value = text;
  }
  value = eval_string_template(value, vars_);
  if (!name.empty()) vars_[name] = value;
  return true;
}

bool Processor::defineArg(const tinyxml2::XMLElement* el) {
  std::string name = getAttr(el, "name");
  std::string def = getAttr(el, "default");
  if (vars_.find(name) == vars_.end()) vars_[name] = eval_string_template(def, vars_);
  return true;
}

bool Processor::passCollectArgsAndProps(std::string* /*error_msg*/) {
  std::vector<tinyxml2::XMLElement*> remove_args;
  for (auto* el = doc_->RootElement(); el; ) {
    // Iterate DFS on elements
    std::vector<tinyxml2::XMLElement*> stack;
    stack.push_back(doc_->RootElement());
    while (!stack.empty()) {
      auto* cur = stack.back(); stack.pop_back();
      if (isXacroElement(cur, "arg")) { defineArg(cur); remove_args.push_back(cur); }
      if (isXacroElement(cur, "property")) defineProperty(cur);
      auto* child = cur->LastChildElement();
      while (child) { stack.push_back(child); child = child->PreviousSiblingElement(); }
    }
    break;
  }
  // Remove xacro:arg elements from output after collecting
  for (auto* a : remove_args) {
    if (auto* p = a->Parent()) p->DeleteChild(a);
  }
  return true;
}

static std::vector<Processor::MacroParam> parse_params(const std::string& s) {
  std::vector<Processor::MacroParam> out;
  std::istringstream iss(s);
  std::string tok;
  while (iss >> tok) {
    Processor::MacroParam p;
    auto pos = tok.find(":=");
    if (pos != std::string::npos) {
      p.name = tok.substr(0, pos);
      p.default_value = tok.substr(pos + 2);
    } else {
      p.name = tok; p.default_value.clear();
    }
    out.push_back(p);
  }
  return out;
}

bool Processor::defineMacro(tinyxml2::XMLElement* el) {
  std::string name = getAttr(el, "name");
  std::string params = getAttr(el, "params");
  MacroDef def; def.name = name; def.params = parse_params(params);
  // Clone children of macro element into a container node we own in doc_
  tinyxml2::XMLNode* container = doc_->NewElement(("__macro__" + name).c_str());
  for (auto* child = el->FirstChild(); child; child = child->NextSibling()) {
    container->InsertEndChild(child->DeepClone(doc_));
  }
  def.content = container;
  macros_[name] = def;
  return true;
}

bool Processor::passCollectMacros(std::string* /*error_msg*/) {
  std::vector<tinyxml2::XMLElement*> to_remove;
  for (auto* el = doc_->RootElement(); el; ) {
    std::vector<tinyxml2::XMLElement*> stack; stack.push_back(doc_->RootElement());
    while (!stack.empty()) {
      auto* cur = stack.back(); stack.pop_back();
      if (isXacroElement(cur, "macro")) {
        defineMacro(cur); to_remove.push_back(cur);
      }
      auto* child = cur->LastChildElement();
      while (child) { stack.push_back(child); child = child->PreviousSiblingElement(); }
    }
    break;
  }
  for (auto* el : to_remove) {
    el->Parent()->DeleteChild(el);
  }
  return true;
}

static void insert_before(tinyxml2::XMLNode* parent, tinyxml2::XMLNode* ref, tinyxml2::XMLNode* child) {
  if (!parent || !ref || !child) return;
  tinyxml2::XMLNode* prev = ref->PreviousSibling();
  if (prev) parent->InsertAfterChild(prev, child);
  else parent->InsertFirstChild(child);
}

bool Processor::expandIncludesInNode(tinyxml2::XMLNode* node, std::string* error_msg) {
  for (auto* el = node->ToElement(); el; ) {
    // Depth-first traversal over elements
    std::vector<tinyxml2::XMLElement*> stack; stack.push_back(node->ToElement());
    while (!stack.empty()) {
      auto* cur = stack.back(); stack.pop_back();
      // Evaluate and splice/remove conditionals early so we don't expand includes in dead branches
      if (handleIfUnless(cur)) {
        // cur is deleted or replaced; skip pushing its children as structure changed
        continue;
      }
      if (isXacroElement(cur, "include")) {
        std::string file = getAttr(cur, "filename");
        file = eval_string_template(file, vars_);
        if (file.empty()) continue;
        std::string full;
#if defined(__cpp_lib_filesystem)
        fs::path pfile(file);
        if (pfile.is_absolute()) full = pfile.string();
        else full = (fs::path(base_dir_) / pfile).string();
#else
        if (!file.empty() && file[0] == '/') full = file;
        else full = base_dir_ + "/" + file;
#endif
        tinyxml2::XMLDocument inc;
        if (inc.LoadFile(full.c_str()) != tinyxml2::XML_SUCCESS) {
          if (error_msg) *error_msg = std::string("Failed to include ") + full;
          return false;
        }
        auto* root = inc.RootElement();
        if (!root) continue;
        // Insert children of included root at include position
        std::vector<tinyxml2::XMLNode*> cloned;
        for (auto* c = root->FirstChild(); c; c = c->NextSibling()) {
          cloned.push_back(c->DeepClone(doc_));
        }
        auto* parent = cur->Parent();
        for (auto* n : cloned) insert_before(parent, cur, n);
        parent->DeleteChild(cur);
        continue;
      }
      auto* child = cur->LastChildElement();
      while (child) { stack.push_back(child); child = child->PreviousSiblingElement(); }
    }
    break;
  }
  return true;
}

bool Processor::passExpandIncludes(std::string* error_msg) {
  return expandIncludesInNode(doc_->RootElement(), error_msg);
}

bool Processor::handleIfUnless(tinyxml2::XMLElement* el) {
  if (isXacroElement(el, "if") || isXacroElement(el, "unless")) {
    std::string cond = getAttr(el, "value");
    bool val = eval_bool(eval_string_template(cond, vars_), vars_);
    if (isXacroElement(el, "unless")) val = !val;
    auto* parent = el->Parent();
    if (!val) { parent->DeleteChild(el); return true; }
    // True: splice children
    std::vector<tinyxml2::XMLNode*> cloned;
    for (auto* c = el->FirstChild(); c; c = c->NextSibling()) cloned.push_back(c->DeepClone(doc_));
    for (auto* n : cloned) insert_before(parent, el, n);
    parent->DeleteChild(el);
    return true;
  }
  return false;
}

void Processor::substituteAttributes(tinyxml2::XMLElement* el) {
  // Replace ${...} in all attribute values
  const tinyxml2::XMLAttribute* a = el->FirstAttribute();
  std::vector<std::pair<std::string,std::string>> updates;
  while (a) {
    std::string name = a->Name();
    std::string val = a->Value();
    std::string newv = eval_string_template(val, vars_);
    if (newv != val) updates.emplace_back(name, newv);
    a = a->Next();
  }
  for (auto& kv : updates) el->SetAttribute(kv.first.c_str(), kv.second.c_str());
}

bool Processor::expandMacroCall(tinyxml2::XMLElement* el) {
  std::string name = el->Name();
  auto it = macros_.find(name);
  if (it == macros_.end()) return false;
  // Build local scope
  std::unordered_map<std::string, std::string> scope = vars_;
  const MacroDef& m = it->second;
  // Attribute arguments
  for (const auto& p : m.params) {
    std::string v;
    const char* av = el->Attribute(p.name.c_str());
    if (av) v = eval_string_template(av, scope);
    else v = eval_string_template(p.default_value, scope);
    scope[p.name] = v;
  }
  // Clone macro content with substitution
  std::vector<tinyxml2::XMLNode*> cloned;
  for (auto* c = m.content->FirstChild(); c; c = c->NextSibling()) {
    cloned.push_back(c->DeepClone(doc_));
  }
  auto* parent = el->Parent();
  for (auto* n : cloned) insert_before(parent, el, n);
  parent->DeleteChild(el);
  // After insertion, we must traverse inserted nodes and substitute
  for (auto* n : cloned) {
    // DFS
    std::vector<tinyxml2::XMLNode*> st; st.push_back(n);
    while (!st.empty()) {
      auto* cur = st.back(); st.pop_back();
      if (auto* ce = cur->ToElement()) {
        // Substitute attrs using local scope
        const tinyxml2::XMLAttribute* a = ce->FirstAttribute();
        std::vector<std::pair<std::string,std::string>> updates;
        while (a) {
          std::string name = a->Name();
          std::string val = a->Value();
          std::string newv = eval_string_template(val, scope);
          if (newv != val) updates.emplace_back(name, newv);
          a = a->Next();
        }
        for (auto& kv : updates) ce->SetAttribute(kv.first.c_str(), kv.second.c_str());
      }
      for (auto* c2 = cur->LastChild(); c2; c2 = c2->PreviousSibling()) st.push_back(c2);
    }
  }
  return true;
}

bool Processor::expandElement(tinyxml2::XMLElement* el) {
  if (handleIfUnless(el)) return true; // el is deleted or spliced
  if (expandMacroCall(el)) return true; // el replaced
  substituteAttributes(el);
  return true;
}

bool Processor::expandNode(tinyxml2::XMLNode* node) {
  // DFS traversal
  std::vector<tinyxml2::XMLNode*> st; st.push_back(node);
  while (!st.empty()) {
    auto* cur = st.back(); st.pop_back();
    if (auto* el = cur->ToElement()) {
      expandElement(el);
    }
    for (auto* c = cur->LastChild(); c; c = c->PreviousSibling()) st.push_back(c);
  }
  return true;
}

bool Processor::passExpand(std::string* /*error_msg*/) {
  return expandNode(doc_->RootElement());
}

} // namespace xacro_cpp
