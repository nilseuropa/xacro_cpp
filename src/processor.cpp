#include "xacro_cpp/processor.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_set>
#include <vector>
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

static constexpr char kDollarMarker = '\x1D'; // placeholder used to keep escaped '$' stable across passes
static constexpr std::string_view kXacroPrefix = "xacro:";

static std::string dirname(const std::string& path) {
  auto pos = path.find_last_of("/\\");
  if (pos == std::string::npos) {
    return std::string(".");
  }
  return path.substr(0, pos);
}

// Current-package fallback for ${find('pkg')} when ament index is unavailable.
static std::string g_current_pkg_name;
static std::string g_current_pkg_root;

namespace {

inline bool is_escaped(const std::string& text, size_t idx) {
  if (idx == 0) {
    return false;
  }
  size_t backslash_count = 0;
  size_t pos = idx;
  while (pos > 0 && text[--pos] == '\\') {
    ++backslash_count;
  }
  return (backslash_count % 2) == 1;
}

static void ensure_balanced_quotes(const std::string& expr) {
  bool in_single = false;
  bool in_double = false;
  for (size_t i = 0; i < expr.size(); ++i) {
    char c = expr[i];
    if (c == '\'' && !in_double && !is_escaped(expr, i)) {
      in_single = !in_single;
    } else if (c == '"' && !in_single && !is_escaped(expr, i)) {
      in_double = !in_double;
    }
  }
  if (in_single || in_double) {
    throw ProcessingError("Unterminated string literal inside expression: " + expr);
  }
}

static void ensure_no_pow_operator(const std::string& expr) {
  bool in_single = false;
  bool in_double = false;
  for (size_t i = 0; i + 1 < expr.size(); ++i) {
    char c = expr[i];
    if (c == '\'' && !in_double && !is_escaped(expr, i)) {
      in_single = !in_single;
      continue;
    }
    if (c == '"' && !in_single && !is_escaped(expr, i)) {
      in_double = !in_double;
      continue;
    }
    if (!in_single && !in_double && c == '*' && expr[i + 1] == '*') {
      throw ProcessingError("Unsupported operator '**' in expression: " + expr);
    }
  }
}

static void validate_expression_chunk(const std::string& expr) {
  if (expr.empty()) {
    return;
  }
  ensure_balanced_quotes(expr);
  ensure_no_pow_operator(expr);
}

[[noreturn]] void throw_processing_error(const std::string& message) {
  if (message.empty()) {
    throw ProcessingError("xacro_cpp processing failed");
  }
  throw ProcessingError(message);
}

inline void throw_from_error_msg(std::string* error_msg, const char* fallback) {
  if (error_msg) {
    if (error_msg->empty() && fallback) {
      *error_msg = fallback;
    }
    throw_processing_error(*error_msg);
  }
  throw_processing_error(fallback ? std::string(fallback) : std::string());
}

} // namespace

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
              if (std::string(c->Name()) == "name") {
                name_el = c;
                break;
              }
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

// Forward decl
static std::string replace_indexing_with_values(const std::string& expr,
                                                const std::unordered_map<std::string, std::string>& vars);

double eval_number(const std::string& expr, const std::unordered_map<std::string, std::string>& vars, bool* ok) {
  // Prepare variables as doubles when possible
  std::vector<te_variable> v;
  v.reserve(vars.size());
  std::vector<double> storage;
  storage.reserve(vars.size() + 1);
  // Inject common constants
  static const double s_pi = 3.14159265358979323846;
  storage.push_back(s_pi);
  v.push_back(te_variable{"pi", &storage.back(), TE_VARIABLE, nullptr});
  auto to_lower = [](std::string s) {
    for (auto& c : s)
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
  };
  for (auto& kv : vars) {
    char* end = nullptr;
    double val = std::strtod(kv.second.c_str(), &end);
    bool consumed = (end && *end == '\0');
    if (!consumed) {
      std::string lowered = to_lower(kv.second);
      if (lowered == "true") {
        val = 1.0;
        consumed = true;
      } else if (lowered == "false") {
        val = 0.0;
        consumed = true;
      }
    }
    if (consumed) {
      storage.push_back(val);
      te_variable t{kv.first.c_str(), &storage.back(), TE_VARIABLE, nullptr};
      v.push_back(t);
    }
  }
  int err = 0;
  std::string ex = replace_indexing_with_values(expr, vars);
  te_expr* comp = te_compile(ex.c_str(), v.empty() ? nullptr : v.data(), (int)v.size(), &err);
  if (err) {
    if (ok) {
      *ok = false;
    }
    if (comp) {
      te_free(comp);
    }
    return 0.0;
  }
  double res = te_eval(comp);
  te_free(comp);
  if (!std::isfinite(res)) {
    if (ok) {
      *ok = false;
    }
    throw ProcessingError("division by zero when evaluating expression '" + expr + "'");
  }
  if (ok) {
    *ok = true;
  }
  return res;
}

bool eval_bool(const std::string& expr, const std::unordered_map<std::string, std::string>& vars) {
  // Normalize optional ${...} wrapper (common in xacro:if/unless).
  auto trim = [](const std::string& s) -> std::string {
    size_t i = 0, j = s.size();
    while (i < j && std::isspace(static_cast<unsigned char>(s[i])))
      ++i;
    while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1])))
      --j;
    return s.substr(i, j - i);
  };
  std::string expr_trim = trim(expr);
  if (expr_trim.size() >= 3 && expr_trim[0] == '$' && expr_trim[1] == '{' && expr_trim.back() == '}') {
    expr_trim = trim(expr_trim.substr(2, expr_trim.size() - 3));
  }
  // Fast path for textual booleans
  auto trim_lower = [](std::string s) {
    size_t i = 0, j = s.size();
    while (i < j && std::isspace(static_cast<unsigned char>(s[i])))
      ++i;
    while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1])))
      --j;
    s = s.substr(i, j - i);
    for (auto& c : s)
      c = (char)std::tolower((unsigned char)c);
    return s;
  };
  std::string t = trim_lower(expr_trim);
  if (t == "true") {
    return true;
  }
  if (t == "false") {
    return false;
  }
  // Numeric expression
  bool ok = false;
  double v = eval_number(expr_trim, vars, &ok);
  if (ok) {
    return v != 0.0;
  }
  // Fallback: comparison operators with identifiers, quoted strings or numeric literals
  try {
    static const std::regex re_cmp(R"(^\s*(.*?)\s*(==|!=|<=|>=|<|>)\s*(.*?)\s*$)");
    std::smatch m;
    if (std::regex_match(expr_trim, m, re_cmp)) {
      auto resolve = [&](std::string s) -> std::string {
        // local trim
        size_t i = 0, j = s.size();
        while (i < j && std::isspace(static_cast<unsigned char>(s[i])))
          ++i;
        while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1])))
          --j;
        s = s.substr(i, j - i);
        if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\''))) {
          return s.substr(1, s.size() - 2);
        }
        // identifier lookup
        if (!s.empty() && (std::isalpha((unsigned char)s[0]) || s[0] == '_')) {
          auto it = vars.find(s);
          if (it != vars.end()) {
            return it->second;
          }
          else {
            return s;
          }
        }
        return s;
      };
      std::string ls = resolve(std::string(m[1]));
      std::string op = m[2];
      std::string rs = resolve(std::string(m[3]));
      // Try numeric compare if both numeric
      auto is_num = [](const std::string& s, double* out) -> bool {
        if (s.empty()) {
          return false;
        }
        char* end = nullptr;
        double d = strtod(s.c_str(), &end);
        if (end && *end == '\0') {
          if (out) {
            *out = d;
          }
          return true;
        }
        return false;
      };
      double ln = 0, rn = 0;
      bool ln_ok = is_num(ls, &ln), rn_ok = is_num(rs, &rn);
      if (ln_ok && rn_ok) {
        if (op == "==") {
          return ln == rn;
        }
        if (op == "!=") {
          return ln != rn;
        }
        if (op == "<") {
          return ln < rn;
        }
        if (op == "<=") {
          return ln <= rn;
        }
        if (op == "<") {
          return ln < rn;
        }
        if (op == "<") {
          return ln < rn;
        }
        if (op == "<=") {
          return ln <= rn;
        }
        if (op == "<") {
          return ln < rn;
        }
        if (op == "<") {
          return ln < rn;
        }
        if (op == "<") {
          return ln < rn;
        }
        if (op == "<") {
          return ln < rn;
        }
        if (op == ">=") {
          return ln >= rn;
        }
        if (op == ">") {
          return ln > rn;
        }
      } else {
        if (op == "==") {
          return ls == rs;
        }
        if (op == "!=") {
          return ls != rs;
        }
        // For non-numeric <,>,<=,>= treat lexicographically
        if (op == "<") {
          return ls < rs;
        }
        if (op == "<=") {
          return ls <= rs;
        }
        if (op == ">") {
          return ls > rs;
        }
        if (op == ">=") {
          return ls >= rs;
        }
      }
    }
  } catch (...) {
  }
  // Fallback: non-empty string that is not "0" or "false"
  if (t == "0" || t.empty()) {
    return false;
  }
  return true;
}

static std::string replace_all(std::string s, const std::string& from, const std::string& to) {
  if (from.empty()) {
    return s;
  }
  size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.size(), to);
    pos += to.size();
  }
  return s;
}

static std::string strip_quotes(const std::string& s) {
  if (s.size() >= 2) {
    if ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\'')) {
      return s.substr(1, s.size() - 2);
    }
  }
  return s;
}

static std::string trim_copy(const std::string& s) {
  size_t i = 0, j = s.size();
  while (i < j && std::isspace(static_cast<unsigned char>(s[i])))
    ++i;
  while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1])))
    --j;
  return s.substr(i, j - i);
}

static bool is_simple_identifier(const std::string& s) {
  if (s.empty()) {
    return false;
  }
  if (!(std::isalpha(static_cast<unsigned char>(s[0])) || s[0] == '_')) {
    return false;
  }
  for (size_t i = 1; i < s.size(); ++i) {
    char c = s[i];
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_')) {
      return false;
    }
  }
  return true;
}

static bool has_unknown_identifier(const std::string& expr, const std::unordered_map<std::string, std::string>& vars) {
  static const std::unordered_set<std::string> kBuiltins
    = {"sin",  "cos",   "tan",   "asin", "acos", "atan", "atan2", "sqrt", "abs", "fabs", "ln", "log",  "exp",
       "floor", "ceil", "round", "trunc", "sinh", "cosh", "tanh", "pow",  "min", "max", "pi",  "true", "false"};
  bool in_single = false;
  bool in_double = false;
  for (size_t i = 0; i < expr.size();) {
    char c = expr[i];
    if (c == '\'' && !in_double && !is_escaped(expr, i)) {
      in_single = !in_single;
      ++i;
      continue;
    }
    if (c == '"' && !in_single && !is_escaped(expr, i)) {
      in_double = !in_double;
      ++i;
      continue;
    }
    if (in_single || in_double) {
      ++i;
      continue;
    }
    if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
      size_t j = i + 1;
      while (j < expr.size()) {
        char cj = expr[j];
        if (std::isalnum(static_cast<unsigned char>(cj)) || cj == '_') {
          ++j;
        }
        else {
          break;
        }
      }
      std::string ident = expr.substr(i, j - i);
      if (!kBuiltins.count(ident) && vars.find(ident) == vars.end()) {
        return true;
      }
      i = j;
    } else {
      ++i;
    }
  }
  return false;
}

static bool split_list_literal(const std::string& expr, std::vector<std::string>* items) {
  if (!items) {
    return false;
  }
  std::string trimmed = trim_copy(expr);
  if (trimmed.size() < 2 || trimmed.front() != '[' || trimmed.back() != ']') {
    return false;
  }

  std::string inner = trimmed.substr(1, trimmed.size() - 2);
  std::vector<std::string> tokens;
  std::string current;
  int depth = 0;
  bool in_single = false;
  bool in_double = false;
  auto flush = [&]() {
    std::string token = trim_copy(current);
    if (!token.empty()) {
      tokens.push_back(token);
    }
    current.clear();
  };
  for (size_t idx = 0; idx < inner.size(); ++idx) {
    char c = inner[idx];
    if (c == ',' && depth == 0 && !in_single && !in_double) {
      flush();
      continue;
    }
    current.push_back(c);
    if (c == '\'' && !in_double && !is_escaped(inner, idx)) {
      in_single = !in_single;
    } else if (c == '"' && !in_single && !is_escaped(inner, idx)) {
      in_double = !in_double;
    } else if (!in_single && !in_double) {
      if (c == '[' || c == '{' || c == '(') {
        ++depth;
      }
      else if (c == ']' || c == '}' || c == ')') {
        if (depth > 0) {
          --depth;
        }
      }
    }
  }
  if (in_single || in_double || depth != 0) {
    return false;
  }
  flush();
  *items = tokens;
  return true;
}

// Replace occurrences of name[index] with the indexed token from vars[name]
static std::string replace_indexing_with_values(const std::string& expr,
                                                const std::unordered_map<std::string, std::string>& vars) {
  try {
    std::regex re(R"(([A-Za-z_][A-Za-z0-9_]*)\s*\[\s*([0-9]+)\s*\])");
    std::smatch m;
    std::string s = expr;
    std::string result;
    result.reserve(s.size());
    std::string::const_iterator searchStart(s.cbegin());
    while (std::regex_search(searchStart, s.cend(), m, re)) {
      result.append(m.prefix().first, m.prefix().second);
      std::string name = std::string(m[1]);
      size_t idx = static_cast<size_t>(std::stoul(std::string(m[2])));
      std::string value;
      auto it = vars.find(name);
      if (it != vars.end()) {
        std::vector<std::string> tokens;
        if (!split_list_literal(it->second, &tokens)) {
          std::istringstream iss(it->second);
          std::string tok;
          while (iss >> tok) {
            while (!tok.empty() && (tok.back() == ',' || tok.back() == ';')) {
              tok.pop_back();
            }
            if (!tok.empty()) {
              tokens.push_back(tok);
            }
          }
        }
        if (idx < tokens.size()) {
          value = tokens[idx];
        }
      }
      result += strip_quotes(trim_copy(value));
      searchStart = m.suffix().first;
    }
    result.append(searchStart, s.cend());
    return result;
  } catch (...) {
    return expr;
  }
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
  if (paths.empty()) {
    paths = get_env("COLCON_PREFIX_PATH");
  }
  if (paths.empty()) {
    return std::string();
  }
  char sep = ':';
  size_t start = 0;
  while (start <= paths.size()) {
    size_t end = paths.find(sep, start);
    std::string prefix = (end == std::string::npos) ? paths.substr(start) : paths.substr(start, end - start);
    if (!prefix.empty()) {
#if defined(__cpp_lib_filesystem)
      fs::path p = fs::path(prefix) / "share" / pkg;
      std::error_code ec;
      if (fs::exists(p, ec)) {
        return p.string();
      }
#else
      // Lightweight existence check via trying to open a known file fails; skip
#endif
    }
    if (end == std::string::npos) {
      break;
    }
    else {
      start = end + 1;
    }
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
    while (!root.empty() && root.filename() != "src")
      root = root.parent_path();
    if (!root.empty() && fs::exists(root, ec) && fs::is_directory(root, ec)) {
      for (auto it = fs::recursive_directory_iterator(root, ec); it != fs::recursive_directory_iterator(); ++it) {
        if (ec) {
          break;
        }
        if (!it->is_regular_file()) {
          continue;
        }
        if (it->path().filename() != "package.xml") {
          continue;
        }
        // read package.xml name
        tinyxml2::XMLDocument d;
        if (d.LoadFile(it->path().string().c_str()) == tinyxml2::XML_SUCCESS) {
          auto* r = d.RootElement();
          if (!r) {
            continue;
          }
          tinyxml2::XMLElement* name_el = r->FirstChildElement("name");
          if (!name_el) {
            for (auto* c = r->FirstChildElement(); c; c = c->NextSiblingElement()) {
              if (std::string(c->Name()) == "name") {
                name_el = c;
                break;
              }
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

// Very small YAML subset parser for maps of numeric lists/scalars.
// Supports patterns like:
//   key: [1.0, 2, 3]
//   key: 1.23
//   key:
//     - 1
//     - 2.0
static std::unordered_map<std::string, std::vector<double>> parse_simple_yaml_map(const std::string& path) {
  std::unordered_map<std::string, std::vector<double>> out;
  std::ifstream ifs(path);
  if (!ifs) {
    return out;
  }
  auto ltrim = [](const std::string& s) {
    size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i])))
      ++i;
    return s.substr(i);
  };
  auto rtrim = [](const std::string& s) {
    if (s.empty()) {
      return s;
    }
    size_t j = s.size();
    while (j > 0 && std::isspace(static_cast<unsigned char>(s[j - 1])))
      --j;
    return s.substr(0, j);
  };
  auto trim = [&](const std::string& s) {
    return rtrim(ltrim(s));
  };
  auto parse_list = [&](const std::string& s) {
    std::vector<double> vec;
    std::string t = trim(s);
    size_t i = 0;
    while (i < t.size()) {
      // skip commas and spaces
      while (i < t.size() && (t[i] == ',' || std::isspace(static_cast<unsigned char>(t[i]))))
        ++i;
      if (i >= t.size()) {
        break;
      }
      size_t j = i;
      while (j < t.size() && t[j] != ',')
        ++j;
      std::string item = trim(t.substr(i, j - i));
      if (!item.empty()) {
        try {
          vec.push_back(std::stod(item));
        } catch (...) {
        }
      }
      i = j + 1;
    }
    return vec;
  };
  std::string line;
  int last_indent = 0;
  std::string pending_key;
  std::vector<double> pending_list;
  bool in_list = false;
  while (std::getline(ifs, line)) {
    // strip comments
    auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    std::string raw = line;
    std::string s = rtrim(raw);
    if (s.empty()) {
      continue;
    }
    // compute indent
    int indent = 0;
    while (indent < (int)raw.size() && std::isspace(static_cast<unsigned char>(raw[indent])))
      ++indent;
    std::string t = ltrim(raw);
    if (in_list) {
      if (indent <= last_indent) {
        // end of list
        out[pending_key] = pending_list;
        pending_list.clear();
        in_list = false;
        pending_key.clear();
      } else {
        // expect list item
        std::string lt = trim(t);
        if (!lt.empty() && lt[0] == '-') {
          std::string item = trim(lt.substr(1));
          try {
            pending_list.push_back(std::stod(item));
          } catch (...) {
          }
          continue;
        } else {
          // unexpected, end list
          out[pending_key] = pending_list;
          pending_list.clear();
          in_list = false;
          pending_key.clear();
        }
      }
    }
    // key: value
    std::smatch m;
    static std::regex re_inline(R"(^([A-Za-z_][A-Za-z0-9_\-]*)\s*:\s*(.*)\s*$)");
    if (std::regex_match(t, m, re_inline)) {
      std::string key = std::string(m[1]);
      std::string val = std::string(m[2]);
      if (!val.empty() && val.front() == '[' && val.back() == ']') {
        val = val.substr(1, val.size() - 2);
        out[key] = parse_list(val);
      } else if (val.empty()) {
        // maybe a multiline list follows
        in_list = true;
        last_indent = indent;
        pending_key = key;
        pending_list.clear();
      } else {
        try {
          out[key] = std::vector<double>{std::stod(trim(val))};
        } catch (...) {
        }
      }
    }
  }
  if (in_list && !pending_key.empty()) {
    out[pending_key] = pending_list;
  }
  return out;
}

std::string Processor::trim(const std::string& s) {
  size_t i = 0, j = s.size();
  while (i < j && isspace((unsigned char)s[i]))
    ++i;
  while (j > i && isspace((unsigned char)s[j - 1]))
    --j;
  return s.substr(i, j - i);
}

std::string Processor::getAttr(const tinyxml2::XMLElement* el, const char* name) {
  const char* v = el->Attribute(name);
  return v ? std::string(v) : std::string();
}

bool Processor::isXacroElement(const tinyxml2::XMLElement* el, const char* local) {
  const char* name = el->Name();
  std::string n = name ? name : "";
  if (n == local) {
    return true;
  }
  const std::string p = std::string("xacro:") + local;
  return n == p;
}

std::string eval_string_with_vars(const std::string& expr,
                                  const std::unordered_map<std::string, std::string>& vars,
                                  bool* resolved = nullptr) {
  if (resolved) {
    *resolved = false;
  }
  // Prefer exact variable replacement in the given scope first
  auto it = vars.find(expr);
  if (it != vars.end()) {
    if (resolved) {
      *resolved = true;
    }
    return it->second;
  }
  if (has_unknown_identifier(expr, vars)) {
    return expr;
  }
  // Else, if numeric expression, evaluate
  bool ok = false;
  double num = eval_number(expr, vars, &ok);
  if (ok) {
    if (resolved) {
      *resolved = true;
    }
    if (num == 0.0) {
      num = 0.0; // normalize -0.0 to 0.0
    }
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10) << num;
    return oss.str();
  }
  return expr; // leave as-is
}

std::string eval_string_template(const std::string& text, const std::unordered_map<std::string, std::string>& vars) {
  // Handle escaped dollars first: each "$$" becomes a marker for a literal "$"
  auto escape_dollars = [](const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
      if (in[i] == '$' && i + 1 < in.size() && in[i + 1] == '$') {
        out.push_back(kDollarMarker);
        ++i; // skip second '$'
      } else {
        out.push_back(in[i]);
      }
    }
    return out;
  };
  std::string work = escape_dollars(text);

  auto trim_str = [](const std::string& s) -> std::string {
    size_t i = 0, j = s.size();
    while (i < j && std::isspace(static_cast<unsigned char>(s[i])))
      ++i;
    while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1])))
      --j;
    return s.substr(i, j - i);
  };
  // Replace ${...} occurrences
  std::string out;
  out.reserve(work.size());
  for (size_t i = 0; i < work.size();) {
    if (work[i] == '$' && i + 1 < work.size() && work[i + 1] == '{') {
      size_t j = i + 2;
      int depth = 1;
      while (j < work.size() && depth > 0) {
        if (work[j] == '{') {
          depth++;
        }
        else if (work[j] == '}') {
          depth--;
        }
        if (depth == 0) {
          break;
        }
        ++j;
      }
      if (j >= work.size() || work[j] != '}') {
        std::string snippet = work.substr(i, std::min<size_t>(work.size() - i, 80));
        throw ProcessingError("Unterminated ${...} expression near: " + snippet);
      }
      if (j < work.size() && work[j] == '}') {
        std::string raw_expr = work.substr(i, j - i + 1);
        std::string expr = trim_str(work.substr(i + 2, j - (i + 2)));
        validate_expression_chunk(expr);
        // Handle inline list literals like [a, b, c]
        std::vector<std::string> list_expr_items;
        if (split_list_literal(expr, &list_expr_items)) {
          bool list_resolved = true;
          std::ostringstream oss;
          oss << "[";
          for (size_t idx_item = 0; idx_item < list_expr_items.size(); ++idx_item) {
            std::string item_expr = replace_indexing_with_values(list_expr_items[idx_item], vars);
            bool item_resolved = false;
            std::string item_val = eval_string_with_vars(trim_copy(item_expr), vars, &item_resolved);
            if (!item_resolved) {
              list_resolved = false;
              break;
            }
            if (idx_item) {
              oss << ", ";
            }
            oss << item_val;
          }
          if (list_resolved) {
            oss << "]";
            out += oss.str();
            i = j + 1;
            continue;
          }
        }
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
        std::string ex2 = replace_indexing_with_values(expr, vars);
        bool resolved = false;
        std::string val = eval_string_with_vars(ex2, vars, &resolved);
        if (!resolved) {
          out += raw_expr;
        } else {
          out += val;
        }
        i = j + 1;
        continue;
      }
    }
    out.push_back(work[i]);
    ++i;
  }
  // First handle $(arg name) outside of ${...}
  // Do this before $(find ...) so nested patterns like $(find $(arg pkg)) work.
  std::regex re_arg("\\$\\(\\s*arg\\s+([^\\)]+)\\)");
  std::smatch m;
  std::string s = out;
  std::string result;
  result.reserve(s.size());
  std::string::const_iterator searchStart(s.cbegin());
  while (std::regex_search(searchStart, s.cend(), m, re_arg)) {
    result.append(m.prefix().first, m.prefix().second);
    std::string name = trim_str(std::string(m[1]));
    name = strip_quotes(name);
    auto it = vars.find(name);
    if (it == vars.end()) {
      throw ProcessingError("Undefined substitution argument '" + name + "'");
    }
    result += it->second;
    searchStart = m.suffix().first;
  }
  result.append(searchStart, s.cend());
  out = std::move(result);

  // Also handle $(find pkg) style outside of ${...}
  try {
    std::regex re("\\$\\(\\s*find\\s+([^\\)]+)\\)");
    std::smatch m;
    std::string s = out;
    std::string result;
    result.reserve(s.size());
    std::string::const_iterator searchStart(s.cbegin());
    while (std::regex_search(searchStart, s.cend(), m, re)) {
      result.append(m.prefix().first, m.prefix().second);
      std::string pkg = trim_str(std::string(m[1]));
      pkg = strip_quotes(pkg);
      result += resolve_find(pkg);
      searchStart = m.suffix().first;
    }
    result.append(searchStart, s.cend());
    out = std::move(result);
  } catch (...) {
    // leave 'out' untouched
  }

  // Restore escaped dollars after all substitutions/checks.
  return out;
}

Processor::Processor() {}
Processor::~Processor() {
  delete doc_;
}

bool Processor::run(const Options& opts, std::string* error_msg) {
  base_dir_ = dirname(opts.input_path);
  vars_.clear();
  macros_.clear();
  for (const auto& kv : opts.cli_args)
    vars_[kv.first] = kv.second;

  // Detect current package for ${find('<this_pkg>')} fallback in source trees.
  detect_current_package_from(base_dir_);

  if (!loadDocument(opts.input_path, error_msg)) {
    throw_from_error_msg(error_msg, "Failed to load XML");
  }
  if (!processDocument(error_msg)) {
    throw_from_error_msg(error_msg, "Failed to process document");
  }

  tinyxml2::XMLPrinter printer;
  doc_->Print(&printer);
  std::string result = printer.CStr();

  if (opts.output_path.empty()) {
    std::cout << result;
  } else {
    std::ofstream ofs(opts.output_path);
    if (!ofs) {
      if (error_msg) {
        *error_msg = "Failed to write output";
      }
      throw_from_error_msg(error_msg, "Failed to write output");
    }
    ofs << result;
  }
  return true;
}

bool Processor::runToString(const Options& opts, std::string* urdf_xml, std::string* error_msg) {
  base_dir_ = dirname(opts.input_path);
  vars_.clear();
  macros_.clear();
  arg_names_.clear();
  for (const auto& kv : opts.cli_args)
    vars_[kv.first] = kv.second;

  detect_current_package_from(base_dir_);

  if (!loadDocument(opts.input_path, error_msg)) {
    throw_from_error_msg(error_msg, "Failed to load XML");
  }
  if (!processDocument(error_msg)) {
    throw_from_error_msg(error_msg, "Failed to process document");
  }

  tinyxml2::XMLPrinter printer;
  doc_->Print(&printer);
  if (urdf_xml) {
    *urdf_xml = printer.CStr();
  }
  return true;
}

bool Processor::collectArgs(const Options& opts, std::map<std::string, std::string>* args_out, std::string* error_msg) {
  if (!args_out) {
    return false;
  }
  args_out->clear();
  base_dir_ = dirname(opts.input_path);
  arg_names_.clear();
  if (!loadDocument(opts.input_path, error_msg)) {
    throw_from_error_msg(error_msg, "Failed to load XML");
  }
  // initialize argument map with CLI overrides
  vars_.clear();
  macros_.clear();
  for (const auto& kv : opts.cli_args)
    vars_[kv.first] = kv.second;
  // Only collect args and properties; do not expand macros/includes
  if (!passCollectArgsAndProps(error_msg)) {
    throw_from_error_msg(error_msg, "Failed to collect args and properties");
  }
  // Copy collected arg values (only those declared via xacro:arg)
  for (const auto& name : arg_names_) {
    auto it = vars_.find(name);
    if (it != vars_.end()) {
      (*args_out)[name] = it->second;
    }
  }
  return true;
}

bool Processor::loadDocument(const std::string& path, std::string* error_msg) {
  doc_ = new tinyxml2::XMLDocument();
  auto rc = doc_->LoadFile(path.c_str());
  if (rc != tinyxml2::XML_SUCCESS) {
    if (error_msg) {
      *error_msg = std::string("Failed to load XML: ") + doc_->ErrorStr();
    }
    return false;
  }
  // xacro ignores comments; strip them up front to simplify later passes.
  removeComments(doc_);
  return true;
}

bool Processor::processDocument(std::string* error_msg) {
  // 1) collect args/props that may affect include paths
  if (!passCollectArgsAndProps(error_msg)) {
    return false;
  }
  // 2) expand includes (uses current vars and resolves ${}/$())
  if (!passExpandIncludes(error_msg)) {
    return false;
  }
  // 3) collect args/props again (includes might have added more)
  if (!passCollectArgsAndProps(error_msg)) {
    return false;
  }
  // 4) collect macros after all includes are inlined
  if (!passCollectMacros(error_msg)) {
    return false;
  }
  // 5) expand remaining xacro constructs (macros, if/unless, substitutions)
  if (!passExpand(error_msg)) {
    return false;
  }
  // Convert escaped dollar markers back to literal '$' in final output tree.
  restoreDollarMarkers(doc_->RootElement());
  return true;
}

bool Processor::defineProperty(const tinyxml2::XMLElement* el) {
  std::string name = getAttr(el, "name");
  std::string value = getAttr(el, "value");
  if (value.empty()) {
    const char* text = el->GetText();
    if (text) {
      value = text;
    }
  }
  auto trim_local = [](const std::string& s) -> std::string {
    size_t i = 0, j = s.size();
    while (i < j && std::isspace(static_cast<unsigned char>(s[i])))
      ++i;
    while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1])))
      --j;
    return s.substr(i, j - i);
  };
  std::string expr = trim_local(value);
  if (expr.size() >= 3 && expr[0] == '$' && expr[1] == '{' && expr.back() == '}') {
    expr = trim_local(expr.substr(2, expr.size() - 3));
  }
  // Handle xacro.load_yaml(file)
  {
    std::smatch m;
    static std::regex re_load(R"(^\s*xacro\.load_yaml\((.*)\)\s*$)");
    if (!name.empty() && std::regex_match(expr, m, re_load)) {
      std::string arg = Processor::trim(std::string(m[1]));
      std::string arg_eval = eval_string_template(arg, vars_);
      if (arg_eval == arg) {
        auto it = vars_.find(arg);
        if (it != vars_.end()) {
          arg_eval = it->second;
        }
      }
      arg = strip_quotes(arg_eval);
      std::string full;
#if defined(__cpp_lib_filesystem)
      fs::path p(arg);
      if (p.is_absolute()) {
        full = p.string();
      }
      else {
        full = (fs::path(base_dir_) / p).string();
      }
#else
      full = (!arg.empty() && arg[0] == '/') ? arg : (base_dir_ + "/" + arg);
#endif
      yaml_maps_[name] = parse_simple_yaml_map(full);
      // store a marker or path for reference (not used later directly)
      vars_[name] = full;
      return true;
    }
  }
  // Handle lookup of YAML key: somevar['key']
  {
    std::smatch m;
    static std::regex re_get(R"(^\s*([A-Za-z_][A-Za-z0-9_]*)\s*\[\s*(?:'([^']+)'|\"([^\"]+)\")\s*\]\s*$)");
    if (!name.empty() && std::regex_match(expr, m, re_get)) {
      std::string container = std::string(m[1]);
      std::string key = m[2].matched ? std::string(m[2]) : std::string(m[3]);
      auto it = yaml_maps_.find(container);
      if (it != yaml_maps_.end()) {
        auto it2 = it->second.find(key);
        if (it2 != it->second.end()) {
          std::ostringstream oss;
          if (it2->second.size() == 1) {
            oss << it2->second.front();
          } else {
            oss << "[";
            for (size_t i = 0; i < it2->second.size(); ++i) {
              if (i) {
                oss << ", ";
              }
              oss << it2->second[i];
            }
            oss << "]";
          }
          vars_[name] = oss.str();
          return true;
        }
      }
      // not found: set empty
      vars_[name].clear();
      return true;
    }
  }
  value = eval_string_template(value, vars_);
  // Normalize simple list literals so later indexing sees evaluated entries.
  if (!value.empty()) {
    std::vector<std::string> list_items;
    if (split_list_literal(value, &list_items)) {
      std::ostringstream oss;
      oss << "[";
      for (size_t i = 0; i < list_items.size(); ++i) {
        std::string resolved = Processor::trim(eval_string_with_vars(list_items[i], vars_));
        if (i) {
          oss << ", ";
        }
        oss << resolved;
      }
      oss << "]";
      value = oss.str();
    }
  }
  if (!name.empty()) {
    vars_[name] = value;
  }
  return true;
}

bool Processor::defineArg(const tinyxml2::XMLElement* el) {
  std::string name = getAttr(el, "name");
  std::string def = getAttr(el, "default");
  if (!name.empty()) {
    arg_names_.insert(name);
  }
  if (vars_.find(name) == vars_.end()) {
    vars_[name] = eval_string_template(def, vars_);
  }
  return true;
}

bool Processor::passCollectArgsAndProps(std::string* /*error_msg*/) {
  std::vector<tinyxml2::XMLElement*> remove_args;
  std::vector<tinyxml2::XMLElement*> remove_props;
  struct Frame {
    tinyxml2::XMLElement* el;
    bool in_macro;
    bool in_conditional;
  };
  std::vector<Frame> stack;
  stack.push_back({doc_->RootElement(), false, false});
  while (!stack.empty()) {
    Frame f = stack.back();
    stack.pop_back();
    auto* cur = f.el;
    bool in_macro = f.in_macro;
    bool in_conditional = f.in_conditional;
    if (isXacroElement(cur, "arg")) {
      if (!in_macro && !in_conditional) {
        defineArg(cur);
        remove_args.push_back(cur);
      }
    }
    if (isXacroElement(cur, "property")) {
      if (!in_macro && !in_conditional) {
        defineProperty(cur);
        remove_props.push_back(cur);
      }
    }
    auto* child = cur->LastChildElement();
    bool child_in_macro = in_macro || isXacroElement(cur, "macro");
    bool child_in_conditional = in_conditional || isXacroElement(cur, "if") || isXacroElement(cur, "unless")
                                || isXacroElement(cur, "else") || isXacroElement(cur, "elseif")
                                || isXacroElement(cur, "elif");
    while (child) {
      stack.push_back({child, child_in_macro, child_in_conditional});
      child = child->PreviousSiblingElement();
    }
  }
  // Remove xacro:arg elements from output after collecting
  for (auto* a : remove_args) {
    if (auto* p = a->Parent()) {
      p->DeleteChild(a);
    }
  }
  // Remove xacro:property elements from output after evaluation
  for (auto* pr : remove_props) {
    if (auto* p = pr->Parent()) {
      p->DeleteChild(pr);
    }
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
      p.name = tok;
      p.default_value.clear();
    }
    if (!p.name.empty() && p.name[0] == '*') {
      p.is_block = true;
      p.name = p.name.substr(1);
    }
    out.push_back(p);
  }
  return out;
}

bool Processor::defineMacro(tinyxml2::XMLElement* el) {
  std::string name = getAttr(el, "name");
  std::string params = getAttr(el, "params");
  MacroDef def;
  def.name = name;
  std::string plain_name = name;
  if (plain_name.rfind("xacro:", 0) == 0) {
    plain_name = plain_name.substr(6);
  }
  if (plain_name == "call") {
    throw ProcessingError("Invalid use of macro name 'call'");
  }
  def.params = parse_params(params);
  // Clone children of macro element into a container node we own in doc_
  tinyxml2::XMLNode* container = doc_->NewElement(("__macro__" + name).c_str());
  for (auto* child = el->FirstChild(); child; child = child->NextSibling()) {
    container->InsertEndChild(child->DeepClone(doc_));
  }
  def.content = container;
  macros_[name] = def;
  return true;
}

void Processor::removeComments(tinyxml2::XMLNode* node) {
  if (!node) {
    return;
  }
  for (auto* child = node->FirstChild(); child;) {
    auto* next = child->NextSibling();
    if (child->ToComment()) {
      // When comments sit between text nodes, deleting them would glue words together.
      // Insert a newline + indentation similar to Python xacro's pretty-printing so
      // spacing is preserved once comments are removed.
      auto* prev_text = child->PreviousSibling() ? child->PreviousSibling()->ToText() : nullptr;
      auto* next_text = next ? next->ToText() : nullptr;
      if (prev_text && next_text) {
        std::string prev_val = prev_text->Value();
        std::string next_val = next_text->Value();
        auto is_ws = [](char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; };
        bool prev_needs_sep = !prev_val.empty() && !is_ws(prev_val.back());
        bool next_needs_sep = !next_val.empty() && !is_ws(next_val.front());
        if (prev_needs_sep && next_needs_sep) {
          auto extract_indent = [](const std::string& s) {
            std::string indent;
            auto pos = s.rfind('\n');
            if (pos != std::string::npos) {
              for (size_t i = pos + 1; i < s.size(); ++i) {
                char ch = s[i];
                if (ch == ' ' || ch == '\t') {
                  indent.push_back(ch);
                }
                else {
                  break;
                }
              }
            }
            return indent;
          };
          std::string indent = extract_indent(prev_val);
          std::string sep = "\n" + indent;
          prev_val += sep;
          next_val = sep + next_val;
          prev_text->SetValue(prev_val.c_str());
          next_text->SetValue(next_val.c_str());
        }
      }
      node->DeleteChild(child);
    } else {
      removeComments(child);
    }
    child = next;
  }
}

void Processor::collectGlobalsInIncludedDoc(tinyxml2::XMLDocument& inc, const std::string& base_dir) {
  auto* root = inc.RootElement();
  if (!root) {
    return;
  }
  const std::string prev_base = base_dir_;
  base_dir_ = base_dir;
  std::vector<tinyxml2::XMLElement*> remove_args;
  std::vector<tinyxml2::XMLElement*> remove_props;
  struct Frame {
    tinyxml2::XMLElement* el;
    bool in_macro;
    bool in_conditional;
  };
  std::vector<Frame> stack;
  stack.push_back({root, false, false});
  while (!stack.empty()) {
    Frame f = stack.back();
    stack.pop_back();
    auto* cur = f.el;
    bool in_macro = f.in_macro;
    bool in_conditional = f.in_conditional;
    if (isXacroElement(cur, "arg")) {
      if (!in_macro && !in_conditional) {
        defineArg(cur);
        remove_args.push_back(cur);
      }
    }
    if (isXacroElement(cur, "property")) {
      if (!in_macro && !in_conditional) {
        defineProperty(cur);
        remove_props.push_back(cur);
      }
    }
    auto* child = cur->LastChildElement();
    bool child_in_macro = in_macro || isXacroElement(cur, "macro");
    bool child_in_conditional = in_conditional || isXacroElement(cur, "if") || isXacroElement(cur, "unless")
                                || isXacroElement(cur, "else") || isXacroElement(cur, "elseif")
                                || isXacroElement(cur, "elif");
    while (child) {
      stack.push_back({child, child_in_macro, child_in_conditional});
      child = child->PreviousSiblingElement();
    }
  }
  for (auto* a : remove_args) {
    if (auto* p = a->Parent()) {
      p->DeleteChild(a);
    }
  }
  for (auto* pr : remove_props) {
    if (auto* p = pr->Parent()) {
      p->DeleteChild(pr);
    }
  }
  base_dir_ = prev_base;
}

bool Processor::passCollectMacros(std::string* /*error_msg*/) {
  std::vector<tinyxml2::XMLElement*> to_remove;
  for (auto* el = doc_->RootElement(); el;) {
    std::vector<tinyxml2::XMLElement*> stack;
    stack.push_back(doc_->RootElement());
    while (!stack.empty()) {
      auto* cur = stack.back();
      stack.pop_back();
      if (isXacroElement(cur, "macro")) {
        defineMacro(cur);
        to_remove.push_back(cur);
      }
      auto* child = cur->LastChildElement();
      while (child) {
        stack.push_back(child);
        child = child->PreviousSiblingElement();
      }
    }
    break;
  }
  // Delete macros from deepest to shallowest to avoid invalidating nested entries.
  for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
    auto* el = *it;
    if (auto* parent = el->Parent()) {
      parent->DeleteChild(el);
    }
  }
  return true;
}

static void insert_before(tinyxml2::XMLNode* parent, tinyxml2::XMLNode* ref, tinyxml2::XMLNode* child) {
  if (!parent || !ref || !child) {
    return;
  }
  tinyxml2::XMLNode* prev = ref->PreviousSibling();
  if (prev) {
    parent->InsertAfterChild(prev, child);
  }
  else {
    parent->InsertFirstChild(child);
  }
}

bool Processor::expandIncludesInNode(tinyxml2::XMLNode* node, const std::string& base_dir, std::string* error_msg) {
  if (!node) {
    return true;
  }
  // Stack of elements with their current base directory for resolving relative includes
  struct Frame {
    tinyxml2::XMLElement* el;
    std::string bdir;
    bool active;
  };
  std::vector<Frame> stack;
  if (auto* root_el = node->ToElement()) {
    stack.push_back({root_el, base_dir, true});
  }
  while (!stack.empty()) {
    Frame fr = stack.back();
    stack.pop_back();
    tinyxml2::XMLElement* cur = fr.el;
    std::string cur_base = fr.bdir;
    bool active = fr.active;
    // For xacro:if/unless, determine child activity but do not splice yet.
    if (isXacroElement(cur, "if") || isXacroElement(cur, "unless")) {
      std::string cond = getAttr(cur, "value");
      bool val = eval_bool(eval_string_template(cond, vars_), vars_);
      if (isXacroElement(cur, "unless")) {
        val = !val;
      }
      for (auto* child = cur->LastChildElement(); child; child = child->PreviousSiblingElement()) {
        stack.push_back({child, cur_base, active && val});
      }
      continue;
    }
    if (isXacroElement(cur, "include")) {
      if (!active) { // skip expanding includes in inactive branches
        continue;
      }
      std::string file = getAttr(cur, "filename");
      file = eval_string_template(file, vars_);
      if (file.empty()) {
        continue;
      }
      std::string full;
#if defined(__cpp_lib_filesystem)
      fs::path pfile(file);
      if (pfile.is_absolute()) {
        full = pfile.string();
      }
      else {
        full = (fs::path(cur_base) / pfile).string();
      }
#else
      if (!file.empty() && file[0] == '/') {
        full = file;
      }
      else {
        full = cur_base + "/" + file;
      }
#endif
      tinyxml2::XMLDocument inc;
      if (inc.LoadFile(full.c_str()) != tinyxml2::XML_SUCCESS) {
        if (error_msg) {
          *error_msg = std::string("Failed to include ") + full;
        }
        return false;
      }
      removeComments(&inc);
      collectGlobalsInIncludedDoc(inc, dirname(full));
      auto* root = inc.RootElement();
      if (!root) {
        continue;
      }
      // Insert children of included root at include position
      std::vector<tinyxml2::XMLNode*> cloned;
      for (auto* c = root->FirstChild(); c; c = c->NextSibling()) {
        cloned.push_back(c->DeepClone(doc_));
      }
      auto* parent = cur->Parent();
      for (auto* n : cloned)
        insert_before(parent, cur, n);
      // Compute new base for items from this include
      std::string new_base = dirname(full);
      // Push newly inserted elements with their base dir for further include resolution
      for (auto* n : cloned) {
        if (auto* e = n->ToElement()) {
          stack.push_back({e, new_base, active});
        }
      }
      parent->DeleteChild(cur);
      continue;
    }
    // Regular element: push its children with same base
    for (auto* child = cur->LastChildElement(); child; child = child->PreviousSiblingElement()) {
      stack.push_back({child, cur_base, active});
    }
  }
  return true;
}

bool Processor::passExpandIncludes(std::string* error_msg) {
  return expandIncludesInNode(doc_->RootElement(), base_dir_, error_msg);
}

bool Processor::handleIfUnless(tinyxml2::XMLElement* el, std::vector<tinyxml2::XMLNode*>* inserted_out) {
  if (isXacroElement(el, "if") || isXacroElement(el, "unless")) {
    std::string cond = getAttr(el, "value");
    std::string cond_eval = eval_string_template(cond, vars_);
    bool val = eval_bool(cond_eval, vars_);
    if (isXacroElement(el, "unless")) {
      val = !val;
    }
    auto* parent = el->Parent();
    if (!val) {
      if (parent) {
        parent->DeleteChild(el);
      }
      modified_ = true;
      return true;
    }
    // True: splice children
    std::vector<tinyxml2::XMLNode*> cloned;
    for (auto* c = el->FirstChild(); c; c = c->NextSibling())
      cloned.push_back(c->DeepClone(doc_));
    for (auto* n : cloned)
      insert_before(parent, el, n);
    parent->DeleteChild(el);
    modified_ = true;
    if (inserted_out) {
      inserted_out->insert(inserted_out->end(), cloned.begin(), cloned.end());
    }
    return true;
  }
  return false;
}

bool Processor::expandXacroElement(tinyxml2::XMLElement* el,
                                   const std::unordered_map<std::string, std::string>& scope) {
  if (!isXacroElement(el, "element")) {
    return false;
  }
  std::string raw_name = getAttr(el, "xacro:name");
  if (raw_name.empty()) {
    raw_name = getAttr(el, "name");
  }
  std::string new_name = eval_string_template(raw_name, scope);
  if (new_name.empty()) {
    throw ProcessingError("xacro:element requires a non-empty name");
  }
  el->SetName(new_name.c_str());
  // Drop xacro:* attributes (e.g., xacro:name) from the renamed element
  std::vector<std::string> to_remove;
  const tinyxml2::XMLAttribute* attr = el->FirstAttribute();
  while (attr) {
    std::string attr_name = attr->Name() ? attr->Name() : "";
    if (attr_name.rfind("xacro:", 0) == 0) {
      to_remove.push_back(attr_name);
    }
    attr = attr->Next();
  }
  for (const auto& an : to_remove)
    el->DeleteAttribute(an.c_str());
  modified_ = true;
  return true;
}

void Processor::substituteAttributes(tinyxml2::XMLElement* el) {
  // Replace ${...} in all attribute values
  const tinyxml2::XMLAttribute* a = el->FirstAttribute();
  std::vector<std::pair<std::string, std::string>> updates;
  while (a) {
    std::string name = a->Name();
    std::string val = a->Value();
    std::string newv = eval_string_template(val, vars_);
    if (newv != val) {
      updates.emplace_back(name, newv);
    }
    a = a->Next();
  }
  for (auto& kv : updates) {
    el->SetAttribute(kv.first.c_str(), kv.second.c_str());
    modified_ = true;
  }
}

bool Processor::applyXacroAttribute(tinyxml2::XMLElement* el,
                                    const std::unordered_map<std::string, std::string>& scope) {
  if (!el) {
    return false;
  }
  tinyxml2::XMLNode* parent_node = el->Parent();
  auto* parent_el = parent_node ? parent_node->ToElement() : nullptr;
  if (!parent_el) {
    return false;
  }

  std::string name = getAttr(el, "name");
  if (name.empty()) {
    name = getAttr(el, "xacro:name");
  }
  std::string value = getAttr(el, "value");
  if (value.empty()) {
    const char* text = el->GetText();
    if (text) {
      value = text;
    }
  }
  name = eval_string_template(name, scope);
  value = eval_string_template(value, scope);
  if (name.empty()) {
    return false;
  }

  parent_el->SetAttribute(name.c_str(), value.c_str());
  parent_node->DeleteChild(el);
  modified_ = true;
  return true;
}

const Processor::MacroDef* Processor::findMacro(const std::vector<std::string>& names) const {
  for (const auto& n : names) {
    auto it = macros_.find(n);
    if (it != macros_.end()) {
      return &it->second;
    }
  }
  return nullptr;
}

bool Processor::expandMacroCall(tinyxml2::XMLElement* el,
                                const std::unordered_map<std::string, std::string>& parent_scope,
                                const std::unordered_map<std::string, std::vector<tinyxml2::XMLNode*>>* parent_blocks) {
  auto uniq_push = [](std::vector<std::string>* v, const std::string& n) {
    if (n.empty()) {
      return;
    }
    if (std::find(v->begin(), v->end(), n) == v->end()) {
      v->push_back(n);
    }
  };

  bool is_call_tag = isXacroElement(el, "call");
  std::string target = is_call_tag ? getAttr(el, "macro") : std::string(el->Name() ? el->Name() : "");
  if (!is_call_tag) {
    if (!target.starts_with(kXacroPrefix)) {
      return false;
    }
  }
  if (is_call_tag && target.empty()) {
    target = getAttr(el, "xacro:macro");
  }
  if (is_call_tag) {
    if (target.empty()) {
      throw ProcessingError("xacro:call: missing attribute 'macro'");
    }
    target = eval_string_template(target, parent_scope);
    if (target.empty()) {
      throw ProcessingError("xacro:call: missing attribute 'macro'");
    }
  }
  std::string plain_target = target;
  if (plain_target.starts_with(kXacroPrefix)) {
    plain_target = plain_target.substr(kXacroPrefix.size());
  }

  std::vector<std::string> candidates;
  uniq_push(&candidates, target);
  uniq_push(&candidates, plain_target);
  if (plain_target != target) {
    uniq_push(&candidates, std::string("xacro:") + plain_target);
  } else if (!plain_target.empty()) {
    uniq_push(&candidates, std::string("xacro:") + plain_target);
  }

  const MacroDef* mptr = findMacro(candidates);
  if (!mptr) {
    if (is_call_tag) {
      std::string msg_name = target.empty() ? plain_target : target;
      if (msg_name.rfind("xacro:", 0) != 0 && !msg_name.empty()) {
        msg_name = "xacro:" + msg_name;
      }
      throw ProcessingError("unknown macro name: " + msg_name);
    }
    return false;
  }
  const MacroDef& m = *mptr;
  // Build local scope seeded from caller (outer macro/global scope).
  std::unordered_map<std::string, std::string> scope = parent_scope;
  // Collect block arguments passed as child nodes of the macro call.
  // If a block argument is itself an xacro:insert_block, resolve it using the parent macro's blocks first
  // to avoid self-recursive block substitution (matches Python xacro behavior).
  std::unordered_map<std::string, std::vector<tinyxml2::XMLNode*>> blocks;
  std::vector<tinyxml2::XMLNode*> call_blocks;
  for (auto* ch = el->FirstChild(); ch; ch = ch->NextSibling()) {
    auto* chel = ch->ToElement();
    if (parent_blocks && chel && isXacroElement(chel, "insert_block")) {
      std::string bname = getAttr(chel, "name");
      auto itpb = parent_blocks->find(bname);
      if (itpb != parent_blocks->end()) {
        for (auto* bn : itpb->second) {
          if (bn) {
            call_blocks.push_back(bn->DeepClone(doc_));
          }
        }
        continue; // resolved this placeholder
      }
    }
    call_blocks.push_back(ch);
  }
  size_t block_arg_idx = 0;
  // First collect raw parameter strings (without evaluating references yet)
  std::unordered_map<std::string, std::string> raw_params;
  for (const auto& p : m.params) {
    if (p.is_block) {
      if (block_arg_idx < call_blocks.size()) {
        blocks[p.name].push_back(call_blocks[block_arg_idx]->DeepClone(doc_));
      }
      ++block_arg_idx;
    } else {
      const char* av = el->Attribute(p.name.c_str());
      raw_params[p.name] = av ? std::string(av) : p.default_value;
    }
  }
  // Resolve parameters allowing cross-references between them without self-recursing
  const std::unordered_map<std::string, std::string> base_scope = scope;
  std::unordered_map<std::string, std::string> evaluated_params;
  // Iteratively resolve templates (a couple of passes are sufficient in practice)
  for (int pass = 0; pass < 3; ++pass) {
    bool changed = false;
    for (auto& kv : raw_params) {
      // Allow references to previously resolved params, but avoid using the current
      // parameter's prior value to prevent self-recursion.
      std::unordered_map<std::string, std::string> eval_scope = base_scope;
      for (const auto& ep : evaluated_params) {
        if (ep.first != kv.first) {
          eval_scope[ep.first] = ep.second;
        }
      }
      std::string v = eval_string_template(kv.second, eval_scope);
      auto it_prev = evaluated_params.find(kv.first);
      if (it_prev == evaluated_params.end() || it_prev->second != v) {
        evaluated_params[kv.first] = v;
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }
  scope = base_scope;
  for (const auto& kv : evaluated_params) {
    scope[kv.first] = kv.second;
  }
  // Clone macro content with substitution
  std::vector<tinyxml2::XMLNode*> cloned;
  for (auto* c = m.content->FirstChild(); c; c = c->NextSibling()) {
    cloned.push_back(c->DeepClone(doc_));
  }
  auto* parent = el->Parent();
  for (auto* n : cloned) {
    insert_before(parent, el, n);
  }
  parent->DeleteChild(el);
  // After insertion, we must traverse inserted nodes and substitute
  for (auto* n : cloned) {
    // DFS
    std::vector<tinyxml2::XMLNode*> st;
    st.push_back(n);
    while (!st.empty()) {
      auto* cur = st.back();
      st.pop_back();
      if (auto* ce = cur->ToElement()) {
        // Nested macro definitions: register and remove from output.
        if (isXacroElement(ce, "macro")) {
          defineMacro(ce);
          if (auto* par = ce->Parent()) {
            par->DeleteChild(ce);
          }
          modified_ = true;
          continue;
        }
        // Expand nested macro calls using the current scope chain.
        if (expandMacroCall(ce, scope, &blocks)) {
          continue;
        }
        // Evaluate local-scope conditionals inside macro bodies
        if (isXacroElement(ce, "if") || isXacroElement(ce, "unless")) {
          std::string cond = getAttr(ce, "value");
          std::string cond_eval = eval_string_template(cond, scope);
          bool val = eval_bool(cond_eval, scope);
          if (isXacroElement(ce, "unless")) {
            val = !val;
          }
          auto* parent = ce->Parent();
          if (parent) {
            if (!val) {
              parent->DeleteChild(ce);
              modified_ = true;
              continue; // skip pushing children
            } else {
              // splice children
              std::vector<tinyxml2::XMLNode*> cloned_if;
              for (auto* c3 = ce->FirstChild(); c3; c3 = c3->NextSibling()) {
                cloned_if.push_back(c3->DeepClone(doc_));
              }
              for (auto* n2 : cloned_if) {
                insert_before(parent, ce, n2);
              }
              parent->DeleteChild(ce);
              for (auto* n2 : cloned_if) {
                st.push_back(n2);
              }
              modified_ = true;
              continue;
            }
          }
        }
        // Turn <xacro:element> into a regular element using the macro's scope
        expandXacroElement(ce, scope);
        // Handle xacro:property with local scoping (default) or global override
        if (isXacroElement(ce, "property")) {
          std::string pname = getAttr(ce, "name");
          std::string scope_attr = getAttr(ce, "scope");
          std::string pval = getAttr(ce, "value");
          if (pval.empty()) {
            const char* text = ce->GetText();
            if (text) {
              pval = text;
            }
          }
          pval = eval_string_template(pval, scope);
          if (!pname.empty()) {
            if (scope_attr == "global") {
              vars_[pname] = pval;
            } else {
              scope[pname] = pval;
            }
          }
          if (auto* par = ce->Parent()) {
            par->DeleteChild(ce);
          }
          modified_ = true;
          continue;
        }
        // Handle xacro:attribute on the containing element
        if (isXacroElement(ce, "attribute")) {
          applyXacroAttribute(ce, scope);
          continue;
        }
        // Handle xacro:insert_block
        if (isXacroElement(ce, "insert_block")) {
          std::string bname = getAttr(ce, "name");
          auto bit = blocks.find(bname);
          auto* par = ce->Parent();
          if (par) {
            std::vector<tinyxml2::XMLNode*> inserted_nodes;
            if (bit != blocks.end()) {
              inserted_nodes.reserve(bit->second.size());
              for (auto* bn : bit->second) {
                auto* clone = bn->DeepClone(doc_);
                insert_before(par, ce, clone);
                inserted_nodes.push_back(clone);
              }
            }
            par->DeleteChild(ce);
            // process newly inserted nodes with the same macro scope
            for (auto it = inserted_nodes.rbegin(); it != inserted_nodes.rend(); ++it) {
              if (*it) {
                st.push_back(*it);
              }
            }
          }
          modified_ = true;
          continue; // done with this node
        }
        // Substitute attrs using local scope
        const tinyxml2::XMLAttribute* a = ce->FirstAttribute();
        std::vector<std::pair<std::string, std::string>> updates;
        while (a) {
          std::string an = a->Name();
          std::string av = a->Value();
          std::string newv = eval_string_template(av, scope);
          if (newv != av) {
            updates.emplace_back(an, newv);
          }
          a = a->Next();
        }
        for (auto& kv : updates) {
          ce->SetAttribute(kv.first.c_str(), kv.second.c_str());
          modified_ = true;
        }
      } else if (auto* tx = cur->ToText()) {
        const char* tv = tx->Value();
        if (tv) {
          std::string nv = eval_string_template(tv, scope);
          if (nv != tv) {
            tx->SetValue(nv.c_str());
            modified_ = true;
          }
        }
      }
      for (auto* c2 = cur->LastChild(); c2; c2 = c2->PreviousSibling()) {
        st.push_back(c2);
      }
    }
  }
  modified_ = true;
  return true;
}

bool Processor::expandElement(tinyxml2::XMLElement* el) {
  // Handle inline properties/args defined inside expanded content
  if (isXacroElement(el, "property")) {
    defineProperty(el);
    if (auto* p = el->Parent()) {
      p->DeleteChild(el);
    }
    modified_ = true;
    return false;
  }
  if (isXacroElement(el, "arg")) {
    defineArg(el);
    if (auto* p = el->Parent()) {
      p->DeleteChild(el);
    }
    modified_ = true;
    return false;
  }
  {
    std::vector<tinyxml2::XMLNode*> inserted;
    if (handleIfUnless(el, &inserted)) {
      return false; // el was deleted or spliced; don't traverse its children
    }
  }
  bool was_xacro_element = expandXacroElement(el, vars_);
  if (!was_xacro_element && expandMacroCall(el, vars_, nullptr)) {
    return false; // el replaced and processed; don't traverse old children
  }
  if (isXacroElement(el, "attribute")) {
    applyXacroAttribute(el, vars_);
    return false;
  }
  substituteAttributes(el);
  return true; // el remains; traverse its children
}

bool Processor::expandNode(tinyxml2::XMLNode* node) {
  // DFS traversal
  std::vector<tinyxml2::XMLNode*> st;
  st.push_back(node);
  while (!st.empty()) {
    auto* cur = st.back();
    st.pop_back();
    if (auto* el = cur->ToElement()) {
      bool keep = expandElement(el);
      if (!keep) {
        // Node was deleted/replaced; don't use 'cur' further.
        continue;
      }
    } else if (auto* txt = cur->ToText()) {
      const char* tv = txt->Value();
      if (tv) {
        std::string nv = eval_string_template(tv, vars_);
        if (nv != tv) {
          txt->SetValue(nv.c_str());
          modified_ = true;
        }
      }
    }
    for (auto* c = cur->LastChild(); c; c = c->PreviousSibling())
      st.push_back(c);
  }
  return true;
}

void Processor::restoreDollarMarkers(tinyxml2::XMLNode* node) {
  if (!node) {
    return;
  }
  std::vector<tinyxml2::XMLNode*> st;
  st.push_back(node);
  while (!st.empty()) {
    auto* cur = st.back();
    st.pop_back();
    if (auto* el = cur->ToElement()) {
      std::vector<std::pair<std::string, std::string>> updates;
      const tinyxml2::XMLAttribute* a = el->FirstAttribute();
      while (a) {
        std::string val = a->Value() ? a->Value() : "";
        if (val.find(kDollarMarker) != std::string::npos) {
          std::replace(val.begin(), val.end(), kDollarMarker, '$');
          updates.emplace_back(a->Name(), val);
        }
        a = a->Next();
      }
      for (auto& kv : updates)
        el->SetAttribute(kv.first.c_str(), kv.second.c_str());
    } else if (auto* txt = cur->ToText()) {
      const char* tv = txt->Value();
      if (tv) {
        std::string val = tv;
        if (val.find(kDollarMarker) != std::string::npos) {
          std::replace(val.begin(), val.end(), kDollarMarker, '$');
          txt->SetValue(val.c_str());
        }
      }
    }
    for (auto* c = cur->LastChild(); c; c = c->PreviousSibling())
      st.push_back(c);
  }
}

bool Processor::passExpand(std::string* /*error_msg*/) {
  // Iterate expansion until a fixed point or safety limit
  for (int iter = 0; iter < 10; ++iter) {
    modified_ = false;
    expandNode(doc_->RootElement());
    if (!modified_) {
      break;
    }
  }
  return true;
}

} // namespace xacro_cpp
