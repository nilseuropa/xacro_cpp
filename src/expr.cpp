/// NowTechnologies Zrt. All rights reserved.
/// Expression evaluation and template substitution.
/// Author: nilseuropa <marton@nowtech.hu>
/// Created: 2026.01.20

#include "xacro_cpp/processor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "processor_internal.hpp"
#include "xacro_cpp/tinyexpr.h"

namespace xacro_cpp {

namespace {

inline bool isEscaped(const std::string& text, size_t idx) {
  if (idx == 0) {
    return false;
  }
  size_t backslashCount = 0;
  size_t pos = idx;
  while (pos > 0 && text[--pos] == '\\') {
    ++backslashCount;
  }
  return (backslashCount % 2) == 1;
}

static void ensureBalancedQuotes(const std::string& expr) {
  bool inSingle = false;
  bool inDouble = false;
  for (size_t i = 0; i < expr.size(); ++i) {
    char c = expr[i];
    if (c == '\'' && !inDouble && !isEscaped(expr, i)) {
      inSingle = !inSingle;
    } else if (c == '"' && !inSingle && !isEscaped(expr, i)) {
      inDouble = !inDouble;
    }
  }
  if (inSingle || inDouble) {
    throw ProcessingError("Unterminated string literal inside expression: " + expr);
  }
}

static void ensureNoPowOperator(const std::string& expr) {
  bool inSingle = false;
  bool inDouble = false;
  for (size_t i = 0; i + 1 < expr.size(); ++i) {
    char c = expr[i];
    if (c == '\'' && !inDouble && !isEscaped(expr, i)) {
      inSingle = !inSingle;
      continue;
    }
    if (c == '"' && !inSingle && !isEscaped(expr, i)) {
      inDouble = !inDouble;
      continue;
    }
    if (!inSingle && !inDouble && c == '*' && expr[i + 1] == '*') {
      throw ProcessingError("Unsupported operator '**' in expression: " + expr);
    }
  }
}

static void validateExpressionChunk(const std::string& expr) {
  if (expr.empty()) {
    return;
  }
  ensureBalancedQuotes(expr);
  ensureNoPowOperator(expr);
}

static std::string trimWs(const std::string& s) {
  size_t i = 0, j = s.size();
  while (i < j && (std::isspace(static_cast<unsigned char>(s[i])) != 0)) {
    ++i;
  }
  while (j > i && (std::isspace(static_cast<unsigned char>(s[j - 1])) != 0)) {
    --j;
  }
  return s.substr(i, j - i);
}

static bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

static void applyDefaultOverrides(const std::unordered_map<std::string, std::string>& vars,
                                  std::unordered_map<std::string, std::string>* out) {
  if (out == nullptr) {
    return;
  }
  *out = vars;
  static const std::string cDefaultSuffix = "__default__";
  for (const auto& kv : vars) {
    if (endsWith(kv.first, cDefaultSuffix)) {
      std::string base = kv.first.substr(0, kv.first.size() - cDefaultSuffix.size());
      (*out)[base] = kv.second;
    }
  }
}

static bool isSimpleIdentifier(const std::string& s) {
  if (s.empty()) {
    return false;
  }
  if (!((std::isalpha(static_cast<unsigned char>(s[0])) != 0) || s[0] == '_')) {
    return false;
  }
  for (size_t i = 1; i < s.size(); ++i) {
    char c = s[i];
    if (!((std::isalnum(static_cast<unsigned char>(c)) != 0) || c == '_')) {
      return false;
    }
  }
  return true;
}

static const std::unordered_set<std::string>& builtinIdentifiers() {
  static const std::unordered_set<std::string> cBuiltins
    = {"sin",   "cos",  "tan",   "asin",  "acos", "atan", "atan2", "sqrt", "abs", "fabs", "ln", "log",  "exp",
       "floor", "ceil", "round", "trunc", "sinh", "cosh", "tanh",  "pow",  "min", "max",  "pi", "true", "false"};
  return cBuiltins;
}

static void scanIdentifiers(const std::string& expr,
                            const std::unordered_set<std::string>& builtins,
                            const std::function<void(const std::string&)>& onIdent) {
  bool inSingle = false;
  bool inDouble = false;
  for (size_t i = 0; i < expr.size();) {
    char c = expr[i];
    if (c == '\'' && !inDouble && !isEscaped(expr, i)) {
      inSingle = !inSingle;
      ++i;
      continue;
    }
    if (c == '"' && !inSingle && !isEscaped(expr, i)) {
      inDouble = !inDouble;
      ++i;
      continue;
    }
    if (inSingle || inDouble) {
      ++i;
      continue;
    }
    if ((std::isalpha(static_cast<unsigned char>(c)) != 0) || c == '_') {
      size_t j = i + 1;
      while (j < expr.size()) {
        char cj = expr[j];
        if ((std::isalnum(static_cast<unsigned char>(cj)) != 0) || cj == '_') {
          ++j;
        } else {
          break;
        }
      }
      std::string ident = expr.substr(i, j - i);
      if (builtins.count(ident) == 0U) {
        onIdent(ident);
      }
      i = j;
    } else {
      ++i;
    }
  }
}

static bool hasUnknownIdentifier(const std::string& expr, const std::unordered_map<std::string, std::string>& vars) {
  const auto& cBuiltins = builtinIdentifiers();
  bool unknown = false;
  scanIdentifiers(expr, cBuiltins, [&](const std::string& ident) {
    static const std::string cDefaultSuffix = "__default__";
    if (vars.find(ident) == vars.end() && vars.find(ident + cDefaultSuffix) == vars.end()) {
      unknown = true;
    }
  });
  return unknown;
}

/// Replace occurrences of name[index] with the indexed token from vars[name]
static std::string replaceIndexingWithValues(const std::string& expr,
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
      static const std::string cDefaultSuffix = "__default__";
      auto it = vars.find(name + cDefaultSuffix);
      bool usedDefault = false;
      if (it != vars.end()) {
        usedDefault = true;
      } else {
        it = vars.find(name);
      }
      bool useStringIndex = false;
      if (usedDefault) {
        auto marker = vars.find(name + "__default_string__");
        useStringIndex = (marker != vars.end());
      }
      if (it != vars.end()) {
        if (useStringIndex) {
          if (idx < it->second.size()) {
            value.assign(1, it->second[idx]);
          } else {
            value.clear();
          }
        } else {
          std::vector<std::string> tokens;
          if (!splitListLiteral(it->second, &tokens)) {
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
      }
      if (useStringIndex) {
        result += value;
      } else {
        result += stripQuotes(trimWs(value));
      }
      searchStart = m.suffix().first;
    }
    result.append(searchStart, s.cend());
    return result;
  } catch (...) {
    return expr;
  }
}

static bool tryEvalListLiteral(const std::string& expr,
                               const std::unordered_map<std::string, std::string>& vars,
                               std::string* out) {
  std::vector<std::string> listExprItems;
  if (!splitListLiteral(expr, &listExprItems)) {
    return false;
  }
  bool listResolved = true;
  std::ostringstream oss;
  oss << "[";
  for (size_t idxItem = 0; idxItem < listExprItems.size(); ++idxItem) {
    std::string itemExpr = replaceIndexingWithValues(listExprItems[idxItem], vars);
    bool itemResolved = false;
    std::string itemVal = evalStringWithVars(trimWs(itemExpr), vars, &itemResolved);
    if (!itemResolved) {
      listResolved = false;
      break;
    }
    if (idxItem != 0U) {
      oss << ", ";
    }
    oss << itemVal;
  }
  if (!listResolved) {
    return false;
  }
  oss << "]";
  if (out != nullptr) {
    *out = oss.str();
  }
  return true;
}

static bool tryEvalFindExpr(const std::string& expr, std::string* out) {
  if (expr.rfind("find", 0) != 0) {
    return false;
  }
  std::string inside;
  size_t lp = expr.find('(');
  size_t rp = expr.find_last_of(')');
  if (lp != std::string::npos && rp != std::string::npos && rp > lp) {
    inside = trimWs(expr.substr(lp + 1, rp - lp - 1));
    inside = stripQuotes(inside);
  }
  std::string found = inside.empty() ? std::string() : resolveFind(inside);
  if (!inside.empty() && found.empty()) {
    throw ProcessingError("package not found: " + inside);
  }
  if (out != nullptr) {
    *out = found;
  }
  return true;
}

static void skipWs(const std::string& s, size_t* pos) {
  while (*pos < s.size() && (std::isspace(static_cast<unsigned char>(s[*pos])) != 0)) {
    ++(*pos);
  }
}

static bool parseIdentifierToken(const std::string& s, size_t* pos, std::string* out) {
  if (*pos >= s.size()) {
    return false;
  }
  char c = s[*pos];
  if (!((std::isalpha(static_cast<unsigned char>(c)) != 0) || c == '_')) {
    return false;
  }
  size_t start = *pos;
  ++(*pos);
  while (*pos < s.size()) {
    char cur = s[*pos];
    if ((std::isalnum(static_cast<unsigned char>(cur)) != 0) || cur == '_') {
      ++(*pos);
    } else {
      break;
    }
  }
  *out = s.substr(start, *pos - start);
  return true;
}

static bool parseQuotedString(const std::string& s, size_t* pos, std::string* out) {
  if (*pos >= s.size()) {
    return false;
  }
  char quote = s[*pos];
  if (quote != '\'' && quote != '"') {
    return false;
  }
  ++(*pos);
  std::string result;
  while (*pos < s.size()) {
    char c = s[*pos];
    if (c == '\\') {
      if (*pos + 1 >= s.size()) {
        return false;
      }
      char next = s[*pos + 1];
      if (next == '\\' || next == '\'' || next == '"') {
        result.push_back(next);
        *pos += 2;
        continue;
      }
      result.push_back(next);
      *pos += 2;
      continue;
    }
    if (c == quote) {
      ++(*pos);
      *out = result;
      return true;
    }
    result.push_back(c);
    ++(*pos);
  }
  return false;
}

static bool resolveYamlExpression(const std::string& expr,
                                  const std::unordered_map<std::string, YamlValue>* yamlDocs,
                                  std::string* out) {
  if ((yamlDocs == nullptr) || (out == nullptr)) {
    return false;
  }
  size_t pos = 0;
  skipWs(expr, &pos);
  std::string root;
  if (!parseIdentifierToken(expr, &pos, &root)) {
    return false;
  }
  auto it = yamlDocs->find(root);
  if (it == yamlDocs->end()) {
    return false;
  }
  const YamlValue* current = &it->second;
  while (true) {
    skipWs(expr, &pos);
    if (pos >= expr.size()) {
      break;
    }
    char c = expr[pos];
    if (c == '.') {
      ++pos;
      skipWs(expr, &pos);
      std::string key;
      if (!parseIdentifierToken(expr, &pos, &key)) {
        return false;
      }
      if (current->mType != YamlValue::Type::cMap) {
        return false;
      }
      auto mit = current->mMapValue.find(key);
      if (mit == current->mMapValue.end()) {
        return false;
      }
      current = &mit->second;
      continue;
    }
    if (c == '[') {
      ++pos;
      skipWs(expr, &pos);
      if (pos >= expr.size()) {
        return false;
      }
      if (expr[pos] == '\'' || expr[pos] == '"') {
        std::string key;
        if (!parseQuotedString(expr, &pos, &key)) {
          return false;
        }
        skipWs(expr, &pos);
        if (pos >= expr.size() || expr[pos] != ']') {
          return false;
        }
        ++pos;
        if (current->mType != YamlValue::Type::cMap) {
          return false;
        }
        auto mit = current->mMapValue.find(key);
        if (mit == current->mMapValue.end()) {
          return false;
        }
        current = &mit->second;
        continue;
      }
      size_t start = pos;
      while (pos < expr.size() && (std::isdigit(static_cast<unsigned char>(expr[pos])) != 0)) {
        ++pos;
      }
      if (start == pos) {
        return false;
      }
      std::string numStr = expr.substr(start, pos - start);
      skipWs(expr, &pos);
      if (pos >= expr.size() || expr[pos] != ']') {
        return false;
      }
      ++pos;
      if (current->mType != YamlValue::Type::cList) {
        return false;
      }
      size_t idx = 0;
      try {
        idx = static_cast<size_t>(std::stoul(numStr));
      } catch (...) {
        return false;
      }
      if (idx >= current->mListValue.size()) {
        return false;
      }
      current = &current->mListValue[idx];
      continue;
    }
    return false;
  }
  if (current->mType == YamlValue::Type::cMap) {
    return false;
  }
  *out = yamlValueToString(*current);
  return true;
}

static bool tryEvalYamlExpr(const std::string& expr,
                            const std::unordered_map<std::string, YamlValue>* yamlDocs,
                            std::string* out) {
  if (yamlDocs == nullptr) {
    return false;
  }
  std::string yamlResolved;
  if (!resolveYamlExpression(expr, yamlDocs, &yamlResolved)) {
    return false;
  }
  if (out != nullptr) {
    *out = yamlResolved;
  }
  return true;
}

static bool tryEvalNumericExpr(const std::string& expr,
                               const std::unordered_map<std::string, std::string>& vars,
                               const std::function<std::string(const std::string&)>& resolveVar,
                               std::string* out) {
  std::unordered_map<std::string, std::string> evalVars = vars;
  std::vector<std::string> idents;
  collectIdentifiers(expr, &idents);
  for (const auto& ident : idents) {
    auto it = vars.find(ident);
    if (it != vars.end()) {
      evalVars[ident] = resolveVar(ident);
    }
  }
  std::string ex2 = replaceIndexingWithValues(expr, evalVars);
  bool resolved = false;
  std::string val = evalStringWithVars(ex2, evalVars, &resolved);
  if (!resolved) {
    return false;
  }
  if (out != nullptr) {
    *out = val;
  }
  return true;
}

} // namespace

void collectIdentifiers(const std::string& expr, std::vector<std::string>* out) {
  if (out == nullptr) {
    return;
  }
  const auto& cBuiltins = builtinIdentifiers();
  scanIdentifiers(expr, cBuiltins, [&](const std::string& ident) {
    if (std::find(out->begin(), out->end(), ident) == out->end()) {
      out->push_back(ident);
    }
  });
}

bool splitListLiteral(const std::string& expr, std::vector<std::string>* items) {
  if (items == nullptr) {
    return false;
  }
  std::string trimmed = trimWs(expr);
  if (trimmed.size() < 2 || trimmed.front() != '[' || trimmed.back() != ']') {
    return false;
  }

  std::string inner = trimmed.substr(1, trimmed.size() - 2);
  std::vector<std::string> tokens;
  std::string current;
  int depth = 0;
  bool inSingle = false;
  bool inDouble = false;
  auto flush = [&]() {
    std::string token = trimWs(current);
    if (!token.empty()) {
      tokens.push_back(token);
    }
    current.clear();
  };
  for (size_t idx = 0; idx < inner.size(); ++idx) {
    char c = inner[idx];
    if (c == ',' && depth == 0 && !inSingle && !inDouble) {
      flush();
      continue;
    }
    current.push_back(c);
    if (c == '\'' && !inDouble && !isEscaped(inner, idx)) {
      inSingle = !inSingle;
    } else if (c == '"' && !inSingle && !isEscaped(inner, idx)) {
      inDouble = !inDouble;
    } else if (!inSingle && !inDouble) {
      if (c == '[' || c == '{' || c == '(') {
        ++depth;
      } else if (c == ']' || c == '}' || c == ')') {
        if (depth > 0) {
          --depth;
        }
      }
    }
  }
  if (inSingle || inDouble || depth != 0) {
    return false;
  }
  flush();
  *items = tokens;
  return true;
}

double evalNumber(const std::string& expr, const std::unordered_map<std::string, std::string>& vars, bool* ok) {
  /// Prepare variables as doubles when possible
  std::unordered_map<std::string, std::string> effectiveVars;
  applyDefaultOverrides(vars, &effectiveVars);
  std::vector<TeVariable> v;
  v.reserve(effectiveVars.size());
  std::vector<double> storage;
  storage.reserve(effectiveVars.size() + 1);
  /// Inject common constants
  static const double cPi = 3.14159265358979323846;
  storage.push_back(cPi);
  v.push_back(TeVariable{"pi", &storage.back(), cTeVariable, nullptr});
  auto toLower = [](std::string s) {
    for (auto& c : s) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
  };
  for (auto& kv : effectiveVars) {
    char* end = nullptr;
    std::string raw = trimWs(kv.second);
    double val = std::strtod(raw.c_str(), &end);
    bool consumed = ((end != nullptr) && *end == '\0');
    if (!consumed) {
      std::string lowered = toLower(raw);
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
      TeVariable t{kv.first.c_str(), &storage.back(), cTeVariable, nullptr};
      v.push_back(t);
    }
  }
  int err = 0;
  std::string ex = replaceIndexingWithValues(expr, effectiveVars);
  TeExpr* comp = nullptr;
  static std::mutex cacheMutex;
  static std::unordered_map<std::string, TeExpr*> cache;
  if (v.empty()) {
    std::scoped_lock lock(cacheMutex);
    auto it = cache.find(ex);
    if (it == cache.end()) {
      comp = teCompile(ex.c_str(), nullptr, 0, &err);
      if (err == 0 && comp != nullptr) {
        cache.emplace(ex, comp);
      }
    } else {
      comp = it->second;
    }
  } else {
    comp = teCompile(ex.c_str(), v.data(), (int)v.size(), &err);
  }
  if (err != 0) {
    if (ok != nullptr) {
      *ok = false;
    }
    if (comp != nullptr) {
      if (!v.empty()) {
        teFree(comp);
      }
    }
    return 0.0;
  }
  double res = teEval(comp);
  if (!v.empty()) {
    teFree(comp);
  }
  if (!std::isfinite(res)) {
    if (ok != nullptr) {
      *ok = false;
    }
    throw ProcessingError("division by zero when evaluating expression '" + expr + "'");
  }
  if (ok != nullptr) {
    *ok = true;
  }
  return res;
}

bool evalBool(const std::string& expr, const std::unordered_map<std::string, std::string>& vars) {
  /// Emulates Python xacro truthiness: strict boolean/number expressions only.
  /// Normalize optional ${...} wrapper (common in xacro:if/unless).
  std::unordered_map<std::string, std::string> effectiveVars;
  applyDefaultOverrides(vars, &effectiveVars);
  auto trim = [](const std::string& s) -> std::string {
    size_t i = 0, j = s.size();
    while (i < j && (std::isspace(static_cast<unsigned char>(s[i])) != 0)) {
      ++i;
    }
    while (j > i && (std::isspace(static_cast<unsigned char>(s[j - 1])) != 0)) {
      --j;
    }
    return s.substr(i, j - i);
  };
  std::string exprTrim = trim(expr);
  if (exprTrim.size() >= 3 && exprTrim[0] == '$' && exprTrim[1] == '{' && exprTrim.back() == '}') {
    exprTrim = trim(exprTrim.substr(2, exprTrim.size() - 3));
  }
  /// Fast path for textual booleans
  auto trimLower = [](std::string s) {
    size_t i = 0, j = s.size();
    while (i < j && (std::isspace(static_cast<unsigned char>(s[i])) != 0)) {
      ++i;
    }
    while (j > i && (std::isspace(static_cast<unsigned char>(s[j - 1])) != 0)) {
      --j;
    }
    s = s.substr(i, j - i);
    for (auto& c : s) {
      c = (char)std::tolower((unsigned char)c);
    }
    return s;
  };
  std::string t = trimLower(exprTrim);
  if (t == "true") {
    return true;
  }
  if (t == "false") {
    return false;
  }
  /// Numeric expression
  bool ok = false;
  bool hasUnknown = hasUnknownIdentifier(exprTrim, effectiveVars);
  double v = evalNumber(exprTrim, effectiveVars, &ok);
  if (ok) {
    if (hasUnknown) {
      throw ProcessingError("invalid boolean expression: " + exprTrim);
    }
    return v != 0.0;
  }
  /// Fallback: comparison operators with identifiers, quoted strings or numeric literals
  try {
    static const std::regex cReCmp(R"(^\s*(.*?)\s*(==|!=|<=|>=|<|>)\s*(.*?)\s*$)");
    std::smatch m;
    if (std::regex_match(exprTrim, m, cReCmp)) {
      auto resolve = [&](std::string s) -> std::string {
        /// local trim
        size_t i = 0, j = s.size();
        while (i < j && (std::isspace(static_cast<unsigned char>(s[i])) != 0)) {
          ++i;
        }
        while (j > i && (std::isspace(static_cast<unsigned char>(s[j - 1])) != 0)) {
          --j;
        }
        s = s.substr(i, j - i);
        if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\''))) {
          return s.substr(1, s.size() - 2);
        }
        /// identifier lookup
        if (!s.empty() && ((std::isalpha((unsigned char)s[0]) != 0) || s[0] == '_')) {
          auto it = effectiveVars.find(s);
          return it != effectiveVars.end() ? it->second : s;
        }
        return s;
      };
      std::string lhs = trim(std::string(m[1]));
      std::string rhs = trim(std::string(m[3]));
      std::string op = m[2];
      bool lnOk = false;
      bool rnOk = false;
      double ln = evalNumber(lhs, effectiveVars, &lnOk);
      double rn = evalNumber(rhs, effectiveVars, &rnOk);
      if (lnOk && rnOk) {
        if (op == "==") {
          return ln == rn;
        } else if (op == "!=") {
          return ln != rn;
        } else if (op == "<") {
          return ln < rn;
        } else if (op == "<=") {
          return ln <= rn;
        } else if (op == ">=") {
          return ln >= rn;
        } else if (op == ">") {
          return ln > rn;
        }
      } else {
        std::string ls = resolve(std::string(m[1]));
        std::string rs = resolve(std::string(m[3]));
        if (op == "==") {
          return ls == rs;
        } else if (op == "!=") {
          return ls != rs;
        } else if (op == "<") {
          /// For non-numeric <,>,<=,>= treat lexicographically
          return ls < rs;
        } else if (op == "<=") {
          return ls <= rs;
        } else if (op == ">") {
          return ls > rs;
        } else if (op == ">=") {
          return ls >= rs;
        }
      }
    }
  } catch (...) {
  }
  if (isSimpleIdentifier(exprTrim)) {
    auto it = effectiveVars.find(exprTrim);
    if (it != effectiveVars.end()) {
      std::string v = trimLower(it->second);
      if (v == "true") {
        return true;
      }
      if (v == "false" || v == "0" || v.empty()) {
        return false;
      }
      char* end = nullptr;
      double d = std::strtod(it->second.c_str(), &end);
      if ((end != nullptr) && *end == '\0') {
        return d != 0.0;
      }
      return true;
    }
    throw ProcessingError("invalid boolean expression: " + exprTrim);
  }
  if (exprTrim.size() >= 2
      && ((exprTrim.front() == '"' && exprTrim.back() == '"')
          || (exprTrim.front() == '\'' && exprTrim.back() == '\''))) {
    return exprTrim.size() > 2;
  }
  throw ProcessingError("invalid boolean expression: " + exprTrim);
}

std::string evalStringWithVars(const std::string& expr,
                               const std::unordered_map<std::string, std::string>& vars,
                               bool* resolved) {
  /// Evaluate a minimal numeric expression if all identifiers are known; otherwise leave literal.
  std::unordered_map<std::string, std::string> effectiveVars;
  applyDefaultOverrides(vars, &effectiveVars);
  if (resolved != nullptr) {
    *resolved = false;
  }
  /// Prefer exact variable replacement in the given scope first
  auto it = effectiveVars.find(expr);
  if (it != effectiveVars.end()) {
    if (resolved != nullptr) {
      *resolved = true;
    }
    return it->second;
  }
  if (hasUnknownIdentifier(expr, effectiveVars)) {
    return expr;
  }
  /// Else, if numeric expression, evaluate
  bool ok = false;
  double num = evalNumber(expr, effectiveVars, &ok);
  if (ok) {
    if (resolved != nullptr) {
      *resolved = true;
    }
    if (num == 0.0) {
      /// normalize -0.0 to 0.0
      num = 0.0;
    }
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10) << num;
    return oss.str();
  }
  /// leave as-is
  return expr;
}

std::string stripQuotes(const std::string& s) {
  if (s.size() >= 2) {
    if ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\'')) {
      return s.substr(1, s.size() - 2);
    }
  }
  return s;
}

std::string getDefaultName(const std::string& name) {
  return name + "__default__";
}

std::string evalStringTemplate(const std::string& text, const std::unordered_map<std::string, std::string>& vars) {
  return evalStringTemplate(text, vars, nullptr);
}

static std::string evalStringTemplate(const std::string& text,
                                      const std::unordered_map<std::string, std::string>& vars,
                                      const std::unordered_map<std::string, YamlValue>* yamlDocs,
                                      std::vector<std::string>* resolveStack);

std::string evalStringTemplate(const std::string& text,
                               const std::unordered_map<std::string, std::string>& vars,
                               const std::unordered_map<std::string, YamlValue>* yamlDocs) {
  std::vector<std::string> resolveStack;
  return evalStringTemplate(text, vars, yamlDocs, &resolveStack);
}

static std::string evalStringTemplate(const std::string& text,
                                      const std::unordered_map<std::string, std::string>& vars,
                                      const std::unordered_map<std::string, YamlValue>* yamlDocs,
                                      std::vector<std::string>* resolveStack) {
  if (text.find('$') == std::string::npos) {
    return text;
  }
  /// Resolve ${...}, then $(arg ...), then $(find ...) while preserving escaped "$$".
  auto resolveVar = [&](const std::string& name) -> std::string {
    auto it = vars.find(getDefaultName(name));
    if (it == vars.end()) {
      it = vars.find(name);
      if (it == vars.end()) {
        return std::string();
      }
    }
    if (resolveStack != nullptr) {
      if (std::find(resolveStack->begin(), resolveStack->end(), name) != resolveStack->end()) {
        std::string msg = "circular variable definition: ";
        for (const auto& entry : *resolveStack) {
          msg += entry;
          msg += " -> ";
        }
        msg += name;
        throw ProcessingError(msg);
      }
      resolveStack->push_back(name);
    }
    std::string val = it->second;
    if (val.find('$') != std::string::npos) {
      val = evalStringTemplate(val, vars, yamlDocs, resolveStack);
    }
    if (resolveStack != nullptr) {
      resolveStack->pop_back();
    }
    return val;
  };

  /// Handle escaped dollars first: each "$$" becomes a marker for a literal "$"
  auto escapeDollars = [](const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
      if (in[i] == '$' && i + 1 < in.size() && in[i + 1] == '$') {
        out.push_back(kDollarMarker);
        /// skip second '$'
        ++i;
      } else {
        out.push_back(in[i]);
      }
    }
    return out;
  };
  std::string work = escapeDollars(text);

  /// Replace ${...} occurrences
  std::string out;
  out.reserve(work.size());
  for (size_t i = 0; i < work.size();) {
    if (work[i] == '$' && i + 1 < work.size() && work[i + 1] == '{') {
      size_t j = i + 2;
      int depth = 1;
      while (j < work.size() && depth > 0) {
        if (work[j] == '{') {
          depth++;
        } else if (work[j] == '}') {
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
        std::string rawExpr = work.substr(i, j - i + 1);
        std::string expr = trimWs(work.substr(i + 2, j - (i + 2)));
        validateExpressionChunk(expr);
        std::string resolvedChunk;
        if (tryEvalListLiteral(expr, vars, &resolvedChunk)) {
          out += resolvedChunk;
          i = j + 1;
          continue;
        }
        if (isSimpleIdentifier(expr)) {
          auto defaultName = getDefaultName(expr);
          /// checking the default first - when it is present for a property, it is the relevant value
          auto it = vars.find(defaultName);
          if (it != vars.end()) {
            out += resolveVar(defaultName);
            i = j + 1;
            continue;
          }
          if ((it = vars.find(expr)) != vars.end()) {
            out += resolveVar(expr);
            i = j + 1;
            continue;
          }
        }
        if (tryEvalFindExpr(expr, &resolvedChunk)) {
          out += resolvedChunk;
          i = j + 1;
          continue;
        }
        if (tryEvalYamlExpr(expr, yamlDocs, &resolvedChunk)) {
          out += resolvedChunk;
          i = j + 1;
          continue;
        }
        if (!tryEvalNumericExpr(expr, vars, resolveVar, &resolvedChunk)) {
          out += rawExpr;
        } else {
          out += resolvedChunk;
        }
        i = j + 1;
        continue;
      }
    }
    out.push_back(work[i]);
    ++i;
  }
  /// First handle $(arg name) outside of ${...}
  /// Do this before $(find ...) so nested patterns like $(find $(arg pkg)) work.
  static const std::regex cReArg("\\$\\(\\s*arg\\s+([^\\)]+)\\)");
  std::smatch m;
  std::string s = out;
  std::string result;
  result.reserve(s.size());
  std::string::const_iterator searchStart(s.cbegin());
  while (std::regex_search(searchStart, s.cend(), m, cReArg)) {
    result.append(m.prefix().first, m.prefix().second);
    std::string name = trimWs(std::string(m[1]));
    name = stripQuotes(name);
    auto it = vars.find(name);
    if (it == vars.end()) {
      throw ProcessingError("Undefined substitution argument '" + name + "'");
    }
    result += it->second;
    searchStart = m.suffix().first;
  }
  result.append(searchStart, s.cend());
  out = std::move(result);

  /// Also handle $(find pkg) style outside of ${...}
  try {
    static const std::regex cReFind("\\$\\(\\s*find\\s+([^\\)]+)\\)");
    std::smatch m;
    std::string s = out;
    std::string result;
    result.reserve(s.size());
    std::string::const_iterator searchStart(s.cbegin());
    while (std::regex_search(searchStart, s.cend(), m, cReFind)) {
      result.append(m.prefix().first, m.prefix().second);
      std::string pkg = trimWs(std::string(m[1]));
      pkg = stripQuotes(pkg);
      std::string found = resolveFind(pkg);
      if (!pkg.empty() && found.empty()) {
        throw ProcessingError("package not found: " + pkg);
      }
      result += found;
      searchStart = m.suffix().first;
    }
    result.append(searchStart, s.cend());
    out = std::move(result);
  } catch (const ProcessingError&) {
    throw;
  } catch (...) {
    /// leave 'out' untouched
  }

  /// Restore escaped dollars after all substitutions/checks.
  return out;
}

} // namespace xacro_cpp
