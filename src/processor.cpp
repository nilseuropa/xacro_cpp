/// NowTechnologies Zrt. All rights reserved.
/// Xacro processing engine implementation.
/// Author: nilseuropa <marton@nowtech.hu>
/// Created: 2026.01.20

#include "xacro_cpp/processor.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <ranges>
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

#include <tinyxml2.h>
#if __has_include(<ament_index_cpp/get_package_share_directory.hpp>)
#include <ament_index_cpp/get_package_share_directory.hpp>
#define XACRO_HAVE_AMENT_INDEX 1
#endif

#include "processor_internal.hpp"

namespace xacro_cpp {

static constexpr std::string_view cXacroPrefix = "xacro:";

/// ---- Path helpers / package discovery ----
static std::string dirname(const std::string& path) {
  auto pos = path.find_last_of("/\\");
  if (pos == std::string::npos) {
    return std::string(".");
  }
  return path.substr(0, pos);
}

/// Current-package fallback for ${find('pkg')} when ament index is unavailable.
static std::string gCurrentPkgName;
static std::string gCurrentPkgRoot;

namespace {

static bool debugEnabled() {
  static const bool cEnabled = std::getenv("XACRO_CPP_DEBUG") != nullptr;
  return cEnabled;
}

static void debugLog(const std::string& msg) {
  if (debugEnabled()) {
    std::cerr << "[xacro_cpp] " << msg << "\n";
  }
}

[[noreturn]] void throwProcessingError(const std::string& message) {
  if (message.empty()) {
    throw ProcessingError("xacro_cpp processing failed");
  }
  throw ProcessingError(message);
}

inline void throwFromErrorMsg(std::string* errorMsg, const char* fallback) {
  if (errorMsg != nullptr) {
    if (errorMsg->empty() && (fallback != nullptr)) {
      *errorMsg = fallback;
    }
    throwProcessingError(*errorMsg);
  }
  throwProcessingError((fallback != nullptr) ? std::string(fallback) : std::string());
}

} // namespace

/// ---- Filesystem / package lookups ----
static void detectCurrentPackageFrom(const std::string& startDir) {
  gCurrentPkgName.clear();
  gCurrentPkgRoot.clear();
#if defined(__cpp_lib_filesystem)
  fs::path p = startDir;
  std::error_code errorCode;
  while (!p.empty()) {
    fs::path pkgxml = p / "package.xml";
    if (fs::exists(pkgxml, errorCode)) {
      tinyxml2::XMLDocument d;
      if (d.LoadFile(pkgxml.string().c_str()) == tinyxml2::XML_SUCCESS) {
        if (auto* root = d.RootElement()) {
          /// Expect <package> then <name>
          auto* nameEl = root->FirstChildElement("name");
          if (nameEl == nullptr) {
            /// Some package.xml may have <package format="3"> then <name>
            for (auto* c = root->FirstChildElement(); c != nullptr; c = c->NextSiblingElement()) {
              if (std::string(c->Name()) == "name") {
                nameEl = c;
                break;
              }
            }
          }
          if ((nameEl != nullptr) && (nameEl->GetText() != nullptr)) {
            gCurrentPkgName = nameEl->GetText();
            gCurrentPkgRoot = p.string();
            return;
          }
        }
      }
    }
    fs::path parentPath = p.parent_path();
    if (parentPath == p) {
      /// reached filesystem root
      break;
    }
    p = parentPath;
  }
#endif
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

template <typename MapT>
class ScopedRestore {
public:
  explicit ScopedRestore(MapT& target) : mTarget(target), mSaved(target) {}
  ~ScopedRestore() {
    mTarget = std::move(mSaved);
  }
  ScopedRestore(const ScopedRestore&) = delete;
  ScopedRestore& operator=(const ScopedRestore&) = delete;
  ScopedRestore(ScopedRestore&&) = delete;
  ScopedRestore& operator=(ScopedRestore&&) = delete;

private:
  MapT& mTarget;
  MapT mSaved;
};

class LocalScope {
public:
  explicit LocalScope(const std::unordered_map<std::string, std::string>& base) : mVars(base) {}
  std::unordered_map<std::string, std::string>& map() {
    return mVars;
  }
  const std::unordered_map<std::string, std::string>& map() const {
    return mVars;
  }
  void setProperty(const std::string& name,
                   const std::string& value,
                   const std::string& scopeAttr,
                   std::unordered_map<std::string, std::string>& globals) {
    if (scopeAttr == "global") {
      globals[name] = value;
    } else {
      mVars[name] = value;
    }
  }

private:
  std::unordered_map<std::string, std::string> mVars;
};

std::string resolveFind(const std::string& pkg) {
  static std::unordered_map<std::string, std::string> cache;
  auto cached = cache.find(pkg);
  if (cached != cache.end()) {
    return cached->second;
  }
  auto normalizePath = [](const std::string& path) -> std::string {
    if (path.empty()) {
      return path;
    }
#if defined(__cpp_lib_filesystem)
    fs::path p(path);
    if (!p.is_absolute()) {
      std::error_code ec;
      fs::path abs = fs::absolute(p, ec);
      if (!ec) {
        return abs.string();
      }
    }
    return p.string();
#else
    return path;
#endif
  };
#ifdef XACRO_HAVE_AMENT_INDEX
  try {
    std::string found = ament_index_cpp::get_package_share_directory(pkg);
    found = normalizePath(found);
    cache[pkg] = found;
    return found;
  } catch (const std::exception&) {
    /// fall through to env scan
  }
#endif
  /// Fallback: scan AMENT_PREFIX_PATH or COLCON_PREFIX_PATH for share/pkg
  auto getEnv = [](const char* name) -> std::string {
    const char* v = std::getenv(name);
    return (v != nullptr) ? std::string(v) : std::string();
  };
  std::string paths = getEnv("AMENT_PREFIX_PATH");
  if (paths.empty()) {
    paths = getEnv("COLCON_PREFIX_PATH");
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
        std::string found = normalizePath(p.string());
        cache[pkg] = found;
        return found;
      }
#else
      /// Lightweight existence check via trying to open a known file fails; skip
#endif
    }
    if (end == std::string::npos) {
      break;
    } else {
      start = end + 1;
    }
  }
  /// Last fallback: if requested package equals current package, use source root
  if (!gCurrentPkgName.empty() && pkg == gCurrentPkgName && !gCurrentPkgRoot.empty()) {
    std::string found = normalizePath(gCurrentPkgRoot);
    cache[pkg] = found;
    return found;
  }
  /// Fallback: search source workspace near current package for a matching package.xml
#if defined(__cpp_lib_filesystem)
  if (!gCurrentPkgRoot.empty()) {
    std::error_code ec;
    fs::path cur = gCurrentPkgRoot;
    /// climb up to find a 'src' dir
    fs::path root = cur;
    while (!root.empty() && root.filename() != "src") {
      fs::path parent = root.parent_path();
      if (parent == root) {
        /// reached filesystem root; avoid infinite loop
        root.clear();
        break;
      }
      root = parent;
    }
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
        /// read package.xml name
        tinyxml2::XMLDocument d;
        if (d.LoadFile(it->path().string().c_str()) == tinyxml2::XML_SUCCESS) {
          auto* r = d.RootElement();
          if (r == nullptr) {
            continue;
          }
          tinyxml2::XMLElement* nameEl = r->FirstChildElement("name");
          if (nameEl == nullptr) {
            for (auto* c = r->FirstChildElement(); c != nullptr; c = c->NextSiblingElement()) {
              if (std::string(c->Name()) == "name") {
                nameEl = c;
                break;
              }
            }
          }
          if ((nameEl != nullptr) && (nameEl->GetText() != nullptr) && pkg == std::string(nameEl->GetText())) {
            std::string found = normalizePath(it->path().parent_path().string());
            cache[pkg] = found;
            return found;
          }
        }
      }
    }
  }
#endif
  cache[pkg] = std::string();
  return std::string();
}

std::string Processor::trim(const std::string& s) {
  return trimWs(s);
}

std::string Processor::getAttr(const tinyxml2::XMLElement* el, const char* name) {
  const char* v = el->Attribute(name);
  return (v != nullptr) ? std::string(v) : std::string();
}

bool Processor::isXacroElement(const tinyxml2::XMLElement* el, const char* local) {
  const char* name = el->Name();
  std::string n = (name != nullptr) ? name : "";
  if (n == local) {
    return true;
  }
  const std::string p = std::string("xacro:") + local;
  return n == p;
}

Processor::Processor() = default;
Processor::~Processor() = default;

bool Processor::run(const Options& opts, std::string* errorMsg) {
  mBaseDir = dirname(opts.mInputPath);
  mVars.clear();
  mMacros.clear();
  mNodeBaseDirs.clear();
  mPropDeps.clear();
  for (const auto& kv : opts.mCliArgs) {
    mVars[kv.first] = kv.second;
  }

  /// Detect current package for ${find('<this_pkg>')} fallback in source trees.
  detectCurrentPackageFrom(mBaseDir);

  if (!loadDocument(opts.mInputPath, errorMsg)) {
    throwFromErrorMsg(errorMsg, "Failed to load XML");
  }
  if (!processDocument(errorMsg)) {
    throwFromErrorMsg(errorMsg, "Failed to process document");
  }

  tinyxml2::XMLPrinter printer;
  mDoc->Print(&printer);
  std::string result = printer.CStr();

  if (opts.mOutputPath.empty()) {
    std::cout << result;
  } else {
    std::ofstream ofs(opts.mOutputPath);
    if (!ofs) {
      if (errorMsg != nullptr) {
        *errorMsg = "Failed to write output";
      }
      throwFromErrorMsg(errorMsg, "Failed to write output");
    }
    ofs << result;
  }
  return true;
}

bool Processor::runToString(const Options& opts, std::string* urdfXml, std::string* errorMsg) {
  mBaseDir = dirname(opts.mInputPath);
  mVars.clear();
  mMacros.clear();
  mArgNames.clear();
  mPropDeps.clear();
  mPropertyBlocks.clear();
  mNodeBaseDirs.clear();
  for (const auto& kv : opts.mCliArgs) {
    mVars[kv.first] = kv.second;
  }

  detectCurrentPackageFrom(mBaseDir);

  if (!loadDocument(opts.mInputPath, errorMsg)) {
    throwFromErrorMsg(errorMsg, "Failed to load XML");
  }
  if (!processDocument(errorMsg)) {
    throwFromErrorMsg(errorMsg, "Failed to process document");
  }

  tinyxml2::XMLPrinter printer;
  mDoc->Print(&printer);
  if (urdfXml != nullptr) {
    *urdfXml = printer.CStr();
  }
  return true;
}

bool Processor::collectArgs(const Options& opts, std::map<std::string, std::string>* argsOut, std::string* errorMsg) {
  if (argsOut == nullptr) {
    return false;
  }
  argsOut->clear();
  mBaseDir = dirname(opts.mInputPath);
  mArgNames.clear();
  mPropDeps.clear();
  if (!loadDocument(opts.mInputPath, errorMsg)) {
    throwFromErrorMsg(errorMsg, "Failed to load XML");
  }
  /// initialize argument map with CLI overrides
  mVars.clear();
  mMacros.clear();
  mPropertyBlocks.clear();
  mNodeBaseDirs.clear();
  for (const auto& kv : opts.mCliArgs) {
    mVars[kv.first] = kv.second;
  }
  /// Only collect args and properties; do not expand macros/includes
  if (!passCollectArgsAndProps(errorMsg)) {
    throwFromErrorMsg(errorMsg, "Failed to collect args and properties");
  }
  /// Copy collected arg values (only those declared via xacro:arg)
  for (const auto& name : mArgNames) {
    auto it = mVars.find(name);
    if (it != mVars.end()) {
      (*argsOut)[name] = it->second;
    }
  }
  return true;
}

bool Processor::loadDocument(const std::string& path, std::string* errorMsg) {
  mDoc = std::make_unique<tinyxml2::XMLDocument>();
  auto rc = mDoc->LoadFile(path.c_str());
  if (rc != tinyxml2::XML_SUCCESS) {
    if (errorMsg != nullptr) {
      *errorMsg = std::string("Failed to load XML: ") + mDoc->ErrorStr();
    }
    return false;
  }
  /// xacro ignores comments; strip them up front to simplify later passes.
  removeComments(mDoc.get());
  return true;
}

bool Processor::processDocument(std::string* errorMsg) {
  /// Pass pipeline: collect globals, expand includes, collect again, then expand macros/ifs.
  /// 1) collect args/props that may affect include paths
  if (!passCollectArgsAndProps(errorMsg)) {
    return false;
  }
  /// 2) expand includes (uses current vars and resolves ${}/$())
  if (!passExpandIncludes(errorMsg)) {
    return false;
  }
  /// 3) collect args/props again (includes might have added more)
  if (!passCollectArgsAndProps(errorMsg)) {
    return false;
  }
  /// 4) expand remaining xacro constructs in-order (macros, if/unless, substitutions)
  if (!passExpand(errorMsg)) {
    return false;
  }
  /// Convert escaped dollar markers back to literal '$' in final output tree.
  restoreDollarMarkers(mDoc->RootElement());
  return true;
}

bool Processor::defineProperty(const tinyxml2::XMLElement* el) {
  std::string name = getAttr(el, "name");
  std::string baseName = name;
  if (!name.empty() && (el->FirstChildElement() != nullptr)) {
    std::vector<tinyxml2::XMLNode*> blocks;
    for (auto* child = el->FirstChild(); child != nullptr; child = child->NextSibling()) {
      blocks.push_back(child->DeepClone(mDoc.get()));
    }
    mPropertyBlocks[name] = std::move(blocks);
    return true;
  }
  std::string value = getAttr(el, "value");
  std::string defaultValue = getAttr(el, "default");
  bool valueHasEval = (value.find("${") != std::string::npos) || (value.find("$(") != std::string::npos);
  bool defaultHasEval
    = (defaultValue.find("${") != std::string::npos) || (defaultValue.find("$(") != std::string::npos);
  if (value.empty()) {
    const char* text = el->GetText();
    if (text != nullptr) {
      value = text;
      valueHasEval = (value.find("${") != std::string::npos) || (value.find("$(") != std::string::npos);
    } else if (!defaultValue.empty()) {
      if (mVars.contains(getDefaultName(name))) {
        return true;
      }
      mVars[name] = "";
      name = getDefaultName(name);
      value = defaultValue;
      valueHasEval = defaultHasEval;
    }
  }
  if (!name.empty()) {
    std::unordered_set<std::string> deps;
    try {
      std::regex reExpr(R"(\$\{([^}]*)\})");
      std::smatch m;
      std::string s = value;
      std::string::const_iterator searchStart(s.cbegin());
      while (std::regex_search(searchStart, s.cend(), m, reExpr)) {
        std::string inner = Processor::trim(std::string(m[1]));
        std::vector<std::string> idents;
        collectIdentifiers(inner, &idents);
        for (const auto& ident : idents) {
          deps.insert(ident);
        }
        searchStart = m.suffix().first;
      }
    } catch (...) {
      /// ignore malformed expressions here; evaluation will raise later if needed
    }
    mPropDeps[name] = std::move(deps);
    std::unordered_set<std::string> visiting;
    std::unordered_set<std::string> visited;
    std::function<bool(const std::string&)> hasCycle = [&](const std::string& cur) -> bool {
      if (visiting.count(cur) != 0U) {
        return true;
      }
      if (visited.count(cur) != 0U) {
        return false;
      }
      visiting.insert(cur);
      auto it = mPropDeps.find(cur);
      if (it != mPropDeps.end()) {
        for (const auto& dep : it->second) {
          if (hasCycle(dep)) {
            return true;
          }
        }
      }
      visiting.erase(cur);
      visited.insert(cur);
      return false;
    };
    if (hasCycle(name)) {
      throw ProcessingError("circular variable definition: " + name);
    }
  }
  std::string expr = trimWs(value);
  if (expr.size() >= 3 && expr[0] == '$' && expr[1] == '{' && expr.back() == '}') {
    expr = trimWs(expr.substr(2, expr.size() - 3));
  }
  /// Handle xacro.load_yaml(file)
  {
    std::smatch m;
    static std::regex cReLoad(R"(^\s*xacro\.load_yaml\((.*)\)\s*$)");
    if (!name.empty() && std::regex_match(expr, m, cReLoad)) {
      std::string arg = Processor::trim(std::string(m[1]));
      std::string argEval = evalStringTemplate(arg, mVars, &mYamlDocs);
      if (argEval == arg) {
        auto it = mVars.find(arg);
        if (it != mVars.end()) {
          argEval = it->second;
        }
      }
      arg = stripQuotes(argEval);
      std::string full;
#if defined(__cpp_lib_filesystem)
      fs::path p(arg);
      if (p.is_absolute()) {
        full = p.string();
      } else {
        full = (fs::path(mBaseDir) / p).string();
      }
#else
      full = (!arg.empty() && arg[0] == '/') ? arg : (mBaseDir + "/" + arg);
#endif
      mYamlDocs[name] = parseYamlFile(full);
      /// store a marker or path for reference (not used later directly)
      mVars.erase(getDefaultName(name));
      mVars[name] = full;
      return true;
    }
  }
  value = evalStringTemplate(value, mVars, &mYamlDocs);
  /// Normalize simple list literals so later indexing sees evaluated entries.
  if (!value.empty() && valueHasEval) {
    std::vector<std::string> listItems;
    if (splitListLiteral(value, &listItems)) {
      std::ostringstream oss;
      oss << "[";
      for (size_t i = 0; i < listItems.size(); ++i) {
        std::string resolved = Processor::trim(evalStringWithVars(listItems[i], mVars));
        if (i != 0U) {
          oss << ", ";
        }
        oss << resolved;
      }
      oss << "]";
      value = oss.str();
    }
  }
  if (!name.empty()) {
    if (!baseName.empty()) {
      if (!valueHasEval) {
        mVars[baseName + "__default_string__"] = "1";
      } else {
        mVars.erase(baseName + "__default_string__");
      }
    }
    mVars.erase(getDefaultName(name));
    mVars[name] = value;
  }
  return true;
}

bool Processor::defineArg(const tinyxml2::XMLElement* el) {
  std::string name = getAttr(el, "name");
  std::string def = getAttr(el, "default");
  if (!name.empty()) {
    mArgNames.insert(name);
  }
  if (mVars.find(name) == mVars.end()) {
    mVars[name] = evalStringTemplate(def, mVars, &mYamlDocs);
  }
  return true;
}

bool Processor::passCollectArgsAndProps([[maybe_unused]] std::string* errorMsg) {
  collectArgsAndProps(mDoc->RootElement(), false, false, true);
  return true;
}

static std::vector<Processor::MacroParam> parseParams(const std::string& s) {
  /// Split macro params while respecting quoted defaults and block params.
  std::vector<Processor::MacroParam> out;
  std::vector<std::string> tokens;
  std::string tok;
  bool inSingle = false;
  bool inDouble = false;
  for (char c : s) {
    if (c == '\'' && !inDouble) {
      inSingle = !inSingle;
      tok.push_back(c);
      continue;
    }
    if (c == '"' && !inSingle) {
      inDouble = !inDouble;
      tok.push_back(c);
      continue;
    }
    if (!inSingle && !inDouble && (std::isspace(static_cast<unsigned char>(c)) != 0)) {
      if (!tok.empty()) {
        tokens.push_back(tok);
        tok.clear();
      }
      continue;
    }
    tok.push_back(c);
  }
  if (!tok.empty()) {
    tokens.push_back(tok);
  }
  for (const auto& rawTok : tokens) {
    Processor::MacroParam p;
    size_t pos = std::string::npos;
    size_t posLen = 0;
    inSingle = false;
    inDouble = false;
    for (size_t i = 0; i < rawTok.size(); ++i) {
      char c = rawTok[i];
      if (c == '\'' && !inDouble) {
        inSingle = !inSingle;
      } else if (c == '"' && !inSingle) {
        inDouble = !inDouble;
      }
      if (inSingle || inDouble) {
        continue;
      }
      if (c == ':' && i + 1 < rawTok.size() && rawTok[i + 1] == '=') {
        pos = i;
        posLen = 2;
        break;
      }
      if (c == '=') {
        pos = i;
        posLen = 1;
        break;
      }
    }
    if (pos != std::string::npos) {
      p.mName = rawTok.substr(0, pos);
      p.mDefaultValue = rawTok.substr(pos + posLen);
      p.mDefaultValue = stripQuotes(trimWs(p.mDefaultValue));
      p.mHasDefault = true;
    } else {
      p.mName = rawTok;
      p.mDefaultValue.clear();
      p.mHasDefault = false;
    }
    p.mName = trimWs(p.mName);
    while (!p.mName.empty() && p.mName[0] == '*') {
      p.mIsBlock = true;
      p.mName = p.mName.substr(1);
    }
    out.push_back(p);
  }
  return out;
}

bool Processor::defineMacro(tinyxml2::XMLElement* el, const std::string& baseDir) {
  /// Register macro and clone its body into the document for later expansion.
  std::string name = getAttr(el, "name");
  std::string params = getAttr(el, "params");
  MacroDef def;
  def.mName = name;
  std::string plainName = name;
  if (plainName.rfind("xacro:", 0) == 0) {
    plainName = plainName.substr(6);
  }
  if (plainName == "call") {
    throw ProcessingError("Invalid use of macro name 'call'");
  }
  def.mParams = parseParams(params);
  /// Clone children of macro element into a container node we own in mDoc
  tinyxml2::XMLNode* container = mDoc->NewElement(("__macro__" + name).c_str());
  for (auto* child = el->FirstChild(); child != nullptr; child = child->NextSibling()) {
    container->InsertEndChild(child->DeepClone(mDoc.get()));
  }
  def.mContent = container;
  def.mBaseDir = baseDir;
  mMacros[name] = def;
  return true;
}

void Processor::removeComments(tinyxml2::XMLNode* node) {
  /// Drop XML comments but preserve whitespace separators between adjacent text nodes.
  if (node == nullptr) {
    return;
  }
  for (auto* child = node->FirstChild(); child != nullptr;) {
    auto* next = child->NextSibling();
    if (child->ToComment() != nullptr) {
      /// When comments sit between text nodes, deleting them would glue words together.
      /// Insert a newline + indentation similar to Python xacro's pretty-printing so
      /// spacing is preserved once comments are removed.
      auto* prevText = (child->PreviousSibling() != nullptr) ? child->PreviousSibling()->ToText() : nullptr;
      auto* nextText = (next != nullptr) ? next->ToText() : nullptr;
      if ((prevText != nullptr) && (nextText != nullptr)) {
        std::string prevVal = prevText->Value();
        std::string nextVal = nextText->Value();
        auto isWs = [](char c) {
          return std::isspace(static_cast<unsigned char>(c)) != 0;
        };
        bool prevNeedsSep = !prevVal.empty() && !isWs(prevVal.back());
        bool nextNeedsSep = !nextVal.empty() && !isWs(nextVal.front());
        if (prevNeedsSep && nextNeedsSep) {
          auto extractIndent = [](const std::string& s) {
            std::string indent;
            auto pos = s.rfind('\n');
            if (pos != std::string::npos) {
              for (size_t i = pos + 1; i < s.size(); ++i) {
                char ch = s[i];
                if (ch == ' ' || ch == '\t') {
                  indent.push_back(ch);
                } else {
                  break;
                }
              }
            }
            return indent;
          };
          std::string indent = extractIndent(prevVal);
          std::string sep = "\n" + indent;
          prevVal += sep;
          nextVal = sep + nextVal;
          prevText->SetValue(prevVal.c_str());
          nextText->SetValue(nextVal.c_str());
        }
      }
      node->DeleteChild(child);
    } else {
      removeComments(child);
    }
    child = next;
  }
}

void Processor::collectArgsAndProps(tinyxml2::XMLElement* root,
                                    bool collectProps,
                                    bool removeProps,
                                    bool removeArgs) {
  if (root == nullptr) {
    return;
  }
  std::vector<tinyxml2::XMLElement*> argsToRemove;
  std::vector<tinyxml2::XMLElement*> propsToRemove;
  struct Frame {
    tinyxml2::XMLElement* mEl;
    bool mInMacro;
    bool mInConditional;
  };
  std::vector<Frame> stack;
  stack.push_back({root, false, false});
  while (!stack.empty()) {
    Frame f = stack.back();
    stack.pop_back();
    auto* cur = f.mEl;
    bool inMacro = f.mInMacro;
    bool inConditional = f.mInConditional;
    if (isXacroElement(cur, "arg")) {
      if (!inMacro && !inConditional) {
        defineArg(cur);
        if (removeArgs) {
          argsToRemove.push_back(cur);
        }
      }
    }
    if (collectProps && isXacroElement(cur, "property")) {
      if (!inMacro && !inConditional) {
        defineProperty(cur);
        if (removeProps) {
          propsToRemove.push_back(cur);
        }
      }
    }
    auto* child = cur->LastChildElement();
    bool childInMacro = inMacro || isXacroElement(cur, "macro");
    bool childInConditional = inConditional || isXacroElement(cur, "if") || isXacroElement(cur, "unless")
                              || isXacroElement(cur, "else") || isXacroElement(cur, "elseif")
                              || isXacroElement(cur, "elif");
    while (child != nullptr) {
      stack.push_back({child, childInMacro, childInConditional});
      child = child->PreviousSiblingElement();
    }
  }
  if (removeArgs) {
    for (auto* a : argsToRemove) {
      if (auto* p = a->Parent()) {
        p->DeleteChild(a);
      }
    }
  }
  if (removeProps) {
    for (auto* pr : propsToRemove) {
      if (auto* p = pr->Parent()) {
        p->DeleteChild(pr);
      }
    }
  }
}

void Processor::collectGlobalsInIncludedDoc(tinyxml2::XMLDocument& inc, const std::string& baseDir) {
  auto* root = inc.RootElement();
  if (root == nullptr) {
    return;
  }
  const std::string prevBase = mBaseDir;
  mBaseDir = baseDir;
  collectArgsAndProps(root, true, true, true);
  mBaseDir = prevBase;
}

bool Processor::loadIncludeFile(const std::string& full,
                                bool collectGlobals,
                                std::vector<tinyxml2::XMLNode*>* cloned,
                                bool* rootPresent,
                                std::string* baseDirOut,
                                std::string* errorMsg) {
  tinyxml2::XMLDocument inc;
  if (inc.LoadFile(full.c_str()) != tinyxml2::XML_SUCCESS) {
    if (errorMsg != nullptr) {
      *errorMsg = std::string("Failed to include ") + full;
    }
    return false;
  }
  removeComments(&inc);
  if (collectGlobals) {
    collectGlobalsInIncludedDoc(inc, dirname(full));
  }
  auto* root = inc.RootElement();
  if (rootPresent != nullptr) {
    *rootPresent = (root != nullptr);
  }
  if (root != nullptr && cloned != nullptr) {
    for (auto* c = root->FirstChild(); c != nullptr; c = c->NextSibling()) {
      cloned->push_back(c->DeepClone(mDoc.get()));
    }
  }
  if (baseDirOut != nullptr) {
    *baseDirOut = dirname(full);
  }
  return true;
}

bool Processor::passCollectMacros([[maybe_unused]] std::string* errorMsg) {
  std::vector<tinyxml2::XMLElement*> toRemove;
  for (auto* el = mDoc->RootElement(); el != nullptr;) {
    std::vector<tinyxml2::XMLElement*> stack;
    stack.push_back(mDoc->RootElement());
    while (!stack.empty()) {
      auto* cur = stack.back();
      stack.pop_back();
      if (isXacroElement(cur, "macro")) {
        auto it = mNodeBaseDirs.find(cur);
        const std::string base = (it != mNodeBaseDirs.end()) ? it->second : mBaseDir;
        defineMacro(cur, base);
        toRemove.push_back(cur);
        /// nested macros are scoped; skip traversing macro body
        continue;
      }
      auto* child = cur->LastChildElement();
      while (child != nullptr) {
        stack.push_back(child);
        child = child->PreviousSiblingElement();
      }
    }
    break;
  }
  /// Delete macros from deepest to shallowest to avoid invalidating nested entries.
  for (auto el : std::ranges::reverse_view(toRemove)) {
    if (auto* parent = el->Parent()) {
      parent->DeleteChild(el);
    }
  }
  return true;
}

static void insertBefore(tinyxml2::XMLNode* parent, tinyxml2::XMLNode* ref, tinyxml2::XMLNode* child) {
  if ((parent == nullptr) || (ref == nullptr) || (child == nullptr)) {
    return;
  }
  tinyxml2::XMLNode* prev = ref->PreviousSibling();
  if (prev != nullptr) {
    parent->InsertAfterChild(prev, child);
  } else {
    parent->InsertFirstChild(child);
  }
}

static bool hasXacroOrSubst(tinyxml2::XMLNode* node) {
  if (node == nullptr) {
    return false;
  }
  std::vector<tinyxml2::XMLNode*> st;
  st.push_back(node);
  while (!st.empty()) {
    auto* cur = st.back();
    st.pop_back();
    if (auto* el = cur->ToElement()) {
      const char* name = el->Name();
      if (name != nullptr && std::string_view(name).rfind("xacro:", 0) == 0) {
        return true;
      }
      const tinyxml2::XMLAttribute* a = el->FirstAttribute();
      while (a != nullptr) {
        const char* val = a->Value();
        if (val != nullptr && std::strchr(val, '$') != nullptr) {
          return true;
        }
        a = a->Next();
      }
    } else if (auto* txt = cur->ToText()) {
      const char* tv = txt->Value();
      if (tv != nullptr && std::strchr(tv, '$') != nullptr) {
        return true;
      }
    }
    for (auto* c = cur->LastChild(); c != nullptr; c = c->PreviousSibling()) {
      st.push_back(c);
    }
  }
  return false;
}

bool Processor::expandIncludesInNode(tinyxml2::XMLNode* node, const std::string& baseDir, std::string* errorMsg) {
  /// Include expansion is conditional-aware: inactive branches are skipped.
  if (node == nullptr) {
    return true;
  }
  /// Stack of elements with their current base directory for resolving relative includes
  struct Frame {
    tinyxml2::XMLElement* mEl;
    std::string mBaseDir;
    bool mActive;
    bool mInMacro;
  };
  std::vector<Frame> stack;
  if (auto* rootEl = node->ToElement()) {
    stack.push_back({rootEl, baseDir, true, false});
  }
  while (!stack.empty()) {
    Frame fr = stack.back();
    stack.pop_back();
    tinyxml2::XMLElement* cur = fr.mEl;
    std::string curBase = fr.mBaseDir;
    bool active = fr.mActive;
    bool inMacro = fr.mInMacro;
    mNodeBaseDirs[cur] = curBase;
    /// For xacro:if/unless, determine child activity but do not splice yet.
    if (isXacroElement(cur, "if") || isXacroElement(cur, "unless")) {
      if (!inMacro) {
        std::string cond = getAttr(cur, "value");
        bool val = evalBool(evalStringTemplate(cond, mVars, &mYamlDocs), mVars);
        if (isXacroElement(cur, "unless")) {
          val = !val;
        }
        for (auto* child = cur->LastChildElement(); child != nullptr; child = child->PreviousSiblingElement()) {
          stack.push_back({child, curBase, active && val, inMacro});
        }
        continue;
      }
      /// Inside macro bodies, defer conditional evaluation to macro expansion.
    }
    if (isXacroElement(cur, "property")) {
      if (active && !inMacro) {
        defineProperty(cur);
      }
      continue;
    }
    if (isXacroElement(cur, "include")) {
      if (!active) {
        /// skip expanding includes in inactive branches
        continue;
      }
      if (inMacro) { // defer include expansion to macro expansion
        continue;
      }
      std::string file = getAttr(cur, "filename");
      file = evalStringTemplate(file, mVars, &mYamlDocs);
      if (file.empty()) {
        continue;
      }
      std::string full;
#if defined(__cpp_lib_filesystem)
      fs::path pfile(file);
      if (pfile.is_absolute()) {
        full = pfile.string();
      } else {
        full = (fs::path(curBase) / pfile).string();
      }
#else
      if (!file.empty() && file[0] == '/') {
        full = file;
      } else {
        full = curBase + "/" + file;
      }
#endif
      /// Insert children of included root at include position
      std::vector<tinyxml2::XMLNode*> cloned;
      std::string newBase;
      bool rootPresent = false;
      if (!loadIncludeFile(full, true, &cloned, &rootPresent, &newBase, errorMsg)) {
        return false;
      }
      if (!rootPresent) {
        continue;
      }
      auto* parent = cur->Parent();
      for (auto* n : cloned) {
        insertBefore(parent, cur, n);
      }
      /// Push newly inserted elements with their base dir for further include resolution
      for (auto* n : cloned) {
        if (auto* e = n->ToElement()) {
          stack.push_back({e, newBase, active, inMacro});
        }
      }
      parent->DeleteChild(cur);
      continue;
    }
    /// Regular element: push its children with same base
    bool childInMacro = inMacro || isXacroElement(cur, "macro");
    for (auto* child = cur->LastChildElement(); child != nullptr; child = child->PreviousSiblingElement()) {
      stack.push_back({child, curBase, active, childInMacro});
    }
  }
  return true;
}

bool Processor::passExpandIncludes(std::string* errorMsg) {
  return expandIncludesInNode(mDoc->RootElement(), mBaseDir, errorMsg);
}

bool Processor::handleIfUnless(tinyxml2::XMLElement* el, std::vector<tinyxml2::XMLNode*>* insertedOut) {
  /// Splice children into the parent if the condition is true; otherwise remove.
  if (isXacroElement(el, "if") || isXacroElement(el, "unless")) {
    std::string cond = getAttr(el, "value");
    std::string condEval = evalStringTemplate(cond, mVars, &mYamlDocs);
    bool val = evalBool(condEval, mVars);
    if (isXacroElement(el, "unless")) {
      val = !val;
    }
    auto* parent = el->Parent();
    if (!val) {
      if (parent != nullptr) {
        parent->DeleteChild(el);
      }
      mModified = true;
      return true;
    }
    /// True: splice children
    std::vector<tinyxml2::XMLNode*> cloned;
    for (auto* c = el->FirstChild(); c != nullptr; c = c->NextSibling()) {
      cloned.push_back(c->DeepClone(mDoc.get()));
    }
    for (auto* n : cloned) {
      insertBefore(parent, el, n);
    }
    parent->DeleteChild(el);
    mModified = true;
    if (insertedOut != nullptr) {
      insertedOut->insert(insertedOut->end(), cloned.begin(), cloned.end());
    }
    return true;
  }
  return false;
}

bool Processor::expandXacroElement(tinyxml2::XMLElement* el,
                                   const std::unordered_map<std::string, std::string>& scope) {
  /// xacro:element dynamically renames the current element.
  if (!isXacroElement(el, "element")) {
    return false;
  }
  std::string rawName = getAttr(el, "xacro:name");
  if (rawName.empty()) {
    rawName = getAttr(el, "name");
  }
  std::string newName = evalStringTemplate(rawName, scope, &mYamlDocs);
  if (newName.empty()) {
    throw ProcessingError("xacro:element requires a non-empty name");
  }
  el->SetName(newName.c_str());
  /// Drop xacro:* attributes (e.g., xacro:name) from the renamed element
  std::vector<std::string> toRemove;
  const tinyxml2::XMLAttribute* attr = el->FirstAttribute();
  while (attr != nullptr) {
    std::string attrName = (attr->Name() != nullptr) ? attr->Name() : "";
    if (attrName.rfind("xacro:", 0) == 0) {
      toRemove.push_back(attrName);
    }
    attr = attr->Next();
  }
  for (const auto& an : toRemove) {
    el->DeleteAttribute(an.c_str());
  }
  mModified = true;
  return true;
}

void Processor::substituteAttributes(tinyxml2::XMLElement* el,
                                     const std::unordered_map<std::string, std::string>& scope) {
  /// Replace ${...} in all attribute values
  const tinyxml2::XMLAttribute* a = el->FirstAttribute();
  std::vector<std::pair<std::string, std::string>> updates;
  while (a != nullptr) {
    std::string name = a->Name();
    std::string val = a->Value();
    std::string newv = evalStringTemplate(val, scope, &mYamlDocs);
    if (newv != val) {
      updates.emplace_back(name, newv);
    }
    a = a->Next();
  }
  for (auto& kv : updates) {
    el->SetAttribute(kv.first.c_str(), kv.second.c_str());
    mModified = true;
  }
}

bool Processor::applyXacroAttribute(tinyxml2::XMLElement* el,
                                    const std::unordered_map<std::string, std::string>& scope) {
  if (el == nullptr) {
    return false;
  }
  tinyxml2::XMLNode* parentNode = el->Parent();
  auto* parentEl = (parentNode != nullptr) ? parentNode->ToElement() : nullptr;
  if (parentEl == nullptr) {
    return false;
  }

  std::string name = getAttr(el, "name");
  if (name.empty()) {
    name = getAttr(el, "xacro:name");
  }
  std::string value = getAttr(el, "value");
  if (value.empty()) {
    const char* text = el->GetText();
    if (text != nullptr) {
      value = text;
    }
  }
  name = evalStringTemplate(name, scope, &mYamlDocs);
  value = evalStringTemplate(value, scope, &mYamlDocs);
  if (name.empty()) {
    return false;
  }

  parentEl->SetAttribute(name.c_str(), value.c_str());
  parentNode->DeleteChild(el);
  mModified = true;
  return true;
}

const Processor::MacroDef* Processor::findMacro(const std::vector<std::string>& names) const {
  for (const auto& n : names) {
    auto it = mMacros.find(n);
    if (it != mMacros.end()) {
      return &it->second;
    }
  }
  return nullptr;
}

bool Processor::expandMacroCall(tinyxml2::XMLElement* el,
                                const std::unordered_map<std::string, std::string>& parentScope,
                                const std::unordered_map<std::string, std::vector<tinyxml2::XMLNode*>>* parentBlocks) {
  /// Expand a macro call in-place, creating a fresh local scope for params and blocks.
  auto isKnownXacroTag = [](const std::string& name) {
    return name == "macro" || name == "property" || name == "arg" || name == "include" || name == "if"
           || name == "unless" || name == "element" || name == "attribute" || name == "insert_block" || name == "call";
  };
  auto uniqPush = [](std::vector<std::string>* v, const std::string& n) {
    if (n.empty()) {
      return;
    }
    if (std::find(v->begin(), v->end(), n) == v->end()) {
      v->push_back(n);
    }
  };

  bool isCallTag = isXacroElement(el, "call");
  std::string target = isCallTag ? getAttr(el, "macro") : std::string((el->Name() != nullptr) ? el->Name() : "");
  if (!isCallTag) {
    if (!target.starts_with(cXacroPrefix)) {
      return false;
    }
  }
  if (isCallTag && target.empty()) {
    target = getAttr(el, "xacro:macro");
  }
  if (isCallTag) {
    if (target.empty()) {
      throw ProcessingError("xacro:call: missing attribute 'macro'");
    }
    target = evalStringTemplate(target, parentScope, &mYamlDocs);
    if (target.empty()) {
      throw ProcessingError("xacro:call: missing attribute 'macro'");
    }
  }
  if (debugEnabled()) {
    debugLog("expandMacroCall target=" + target);
  }
  std::string plainTarget = target;
  if (plainTarget.starts_with(cXacroPrefix)) {
    plainTarget = plainTarget.substr(cXacroPrefix.size());
  }

  std::vector<std::string> candidates;
  uniqPush(&candidates, target);
  uniqPush(&candidates, plainTarget);
  if (plainTarget != target) {
    uniqPush(&candidates, std::string("xacro:") + plainTarget);
  } else if (!plainTarget.empty()) {
    uniqPush(&candidates, std::string("xacro:") + plainTarget);
  }

  const MacroDef* mptr = findMacro(candidates);
  if (mptr == nullptr) {
    if (isCallTag) {
      std::string msgName = target.empty() ? plainTarget : target;
      if (msgName.rfind("xacro:", 0) != 0 && !msgName.empty()) {
        msgName = "xacro:" + msgName;
      }
      throw ProcessingError("unknown macro name: " + msgName);
    }
    if (target.starts_with(cXacroPrefix) && !isKnownXacroTag(plainTarget)) {
      throw ProcessingError("unknown macro name: " + target);
    }
    return false;
  }
  MacroDef m = *mptr;
  if (debugEnabled()) {
    debugLog("expandMacroCall resolved=" + m.mName + " params=" + std::to_string(m.mParams.size()));
  }
  ScopedRestore<std::unordered_map<std::string, MacroDef>> macroScopeGuard(mMacros);
  /// Build local scope seeded from caller (outer macro/global scope).
  LocalScope scopeFrame(parentScope);
  auto& scope = scopeFrame.map();
  /// Collect block arguments passed as child nodes of the macro call.
  /// If a block argument is itself an xacro:insert_block, resolve it using the parent macro's blocks first
  /// to avoid self-recursive block substitution (matches Python xacro behavior).
  std::unordered_map<std::string, std::vector<tinyxml2::XMLNode*>> blocks;
  std::vector<tinyxml2::XMLNode*> callBlocks;
  for (auto* ch = el->FirstChild(); ch != nullptr; ch = ch->NextSibling()) {
    auto* chel = ch->ToElement();
    if (chel == nullptr) {
      /// block args ignore non-element nodes
      continue;
    }
    if ((parentBlocks != nullptr) && isXacroElement(chel, "insert_block")) {
      std::string bname = getAttr(chel, "name");
      auto itpb = parentBlocks->find(bname);
      if (itpb != parentBlocks->end()) {
        for (auto* bn : itpb->second) {
          if (bn != nullptr) {
            callBlocks.push_back(bn->DeepClone(mDoc.get()));
          }
        }
        /// resolved this placeholder
        continue;
      }
    }
    callBlocks.push_back(ch);
  }
  size_t blockArgIdx = 0;
  struct RawParam {
    std::string mName;
    std::string mValue;
    bool mFromAttr = false;
  };
  std::vector<RawParam> rawParams;
  rawParams.reserve(m.mParams.size());
  for (const auto& p : m.mParams) {
    if (p.mIsBlock) {
      if (blockArgIdx < callBlocks.size()) {
        blocks[p.mName].push_back(callBlocks[blockArgIdx]->DeepClone(mDoc.get()));
      }
      ++blockArgIdx;
    } else {
      const char* av = el->Attribute(p.mName.c_str());
      if ((av == nullptr) && !p.mHasDefault) {
        throw ProcessingError("missing required parameter '" + p.mName + "' for macro '" + m.mName + "'");
      }
      rawParams.push_back({p.mName, (av != nullptr) ? std::string(av) : p.mDefaultValue, av != nullptr});
    }
  }
  /// Resolve parameters in definition order. Call-site attributes and defaults are evaluated
  /// strictly in the parent scope (caller), not in terms of other macro params.
  const std::unordered_map<std::string, std::string> baseScope = scope;
  std::unordered_map<std::string, std::string> evaluatedParams;
  for (const auto& rp : rawParams) {
    std::string v = evalStringTemplate(rp.mValue, baseScope, &mYamlDocs);
    evaluatedParams[rp.mName] = v;
  }
  scope = baseScope;
  for (const auto& kv : evaluatedParams) {
    scope[kv.first] = kv.second;
  }
  /// Clone macro content with substitution
  std::vector<tinyxml2::XMLNode*> cloned;
  for (auto* c = m.mContent->FirstChild(); c != nullptr; c = c->NextSibling()) {
    cloned.push_back(c->DeepClone(mDoc.get()));
  }
  auto* parent = el->Parent();
  for (auto* n : cloned) {
    insertBefore(parent, el, n);
  }
  parent->DeleteChild(el);
  if (debugEnabled()) {
    debugLog("expandMacroCall inserted=" + std::to_string(cloned.size()) + " children for " + m.mName);
  }
  /// After insertion, we must traverse inserted nodes and substitute
  for (auto* n : cloned) {
    struct NodeFrame {
      tinyxml2::XMLNode* mNode;
      std::string mBaseDir;
    };
    /// DFS
    std::vector<NodeFrame> st;
    st.push_back({n, m.mBaseDir.empty() ? mBaseDir : m.mBaseDir});
    while (!st.empty()) {
      NodeFrame curFrame = st.back();
      st.pop_back();
      auto* cur = curFrame.mNode;
      const std::string& curBase = curFrame.mBaseDir;
      if (auto* ce = cur->ToElement()) {
        /// Nested macro definitions: register and remove from output.
        if (isXacroElement(ce, "macro")) {
          defineMacro(ce, curBase);
          if (auto* par = ce->Parent()) {
            par->DeleteChild(ce);
          }
          mModified = true;
          continue;
        }
        /// Expand nested macro calls using the current scope chain.
        if (expandMacroCall(ce, scope, &blocks)) {
          continue;
        }
        /// Evaluate local-scope conditionals inside macro bodies
        if (isXacroElement(ce, "if") || isXacroElement(ce, "unless")) {
          std::string cond = getAttr(ce, "value");
          std::string condEval = evalStringTemplate(cond, scope, &mYamlDocs);
          bool val = evalBool(condEval, scope);
          if (isXacroElement(ce, "unless")) {
            val = !val;
          }
          auto* parent = ce->Parent();
          if (parent != nullptr) {
            if (!val) {
              parent->DeleteChild(ce);
              mModified = true;
              /// skip pushing children
              continue;
            } else {
              /// splice children
              std::vector<tinyxml2::XMLNode*> clonedIf;
              for (auto* c3 = ce->FirstChild(); c3 != nullptr; c3 = c3->NextSibling()) {
              clonedIf.push_back(c3->DeepClone(mDoc.get()));
              }
              for (auto* n2 : clonedIf) {
                insertBefore(parent, ce, n2);
              }
              parent->DeleteChild(ce);
              for (auto* n2 : clonedIf) {
                st.push_back({n2, curBase});
              }
              mModified = true;
              continue;
            }
          }
        }
        if (isXacroElement(ce, "include")) {
          std::string file = getAttr(ce, "filename");
          file = evalStringTemplate(file, scope, &mYamlDocs);
          if (!file.empty()) {
            std::string full;
            fs::path pfile(file);
            if (pfile.is_absolute()) {
              full = pfile.string();
            } else {
              full = (fs::path(curBase) / pfile).string();
            }
            std::vector<tinyxml2::XMLNode*> clonedInc;
            std::string newBase;
            bool rootPresent = false;
            if (!loadIncludeFile(full, false, &clonedInc, &rootPresent, &newBase, nullptr)) {
              throw ProcessingError(std::string("Failed to include ") + full);
            }
            if (rootPresent) {
              auto* parent = ce->Parent();
              for (auto* nInc : clonedInc) {
                insertBefore(parent, ce, nInc);
              }
              parent->DeleteChild(ce);
              for (auto& it : std::ranges::reverse_view(clonedInc)) {
                st.push_back({it, newBase});
              }
            } else if (auto* parent = ce->Parent()) {
              parent->DeleteChild(ce);
            }
          } else if (auto* parent = ce->Parent()) {
            parent->DeleteChild(ce);
          }
          mModified = true;
          continue;
        }
        /// Turn <xacro:element> into a regular element using the macro's scope
        expandXacroElement(ce, scope);
        /// Handle xacro:property with local scoping (default) or global override
        if (isXacroElement(ce, "property")) {
          std::string pname = getAttr(ce, "name");
          std::string scopeAttr = getAttr(ce, "scope");
          std::string pval = getAttr(ce, "value");
          if (pval.empty()) {
            const char* text = ce->GetText();
            if (text != nullptr) {
              pval = text;
            }
          }
          pval = evalStringTemplate(pval, scope, &mYamlDocs);
          if (!pname.empty()) {
            scopeFrame.setProperty(pname, pval, scopeAttr, mVars);
          }
          if (auto* par = ce->Parent()) {
            par->DeleteChild(ce);
          }
          mModified = true;
          continue;
        }
        /// Handle xacro:attribute on the containing element
        if (isXacroElement(ce, "attribute")) {
          applyXacroAttribute(ce, scope);
          continue;
        }
        /// Handle xacro:insert_block
        if (isXacroElement(ce, "insert_block")) {
          std::string bname = getAttr(ce, "name");
          auto bit = blocks.find(bname);
          auto* par = ce->Parent();
          if (par != nullptr) {
            std::vector<tinyxml2::XMLNode*> insertedNodes;
            if (bit != blocks.end()) {
              insertedNodes.reserve(bit->second.size());
              for (auto* bn : bit->second) {
                auto* clone = bn->DeepClone(mDoc.get());
                insertBefore(par, ce, clone);
                insertedNodes.push_back(clone);
              }
            }
            par->DeleteChild(ce);
            /// process newly inserted nodes with the same macro scope
            for (auto& insertedNode : std::ranges::reverse_view(insertedNodes)) {
              if (insertedNode != nullptr) {
                st.push_back({insertedNode, curBase});
              }
            }
          }
          mModified = true;
          /// done with this node
          continue;
        }
        /// Substitute attrs using local scope
        substituteAttributes(ce, scope);
      } else if (auto* tx = cur->ToText()) {
        const char* tv = tx->Value();
        if (tv != nullptr) {
          std::string nv = evalStringTemplate(tv, scope, &mYamlDocs);
          if (nv != tv) {
            tx->SetValue(nv.c_str());
            mModified = true;
          }
        }
      }
      for (auto* c2 = cur->LastChild(); c2 != nullptr; c2 = c2->PreviousSibling()) {
        st.push_back({c2, curBase});
      }
    }
  }
  mModified = true;
  return true;
}

bool Processor::expandElement(tinyxml2::XMLElement* el, std::vector<tinyxml2::XMLNode*>* inserted) {
  /// Expand one xacro element; return whether the node should remain for traversal.
  /// Handle inline properties/args defined inside expanded content
  if (isXacroElement(el, "property")) {
    defineProperty(el);
    if (auto* p = el->Parent()) {
      p->DeleteChild(el);
    }
    mModified = true;
    return false;
  }
  if (isXacroElement(el, "arg")) {
    defineArg(el);
    if (auto* p = el->Parent()) {
      p->DeleteChild(el);
    }
    mModified = true;
    return false;
  }
  {
    if (handleIfUnless(el, inserted)) {
      /// el was deleted or spliced; don't traverse its children
      return false;
    }
  }
  if (isXacroElement(el, "macro")) {
    auto it = mNodeBaseDirs.find(el);
    const std::string base = (it != mNodeBaseDirs.end()) ? it->second : mBaseDir;
    defineMacro(el, base);
    if (auto* p = el->Parent()) {
      p->DeleteChild(el);
    }
    mModified = true;
    return false;
  }
  bool wasXacroElement = expandXacroElement(el, mVars);
  if (!wasXacroElement && expandMacroCall(el, mVars, nullptr)) {
    /// el replaced and processed; don't traverse old children
    return false;
  }
  if (isXacroElement(el, "attribute")) {
    applyXacroAttribute(el, mVars);
    return false;
  }
  if (isXacroElement(el, "insert_block")) {
    std::string bname = getAttr(el, "name");
    auto it = mPropertyBlocks.find(bname);
    auto* parent = el->Parent();
    if (parent != nullptr) {
      if (it != mPropertyBlocks.end()) {
        for (auto* bn : it->second) {
          if (bn != nullptr) {
            insertBefore(parent, el, bn->DeepClone(mDoc.get()));
          }
        }
      }
      parent->DeleteChild(el);
    }
    mModified = true;
    return false;
  }
  substituteAttributes(el, mVars);
  /// el remains; traverse its children
  return true;
}

bool Processor::expandNode(tinyxml2::XMLNode* node) {
  /// DFS traversal that expands xacro elements and performs text substitutions.
  std::vector<tinyxml2::XMLNode*> st;
  st.push_back(node);
  while (!st.empty()) {
    auto* cur = st.back();
    st.pop_back();
    if (auto* el = cur->ToElement()) {
      std::vector<tinyxml2::XMLNode*> inserted;
      bool keep = expandElement(el, &inserted);
      if (!keep) {
        /// Node was deleted/replaced; don't use 'cur' further.
        for (auto& i : std::ranges::reverse_view(inserted)) {
          st.push_back(i);
        }
        /// Node was deleted/replaced; don't use 'cur' further.
        continue;
      }
    } else if (auto* txt = cur->ToText()) {
      const char* tv = txt->Value();
      if (tv != nullptr) {
        std::string nv = evalStringTemplate(tv, mVars, &mYamlDocs);
        if (nv != tv) {
          txt->SetValue(nv.c_str());
          mModified = true;
        }
      }
    }
    for (auto* c = cur->LastChild(); c != nullptr; c = c->PreviousSibling()) {
      st.push_back(c);
    }
  }
  return true;
}

void Processor::restoreDollarMarkers(tinyxml2::XMLNode* node) {
  /// Restore escaped "$$" markers after all substitutions complete.
  if (node == nullptr) {
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
      while (a != nullptr) {
        std::string val = (a->Value() != nullptr) ? a->Value() : "";
        if (val.find(kDollarMarker) != std::string::npos) {
          std::replace(val.begin(), val.end(), kDollarMarker, '$');
          updates.emplace_back(a->Name(), val);
        }
        a = a->Next();
      }
      for (auto& kv : updates) {
        el->SetAttribute(kv.first.c_str(), kv.second.c_str());
      }
    } else if (auto* txt = cur->ToText()) {
      const char* tv = txt->Value();
      if (tv != nullptr) {
        std::string val = tv;
        if (val.find(kDollarMarker) != std::string::npos) {
          std::replace(val.begin(), val.end(), kDollarMarker, '$');
          txt->SetValue(val.c_str());
        }
      }
    }
    for (auto* c = cur->LastChild(); c != nullptr; c = c->PreviousSibling()) {
      st.push_back(c);
    }
  }
}

bool Processor::passExpand([[maybe_unused]] std::string* errorMsg) {
  /// Re-run expansion until a fixed point (or safety cap) to match Python xacro behavior.
  /// Iterate expansion until a fixed point or safety limit
  if (!hasXacroOrSubst(mDoc->RootElement())) {
    return true;
  }
  for (int iter = 0; iter < 10; ++iter) {
    mModified = false;
    if (debugEnabled()) {
      debugLog("passExpand iter=" + std::to_string(iter));
    }
    expandNode(mDoc->RootElement());
    if (!mModified) {
      break;
    }
  }
  return true;
}

} // namespace xacro_cpp
