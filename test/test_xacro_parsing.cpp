//
// Created by barabas on 2025. 10. 28..
//


#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <map>
#include <string>
#include "gtest/gtest.h"
#include <filesystem>

#include "xacro_cpp/processor.hpp"
#include <filesystem>


namespace fs = std::filesystem;
namespace py = pybind11;

using xacro_cpp::Options;
using xacro_cpp::Processor;

using namespace testing;



static const std::string test_files_path = TEST_FILES_PATH;
static const std::string result_files_path = TEST_RESULTS_PATH;



auto vectorToString(const std::vector<std::string>& list)
{
  std::string result;

  for(const auto& l : list) {
    result += l + "\n";
  }

  return result;
}

class XacroTestFixture : public Test
{
  inline static py::module_ os;
  inline static py::module_ sys;
  inline static py::module_ xacro;
protected:
  static void SetUpTestSuite()
  {
    fs::remove_all(TEST_RESULTS_PATH);
    fs::create_directories(TEST_RESULTS_PATH);

    py::exec(R"(
import sys
import site
sys.path.extend([
    '/usr/lib/python3.12',
    '/usr/lib/python3.12/lib-dynload',
    site.getusersitepackages()
])
)");
    os = py::module_::import("os");
    sys = py::module_::import("sys");
    xacro = py::module_::import("xacro");
  }

  static void parse_python(const std::string& input, const std::string& output) {
    sys.attr("argv") = py::make_tuple("xacro", input, "-o", output);
    xacro.attr("main")();
  }

  static void parse_cpp(const std::string& input, const std::string& output) {

    Options opt;

    opt.input_path = input;
    opt.output_path = output;

    Processor p;
    std::string err;
    if (!p.run(opt, &err)) {
      std::cerr << "xacro_cpp error: " << err << "\n";
      return;
    }
  }

  static bool compareXML(const std::string& path1, const std::string& path2, std::vector<std::string>& diff) {
    const char* script = R"(
import xml.etree.ElementTree as ET
import re

def normalize_numbers(s: str) -> str:
    def replacer(match):
        num = float(match.group())
        if num.is_integer():
            return str(int(num))
        return str(num)
    return re.sub(r'\d+\.\d+', replacer, s)

def parse_attributes(attr: dict) -> dict:
  return {k: normalize_numbers(v) for k, v in attr.items()}

def compare_xml_files(a, b):
  diff = []
  def same_xml(x, y):
    if x.tag != y.tag:
      err_msg = "tag mismatch: " + str(x) + " != " + str(y)
      diff.append(err_msg)
      return False
    if parse_attributes(x.attrib) != parse_attributes(y.attrib):
      err_msg = "attribute mismatch in " + x.tag + ": " + str(x.attrib) + " != " + str(y.attrib)
      diff.append(err_msg)
      return False
    if (x.text or '').strip() != (y.text or '').strip():
      err_msg = "text mismatch in " + x.tag + ": " + str(x.text or '') + " != " + str(y.text or '')
      diff.append(err_msg)
      return False
    if len(x) != len(y):
      err_msg = "length mismatch in " + x.tag + ": " + str(len(x) or '') + " != " + str(len(x) or '')
      diff.append(err_msg)
      return False
    return all(same_xml(c1, c2) for c1, c2 in zip(x, y))
  same_xml(ET.parse(a).getroot(), ET.parse(b).getroot())
  return diff
  )";
    py::exec(script);
    auto compare_xml_files = py::globals()["compare_xml_files"];
    diff = compare_xml_files(path1, path2).cast<std::vector<std::string>>();

    return diff.empty();
  }

  static void parseAndCompare(const std::string& file_name) {
    std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
    std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";

    parse_python(test_files_path + file_name + ".xacro", pythonResultFile);
    parse_cpp(test_files_path + file_name + ".xacro", cppResultFile);

    std::vector<std::string> diff;
    ASSERT_TRUE(compareXML(pythonResultFile, cppResultFile, diff)) << vectorToString(diff);
  }

};


TEST_F(XacroTestFixture, basic_operations)
{
  // basic operations: + - * /
    parseAndCompare("basic_operations");
}

TEST_F(XacroTestFixture, parenthesis)
{
  parseAndCompare("parenthesis");
}

TEST_F(XacroTestFixture, precedence_order)
{
  parseAndCompare("precedence_order");
}

TEST_F(XacroTestFixture, div_by_zero)
{
  std::string file_name = "div_by_zero";

  std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
  std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";

  EXPECT_ANY_THROW(parse_python(test_files_path + file_name + ".xacro", pythonResultFile));
  EXPECT_ANY_THROW(parse_cpp(test_files_path + file_name + ".xacro", cppResultFile));
}

TEST_F(XacroTestFixture, syntax_error)
{
  std::string file_name = "syntax_error";

  std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
  std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";

  EXPECT_NO_THROW(parse_python(test_files_path + file_name + ".xacro", pythonResultFile));
  EXPECT_ANY_THROW(parse_cpp(test_files_path + file_name + ".xacro", cppResultFile));
}

TEST_F(XacroTestFixture, list_params)
{
  parseAndCompare("list_params");
}

TEST_F(XacroTestFixture, properties)
{
  parseAndCompare("properties");
}

TEST_F(XacroTestFixture, conditional_blocks)
{
  parseAndCompare("conditional");
}

TEST_F(XacroTestFixture, including_other_xacro_files)
{
  parseAndCompare("including_other");
}

TEST_F(XacroTestFixture, macro_cross_parameter_cpp_only)
{
  std::string file = "macros_advanced";
  // std::string pythonResultFile = result_files_path + file + "_python_result.xml";
  // EXPECT_ANY_THROW(parse_python(test_files_path + file + ".xacro", pythonResultFile));

  std::string cppResultFile = result_files_path + file + "_cpp_result.xml";
  parse_cpp(test_files_path + file + ".xacro", cppResultFile);

  std::vector<std::string> diff;
  ASSERT_TRUE(compareXML(test_files_path + file + "_expected.xml", cppResultFile, diff)) << vectorToString(diff);
}

TEST_F(XacroTestFixture, macro_block_parameters)
{
  parseAndCompare("macros_block");
}

TEST_F(XacroTestFixture, conditional_else_chains)
{
  parseAndCompare("conditional_else");
}

TEST_F(XacroTestFixture, include_failure)
{
  std::string file_name = "include_failure";
  std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
  std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";

  EXPECT_ANY_THROW(parse_python(test_files_path + file_name + ".xacro", pythonResultFile));
  EXPECT_ANY_THROW(parse_cpp(test_files_path + file_name + ".xacro", cppResultFile));
}

TEST_F(XacroTestFixture, duplicate_property_definitions)
{
  parseAndCompare("duplicate_property");
}


int main(int argc, char** argv) {
  Py_Initialize();
  std::string pythonVersion = Py_GetVersion();

  std::cout << "python version: " << pythonVersion << std::endl;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
