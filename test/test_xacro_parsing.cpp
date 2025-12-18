//
// Created by barabas on 2025. 10. 28..
//


#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <filesystem>
#include <iostream>
#include <map>
#include <string>

#include "gtest/gtest.h"

#include "xacro_cpp/processor.hpp"


namespace fs = std::filesystem;
namespace py = pybind11;

using xacro_cpp::Options;
using xacro_cpp::Processor;

using namespace testing;


static const std::string test_files_path = TEST_FILES_PATH;
static const std::string result_files_path = TEST_RESULTS_PATH;


auto vectorToString(const std::vector<std::string>& list) {
  std::string result;

  for (const auto& l : list) {
    result += l + "\n";
  }

  return result;
}

class XacroTestFixture : public Test {
  inline static py::module_ os;
  inline static py::module_ sys;
  inline static py::module_ xacro;

protected:
  static void SetUpTestSuite() {
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

    // Some distro builds of python xacro don't expose builtin abs() at the top level.
    // Mirror the tinyexpr support so the Python reference parser can evaluate test files.
    py::exec(R"(
import builtins
import xacro as _xacro
if isinstance(getattr(_xacro, "_global_symbols", None), dict):
    _xacro._global_symbols.setdefault("abs", builtins.abs)
    py_ns = _xacro._global_symbols.get("python")
    try:
        py_ns.setdefault("abs", builtins.abs)
    except Exception:
        pass
)");
  }

  static void parse_python(const std::string& input, const std::string& output) {
    sys.attr("argv") = py::make_tuple("xacro", input, "-o", output);
    xacro.attr("main")();
  }

  static void parse_python_with_arg(const std::string& input,
                                    const std::string& output,
                                    const std::string& key,
                                    const std::string& value) {
    sys.attr("argv") = py::make_tuple("xacro", input, key + ":=" + value, "-o", output);
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

  static void parse_cpp_with_arg(const std::string& input,
                                 const std::string& output,
                                 const std::string key,
                                 const std::string value) {

    Options opt;

    opt.input_path = input;
    opt.output_path = output;
    opt.cli_args = {{key, value}};

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

num_re = re.compile(r'[-+]?\d+\.\d+')

def normalize_numbers(s: str) -> str:
  if not isinstance(s, str):
    return s
  def replacer(match):
    num = float(match.group())
    if num.is_integer():
      return str(int(num))
    return str(num)
  return num_re.sub(replacer, s)

def normalize_text(s: str) -> str:
  return normalize_numbers((s or '').strip())

def parse_attributes(attr: dict) -> dict:
  return {k: normalize_numbers(v) for k, v in attr.items()}

def compare_xml_files(a, b):
  diff = []
  def same_xml(x, y, path):
    if x.tag != y.tag:
      diff.append(f"{path}: tag mismatch '{x.tag}' != '{y.tag}'")
    if parse_attributes(x.attrib) != parse_attributes(y.attrib):
      diff.append(f"{path}/{x.tag}: attributes {x.attrib} != {y.attrib}")
    if normalize_text(x.text) != normalize_text(y.text):
      diff.append(f"{path}/{x.tag}: text '{(x.text or '').strip()}' != '{(y.text or '').strip()}'")
    if normalize_text(x.tail) != normalize_text(y.tail):
      diff.append(f"{path}/{x.tag}: tail '{(x.tail or '').strip()}' != '{(y.tail or '').strip()}'")
    if len(x) != len(y):
      diff.append(f"{path}/{x.tag}: child count {len(x)} != {len(y)}")
    for idx, (c1, c2) in enumerate(zip(x, y)):
      same_xml(c1, c2, f"{path}/{x.tag}[{idx}]")
  same_xml(ET.parse(a).getroot(), ET.parse(b).getroot(), "")
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

  static void parseAndCompareWithArg(const std::string& file_name, const std::string key, const std::string value) {
    std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
    std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";

    parse_python_with_arg(test_files_path + file_name + ".xacro", pythonResultFile, key, value);
    parse_cpp_with_arg(test_files_path + file_name + ".xacro", cppResultFile, key, value);

    std::vector<std::string> diff;
    ASSERT_TRUE(compareXML(pythonResultFile, cppResultFile, diff)) << vectorToString(diff);
  }

  static void parseAndCompareWithExpectedFixture(const std::string& file_name) {
    std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";
    parse_cpp(test_files_path + file_name + ".xacro", cppResultFile);

    std::vector<std::string> diff;
    ASSERT_TRUE(compareXML(test_files_path + file_name + "_expected.xml", cppResultFile, diff))
      << vectorToString(diff);
  }

  static void parseAndExpectFailure(const std::string& file_name) {
    std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
    std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";
    EXPECT_ANY_THROW(parse_python(test_files_path + file_name + ".xacro", pythonResultFile));
    EXPECT_ANY_THROW(parse_cpp(test_files_path + file_name + ".xacro", cppResultFile));
  }
};

TEST_F(XacroTestFixture, basic_operations) {
  // basic operations: + - * /
  parseAndCompare("basic_operations");
}

TEST_F(XacroTestFixture, parenthesis) {
  parseAndCompare("parenthesis");
}

TEST_F(XacroTestFixture, precedence_order) {
  parseAndCompare("precedence_order");
}

TEST_F(XacroTestFixture, test_remove_comments) {
  parseAndCompare("test_remove_comments");
}

TEST_F(XacroTestFixture, div_by_zero) {
  std::string file_name = "div_by_zero";

  std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
  std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";

  EXPECT_ANY_THROW(parse_python(test_files_path + file_name + ".xacro", pythonResultFile));
  EXPECT_ANY_THROW(parse_cpp(test_files_path + file_name + ".xacro", cppResultFile));
}

TEST_F(XacroTestFixture, syntax_error) {
  std::string file_name = "syntax_error";

  std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
  std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";

  EXPECT_NO_THROW(parse_python(test_files_path + file_name + ".xacro", pythonResultFile));
  EXPECT_ANY_THROW(parse_cpp(test_files_path + file_name + ".xacro", cppResultFile));
}

TEST_F(XacroTestFixture, list_params) {
  parseAndCompare("list_params");
}

TEST_F(XacroTestFixture, properties) {
  parseAndCompare("properties");
}

TEST_F(XacroTestFixture, conditional_blocks) {
  parseAndCompare("conditional");
}

TEST_F(XacroTestFixture, including_other_xacro_files) {
  parseAndCompare("including_other");
}

TEST_F(XacroTestFixture, including_nested) {
  parseAndCompare("including_nested");
}

TEST_F(XacroTestFixture, macro_cross_parameter_cpp_only) {
  std::string file = "macros_advanced";
  // std::string pythonResultFile = result_files_path + file + "_python_result.xml";
  // EXPECT_ANY_THROW(parse_python(test_files_path + file + ".xacro", pythonResultFile));

  std::string cppResultFile = result_files_path + file + "_cpp_result.xml";
  parse_cpp(test_files_path + file + ".xacro", cppResultFile);

  std::vector<std::string> diff;
  ASSERT_TRUE(compareXML(test_files_path + file + "_expected.xml", cppResultFile, diff)) << vectorToString(diff);
}

TEST_F(XacroTestFixture, macro_block_parameters) {
  parseAndCompare("macros_block");
}

TEST_F(XacroTestFixture, conditional_else_chains) {
  parseAndCompare("conditional_else");
}

TEST_F(XacroTestFixture, include_failure) {
  std::string file_name = "include_failure";
  std::string pythonResultFile = result_files_path + file_name + "_python_result.xml";
  std::string cppResultFile = result_files_path + file_name + "_cpp_result.xml";

  EXPECT_ANY_THROW(parse_python(test_files_path + file_name + ".xacro", pythonResultFile));
  EXPECT_ANY_THROW(parse_cpp(test_files_path + file_name + ".xacro", cppResultFile));
}

TEST_F(XacroTestFixture, duplicate_property_definitions) {
  parseAndCompare("duplicate_property");
}

/// Tests from the official Xacro python package

TEST_F(XacroTestFixture, test_invalid_property_name) {
  parseAndExpectFailure("test_invalid_property_name");
  // Expected: exception complaining about 'invalid.name'
}
TEST_F(XacroTestFixture, test_double_underscore_property_name) {
  parseAndExpectFailure("test_double_underscore_property_name");
  // Expected: exception complaining about '__hidden'
}
TEST_F(XacroTestFixture, test_call_simple) {
  parseAndCompare("test_call_simple");
}
TEST_F(XacroTestFixture, test_dynamic_macro_names) {
  parseAndCompare("test_dynamic_macro_names");
  // Expected: <a><b>bar</b>/a>
}
TEST_F(XacroTestFixture, test_dynamic_macro_name_clash) {
  parseAndExpectFailure("test_dynamic_macro_name_clash");
}
TEST_F(XacroTestFixture, test_dynamic_macro_undefined) {
  parseAndExpectFailure("test_dynamic_macro_undefined");
}
TEST_F(XacroTestFixture, test_call_missing_macro_attr) {
  parseAndExpectFailure("test_call_missing_macro_attr");
}
TEST_F(XacroTestFixture, test_macro_undefined) {
  parseAndExpectFailure("test_macro_undefined");
}
TEST_F(XacroTestFixture, test_xacro_element) {
  parseAndCompare("test_xacro_element");
}
TEST_F(XacroTestFixture, test_xacro_attribute) {
  parseAndCompare("test_xacro_attribute");
  // Expected: <a><tag A="foo"/><tag B="bar"/></a>
}
TEST_F(XacroTestFixture, test_inorder_processing) {
  parseAndCompare("test_inorder_processing");
  /// Expected:
  /// <xml> <a foo="1 1.0"/> <b bar="2 2.0"/> </xml>
}
TEST_F(XacroTestFixture, test_should_replace_before_macroexpand) {
  parseAndCompare("test_should_replace_before_macroexpand");
  // Expected: <a><in_the_outer><in_the_inner><woot /></in_the_inner></in_the_outer></a>
}
TEST_F(XacroTestFixture, test_evaluate_macro_params_before_body) {
  parseAndCompare("test_evaluate_macro_params_before_body");
}
TEST_F(XacroTestFixture, test_macro_params_escaped_string) {
  parseAndCompare("test_macro_params_escaped_string");
  // Expected: <a><bar a="1 -2" c="3"/></a>
}
TEST_F(XacroTestFixture, test_property_replacement) {
  parseAndCompare("test_property_replacement");
}

/// Note: test_property_scope_parent is ommitted

TEST_F(XacroTestFixture, test_property_scope_global) {
  parseAndCompare("test_property_scope_global");
}
TEST_F(XacroTestFixture, test_property_in_comprehension) {
  parseAndCompare("test_property_in_comprehension");
}
TEST_F(XacroTestFixture, test_math_ignores_spaces) {
  parseAndCompare("test_math_ignores_spaces");
}
TEST_F(XacroTestFixture, test_substitution_args_find) {
  parseAndCompare("test_substitution_args_find");
}
TEST_F(XacroTestFixture, test_substitution_args_arg) {
  parseAndCompareWithArg("test_substitution_args_arg", "sub_arg", "my_arg");
}
TEST_F(XacroTestFixture, test_escaping_dollar_braces) {
  parseAndCompare("test_escaping_dollar_braces");
  // Note: expected <a b="${foo}" c="$${foo}" d="text ${foo}" e="text $${foo}" f="$(pwd)" />
}
TEST_F(XacroTestFixture, test_just_a_dollar_sign) {
  parseAndCompare("test_just_a_dollar_sign");
}
TEST_F(XacroTestFixture, test_multiple_insert_blocks) {
  parseAndCompare("test_multiple_insert_blocks");
  // Expected: <a> <a_block/> <a_block/> </a>
}
TEST_F(XacroTestFixture, test_multiple_blocks) {
  parseAndCompare("test_multiple_blocks");
  // Expected: <a><block2/><block1/></a>, note the reversed order of block parameters!
}
TEST_F(XacroTestFixture, test_integer_stays_integer) {
  parseAndCompare("test_integer_stays_integer");
}
TEST_F(XacroTestFixture, test_insert_block_property) {
  parseAndCompare("test_insert_block_property");
  // Expected: <a><foo><some_block attr="2">bar</some_block></foo></a>
}
TEST_F(XacroTestFixture, test_include) {
  parseAndCompare("test_include");
}
TEST_F(XacroTestFixture, test_include_glob) {
  GTEST_SKIP() << "Disabled, include patterns not supported";
  parseAndCompare("test_include_glob");
}
/// Note: test_include_nonexistent already present in our tests
TEST_F(XacroTestFixture, test_include_from_variable) {
  parseAndCompare("test_include_from_variable");
}
TEST_F(XacroTestFixture, test_include_recursive) {
  parseAndCompare("test_include_recursive");
}
TEST_F(XacroTestFixture, test_include_with_namespace) {
  parseAndCompare("test_include_with_namespace");
  // Expected: <a> <inc1/><inc2/><main var="main" A="2" B="3"/> </a>
}
TEST_F(XacroTestFixture, test_macro_has_new_scope) {
  parseAndCompare("test_macro_has_new_scope");
  /// Expected:
  /// <root>
  ///  <outer prop="outer"/>
  ///  <inner prop="inner"/>
  ///  <outer prop="outer"/>
  ///  <inner prop="inner"/>
  /// </root>
}
TEST_F(XacroTestFixture, test_macro_property_scoping) {
  parseAndCompare("test_macro_property_scoping");
}
TEST_F(XacroTestFixture, test_boolean_if_statement) {
  parseAndCompare("test_boolean_if_statement");
}
TEST_F(XacroTestFixture, test_invalid_if_statement) {
  parseAndExpectFailure("test_invalid_if_statement");
  // Expected: exception
}
TEST_F(XacroTestFixture, test_integer_if_statement) {
  parseAndCompare("test_integer_if_statement");
}
TEST_F(XacroTestFixture, test_float_if_statement) {
  parseAndCompare("test_float_if_statement");
}
TEST_F(XacroTestFixture, test_property_if_statement) {
  parseAndCompare("test_property_if_statement");
  // Expected: <robot> <b/><c/> </robot>
}
TEST_F(XacroTestFixture, test_consecutive_if) {
  parseAndCompare("test_consecutive_if");
}
TEST_F(XacroTestFixture, test_equality_expression_in_if_statement) {
  parseAndCompare("test_equality_expression_in_if_statement");
  // Expected: <a> <foo>bar</foo> </a>
}
TEST_F(XacroTestFixture, test_no_evaluation) {
  parseAndCompare("test_no_evaluation");
}
TEST_F(XacroTestFixture, test_math_expressions) {
  parseAndCompare("test_math_expressions");
  // Expected: <a><foo function="0.0"/></a>
}
TEST_F(XacroTestFixture, test_consider_non_elements_if) {
  parseAndCompare("test_consider_non_elements_if");
}
TEST_F(XacroTestFixture, test_consider_non_elements_block) {
  parseAndCompare("test_consider_non_elements_block");
  /// Expected
  /// <a>
  ///  <!-- comment -->
  ///  foo
  ///  <a_block />
  /// </a>
}
TEST_F(XacroTestFixture, test_recursive_evaluation) {
  parseAndCompare("test_recursive_evaluation");
  /// Expected: <robot> <a doubled="84"/> </robot>
}
TEST_F(XacroTestFixture, test_recursive_evaluation_wrong_order) {
  parseAndCompare("test_recursive_evaluation_wrong_order");
}
TEST_F(XacroTestFixture, test_recursive_definition) {
  parseAndExpectFailure("test_recursive_definition");
  // Expected: exception on circular variable definition
}
TEST_F(XacroTestFixture, test_greedy_property_evaluation) {
  parseAndCompare("test_greedy_property_evaluation");
}
TEST_F(XacroTestFixture, test_multiple_recursive_evaluation) {
  parseAndCompare("test_multiple_recursive_evaluation");
}
TEST_F(XacroTestFixture, test_multiple_definition_and_evaluation) {
  parseAndCompare("test_multiple_definition_and_evaluation");
}
TEST_F(XacroTestFixture, test_transitive_evaluation) {
  parseAndCompare("test_transitive_evaluation");
}
TEST_F(XacroTestFixture, test_multi_tree_evaluation) {
  parseAndCompare("test_multi_tree_evaluation");
}
TEST_F(XacroTestFixture, test_from_issue) {
  parseAndCompare("test_from_issue");
}
TEST_F(XacroTestFixture, test_recursive_bad_math) {
  parseAndExpectFailure("test_recursive_bad_math");
}
TEST_F(XacroTestFixture, test_default_param) {
  parseAndCompare("test_default_param");
  /// Expected:
  /// <robot>
  ///  <link name="foo"/>
  ///  <joint name="foo_joint" type="fixed">
  ///   <origin rpy="0 0 0" xyz="0 0 0"/>
  ///   <parent link="base_link"/>
  ///   <child link="foo"/>
  ///  </joint>
  /// </robot>
}
TEST_F(XacroTestFixture, test_default_param_override) {
  parseAndCompare("test_default_param_override");
  /// Expected:
  /// <robot>
  ///  <link name="foo"/>
  ///  <joint name="foo_joint" type="fixed">
  ///   <origin rpy="0 0 0" xyz="0 0 0"/>
  ///   <parent link="bar"/>
  ///   <child link="foo"/>
  ///  </joint>
  /// </robot>
}
TEST_F(XacroTestFixture, test_param_missing) {
  parseAndExpectFailure("test_param_missing");
  // Expected: exception
}
TEST_F(XacroTestFixture, test_default_arg) {
  parseAndCompare("test_default_arg");
}
TEST_F(XacroTestFixture, test_default_arg_override) {
  parseAndCompareWithArg("test_default_arg_override", "foo", "4");
}
TEST_F(XacroTestFixture, test_default_arg_missing) {
  parseAndExpectFailure("test_default_arg_missing");
  // Expected: exception
}
TEST_F(XacroTestFixture, test_default_arg_empty) {
  parseAndCompare("test_default_arg_empty");
}
TEST_F(XacroTestFixture, test_broken_include_error_reporting) {
  parseAndExpectFailure("test_broken_include_error_reporting");
}
// Note: test_create_subdirs and test_set_root_directory are ommitted
// Note: tests related to iterable literals are ommitted
TEST_F(XacroTestFixture, test_issue_68_numeric_arg) {
  parseAndCompare("test_issue_68_numeric_arg");
}
TEST_F(XacroTestFixture, test_transitive_arg_evaluation) {
  parseAndCompare("test_transitive_arg_evaluation");
}
TEST_F(XacroTestFixture, test_macro_name_with_colon) {
  parseAndCompare("test_macro_name_with_colon");
  // Note: should raise warning about
  //  macro names must not contain prefix 'xacro:'
}
TEST_F(XacroTestFixture, test_overwrite_globals) {
  parseAndCompare("test_overwrite_globals");
  // TODO: Should raise some warning about overwritten value
}
TEST_F(XacroTestFixture, test_no_double_evaluation) {
  parseAndCompare("test_no_double_evaluation");
  // Expected: <a><d d="${a}"> a=2 b=1 c=${a} </d></a>
}
TEST_F(XacroTestFixture, test_property_forwarding) {
  GTEST_SKIP() << "Disabled, property forwarding not supported";
  parseAndCompare("test_property_forwarding");
}
TEST_F(XacroTestFixture, test_redefine_global_symbol) {
  parseAndCompare("test_redefine_global_symbol");
  // TODO: Should raise some warning about global redefinition
}
TEST_F(XacroTestFixture, test_include_lazy) {
  parseAndCompare("test_include_lazy");
}
TEST_F(XacroTestFixture, test_issue_63_fixed_with_inorder_processing) {
  parseAndCompare("test_issue_63_fixed_with_inorder_processing");
}
// Note: tests related to advanced namespacing and scoping are ommitted
TEST_F(XacroTestFixture, test_yaml_support) {
  parseAndCompare("test_yaml_support");
}
TEST_F(XacroTestFixture, test_yaml_support_list_of_x) {
  parseAndCompare("test_yaml_support_list_of_x");
}
// Note: test_yaml_* for advanced yaml types are ommitted
TEST_F(XacroTestFixture, test_xacro_exist_optional) {
  GTEST_SKIP() << "Disabled, xacro 'include optional' not supported";
  parseAndCompare("test_xacro_exist_optional");
}
TEST_F(XacroTestFixture, test_macro_default_param_evaluation_order) {
  parseAndCompare("test_macro_default_param_evaluation_order");
  // Expected: <a><f val="21"/><f val="someString"/></a>
}
TEST_F(XacroTestFixture, test_default_property) {
  parseAndCompare("test_default_property");
  // Expected: <a><foo/></a>
}
// Note: test_unicode_* are ommitted, tests related to process return values and command line syntax are ommitted
TEST_F(XacroTestFixture, test_remove_property) {
  GTEST_SKIP() << "Disabled, property removal is not supported";
  parseAndCompare("test_remove_property");
}
TEST_F(XacroTestFixture, test_invalid_property_definitions) {
  parseAndExpectFailure("test_invalid_property_definitions");
}

// Tests made from examples on https://github.com/ros/xacro/wiki
TEST_F(XacroTestFixture, test_block_double_star_insert) {
  parseAndCompare("test_block_double_star_insert");
  // Note: in-place block parameter does not seem to work
}
TEST_F(XacroTestFixture, test_introduction_example) {
  parseAndCompare("test_introduction_example");
}


int main(int argc, char** argv) {
  Py_Initialize();
  std::string pythonVersion = Py_GetVersion();

  std::cout << "python version: " << pythonVersion << std::endl;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
