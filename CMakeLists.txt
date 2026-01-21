cmake_minimum_required(VERSION 3.10)
project(xacro_cpp LANGUAGES C CXX)

# Optionally build as a ROS 2 ament package if ament_cmake is available
find_package(ament_cmake QUIET)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Try both common TinyXML2 package names and fall back to a raw library
find_package(TinyXML2 QUIET)
find_package(tinyxml2 QUIET)
find_package(yaml_cpp_vendor QUIET)
find_package(yaml-cpp REQUIRED)

set(TINYXML2_TARGET "")
if(TARGET TinyXML2::TinyXML2)
  set(TINYXML2_TARGET TinyXML2::TinyXML2)
elseif(TARGET tinyxml2::tinyxml2)
  set(TINYXML2_TARGET tinyxml2::tinyxml2)
else()
  find_library(TINYXML2_LIB NAMES tinyxml2)
  if(NOT TINYXML2_LIB)
    message(FATAL_ERROR "tinyxml2 not found. Please install tinyxml2 dev package.")
  endif()
  add_library(tinyxml2_unknown STATIC IMPORTED)
  set_target_properties(tinyxml2_unknown PROPERTIES IMPORTED_LOCATION "${TINYXML2_LIB}")
  set(TINYXML2_TARGET tinyxml2_unknown)
endif()

add_library(xacro_cpp_lib
  src/expr.cpp
  src/processor.cpp
  src/tinyexpr.c
  src/yaml.cpp
)
target_include_directories(xacro_cpp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
find_package(ament_index_cpp QUIET)
if(TARGET ament_index_cpp::ament_index_cpp)
  target_compile_definitions(xacro_cpp_lib PRIVATE XACRO_HAVE_AMENT_INDEX=1)
  target_link_libraries(xacro_cpp_lib PUBLIC ament_index_cpp::ament_index_cpp)
endif()
target_link_libraries(xacro_cpp_lib PRIVATE ${TINYXML2_TARGET})
target_link_libraries(xacro_cpp_lib PUBLIC yaml-cpp)

# Provide a convenient namespaced alias for consumers
add_library(xacro_cpp::xacro_cpp ALIAS xacro_cpp_lib)

add_executable(xacro_cpp src/main.cpp)
target_link_libraries(xacro_cpp PRIVATE xacro_cpp_lib)

# Install and export library target for downstream packages
install(TARGETS xacro_cpp_lib
  EXPORT xacro_cppTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# The CLI executable is optional for consumers; install without exporting
install(TARGETS xacro_cpp
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

# Export CMake targets and include dirs for both ament and plain CMake consumers
install(EXPORT xacro_cppTargets
  NAMESPACE xacro_cpp::
  DESTINATION share/${PROJECT_NAME}/cmake
)



if(BUILD_TESTING)
  find_package(pybind11 REQUIRED)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  find_package(GTest REQUIRED)

  add_executable(test_xacro_parsing test/test_xacro_parsing.cpp)
  target_link_libraries(test_xacro_parsing PRIVATE pybind11::embed Python3::Python gtest xacro_cpp_lib)
  target_include_directories(test_xacro_parsing PRIVATE include)

  add_test(NAME test_xacro_parsing
          COMMAND ${CMAKE_BINARY_DIR}/test_xacro_parsing
  )

  target_compile_definitions(test_xacro_parsing PRIVATE TEST_FILES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/test_files/")
  target_compile_definitions(test_xacro_parsing PRIVATE TEST_RESULTS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/result_files/")

  gtest_discover_tests(test_xacro_parsing)

  install(TARGETS test_xacro_parsing
          DESTINATION lib/${PROJECT_NAME})
endif()


if(ament_cmake_FOUND)
  # Export interface details for ament consumers
  ament_export_targets(xacro_cppTargets HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  if(TARGET ament_index_cpp::ament_index_cpp)
    ament_export_dependencies(ament_index_cpp)
  endif()
  ament_export_dependencies(yaml_cpp_vendor yaml-cpp)
  ament_package()
endif()
