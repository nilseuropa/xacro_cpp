cmake_minimum_required(VERSION 3.10)
project(xacro_cpp LANGUAGES C CXX)

# Optionally build as a ROS 2 ament package if ament_cmake is available
find_package(ament_cmake QUIET)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Try both common TinyXML2 package names and fall back to a raw library
find_package(TinyXML2 QUIET)
find_package(tinyxml2 QUIET)

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

add_library(tinyexpr STATIC
  src/tinyexpr.c
)
target_include_directories(tinyexpr PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(xacro_cpp_lib
  src/processor.cpp
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
target_link_libraries(xacro_cpp_lib PRIVATE tinyexpr ${TINYXML2_TARGET})

add_executable(xacro_cpp src/main.cpp)
target_link_libraries(xacro_cpp PRIVATE xacro_cpp_lib)

install(TARGETS xacro_cpp xacro_cpp_lib tinyexpr
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

if(ament_cmake_FOUND)
  ament_package()
endif()
