Overview
This is a minimal, pure C++ port of core xacro functionality focused on common URDF workflows. It aims to be Python-free and dependency-light.

Supported (initial MVP)
- Properties: <xacro:property name="..." value="..."/>
- Arguments: <xacro:arg name="..." default="..."/>, CLI overrides as name:=value
- Substitutions: ${...} expressions using a tiny evaluator (numbers, + - * /, comparisons)
- Package lookup: ${find('pkg')} and $(find pkg) resolve to the package share directory (via ament_index_cpp or env fallback)
  - If ament_index/env paths fail and the requested package is the one containing the input file, it falls back to that package’s source root.
- Macros: <xacro:macro name="m" params="a b:=1">...</xacro:macro> and calls via <m a="..."/>
- Conditionals: <xacro:if value="...">...</xacro:if>, <xacro:unless value="...">...</xacro:unless>
- Includes: <xacro:include filename="..."/>

Not yet implemented
- Python blocks, YAML, ROS param substitution, advanced namespaces, property scopes identical to Python xacro, block macros, and plugin features. These can be added incrementally if needed.

Build
- Requires TinyXML2 development package installed on the system (e.g., libtinyxml2-dev).
- As a plain CMake project: mkdir build && cd build && cmake .. && make -j
- In a ROS 2 workspace: colcon build --packages-select xacro_cpp

CLI
- xacro_cpp input.xacro [-o output.xml] [name:=value ...]
