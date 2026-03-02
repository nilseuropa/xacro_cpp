# xacro_cpp

Minimal C++ implementation of core xacro processing for common URDF workflows.

## Current status

`xacro_cpp` supports:
- `xacro:arg` with CLI overrides (`name:=value`)
- `xacro:property` (`value`, `default`, property blocks, `scope="global"` in macro scope)
- `xacro:include filename="..."` (relative path per including file)
- conditional include expansion (`xacro:if`, `xacro:unless`)
- package lookup via `${find('pkg')}` and `$(find pkg)`
  - `ament_index_cpp` when available
  - fallback to `AMENT_PREFIX_PATH` / `COLCON_PREFIX_PATH`
  - source-tree package fallback by scanning nearby `src/**/package.xml`
- substitutions in attributes and text
  - `${...}` templates
  - `$(arg name)`
  - numeric expressions (tinyexpr)
  - list/index access like `${var[2]}`
- macros
  - definition: `<xacro:macro ...>`
  - invocation: `<xacro:m .../>`, `<m .../>`, `<xacro:call macro="..."/>`
  - block params (`*block`) with `<xacro:insert_block .../>`
  - dynamic macro names via `xacro:call`
- dynamic XML helpers
  - `<xacro:element .../>`
  - `<xacro:attribute .../>`
- YAML loading with `xacro.load_yaml(...)`
  - backed by `yaml-cpp`
  - nested map/list/scalar parsing
  - access forms: `cfg.key`, `cfg['key']`, `cfg[0]`, `cfg.get('key', default)`

Pipeline order:
1. collect args/properties
2. expand includes
3. collect args/properties again
4. iterative expansion to fixed point (macros, conditionals, substitutions)

## Known gaps vs Python xacro

Not supported:
- `lazy_eval="false"`
- property forwarding (`:=^`)
- property removal (`remove="true"`)
- include glob patterns
- include namespace (`ns="..."`)
- optional include (`optional="true"`)
- property name validation parity
- double-underscore property guard parity
- headless block params (`**block`)
- rospack-style `$(cwd)`
- Python boolean operators (`and`, `or`, `not`) and full Python eval semantics

Behavior differences:
- macro parameter defaults/values are evaluated in caller scope, not in terms of earlier macro params
- YAML map values are not stringified directly
- output formatting/attribute ordering may differ slightly
- warning behavior is not fully aligned with Python xacro

## API

Public headers: `include/xacro_cpp/`

Main entry points (`xacro_cpp/processor.hpp`):
- `struct Options`
  - `mInputPath`, `mOutputPath`, `mCliArgs`
- `class Processor`
  - `bool run(const Options&, std::string* errorMsg)`
  - `bool runToString(const Options&, std::string* urdfXml, std::string* errorMsg)`
  - `bool collectArgs(const Options&, std::map<std::string, std::string>* argsOut, std::string* errorMsg)`
- expression helpers
  - `evalNumber`, `evalBool`, `evalStringTemplate`

Failures are reported via `xacro_cpp::ProcessingError` exceptions (and `errorMsg` when provided).

## Build

Dependencies:
- C++20 compiler
- `tinyxml2`
- `yaml-cpp`

Optional (if available):
- `ament_index_cpp`
- `ament_cmake` (for ROS 2 package mode)

Plain CMake:
```bash
mkdir -p build
cd build
cmake ..
make -j
```

ROS 2 workspace:
```bash
colcon build --packages-select xacro_cpp
```

## CLI

```bash
xacro_cpp input.xacro [-o output.xml] [name:=value ...]
```

## Tests

Test binary: `test_xacro_parsing`

It compares generated XML against Python `xacro` outputs across fixtures under `test/test_files/`, with a few currently skipped parity tests for unsupported features.

Test dependencies (`BUILD_TESTING=ON`):
- `pybind11`
- Python 3 interpreter + development headers
- `gtest`
- Python package `xacro`

Run:
```bash
colcon build --packages-select xacro_cpp --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run xacro_cpp test_xacro_parsing
```

or:

```bash
colcon test --packages-select xacro_cpp --event-handlers console_direct+ --ctest-args -V
```
