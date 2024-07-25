cmake_minimum_required(VERSION 3.8)
project(rapid_bag)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# find system dependencies
find_package(yaml-cpp 0.6.2 REQUIRED)

# find ros dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# build recorder components
ament_auto_add_library(recorder SHARED
  src/recorder.cpp
)
target_include_directories(recorder PUBLIC 
  include
  ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(recorder
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(recorder ${ros2_interface_packages})
rclcpp_components_register_node(recorder
  PLUGIN "rapid_bag::Recorder"
  EXECUTABLE recorder_node
)

# build converter node
ament_auto_add_executable(converter_node
  src/converter.cpp
)
target_include_directories(converter_node PUBLIC 
  include
  ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(converter_node
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(converter_node ${ros2_interface_packages})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()