cmake_minimum_required(VERSION 3.8)
project(follower_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  follower_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

file(GLOB SRC src/*.cpp src/*/*.cpp)

add_library(follower_follow SHARED ${SRC})

# target_include_directories(follower_follow PUBLIC
#   include
#   /usr/local/include
# )
ament_export_include_directories(include)

ament_target_dependencies(follower_follow PUBLIC
${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_follower.xml)

add_executable(follower_follow_node src/follower_follow_node.cpp)

target_link_libraries(follower_follow_node follower_follow)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS
  follower_follow
  follower_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config control
  DESTINATION share/${PROJECT_NAME})

install(TARGETS follower_follow
  EXPORT export_follower_follow
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


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

## EXPORTS
ament_export_targets(export_follower_follow HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
