cmake_minimum_required(VERSION 3.8)
project(follower_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(follower_msgs REQUIRED)

include_directories(
  include
)

file(GLOB SRC src/*.cpp src/*/*.cpp)

add_library(FOLLOWER_FOLLOW SHARED ${SRC})

target_include_directories(FOLLOWER_FOLLOW PUBLIC
  include
  /usr/local/include
)

ament_target_dependencies(FOLLOWER_FOLLOW
  rclcpp
  std_msgs
  follower_msgs
)

add_executable(follower_follow_node src/follower_follow_node.cpp)

target_link_libraries(follower_follow_node FOLLOWER_FOLLOW)

install(TARGETS
follower_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


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

ament_package()
