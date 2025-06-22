cmake_minimum_required(VERSION 3.8)
project(pub_sub_exercise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED) # for service example (std_srvs/srv/empty.hpp)

set(dependencies 
  rclcpp
  std_msgs
  std_srvs
)

# This tells the compiler where to find header files for c++ code
include_directories(
  include
)

# The following creates an executable program using the specified source file.
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher ${dependencies})

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber ${dependencies})

install(
  DIRECTORY scripts launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install cpp executables
install (
  TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
