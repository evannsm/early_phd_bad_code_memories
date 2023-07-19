cmake_minimum_required(VERSION 3.5)
project(one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)


# install(TARGETS
# run1
# DESTINATION lib/${PROJECT_NAME})


add_executable(simplepub src/simplepub.cpp)
ament_target_dependencies(simplepub rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)
add_executable(simplesub src/simplesub.cpp)
ament_target_dependencies(simplesub rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)


add_executable(fly src/fly.cpp)
ament_target_dependencies(fly rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)

add_executable(ratesctrl src/ratesctrl.cpp)
ament_target_dependencies(ratesctrl rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)

add_executable(run1 src/run.cpp)
ament_target_dependencies(run1 rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)

add_executable(first_orig src/first_orig.cpp)
ament_target_dependencies(first_orig rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)


install(TARGETS
run1
fly
ratesctrl
simplepub
simplesub
first_orig

DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
