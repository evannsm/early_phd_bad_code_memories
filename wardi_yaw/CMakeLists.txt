cmake_minimum_required(VERSION 3.8)
project(wardi_yaw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(thrust_pub src/thrust_pub.cpp)
ament_target_dependencies(thrust_pub rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)
add_executable(yaw_pub src/yaw_pub.cpp)
ament_target_dependencies(yaw_pub rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)
add_executable(move_pub1 src/move_pub1.cpp)
ament_target_dependencies(move_pub1 rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)
add_executable(move2 src/move2.cpp)
ament_target_dependencies(move2 rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)
add_executable(loc1 src/loc1.cpp)
ament_target_dependencies(loc1 rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)
add_executable(move3 src/move3.cpp)
ament_target_dependencies(move3 rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)
add_executable(move4 src/move4.cpp)
ament_target_dependencies(move4 rclcpp std_msgs geometry_msgs sensor_msgs px4_msgs)

# Install Cpp Executables
install(TARGETS
thrust_pub
yaw_pub
move_pub1
move2
loc1
move3
move4
DESTINATION lib/${PROJECT_NAME})


# Install Python Modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/thrust_done.py
  scripts/yaw_work.py
  scripts/yaw_done.py
  scripts/move_work.py
  scripts/move_work2.py
  scripts/circle.py
  scripts/yaw_all_work.py
  scripts/yaw_all_work2.py
  scripts/final_xy_b4_yaw.py
  scripts/work_angles.py
  scripts/work_angles2.py
  scripts/rosbagreader.py
  scripts/finnal_with_data.py
  scripts/implement_CBF.py
  DESTINATION lib/${PROJECT_NAME}
)



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
