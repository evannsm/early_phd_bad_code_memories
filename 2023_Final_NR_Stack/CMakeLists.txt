cmake_minimum_required(VERSION 3.8)
project(Final_NR_Wardi_Tracker_Stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) #ROS2 Dependencies
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED) #Pakage Dependencies
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create CPP Executables
add_executable(offboard_pub src/offboard_pub.cpp)
ament_target_dependencies(offboard_pub rclcpp std_msgs px4_msgs)

# Install Cpp Executables
install(TARGETS
offboard_pub
DESTINATION lib/${PROJECT_NAME})

# Install Python Modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  python_scripts/nr_tracker_final.py
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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
