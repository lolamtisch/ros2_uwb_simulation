cmake_minimum_required(VERSION 3.8)
project(ros2_uwb_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs)
find_package(std_srvs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/UwbPosition.srv"
  "srv/RealPosition.srv"
  "msg/UwbData.msg"
  DEPENDENCIES std_msgs std_srvs
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  src/uwb_distance.py
  src/uwb_broadcaster.py
  src/uwb_position.py
  src/test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY rviz/
  DESTINATION rviz
)

install(
  DIRECTORY launch/
  DESTINATION launch
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
