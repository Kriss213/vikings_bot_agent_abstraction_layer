cmake_minimum_required(VERSION 3.8)
project(vikings_bot_agent_abstraction_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(vikings_bot_agent_abstraction_layer REQUIRED)

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


# install custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  "msg/Bid.msg"
  "msg/UGVStatus.msg"
  "msg/LoaderStatus.msg"
  "msg/UnloaderStatus.msg"
  
  "srv/LoadUnloadRequest.srv"

  DEPENDENCIES builtin_interfaces
)

# install Python executables
#ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(agent_layer)
# ament_python_install_module(scripts/classes.py)
# ament_python_install_module(scripts/CBBA.py)

install(
  PROGRAMS
    agent_layer/agents/courier.py
    agent_layer/agents/loader.py
    agent_layer/agents/unloader.py
  DESTINATION
    lib/${PROJECT_NAME}
)
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
