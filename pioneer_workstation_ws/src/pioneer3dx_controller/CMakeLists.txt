cmake_minimum_required(VERSION 3.8)
project(pioneer3dx_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "messages/ChangeMode.msg"
  "messages/ConfigLed.msg"
  "messages/ConfigPid.msg"
  "messages/ConfigRobot.msg"
  "services/ChangeMode.srv"
  "services/ConfigLed.srv"
  "services/ConfigPid.srv"
  "services/ConfigRobot.srv"
  DEPENDENCIES service_msgs
)

add_executable(Pioneer3dx_Controller src/main.cpp src/p3dx_node.cpp)
ament_target_dependencies(Pioneer3dx_Controller rclcpp nav_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(Pioneer3dx_Controller ${cpp_typesupport_target})
install(TARGETS Pioneer3dx_Controller DESTINATION lib/${PROJECT_NAME})

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
