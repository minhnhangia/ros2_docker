cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim)

install(TARGETS
  turtle_spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
