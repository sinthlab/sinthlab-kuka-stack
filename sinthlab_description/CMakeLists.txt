cmake_minimum_required(VERSION 3.10)
project(sinthlab_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install ros2_control resources (initial joint positions, configs)
install(
  DIRECTORY ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
