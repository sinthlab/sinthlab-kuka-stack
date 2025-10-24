cmake_minimum_required(VERSION 3.22)
project(sinthlab_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scenario script
set(SINTHLAB_BRINGUP_PY_SCRIPTS
  scripts/apple_pull_controller.py
  scripts/apple_pluck_impedance_control.py
)
foreach(_script ${SINTHLAB_BRINGUP_PY_SCRIPTS})
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${_script})
    file(CHMOD ${CMAKE_CURRENT_SOURCE_DIR}/${_script}
      PERMISSIONS
        OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE)
  endif()
endforeach()
install(
  PROGRAMS
    ${SINTHLAB_BRINGUP_PY_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
