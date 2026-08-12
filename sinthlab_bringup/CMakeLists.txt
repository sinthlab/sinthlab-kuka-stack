cmake_minimum_required(VERSION 3.22)
project(sinthlab_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python scenario script
set(SINTHLAB_BRINGUP_PY_SCRIPTS
  orchestrators/apple_pluck_orchestrator.py
  orchestrators/perturb_orchestrator.py
  orchestrators/restricted_plane_orchestrator.py
  orchestrators/maze_orchestrator.py
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

# Install config files (YAML, etc.)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
