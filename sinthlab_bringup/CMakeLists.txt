cmake_minimum_required(VERSION 3.22)
project(sinthlab_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scenario script
set(SINTHLAB_BRINGUP_PY_SCRIPTS
  scripts/apple_pull_controller.py
  scripts/admittance_control_mode.py
  scripts/admittance_gains_gui.py
  scripts/apple_pluck_impedance_control_displacement.py
  scripts/audio_cue_play.py
  scripts/force_torque_bias_calibrator.py
  scripts/move_to_start.py
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

# Install helpers package alongside the scripts so `from helpers...` works
install(
  DIRECTORY scripts/helpers
  DESTINATION lib/${PROJECT_NAME}
)

# Install actions package alongside the scripts so `from actions...` works
install(
  DIRECTORY scripts/actions
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
