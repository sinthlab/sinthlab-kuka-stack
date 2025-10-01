cmake_minimum_required(VERSION 3.22) # Need >=3.22 for file(CHMOD)
project(kuka_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scenario script
set(KUKA_BRINGUP_PY_SCRIPTS
  scripts/apple_pluck_controller.py
)
foreach(_script ${KUKA_BRINGUP_PY_SCRIPTS})
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
    ${KUKA_BRINGUP_PY_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
