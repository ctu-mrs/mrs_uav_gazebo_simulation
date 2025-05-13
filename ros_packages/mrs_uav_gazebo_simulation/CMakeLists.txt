cmake_minimum_required(VERSION 3.5)
project(mrs_uav_gazebo_simulation)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_px4_api]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(MRS_ENABLE_TESTING)
  message(WARNING "Setting optimizaton levels for tests")
  add_definitions("-O0")
  add_definitions("-g")
endif()

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

set(DEPENDENCIES
  rclcpp
  rclpy
  ros_gz
  ros_gz_sim
  px4
  mavros
  mrs_gazebo_common_resources
  std_msgs
  mrs_msgs
  mrs_uav_px4_api
  
  ## TODO: stibipet@fel.cvut.cz
  # mrs_uav_testing
  # mrs_uav_gazebo_testing
  ## TODO
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(Python3 COMPONENTS Interpreter REQUIRED)
message(STATUS "Checking for python3-jinja2...")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "-c" "import jinja2"
  RESULT_VARIABLE _jinja2_check_result
  OUTPUT_QUIET
  ERROR_QUIET
)

if(NOT _jinja2_check_result EQUAL 0)
  # The command failed, meaning jinja2 is likely not installed or not in PYTHONPATH
  message(FATAL_ERROR "Python module 'jinja2' (from python3-jinja2) could not be imported. "
                      "You might need to run:\n"
                      "  sudo apt update && sudo apt install python3-jinja2\n")
else()
  message(STATUS "Python module 'jinja2' found and importable.")
endif()

include_directories(
  ${rclcpp_INCLUDE_DIRECTORIES}
  ${ros_gz_INCLUDE_DIRECTORIES}
  ${ros_gz_sim_INCLUDE_DIRECTORIES}
  ${mavros_msgs_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------


set(INSTALLED_PROGRAMS
  scripts/mrs_drone_spawner/mrs_drone_spawner.py
  scripts/dangerous_pixhawk_only_takeoff
  scripts/kill_previous_session.sh
)

foreach(PROG IN LISTS INSTALLED_PROGRAMS)
  install(
    PROGRAMS ${PROG}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY tmux/
#   DESTINATION share/${PROJECT_NAME}
# )

install(DIRECTORY ROMFS
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY scripts
#   DESTINATION lib/${PROJECT_NAME}
#   USE_SOURCE_PERMISSIONS
# )

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
