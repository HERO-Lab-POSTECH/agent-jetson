cmake_minimum_required(VERSION 2.8.3)
project(characterization_tests)

add_compile_options(-std=c++11 -Wall)

# hero_agent + albc_control export INCLUDE_DIRS include -> their headers arrive
# via catkin_INCLUDE_DIRS, so package-style includes (hero_agent/..., albc_control/...)
# resolve under catkin exactly as run.sh resolves them via -I flags.
find_package(catkin REQUIRED COMPONENTS hero_agent albc_control)

catkin_package()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}   # local "processkey_oracle.h" + Arduino.h host shim
  ${catkin_INCLUDE_DIRS}
)

# test_yaw_deadband_ff.cpp reads agent.ino as TEXT (it cannot be linked on a host)
# to pin the boot setpoint and the two I_yaw resets. run.sh chdir's to this
# directory so the test's built-in relative default works there; ctest runs from
# the build tree, so it needs the absolute path.
add_definitions(-DAGENT_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/agent")

# Standalone golden tests: each test_*.cpp is a self-contained int main() that
# returns non-zero on divergence (CTest pass/fail contract). NOT gtest -- wrapped
# as-is with add_executable + add_test so the verification logic stays byte-identical.
# run.sh remains the ROS-free local/board path; this block is the catkin path.
if(CATKIN_ENABLE_TESTING)
  file(GLOB _char_tests "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
  foreach(_src ${_char_tests})
    get_filename_component(_name ${_src} NAME_WE)
    add_executable(${_name} ${_src})
    set_target_properties(${_name} PROPERTIES EXCLUDE_FROM_ALL TRUE)
    # 'tests' is catkin's target that run_tests builds before invoking CTest;
    # attaching here is what makes `catkin_make run_tests` actually run them
    # (a bare add_test is seen by ctest/make test but not by catkin run_tests).
    add_dependencies(tests ${_name})
    add_test(NAME ${_name} COMMAND ${_name})
  endforeach()
endif()
