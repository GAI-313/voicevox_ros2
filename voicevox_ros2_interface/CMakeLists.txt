cmake_minimum_required(VERSION 3.8)
project(voicevox_ros2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "srv/Speaker.srv"
    "msg/Speaker.msg"
)

# execute setup
execute_process(
    COMMAND gnome-terminal -- bash -c "./setup.bash"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error_output
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)
message(STATUS "Output of setup.bash: ${output}")
if(result EQUAL 0)
    message(STATUS "Command executed successfully. Output: ${output}")
else()
    message(FATAL_ERROR "Command failed. Error output: ${error_output}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
