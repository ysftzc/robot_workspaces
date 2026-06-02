cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)
if(NOT ROBOT_IP)
  set(ROBOT_IP "172.16.0.2")
endif()

project(integration_launch_testing)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
    find_package(ament_cmake_ros REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)

    ament_copyright(package.xml)
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()


    function(add_ros_isolated_launch_test path)
      set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
      add_launch_test("${path}" RUNNER "${RUNNER}" ARGS "robot_ip:=${ROBOT_IP}" TIMEOUT 500)
    endfunction()

    add_ros_isolated_launch_test(test/test_example_controllers.py)
    add_ros_isolated_launch_test(test/test_franka_gripper_position.py)
    add_ros_isolated_launch_test(test/generic_controller_test.py)

    ament_python_install_package(${PROJECT_NAME})

    install(
      DIRECTORY config
      DESTINATION share/${PROJECT_NAME}
    )

endif()

ament_export_libraries(
        ${PROJECT_NAME}
)
ament_package()