cmake_minimum_required(VERSION 3.16)
project(combined_robot_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(combined_robot_contact_attach_system SHARED
  src/contact_attach_system.cpp
)

target_link_libraries(combined_robot_contact_attach_system
  gz-msgs10
  gz-plugin2
  gz-sim8
  gz-transport13
)

install(
  TARGETS combined_robot_contact_attach_system
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  gz-msgs10
  gz-plugin2
  gz-sim8
  gz-transport13
)

ament_package()
