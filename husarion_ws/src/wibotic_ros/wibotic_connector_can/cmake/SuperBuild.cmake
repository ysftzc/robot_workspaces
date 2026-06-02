# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

include(ExternalProject)

ExternalProject_Add(
  uavcan
  # Probable newer repo also will work: https://github.com/dronecan/libuavcan
  GIT_REPOSITORY https://github.com/OpenCyphal-Garage/libcyphal/
  GIT_TAG dcc3a4de237b7482e04543d2393c3a9385685312
  PREFIX uavcan
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/uavcan)

ExternalProject_Add(
  linux_uavcan
  DEPENDS uavcan
  GIT_REPOSITORY
    https://github.com/OpenCyphal-Garage/platform_specific_components/
  GIT_TAG 4745ef59f57b7e1c34705b127ea8c7a35e3874c1
  PREFIX linux_uavcan
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND
    ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_BINARY_DIR}/linux_uavcan/src/linux_uavcan/linux/libuavcan/include
    ${CMAKE_BINARY_DIR}/linux_uavcan/include)

install(DIRECTORY ${CMAKE_BINARY_DIR}/uavcan ${CMAKE_BINARY_DIR}/linux_uavcan
        DESTINATION include)
