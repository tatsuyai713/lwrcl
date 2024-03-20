# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16.3)

project(CyclicROSDataPublisher VERSION 1 LANGUAGES CXX)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT foonathan_memory_FOUND)
    find_package(foonathan_memory REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)

if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    include_directories(/opt/qnx/fast-dds-libs/include 
                        /opt/qnx/fast-dds-libs/include/optionparser)
    link_directories(/opt/qnx/fast-dds-libs/lib)
else()
    include_directories(/opt/fast-dds-libs/include
                        /opt/fast-dds-libs/include/optionparser)
    link_directories(/opt/fast-dds-libs/lib)
endif()

#Check C++11
# include(CheckCXXCompilerFlag)
# if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
#     check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
#     if(NOT SUPPORTS_CXX11)
#         message(FATAL_ERROR "Compiler doesn't support C++11")
#     endif()
# endif()

message(STATUS "Configuring CyclicROSDataPublisher...")
file(GLOB DDS_SHM_ROS_TYPE_DATA_SOURCES_CXX "src/*.cxx")
file(GLOB DDS_SHM_ROS_TYPE_DATA_SOURCES_CPP "src/*.cpp")


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config/config.yaml  COPYONLY)


include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR})
add_executable(CyclicROSDataPublisher ${DDS_SHM_ROS_TYPE_DATA_SOURCES_CXX} ${DDS_SHM_ROS_TYPE_DATA_SOURCES_CPP})
target_compile_definitions(CyclicROSDataPublisher PRIVATE
    $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
    $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
    )
target_link_libraries(CyclicROSDataPublisher fastrtps fastcdr foonathan_memory geometry_msgs yaml-cpp)
install(TARGETS CyclicROSDataPublisher
    RUNTIME DESTINATION bin/CyclicROSDataPublisher/${BIN_INSTALL_DIR})
install(FILES config/config.yaml DESTINATION bin/CyclicROSDataPublisher/${BIN_INSTALL_DIR}/config PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
