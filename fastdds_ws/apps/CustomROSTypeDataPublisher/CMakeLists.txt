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

project(CustomROSTypeDataPublisher VERSION 1 LANGUAGES CXX)

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
    set(ROS_DATA_TYPES_PATH /opt/qnx/fast-dds-libs/include)
else()
    set(ROS_DATA_TYPES_PATH /opt/fast-dds-libs/include)
endif()

file(GLOB_RECURSE Original_IDL_Files
  "${CMAKE_CURRENT_SOURCE_DIR}/*/*.idl"
)

if (NOT Original_I${Original_IDL_Files}DL_Files)
    #create an initial version of these files
    foreach(idl_file ${Original_IDL_Files})
        get_filename_component(idl ${idl_file} NAME_WE)
        get_filename_component(idl_dir "${idl_file}" PATH)
        set(workingdir ${CMAKE_CURRENT_BINARY_DIR})
        configure_file(${idl_file} ${workingdir}/${idl}.idl COPYONLY)
        execute_process(COMMAND fastddsgen ${idl}.idl -I ${ROS_DATA_TYPES_PATH} -I ${idl_dir} -typeros2 -replace -cs
        WORKING_DIRECTORY ${workingdir}
        INPUT_FILE ${idl}.idl)
    endforeach(idl_file)
endif()

file(GLOB_RECURSE in1_files
  "${CMAKE_CURRENT_BINARY_DIR}/*.cxx"
  "${CMAKE_CURRENT_BINARY_DIR}/*.h"
)

#Check C++11
# include(CheckCXXCompilerFlag)
# if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
#     check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
#     if(NOT SUPPORTS_CXX11)
#         message(FATAL_ERROR "Compiler doesn't support C++11")
#     endif()
# endif()

message(STATUS "Configuring CustomROSTypeDataPublisher...")
file(GLOB ROS_TYPE_DATA_PUBLISHER_SOURCES_CXX "src/*.cxx")
file(GLOB ROS_TYPE_DATA_PUBLISHER_SOURCES_CPP "src/*.cpp")

if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    include_directories(/opt/qnx/fast-dds-libs/include 
                        /opt/qnx/fast-dds-libs/include/optionparser)
    link_directories(/opt/qnx/fast-dds-libs/lib)
else()
    include_directories(/opt/fast-dds-libs/include
                        /opt/fast-dds-libs/include/optionparser)
    link_directories(/opt/fast-dds-libs/lib)
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config/config.yaml  COPYONLY)


include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR})
add_executable(CustomROSTypeDataPublisher ${ROS_TYPE_DATA_PUBLISHER_SOURCES_CXX} ${ROS_TYPE_DATA_PUBLISHER_SOURCES_CPP} ${in1_files})
target_compile_definitions(CustomROSTypeDataPublisher PRIVATE
    $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
    $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
    )
# target_include_directories(CustomROSTypeDataSubscriber PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(CustomROSTypeDataPublisher fastrtps fastcdr foonathan_memory geometry_msgs yaml-cpp)
install(TARGETS CustomROSTypeDataPublisher
    RUNTIME DESTINATION bin/CustomROSTypeDataPublisher/${BIN_INSTALL_DIR})
install(FILES config/config.yaml DESTINATION bin/CustomROSTypeDataPublisher/${BIN_INSTALL_DIR}/config PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
