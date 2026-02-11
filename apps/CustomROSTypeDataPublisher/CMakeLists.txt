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

file(GLOB_RECURSE Original_IDL_Files
  "${CMAKE_CURRENT_SOURCE_DIR}/*/*.idl"
)

if (NOT Original_I${Original_IDL_Files}DL_Files)
    #create an initial version of these files
    foreach(idl_file ${Original_IDL_Files})
        get_filename_component(idl ${idl_file} NAME_WE)
        get_filename_component(idl_dir "${idl_file}" PATH)
        set(workingdir ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/msg)
        configure_file(${idl_file} ${workingdir}/${idl}.idl COPYONLY)
        execute_process(COMMAND fastddsgen ${idl}.idl -I ${ROS_DATA_TYPES_INCLUDE_PATH} -I ${idl_dir} -typeros2 -replace -cs
        WORKING_DIRECTORY ${workingdir}
        INPUT_FILE ${idl}.idl)
    endforeach(idl_file)
endif()

file(GLOB_RECURSE in1_files
  "${CMAKE_CURRENT_BINARY_DIR}/*.cxx"
  "${CMAKE_CURRENT_BINARY_DIR}/*.h"
)

message(STATUS "Configuring CustomROSTypeDataPublisher...")
file(GLOB ROS_TYPE_DATA_PUBLISHER_SOURCES_CXX "src/*.cxx")
file(GLOB ROS_TYPE_DATA_PUBLISHER_SOURCES_CPP "src/*.cpp")


add_custom_command(
    OUTPUT update_headers_done
    COMMAND ${CMAKE_COMMAND} -E echo "Running update_headers.sh"
    COMMAND ${CMAKE_COMMAND} -E env bash ${ROS_DATA_TYPES_INCLUDE_PATH}/../bin/update_headers.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Updating header files"
    VERBATIM
)
add_custom_target(Custom_Message_CompileIDLFiles ALL
    DEPENDS update_headers_done)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config/config.yaml  COPYONLY)

include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}    
    ${ROS_DATA_TYPES_INCLUDE_PATH})
add_executable(CustomROSTypeDataPublisher ${ROS_TYPE_DATA_PUBLISHER_SOURCES_CXX} ${ROS_TYPE_DATA_PUBLISHER_SOURCES_CPP} ${in1_files})

add_dependencies(CustomROSTypeDataPublisher Custom_Message_CompileIDLFiles)

target_link_libraries(CustomROSTypeDataPublisher fastrtps fastcdr lwrcl geometry_msgs sensor_msgs yaml-cpp)

set_target_properties(CustomROSTypeDataPublisher PROPERTIES OUTPUT_NAME "CustomROSTypeDataPublisherExecutable")
install(TARGETS CustomROSTypeDataPublisher
    RUNTIME DESTINATION bin/CustomROSTypeDataPublisher/${BIN_INSTALL_DIR})
install(FILES config/config.yaml DESTINATION bin/CustomROSTypeDataPublisher/${BIN_INSTALL_DIR}/config PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
