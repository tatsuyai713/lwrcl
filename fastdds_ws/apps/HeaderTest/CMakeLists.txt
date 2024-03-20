# Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

project(DDSHeaderTest VERSION 1 LANGUAGES CXX)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
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

# Set C++11
# include(CheckCXXCompilerFlag)
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
#         CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
#     if(SUPPORTS_CXX11)
#         add_compile_options(-std=c++11)
#     else()
#         message(FATAL_ERROR "Compiler doesn't support C++11")
#     endif()
# endif()

# #Check C++11
# include(CheckCXXCompilerFlag)
# if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
#     check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
#     if(NOT SUPPORTS_CXX11)
#         message(FATAL_ERROR "Compiler doesn't support C++11")
#     endif()
# endif()

message(STATUS "Configuring Header example...")
file(GLOB DDS_HELLOWORLD_TEST_SOURCES_CXX "*.cxx")
file(GLOB DDS_HELLOWORLD_TEST_SOURCES_CPP "*.cpp")

add_executable(DDSHeaderTest ${DDS_HELLOWORLD_TEST_SOURCES_CXX} ${DDS_HELLOWORLD_TEST_SOURCES_CPP})
target_compile_definitions(DDSHeaderTest PRIVATE
    $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
    $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
)
target_link_libraries(DDSHeaderTest std_msgs fastrtps fastcdr fastdds::optionparser)
install(TARGETS DDSHeaderTest
    RUNTIME DESTINATION examples/cpp/dds/HeaderTest/${BIN_INSTALL_DIR})
