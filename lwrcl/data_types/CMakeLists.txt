cmake_minimum_required(VERSION 3.16.3)
project(data_types)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif()

if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT foonathan_memory_FOUND)
    find_package(foonathan_memory REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

if(NOT tinyxml2_FOUND)
    find_package(tinyxml2 REQUIRED)
endif()

set(ROS_DATA_TYPES_INCLUDE_PATH /opt/fast-dds-libs/include)
include_directories(/opt/fast-dds-libs/include
                    /opt/fast-dds-libs/include/optionparser)
link_directories(/opt/fast-dds-libs/lib)

add_subdirectory(src/ros-data-types-for-fastdds)
