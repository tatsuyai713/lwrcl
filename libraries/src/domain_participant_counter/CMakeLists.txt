cmake_minimum_required(VERSION 3.16.3)
project(domain_participant_counter)

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


if(NOT LWRCL_FASTDDS_PREFIX)
    set(LWRCL_FASTDDS_PREFIX /opt/fast-dds-libs)
endif()

set(ROS_DATA_TYPES_INCLUDE_PATH ${LWRCL_FASTDDS_PREFIX}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lwrcl/fastdds/lwrcl/include
                    ${LWRCL_FASTDDS_PREFIX}/include
                    ${LWRCL_FASTDDS_PREFIX}/include/optionparser)
link_directories(${LWRCL_FASTDDS_PREFIX}/lib)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${ROS_DATA_TYPES_INCLUDE_PATH})

target_link_libraries(${PROJECT_NAME} fastrtps)

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
