cmake_minimum_required(VERSION 3.16.3)
project(rcl_like_wrapper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    set(ROS_DATA_TYPES_INCLUDE_PATH /opt/qnx/fast-dds-libs/include)
    include_directories(/opt/qnx/fast-dds-libs/include 
                        /opt/qnx/fast-dds-libs/include/optionparser)
    link_directories(/opt/qnx/fast-dds-libs/lib)
else()
    set(ROS_DATA_TYPES_INCLUDE_PATH /opt/fast-dds-libs/include)
    include_directories(/opt/fast-dds-libs/include
                        /opt/fast-dds-libs/include/optionparser)
    link_directories(/opt/fast-dds-libs/lib)
endif()

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${ROS_DATA_TYPES_INCLUDE_PATH})

target_link_libraries(${PROJECT_NAME} fastrtps)

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install headers
install(FILES include/rcl_like_wrapper.hpp include/eprosima_namespace.hpp DESTINATION include/)
