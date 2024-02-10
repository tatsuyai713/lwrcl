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

set(FAST_DDS_PATH /opt/fast-dds/include)
set(ROS_DATA_TYPES_PATH /opt/fast-dds-libs/include)


add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${FAST_DDS_PATH} ${ROS_DATA_TYPES_PATH})

link_directories(/opt/fast-dds-libs/lib)

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install headers
install(FILES include/rcl_like_wrapper.hpp include/eprosima_namespace.hpp DESTINATION include/)