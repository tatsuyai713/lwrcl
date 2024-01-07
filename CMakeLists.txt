cmake_minimum_required(VERSION 3.16.3)
project(rcl_like_wrapper)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

set(FAST_DDS_PATH /opt/fast-dds/include)
set(ROS_DATA_TYPES_PATH /opt/fast-dds-libs/include)


add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp src/message_type.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${FAST_DDS_PATH} ${ROS_DATA_TYPES_PATH})

link_directories(/opt/fast-dds-libs/lib)
add_executable(example src/example.cpp)
target_link_libraries(example PRIVATE pthread ${PROJECT_NAME} fastrtps std_msgs sensor_msgs)
