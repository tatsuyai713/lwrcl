cmake_minimum_required(VERSION 3.16.3)
project(lwrcl_example)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find required packages
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT foonathan_memory_FOUND)
    find_package(foonathan_memory REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

# Set include paths
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

add_executable(example_spin src/example_spin.cpp)
add_executable(example_timer src/example_timer.cpp)
add_executable(example_waitset src/example_waitset.cpp)
add_executable(example_class_pub src/example_class_pub.cpp)
add_executable(example_class_sub src/example_class_sub.cpp)
add_executable(example_class_pubsub_executor src/example_class_pubsub_executor.cpp)
add_executable(example_service_server src/example_service_server.cpp)
add_executable(example_service_client src/example_service_client.cpp)

# New examples for added features
add_executable(example_qos_presets src/example_qos_presets.cpp)
add_executable(example_namespace src/example_namespace.cpp)
add_executable(example_zero_copy_pub src/example_zero_copy_pub.cpp)
add_executable(example_zero_copy_sub src/example_zero_copy_sub.cpp)
add_executable(example_timer_control src/example_timer_control.cpp)

target_link_libraries(example_spin PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_timer PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_waitset PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_class_pub PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_class_sub PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_class_pubsub_executor PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_service_server PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_service_client PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)

# New examples
target_link_libraries(example_qos_presets PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_namespace PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_zero_copy_pub PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_zero_copy_sub PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_timer_control PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)

# Install targets
install(TARGETS 
        example_spin example_timer example_waitset 
        example_class_pub example_class_sub example_class_pubsub_executor 
        example_service_server example_service_client
        example_qos_presets example_namespace 
        example_zero_copy_pub example_zero_copy_sub example_timer_control
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
