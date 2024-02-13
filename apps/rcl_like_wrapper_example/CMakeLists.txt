cmake_minimum_required(VERSION 3.16.3)
project(rcl_like_wrapper_example)


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
target_link_libraries(example_spin PRIVATE fastrtps std_msgs sensor_msgs rcl_like_wrapper)
target_link_libraries(example_timer PRIVATE fastrtps std_msgs sensor_msgs rcl_like_wrapper)

# Install targets
install(TARGETS example_spin example_timer
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install headers
install(DIRECTORY include/ DESTINATION include/rcl_like_wrapper)