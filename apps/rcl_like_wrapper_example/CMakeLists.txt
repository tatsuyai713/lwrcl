cmake_minimum_required(VERSION 3.16.3)
project(rcl_like_wrapper_example)

set(FAST_DDS_PATH /opt/fast-dds/include)
set(ROS_DATA_TYPES_PATH /opt/fast-dds-libs/include)


link_directories(/opt/fast-dds-libs/lib /opt/fast-dds/lib)
include_directories(include ${FAST_DDS_PATH} ${ROS_DATA_TYPES_PATH})
add_executable(example_spin src/example_spin.cpp)
add_executable(example_timer src/example_timer.cpp)
target_link_libraries(example_spin PRIVATE pthread fastrtps std_msgs sensor_msgs rcl_like_wrapper)
target_link_libraries(example_timer PRIVATE pthread fastrtps std_msgs sensor_msgs rcl_like_wrapper)

# Install targets
install(TARGETS example_spin example_timer
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install headers
install(DIRECTORY include/ DESTINATION include/rcl_like_wrapper)