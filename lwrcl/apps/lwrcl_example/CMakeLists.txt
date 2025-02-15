cmake_minimum_required(VERSION 3.16.3)
project(lwrcl_example)

add_executable(example_spin src/example_spin.cpp)
add_executable(example_timer src/example_timer.cpp)
add_executable(example_waitset src/example_waitset.cpp)
add_executable(example_class_pub src/example_class_pub.cpp)
add_executable(example_class_sub src/example_class_sub.cpp)
add_executable(example_class_pubsub_executor src/example_class_pubsub_executor.cpp)
add_executable(example_service_server src/example_service_server.cpp)
add_executable(example_service_client src/example_service_client.cpp)

target_link_libraries(example_spin PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_timer PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_waitset PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_class_pub PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_class_sub PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_class_pubsub_executor PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_service_server PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)
target_link_libraries(example_service_client PRIVATE fastrtps std_msgs sensor_msgs lwrcl yaml-cpp)

# Install targets
install(TARGETS example_spin example_timer example_waitset example_class_pub example_class_sub example_class_pubsub_executor example_service_server
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
