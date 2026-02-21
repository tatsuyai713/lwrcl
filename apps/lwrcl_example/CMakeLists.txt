cmake_minimum_required(VERSION 3.16.3)
project(lwrcl_example)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Common examples (both backends)
add_executable(example_spin src/example_spin.cpp)
add_executable(example_timer src/example_timer.cpp)
add_executable(example_waitset src/example_waitset.cpp)
add_executable(example_class_pub src/example_class_pub.cpp)
add_executable(example_class_sub src/example_class_sub.cpp)
add_executable(example_class_pubsub_executor src/example_class_pubsub_executor.cpp)
add_executable(example_service_server src/example_service_server.cpp)
add_executable(example_service_client src/example_service_client.cpp)

set(LWRCL_EXAMPLE_MESSAGE_LIBS std_msgs sensor_msgs geometry_msgs builtin_interfaces)

target_link_libraries(example_spin PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
target_link_libraries(example_timer PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
target_link_libraries(example_waitset PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
target_link_libraries(example_class_pub PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
target_link_libraries(example_class_sub PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
target_link_libraries(example_class_pubsub_executor PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
target_link_libraries(example_service_server PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
target_link_libraries(example_service_client PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)

# Fast DDS / CycloneDDS zero-copy examples
if(DDS_BACKEND STREQUAL "fastdds" OR DDS_BACKEND STREQUAL "cyclonedds")
    add_executable(example_zero_copy_pub src/example_zero_copy_pub.cpp)
    add_executable(example_zero_copy_sub src/example_zero_copy_sub.cpp)

    target_link_libraries(example_zero_copy_pub PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
    target_link_libraries(example_zero_copy_sub PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
endif()

# Fast DDS only examples
if(DDS_BACKEND STREQUAL "fastdds")
    add_executable(example_qos_presets src/example_qos_presets.cpp)
    add_executable(example_namespace src/example_namespace.cpp)
    add_executable(example_timer_control src/example_timer_control.cpp)

    target_link_libraries(example_qos_presets PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
    target_link_libraries(example_namespace PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
    target_link_libraries(example_timer_control PRIVATE ${DDS_LIBRARIES} ${LWRCL_EXAMPLE_MESSAGE_LIBS} lwrcl yaml-cpp)
endif()

# Install targets
set(COMMON_TARGETS
    example_spin example_timer example_waitset
    example_class_pub example_class_sub example_class_pubsub_executor
    example_service_server example_service_client)

if(DDS_BACKEND STREQUAL "fastdds")
    install(TARGETS ${COMMON_TARGETS}
            example_qos_presets example_namespace
            example_zero_copy_pub example_zero_copy_sub example_timer_control
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin)
elseif(DDS_BACKEND STREQUAL "cyclonedds")
    install(TARGETS ${COMMON_TARGETS}
            example_zero_copy_pub example_zero_copy_sub
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin)
else()
    install(TARGETS ${COMMON_TARGETS}
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin)
endif()
