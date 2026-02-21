cmake_minimum_required(VERSION 3.16.3)
project(lwrcl)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include)

target_link_libraries(${PROJECT_NAME} fastrtps yaml-cpp)

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install headers
install(FILES 
include/lwrcl.hpp 
include/fast_dds_header.hpp
include/publisher.hpp 
include/subscription.hpp 
include/clock_time_duration.hpp
include/timer.hpp 
include/channel.hpp
include/qos.hpp
DESTINATION include/)

install(DIRECTORY ../tf2/include/rcutils DESTINATION include/)

install(FILES
include/rclcpp/rclcpp.hpp
include/rclcpp/qos.hpp
DESTINATION include/rclcpp)
