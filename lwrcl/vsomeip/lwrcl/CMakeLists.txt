cmake_minimum_required(VERSION 3.16.3)
project(lwrcl)

# vsomeip backend requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include/link directories are inherited from the parent CMakeLists.txt

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include)

# Link against vsomeip3 for transport, standalone CDR for serialization, and yaml-cpp
target_link_libraries(${PROJECT_NAME} vsomeip3 lwrcl_cdr yaml-cpp)

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install headers
install(FILES
include/lwrcl.hpp
include/vsomeip_header.hpp
include/publisher.hpp
include/subscription.hpp
include/serialized_message.hpp
include/clock_time_duration.hpp
include/timer.hpp
include/channel.hpp
include/qos.hpp
DESTINATION include/)

install(FILES
include/rclcpp/rclcpp.hpp
include/rclcpp/qos.hpp
DESTINATION include/rclcpp)
