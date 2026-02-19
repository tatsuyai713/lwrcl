cmake_minimum_required(VERSION 3.16.3)
project(lwrcl)

# Include/link directories are inherited from the parent CMakeLists.txt

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include)

# Adaptive AUTOSAR DDS headers require this compile definition
target_compile_definitions(${PROJECT_NAME} PUBLIC ARA_COM_USE_CYCLONEDDS=1)

# Link against Adaptive AUTOSAR libraries + yaml-cpp
# CycloneDDS (ddsc/ddscxx) is a transitive dependency via ara_com — not linked directly.
target_link_libraries(${PROJECT_NAME}
  AdaptiveAutosarAP::ara_core
  AdaptiveAutosarAP::ara_com
  yaml-cpp
)

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install headers
install(FILES
include/lwrcl.hpp
include/adaptive_autosar_header.hpp
include/publisher.hpp
include/subscription.hpp
include/clock_time_duration.hpp
include/timer.hpp
include/channel.hpp
include/qos.hpp
DESTINATION include/)

install(FILES
include/rclcpp/rclcpp.hpp
include/rclcpp/qos.hpp
DESTINATION include/rclcpp)
