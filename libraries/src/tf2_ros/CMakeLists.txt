cmake_minimum_required(VERSION 3.5)
project(tf2_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# tf2_ros library
add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/transform_listener.cpp
  src/transform_broadcaster.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${CMAKE_CURRENT_SOURCE_DIR}/../tf2)
target_link_libraries(${PROJECT_NAME} PUBLIC
  geometry_msgs
  rcl_like_wrapper
  tf2
  tf2_msgs)

target_compile_definitions(${PROJECT_NAME} PRIVATE "TF2_ROS_BUILDING_DLL")

add_executable(tf2_echo
  src/tf2_echo.cpp
)
target_link_libraries(tf2_echo
  ${PROJECT_NAME}
  rcl_like_wrapper
)

# add_executable(tf2_monitor
#   src/tf2_monitor.cpp
# )
# target_link_libraries(tf2_monitor
#   ${PROJECT_NAME}
#   rcl_like_wrapper
#   ${tf2_msgs_TARGETS}
# )

# Install rules
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
