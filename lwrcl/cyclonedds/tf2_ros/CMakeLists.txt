cmake_minimum_required(VERSION 3.16.3)
project(tf2_ros)

# tf2_ros library
add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/transform_listener.cpp
  src/transform_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${CMAKE_CURRENT_SOURCE_DIR}/../tf2
)

# Link core libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
  ddscxx
  builtin_interfaces
  std_msgs
  geometry_msgs
  tf2_msgs
  lwrcl
  tf2
)

target_compile_definitions(${PROJECT_NAME} PRIVATE 
    "TF2_ROS_BUILDING_DLL"
)

# static_transform_broadcaster_node library
add_library(static_transform_broadcaster_node SHARED
  src/static_transform_broadcaster_node.cpp
  src/static_transform_broadcaster.cpp
)

target_compile_definitions(static_transform_broadcaster_node PRIVATE 
    "STATIC_TRANSFORM_BROADCASTER_BUILDING_DLL"
)

target_include_directories(static_transform_broadcaster_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(static_transform_broadcaster_node PUBLIC
  ddscxx
  builtin_interfaces
  std_msgs
  geometry_msgs
  tf2_msgs
  lwrcl
  tf2
  ${PROJECT_NAME}
)

# static_transform_publisher executable
add_executable(static_transform_publisher
  src/static_transform_broadcaster_program.cpp
)

target_link_libraries(static_transform_publisher
  ddscxx
  builtin_interfaces
  std_msgs
  geometry_msgs
  tf2_msgs
  lwrcl
  tf2
  static_transform_broadcaster_node
  ${PROJECT_NAME}
)

# tf2_echo executable
add_executable(tf2_echo
  src/tf2_echo.cpp
)

target_link_libraries(tf2_echo
  ddscxx
  builtin_interfaces
  std_msgs
  geometry_msgs
  tf2_msgs
  lwrcl
  tf2
  ${PROJECT_NAME}
)

# tf2_monitor executable
add_executable(tf2_monitor
  src/tf2_monitor.cpp
)

target_link_libraries(tf2_monitor
  ddscxx
  builtin_interfaces
  std_msgs
  geometry_msgs
  tf2_msgs
  lwrcl
  tf2
  ${PROJECT_NAME}
)

# Install rules
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(TARGETS static_transform_broadcaster_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(TARGETS static_transform_publisher tf2_echo tf2_monitor
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})


