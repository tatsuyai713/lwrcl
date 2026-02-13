cmake_minimum_required(VERSION 3.16.3)
project(tf2_ros)

# vsomeip backend requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Reference source/include files from the cyclonedds backend to avoid code duplication
set(TF2_ROS_CYCLONEDDS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cyclonedds/tf2_ros)
set(TF2_CYCLONEDDS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cyclonedds/tf2)

# tf2_ros library
add_library(${PROJECT_NAME} SHARED
  ${TF2_ROS_CYCLONEDDS_DIR}/src/buffer.cpp
  ${TF2_ROS_CYCLONEDDS_DIR}/src/transform_listener.cpp
  ${TF2_ROS_CYCLONEDDS_DIR}/src/transform_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${TF2_ROS_CYCLONEDDS_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${TF2_CYCLONEDDS_DIR}
)

# Link core libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
  lwrcl_cdr
  vsomeip3
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
  ${TF2_ROS_CYCLONEDDS_DIR}/src/static_transform_broadcaster_node.cpp
  ${TF2_ROS_CYCLONEDDS_DIR}/src/static_transform_broadcaster.cpp
)

target_compile_definitions(static_transform_broadcaster_node PRIVATE
    "STATIC_TRANSFORM_BROADCASTER_BUILDING_DLL"
)

target_include_directories(static_transform_broadcaster_node PUBLIC
  "$<BUILD_INTERFACE:${TF2_ROS_CYCLONEDDS_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(static_transform_broadcaster_node PUBLIC
  lwrcl_cdr
  vsomeip3
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
  ${TF2_ROS_CYCLONEDDS_DIR}/src/static_transform_broadcaster_program.cpp
)

target_link_libraries(static_transform_publisher
  lwrcl_cdr
  vsomeip3
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
  ${TF2_ROS_CYCLONEDDS_DIR}/src/tf2_echo.cpp
)

target_link_libraries(tf2_echo
  lwrcl_cdr
  vsomeip3
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
  ${TF2_ROS_CYCLONEDDS_DIR}/src/tf2_monitor.cpp
)

target_link_libraries(tf2_monitor
  lwrcl_cdr
  vsomeip3
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

install(DIRECTORY ${TF2_ROS_CYCLONEDDS_DIR}/include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
