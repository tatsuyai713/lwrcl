cmake_minimum_required(VERSION 3.16.3)
project(tf2_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(DDS_PATH "/opt/fast-dds/")
set(LIB_PATH "/opt/fast-dds-libs/")
set(OPT_PATH "/opt/")
list(APPEND CMAKE_PREFIX_PATH ${DDS_PATH} ${LIB_PATH})
list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${DDS_PATH} ${LIB_PATH})

if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT foonathan_memory_FOUND)
    find_package(foonathan_memory REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

if(NOT tinyxml2_FOUND)
    find_package(tinyxml2 REQUIRED)
endif()


set(ROS_DATA_TYPES_INCLUDE_PATH /opt/fast-dds-libs/include)
include_directories(/opt/fast-dds-libs/include
                    /opt/fast-dds-libs/include/optionparser)
link_directories(/opt/fast-dds-libs/lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/dds/include)
set(fastcdr_DIR ${DDS_PATH}/lib/cmake/fastcdr/)
set(fastrtps_DIR ${DDS_PATH}/share/fastrtps/cmake/)
set(foonathan_memory_DIR ${DDS_PATH}/lib/foonathan_memory/cmake/)
set(tinyxml2_DIR ${DDS_PATH}/lib/cmake/tinyxml2/)


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
  lwrcl
  tf2
  tf2_msgs)

target_compile_definitions(${PROJECT_NAME} PRIVATE "TF2_ROS_BUILDING_DLL")

add_library(static_transform_broadcaster_node SHARED
  src/static_transform_broadcaster_node.cpp
  src/static_transform_broadcaster.cpp
)
target_compile_definitions(static_transform_broadcaster_node PRIVATE "STATIC_TRANSFORM_BROADCASTER_BUILDING_DLL")
target_include_directories(static_transform_broadcaster_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(static_transform_broadcaster_node PUBLIC
  ${PROJECT_NAME}
  geometry_msgs
  lwrcl
  tf2_msgs)
target_link_libraries(static_transform_broadcaster_node PRIVATE)

# static_transform_publisher
add_executable(static_transform_publisher
  src/static_transform_broadcaster_program.cpp
)
target_link_libraries(static_transform_publisher
  ${PROJECT_NAME}
  lwrcl
  static_transform_broadcaster_node
  tf2
)

add_executable(tf2_echo
  src/tf2_echo.cpp
)
target_link_libraries(tf2_echo
  ${PROJECT_NAME}
  lwrcl
)

add_executable(tf2_monitor
  src/tf2_monitor.cpp
)
target_link_libraries(tf2_monitor
  ${PROJECT_NAME}
  lwrcl
  tf2_msgs
)

# Install rules
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
