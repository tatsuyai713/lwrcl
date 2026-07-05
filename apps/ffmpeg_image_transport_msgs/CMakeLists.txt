cmake_minimum_required(VERSION 3.18)

project(ffmpeg_image_transport_msgs LANGUAGES CXX VERSION 1)

message(STATUS "Configuring ffmpeg_image_transport_msgs...")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wnon-virtual-dtor -Wno-deprecated-declarations -Wno-unused-result)

set(UPDATE_HEADERS_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/update_headers.sh")
if(DEFINED DDS_PREFIX)
  set(IDLC_EXECUTABLE "${DDS_PREFIX}/bin/idlc")
else()
  set(IDLC_EXECUTABLE /opt/cyclonedds/bin/idlc)
endif()

include(../cmake/IdlcGenerate.cmake)
include(../cmake/RosMsgToIdl.cmake)

set(IDL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/msg)

# Auto-convert any ROS 2 .msg files in msg/ into matching .idl files.
ros_msg_to_idl_convert(
  PACKAGE ${PROJECT_NAME}
  DIRS ${IDL_ROOT}
)

idlc_generate(
  NAME  ffmpeg_image_transport_msgs
  ALIAS ffmpeg_image_transport_msgs
  IDL_ROOT "${IDL_ROOT}"
  IDLC     "${IDLC_EXECUTABLE}"
  UPDATE_HEADERS_SCRIPT "${UPDATE_HEADERS_SCRIPT}"
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_BINARY_DIR}/generated
  ${ffmpeg_image_transport_msgs_GEN_PKG_DIR}
)
if(DEFINED ROS_DATA_TYPES_INCLUDE_PATH)
  include_directories(${ROS_DATA_TYPES_INCLUDE_PATH})
endif()

file(GLOB FFMPEG_IMAGE_TRANSPORT_MSGS_SOURCES_CXX "src/*.cxx")
file(GLOB FFMPEG_IMAGE_TRANSPORT_MSGS_SOURCES_CPP "src/*.cpp")

add_library(ffmpeg_image_transport_msgs SHARED
  ${FFMPEG_IMAGE_TRANSPORT_MSGS_SOURCES_CXX}
  ${FFMPEG_IMAGE_TRANSPORT_MSGS_SOURCES_CPP}
  ${ffmpeg_image_transport_msgs_FINAL_SRCS}
)

add_dependencies(ffmpeg_image_transport_msgs
  ffmpeg_image_transport_msgs_final_sources
  ffmpeg_image_transport_msgs_idl_gen
)

target_include_directories(ffmpeg_image_transport_msgs
  SYSTEM PRIVATE
    /opt/qnx/cyclonedds/include
    /opt/qnx/cyclonedds-libs/include
)

target_link_libraries(ffmpeg_image_transport_msgs
  PRIVATE
    ddsc
    ddscxx
    std_msgs
)

set_target_properties(ffmpeg_image_transport_msgs PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME ffmpeg_image_transport_msgs
)

# Per-target RPATH for the installed artifact
set_target_properties(ffmpeg_image_transport_msgs PROPERTIES
  INSTALL_RPATH "${LWRCL_INSTALL_PREFIX}/lib"
)

install(TARGETS ffmpeg_image_transport_msgs
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "bin"
)

install(DIRECTORY "${CMAKE_BINARY_DIR}/generated/"
  DESTINATION "include"
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
  DESTINATION "include/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.idl"
)

message(STATUS "IDL files (msg/):")
foreach(F IN LISTS ffmpeg_image_transport_msgs_IDL_FILES)
  get_filename_component(N "${F}" NAME)
  message(STATUS "  - ${N}")
endforeach()
message(STATUS "Generated tree: ${ffmpeg_image_transport_msgs_GEN_MSG_DIR}")
