cmake_minimum_required(VERSION 3.18)

project(ffmpeg_image_transport_msgs LANGUAGES CXX VERSION 1)

message(STATUS "Configuring ffmpeg_image_transport_msgs...")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wnon-virtual-dtor -Wno-deprecated-declarations -Wno-unused-result)

set(UPDATE_HEADERS_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/update_headers.sh")
set(IDLC_EXECUTABLE /opt/cyclonedds/bin/idlc)

include(../cmake/IdlcGenerate.cmake)

set(IDL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/msg)

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

# Choose sysroot per-OS without touching CMAKE_INSTALL_PREFIX
if(CMAKE_SYSTEM_NAME STREQUAL "QNX")
  set(CYCLONEDDS_SYSROOT "/opt/qnx/cyclonedds-libs")
else()
  set(CYCLONEDDS_SYSROOT "/opt/cyclonedds-libs")
endif()

# Per-target RPATH for the installed artifact
set_target_properties(ffmpeg_image_transport_msgs PROPERTIES
  INSTALL_RPATH "${CYCLONEDDS_SYSROOT}/lib"
)

# Absolute DESTINATION paths (ignore CMAKE_INSTALL_PREFIX)
install(TARGETS ffmpeg_image_transport_msgs
  LIBRARY DESTINATION "${CYCLONEDDS_SYSROOT}/lib"
  ARCHIVE DESTINATION "${CYCLONEDDS_SYSROOT}/lib"
  RUNTIME DESTINATION "${CYCLONEDDS_SYSROOT}/bin"
)

install(DIRECTORY "${CMAKE_BINARY_DIR}/generated/"
  DESTINATION "${CYCLONEDDS_SYSROOT}/include"
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
  DESTINATION "${CYCLONEDDS_SYSROOT}/include/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.idl"
)

message(STATUS "IDL files (msg/):")
foreach(F IN LISTS ffmpeg_image_transport_msgs_IDL_FILES)
  get_filename_component(N "${F}" NAME)
  message(STATUS "  - ${N}")
endforeach()
message(STATUS "Generated tree: ${ffmpeg_image_transport_msgs_GEN_MSG_DIR}")
