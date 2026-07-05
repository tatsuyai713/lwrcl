cmake_minimum_required(VERSION 3.18)

project(h265_image_transport LANGUAGES CXX VERSION 1)

message(STATUS "Configuring h265_image_transport...")

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
  NAME  h265_image_transport
  ALIAS h265_image_transport
  IDL_ROOT "${IDL_ROOT}"
  IDLC     "${IDLC_EXECUTABLE}"
  UPDATE_HEADERS_SCRIPT "${UPDATE_HEADERS_SCRIPT}"
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_BINARY_DIR}/generated
  ${h265_image_transport_GEN_PKG_DIR}
)
if(DEFINED ROS_DATA_TYPES_INCLUDE_PATH)
  include_directories(${ROS_DATA_TYPES_INCLUDE_PATH})
endif()

file(GLOB FFMPEG_IMAGE_TRANSPORT_MSGS_SOURCES_CXX "src/*.cxx")
file(GLOB FFMPEG_IMAGE_TRANSPORT_MSGS_SOURCES_CPP "src/*.cpp")

add_library(h265_image_transport SHARED
  ${FFMPEG_IMAGE_TRANSPORT_MSGS_SOURCES_CXX}
  ${FFMPEG_IMAGE_TRANSPORT_MSGS_SOURCES_CPP}
  ${h265_image_transport_FINAL_SRCS}
)

add_dependencies(h265_image_transport
  h265_image_transport_final_sources
  h265_image_transport_idl_gen
)

target_include_directories(h265_image_transport
  SYSTEM PRIVATE
    /opt/qnx/cyclonedds/include
    /opt/qnx/cyclonedds-libs/include
)

target_link_libraries(h265_image_transport
  PRIVATE
    ddsc
    ddscxx
    std_msgs
)

set_target_properties(h265_image_transport PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME h265_image_transport
)

# Per-target RPATH for the installed artifact
set_target_properties(h265_image_transport PROPERTIES
  INSTALL_RPATH "${LWRCL_INSTALL_PREFIX}/lib"
)

install(TARGETS h265_image_transport
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
foreach(F IN LISTS h265_image_transport_IDL_FILES)
  get_filename_component(N "${F}" NAME)
  message(STATUS "  - ${N}")
endforeach()
message(STATUS "Generated tree: ${h265_image_transport_GEN_MSG_DIR}")
