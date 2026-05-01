cmake_minimum_required(VERSION 3.18)

project(ros_msg_mixed_example LANGUAGES CXX VERSION 1)

message(STATUS
  "Configuring ros_msg_mixed_example (mixed .msg + external IDL demo)...")

if(NOT CMAKE_CXX_STANDARD OR CMAKE_CXX_STANDARD LESS 14)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
add_compile_options(-Wall -Wextra -Wnon-virtual-dtor
                    -Wno-deprecated-declarations -Wno-unused-result)

# ── Auto-convert *.msg -> *.idl ───────────────────────────────────────────
# Multiple .msg files in this directory cross-reference each other and
# external IDL types from ros-data-types. msg_to_idl.py emits the right
# IDL #include lines for both kinds of references.
set(IDL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/msg)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/RosMsgToIdl.cmake)
ros_msg_to_idl_convert(
  PACKAGE ${PROJECT_NAME}
  DIRS ${IDL_ROOT}
)

# ── Resolve idlc path (Cyclone-family backends) ───────────────────────────
if(DEFINED DDS_PREFIX)
  set(_RMME_IDLC "${DDS_PREFIX}/bin/idlc")
else()
  set(_RMME_IDLC "/opt/qnx/cyclonedds/bin/idlc")
endif()
if(NOT EXISTS "${_RMME_IDLC}")
  set(_RMME_IDLC "/opt/cyclonedds/bin/idlc")
endif()

# ── IDL generation (backend-specific) ─────────────────────────────────────
if(DDS_BACKEND STREQUAL "fastdds")
  file(GLOB_RECURSE _RMME_IDL_FILES "${IDL_ROOT}/*.idl")
  foreach(idl_file ${_RMME_IDL_FILES})
    get_filename_component(idl ${idl_file} NAME_WE)
    get_filename_component(idl_dir "${idl_file}" PATH)
    set(workingdir ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/msg)
    file(MAKE_DIRECTORY ${workingdir})
    configure_file(${idl_file} ${workingdir}/${idl}.idl COPYONLY)
    execute_process(
      COMMAND fastddsgen ${idl}.idl
              -I ${ROS_DATA_TYPES_INCLUDE_PATH}
              -I ${idl_dir}
              -I ${CMAKE_CURRENT_BINARY_DIR}
              -typeros2 -replace -cs
      WORKING_DIRECTORY ${workingdir}
      INPUT_FILE ${idl}.idl
    )
  endforeach()

  file(GLOB_RECURSE _RMME_GEN_CXX "${CMAKE_CURRENT_BINARY_DIR}/*.cxx")
  file(GLOB_RECURSE _RMME_GEN_H   "${CMAKE_CURRENT_BINARY_DIR}/*.h")

  add_custom_command(
    OUTPUT ros_msg_mixed_example_update_headers_done
    COMMAND ${CMAKE_COMMAND} -E echo "Running update_headers.sh for ${PROJECT_NAME}"
    COMMAND ${CMAKE_COMMAND} -E env bash
            ${ROS_DATA_TYPES_INCLUDE_PATH}/../bin/update_headers.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Patching generated headers for ${PROJECT_NAME}"
    VERBATIM
  )
  add_custom_target(ros_msg_mixed_example_compile_idl ALL
    DEPENDS ros_msg_mixed_example_update_headers_done)

  include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${ROS_DATA_TYPES_INCLUDE_PATH}
  )

  set(_RMME_IDL_SOURCES ${_RMME_GEN_CXX})
  set(_RMME_IDL_TARGET  ros_msg_mixed_example_compile_idl)

elseif(DDS_BACKEND STREQUAL "cyclonedds"
       OR DDS_BACKEND STREQUAL "vsomeip"
       OR DDS_BACKEND STREQUAL "adaptive-autosar")
  set(UPDATE_HEADERS_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/update_headers.sh")
  set(IDLC_EXECUTABLE "${_RMME_IDLC}")

  include(../cmake/IdlcGenerate.cmake)

  idlc_generate(
    NAME  ros_msg_mixed_example_msgs
    ALIAS ${PROJECT_NAME}
    IDL_ROOT "${IDL_ROOT}"
    IDLC     "${IDLC_EXECUTABLE}"
    UPDATE_HEADERS_SCRIPT "${UPDATE_HEADERS_SCRIPT}"
    IDL_INCLUDE_PREFIX "${LWRCL_INSTALL_PREFIX}/include"
  )

  include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/generated
    ${ros_msg_mixed_example_msgs_GEN_PKG_DIR}
  )
  if(DEFINED ROS_DATA_TYPES_INCLUDE_PATH)
    include_directories(${ROS_DATA_TYPES_INCLUDE_PATH})
  endif()

  set(_RMME_IDL_SOURCES ${ros_msg_mixed_example_msgs_FINAL_SRCS})
  set(_RMME_IDL_TARGET_DEPS
    ros_msg_mixed_example_msgs_final_sources
    ros_msg_mixed_example_msgs_idl_gen)

else()
  message(FATAL_ERROR
    "ros_msg_mixed_example: unsupported DDS_BACKEND='${DDS_BACKEND}'")
endif()

# ── Executables ───────────────────────────────────────────────────────────
add_executable(sensor_bundle_publisher
  src/sensor_bundle_publisher.cpp
  ${_RMME_IDL_SOURCES}
)
add_executable(sensor_bundle_subscriber
  src/sensor_bundle_subscriber.cpp
  ${_RMME_IDL_SOURCES}
)

if(DDS_BACKEND STREQUAL "fastdds")
  add_dependencies(sensor_bundle_publisher  ${_RMME_IDL_TARGET})
  add_dependencies(sensor_bundle_subscriber ${_RMME_IDL_TARGET})
else()
  add_dependencies(sensor_bundle_publisher  ${_RMME_IDL_TARGET_DEPS})
  add_dependencies(sensor_bundle_subscriber ${_RMME_IDL_TARGET_DEPS})
endif()

set(_RMME_LINK_LIBS
  ${DDS_LIBRARIES}
  lwrcl
  std_msgs
  geometry_msgs
  builtin_interfaces
)
target_link_libraries(sensor_bundle_publisher  PRIVATE ${_RMME_LINK_LIBS})
target_link_libraries(sensor_bundle_subscriber PRIVATE ${_RMME_LINK_LIBS})

# ── Install ───────────────────────────────────────────────────────────────
if(DDS_BACKEND STREQUAL "fastdds")
  set(_RMME_INSTALL_PREFIX "/opt/fast-dds-apps")
elseif(DDS_BACKEND STREQUAL "cyclonedds")
  set(_RMME_INSTALL_PREFIX "/opt/cyclonedds-apps")
elseif(DDS_BACKEND STREQUAL "vsomeip")
  set(_RMME_INSTALL_PREFIX "/opt/vsomeip-apps")
elseif(DDS_BACKEND STREQUAL "adaptive-autosar")
  set(_RMME_INSTALL_PREFIX "/opt/autosar-ap-apps")
endif()

install(TARGETS sensor_bundle_publisher sensor_bundle_subscriber
        RUNTIME DESTINATION "${_RMME_INSTALL_PREFIX}/bin/${PROJECT_NAME}")
