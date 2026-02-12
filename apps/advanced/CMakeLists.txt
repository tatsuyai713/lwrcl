cmake_minimum_required(VERSION 3.18)

project(advanced_examples LANGUAGES CXX VERSION 1)

message(STATUS "Configuring advanced examples...")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wnon-virtual-dtor
                    -Wno-deprecated-declarations -Wno-unused-result)

find_package(yaml-cpp QUIET)

# ── Copy config files ─────────────────────────────────────────────────────
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/video_stream_pipeline.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/config/video_stream_pipeline.yaml
  COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/diagnostic_aggregator.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/config/diagnostic_aggregator.yaml
  COPYONLY)

# ── IDL generation (backend-specific) ─────────────────────────────────────
set(IDL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/msg)

if(DDS_BACKEND STREQUAL "fastdds")
  # ── Fast DDS: fastddsgen to compile custom IDL ──────────────────────────
  file(GLOB_RECURSE ADVANCED_IDL_FILES "${IDL_ROOT}/*.idl")

  foreach(idl_file ${ADVANCED_IDL_FILES})
    get_filename_component(idl ${idl_file} NAME_WE)
    get_filename_component(idl_dir "${idl_file}" PATH)
    set(workingdir ${CMAKE_CURRENT_BINARY_DIR}/advanced_msgs/msg)
    file(MAKE_DIRECTORY ${workingdir})
    configure_file(${idl_file} ${workingdir}/${idl}.idl COPYONLY)
    execute_process(
      COMMAND fastddsgen ${idl}.idl
              -I ${ROS_DATA_TYPES_INCLUDE_PATH}
              -I ${idl_dir}
              -typeros2 -replace -cs
      WORKING_DIRECTORY ${workingdir}
      INPUT_FILE ${idl}.idl
    )
  endforeach()

  file(GLOB_RECURSE ADVANCED_GEN_CXX "${CMAKE_CURRENT_BINARY_DIR}/*.cxx")
  file(GLOB_RECURSE ADVANCED_GEN_H   "${CMAKE_CURRENT_BINARY_DIR}/*.h")

  # Run update_headers.sh
  add_custom_command(
    OUTPUT advanced_update_headers_done
    COMMAND ${CMAKE_COMMAND} -E echo "Running update_headers.sh for advanced_msgs"
    COMMAND ${CMAKE_COMMAND} -E env bash ${ROS_DATA_TYPES_INCLUDE_PATH}/../bin/update_headers.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Updating header files for advanced_msgs"
    VERBATIM
  )
  add_custom_target(advanced_compile_idl ALL DEPENDS advanced_update_headers_done)

  include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${ROS_DATA_TYPES_INCLUDE_PATH}
  )

  set(ADVANCED_IDL_SOURCES ${ADVANCED_GEN_CXX})
  set(ADVANCED_IDL_HEADERS ${ADVANCED_GEN_H})
  set(ADVANCED_IDL_TARGET  advanced_compile_idl)

elseif(DDS_BACKEND STREQUAL "cyclonedds")
  # ── Cyclone DDS: idlc ───────────────────────────────────────────────────
  set(UPDATE_HEADERS_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/update_headers.sh")
  set(IDLC_EXECUTABLE /opt/cyclonedds/bin/idlc)

  include(../cmake/IdlcGenerate.cmake)

  idlc_generate(
    NAME  advanced_msgs
    ALIAS advanced_msgs
    IDL_ROOT "${IDL_ROOT}"
    IDLC     "${IDLC_EXECUTABLE}"
    UPDATE_HEADERS_SCRIPT "${UPDATE_HEADERS_SCRIPT}"
  )

  include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/generated
    ${advanced_msgs_GEN_PKG_DIR}
  )
  if(DEFINED ROS_DATA_TYPES_INCLUDE_PATH)
    include_directories(${ROS_DATA_TYPES_INCLUDE_PATH})
  endif()

  set(ADVANCED_IDL_SOURCES ${advanced_msgs_FINAL_SRCS})
  set(ADVANCED_IDL_TARGET_DEPS advanced_msgs_final_sources advanced_msgs_idl_gen)

endif()

# ── Executables ───────────────────────────────────────────────────────────

# 1) Video Stream Pipeline (custom IDL + params + executor + shared participant)
add_executable(video_stream_pipeline
  src/video_stream_pipeline.cpp
  ${ADVANCED_IDL_SOURCES}
  ${ADVANCED_IDL_HEADERS}
)

# 2) Diagnostic Aggregator (custom IDL + service + multi-threaded executor)
add_executable(diagnostic_aggregator
  src/diagnostic_aggregator.cpp
  ${ADVANCED_IDL_SOURCES}
  ${ADVANCED_IDL_HEADERS}
)

# 3) Adaptive Image Pipeline (WaitSet + QoS + adaptive rate)
add_executable(adaptive_image_pipeline
  src/adaptive_image_pipeline.cpp
)

# 4) State Machine Controller (timer control + service + state patterns)
add_executable(state_machine_controller
  src/state_machine_controller.cpp
)

# ── Dependencies ──────────────────────────────────────────────────────────
if(DDS_BACKEND STREQUAL "fastdds")
  add_dependencies(video_stream_pipeline ${ADVANCED_IDL_TARGET})
  add_dependencies(diagnostic_aggregator ${ADVANCED_IDL_TARGET})
elseif(DDS_BACKEND STREQUAL "cyclonedds")
  add_dependencies(video_stream_pipeline ${ADVANCED_IDL_TARGET_DEPS})
  add_dependencies(diagnostic_aggregator ${ADVANCED_IDL_TARGET_DEPS})
endif()

# ── Link libraries ────────────────────────────────────────────────────────
set(COMMON_LINK_LIBS
  ${DDS_LIBRARIES}
  lwrcl
  std_msgs
  sensor_msgs
  yaml-cpp
)

target_link_libraries(video_stream_pipeline   PRIVATE ${COMMON_LINK_LIBS})
target_link_libraries(diagnostic_aggregator   PRIVATE ${COMMON_LINK_LIBS})
target_link_libraries(adaptive_image_pipeline PRIVATE ${COMMON_LINK_LIBS})
target_link_libraries(state_machine_controller PRIVATE ${COMMON_LINK_LIBS})

# ── Install ───────────────────────────────────────────────────────────────
if(DDS_BACKEND STREQUAL "fastdds")
  set(APP_INSTALL_PREFIX "/opt/fast-dds-apps")
elseif(DDS_BACKEND STREQUAL "cyclonedds")
  set(APP_INSTALL_PREFIX "/opt/cyclonedds-apps")
endif()

install(TARGETS
  video_stream_pipeline
  diagnostic_aggregator
  adaptive_image_pipeline
  state_machine_controller
  RUNTIME DESTINATION "${APP_INSTALL_PREFIX}/bin/advanced"
)

install(DIRECTORY config/
  DESTINATION "${APP_INSTALL_PREFIX}/bin/advanced/config"
  FILES_MATCHING PATTERN "*.yaml"
)

message(STATUS "Advanced examples configured for ${DDS_BACKEND}")
