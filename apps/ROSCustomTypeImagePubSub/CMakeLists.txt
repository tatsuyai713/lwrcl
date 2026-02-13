cmake_minimum_required(VERSION 3.18)

project(ROSCustomTypeImagePubSub LANGUAGES CXX VERSION 1)

message(STATUS "Configuring ROSCustomTypeImagePubSub...")

# --- C++ / warnings ---
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wnon-virtual-dtor
                    -Wno-deprecated-declarations -Wno-unused-result)

# --- Dependencies ---
find_package(yaml-cpp QUIET)

# --- Paths ---
set(UPDATE_HEADERS_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/update_headers.sh")
set(IDLC_EXECUTABLE /opt/cyclonedds/bin/idlc)


include(../cmake/IdlcGenerate.cmake)

# *.idl は msg/ 直下
set(IDL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/msg)

# idlc_generate() 呼び出し
idlc_generate(
  NAME  ROSCustomTypeImagePubSub
  ALIAS ROSCustomTypeImagePubSub
  IDL_ROOT "${IDL_ROOT}"
  IDLC     "${IDLC_EXECUTABLE}"
  UPDATE_HEADERS_SCRIPT "${UPDATE_HEADERS_SCRIPT}"
)

# --- Config file ---
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/config/config.yaml
  COPYONLY
)

# --- Include directories ---
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_BINARY_DIR}/generated
  ${ROSCustomTypeImagePubSub_GEN_PKG_DIR}
)
if(DEFINED ROS_DATA_TYPES_INCLUDE_PATH)
  include_directories(${ROS_DATA_TYPES_INCLUDE_PATH})
endif()

# --- Sources ---
file(GLOB ROSTYPEIMAGEPUBSUB_SOURCES_CXX "src/*.cxx")
file(GLOB ROSTYPEIMAGEPUBSUB_SOURCES_CPP "src/*.cpp")

# --- Executable ---
add_executable(ROSCustomTypeImagePubSub
  ${ROSTYPEIMAGEPUBSUB_SOURCES_CXX}
  ${ROSTYPEIMAGEPUBSUB_SOURCES_CPP}
  ${ROSCustomTypeImagePubSub_FINAL_SRCS}
)

# 生成依存
add_dependencies(ROSCustomTypeImagePubSub
  ROSCustomTypeImagePubSub_final_sources
  ROSCustomTypeImagePubSub_idl_gen
)

# --- Link ---
target_include_directories(ROSCustomTypeImagePubSub
  SYSTEM PRIVATE
    /opt/qnx/cyclonedds/include
    /opt/qnx/cyclonedds-libs/include
)
target_link_libraries(ROSCustomTypeImagePubSub
  PRIVATE
    ${DDS_LIBRARIES}
    lwrcl geometry_msgs sensor_msgs std_msgs builtin_interfaces yaml-cpp
)
if(TARGET yaml-cpp)
  target_link_libraries(ROSCustomTypeImagePubSub PRIVATE yaml-cpp)
endif()
if(OpenCV_FOUND)
  target_link_libraries(ROSCustomTypeImagePubSub PRIVATE ${OpenCV_LIBRARIES})
endif()

# --- Install ---
# Choose sysroot per-OS without touching CMAKE_INSTALL_PREFIX
if(CMAKE_SYSTEM_NAME STREQUAL "QNX")
  set(CYCLONEDDS_SYSROOT "/opt/qnx/cyclonedds-apps")
else()
  set(CYCLONEDDS_SYSROOT "/opt/cyclonedds-apps")
endif()

install(TARGETS ROSCustomTypeImagePubSub
    RUNTIME DESTINATION "${CYCLONEDDS_SYSROOT}/bin/ROSCustomTypeImagePubSub/${BIN_INSTALL_DIR}"
)

install(FILES config/config.yaml
        DESTINATION "${CYCLONEDDS_SYSROOT}/bin/ROSCustomTypeImagePubSub/${BIN_INSTALL_DIR}/config"
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
)

# --- Info ---
message(STATUS "IDL files (msg/):")
foreach(F IN LISTS ROSCustomTypeImagePubSub_IDL_FILES)
  get_filename_component(N "${F}" NAME)
  message(STATUS "  - ${N}")
endforeach()
message(STATUS "Generated tree: ${ROSCustomTypeImagePubSub_GEN_MSG_DIR}")
