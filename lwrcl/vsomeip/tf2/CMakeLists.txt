cmake_minimum_required(VERSION 3.16.3)
project(tf2 VERSION 1 LANGUAGES CXX)

# vsomeip backend requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Reference source files from the cyclonedds backend to avoid code duplication
set(TF2_CYCLONEDDS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cyclonedds/tf2)

# CPP Libraries
add_library(tf2 SHARED
  ${TF2_CYCLONEDDS_DIR}/src/cache.cpp
  ${TF2_CYCLONEDDS_DIR}/src/buffer_core.cpp
  ${TF2_CYCLONEDDS_DIR}/src/static_cache.cpp
  ${TF2_CYCLONEDDS_DIR}/src/time.cpp
)

target_include_directories(tf2 PUBLIC
  ${TF2_CYCLONEDDS_DIR}/include
)

target_link_libraries(tf2
  builtin_interfaces
  std_msgs
  geometry_msgs
  tf2_msgs
  vsomeip3
  lwrcl_cdr
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tf2 PRIVATE "TF2_BUILDING_DLL")

# Install rules
install(TARGETS tf2 EXPORT export_tf2
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(DIRECTORY ${TF2_CYCLONEDDS_DIR}/include/tf2 DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
