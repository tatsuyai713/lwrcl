cmake_minimum_required(VERSION 3.16.3)
project(tf2 VERSION 1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(QNX_HOST $ENV{QNX_HOST})
set(QNX_TARGET $ENV{QNX_TARGET})

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    set(DDS_PATH "/opt/qnx/fast-dds/aarch64le/usr")
    set(LIB_PATH "/opt/qnx/fast-dds-libs/")
    set(OPT_PATH "/opt/qnx/")
    list(APPEND CMAKE_PREFIX_PATH ${DDS_PATH} ${LIB_PATH} ${QNX_TARGET}/aarch64le/usr)
    list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${DDS_PATH} ${LIB_PATH} ${QNX_TARGET}/aarch64le/usr)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
    set(DDS_PATH "/opt/fast-dds/")
    set(LIB_PATH "/opt/fast-dds-libs/")
    set(OPT_PATH "/opt/")
    list(APPEND CMAKE_PREFIX_PATH ${DDS_PATH} ${LIB_PATH})
    list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${DDS_PATH} ${LIB_PATH})
endif()

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

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
    set(ROS_DATA_TYPES_INCLUDE_PATH /opt/qnx/fast-dds-libs/include)
    include_directories(/opt/qnx/fast-dds/usr/include/
                        /opt/qnx/fast-dds-libs/include/
                        /opt/qnx/fast-dds-libs/include/optionparser)
    link_directories(/opt/qnx/fast-dds/aarch64le/usr/lib/
                     /opt/qnx/fast-dds-libs/lib/)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/dds/include)

    set(OPENSSL_ROOT_DIR ${QNX_TARGET}/aarch64le/usr)
    set(OPENSSL_INCLUDE_DIR ${QNX_TARGET}/aarch64le/usr/include)
    set(OPENSSL_LIBRARIE ${QNX_TARGET}/aarch64le/usr/lib)
    
    set(fastcdr_DIR ${DDS_PATH}/aarch64le/usr/lib/cmake/fastcdr)
    set(fastrtps_DIR ${DDS_PATH}/aarch64le/usr/share/fastrtps/cmake/)
    set(foonathan_memory_DIR ${DDS_PATH}/aarch64le/usr/lib/foonathan_memory/cmake/)
    set(tinyxml2_DIR ${DDS_PATH}/aarch64le/usr/lib/cmake/tinyxml2/)
    set(OpenCV_DIR ${OPT_PATH}/qnx/local/share/OpenCV/)
else()
    set(ROS_DATA_TYPES_INCLUDE_PATH /opt/fast-dds-libs/include)
    include_directories(/opt/fast-dds-libs/include
                        /opt/fast-dds-libs/include/optionparser)
    link_directories(/opt/fast-dds-libs/lib)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/dds/include)
    set(fastcdr_DIR ${DDS_PATH}/lib/cmake/fastcdr/)
    set(fastrtps_DIR ${DDS_PATH}/share/fastrtps/cmake/)
    set(foonathan_memory_DIR ${DDS_PATH}/lib/foonathan_memory/cmake/)
    set(tinyxml2_DIR ${DDS_PATH}/lib/cmake/tinyxml2/)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# export user definitions

#CPP Libraries
add_library(tf2 SHARED src/cache.cpp src/buffer_core.cpp src/static_cache.cpp src/time.cpp)
target_include_directories(tf2 PUBLIC
  include
)

target_link_libraries(tf2 
  std_msgs
  geometry_msgs
  fastrtps fastcdr)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tf2 PRIVATE "TF2_BUILDING_DLL")


# Tests
# if(BUILD_TESTING)

# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
# )
# # For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(googletest)

# include(GoogleTest)

# add_executable(test_cache_unittest test/cache_unittest.cpp)
# target_link_libraries(test_cache_unittest tf2 gtest_main)
# # add_dependencies(test_cache_unittest ${catkin_EXPORTED_TARGETS})

# add_executable(test_static_cache_unittest test/static_cache_test.cpp)
# target_link_libraries(test_static_cache_unittest tf2 gtest gtest_main)
# # add_dependencies(test_static_cache_unittest ${catkin_EXPORTED_TARGETS})

# add_executable(test_simple test/simple_tf2_core.cpp)
# target_link_libraries(test_simple geometry_msgs fastrtps fastcdr tf2 gtest gtest_main)
# # add_dependencies(test_simple ${catkin_EXPORTED_TARGETS})

# add_executable(test_time test/test_time.cpp)
# target_link_libraries(test_time tf2 gtest gtest_main)
# # add_dependencies(test_transform_datatypes ${catkin_EXPORTED_TARGETS})
# enable_testing()

# # endif()


install(TARGETS tf2 EXPORT export_tf2
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(DIRECTORY include/tf2 DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
