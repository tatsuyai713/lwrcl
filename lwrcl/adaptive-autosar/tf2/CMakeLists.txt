cmake_minimum_required(VERSION 3.16.3)
project(tf2 VERSION 1 LANGUAGES CXX)

#CPP Libraries
add_library(tf2 SHARED src/cache.cpp src/buffer_core.cpp src/static_cache.cpp src/time.cpp)
target_include_directories(tf2 PUBLIC
  include
)

target_link_libraries(tf2 
  builtin_interfaces
  std_msgs
  geometry_msgs
  tf2_msgs
  ddsc ddscxx)

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
# target_link_libraries(test_simple geometry_msgs ddsc ddscxx tf2 gtest gtest_main)
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
