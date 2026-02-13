cmake_minimum_required(VERSION 3.16.3)
project(lwrcl_tests)

set(DDS_BACKEND "fastdds" CACHE STRING "DDS backend to use")
set_property(CACHE DDS_BACKEND PROPERTY STRINGS "fastdds" "cyclonedds" "vsomeip")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# --- Google Test via FetchContent -------------------------------------------
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.13.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# --- DDS backend setup ------------------------------------------------------
if(DDS_BACKEND STREQUAL "fastdds")
    find_package(fastcdr REQUIRED)
    find_package(foonathan_memory REQUIRED)
    find_package(fastrtps REQUIRED)
    find_package(tinyxml2 REQUIRED)

    set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/fast-dds-libs)
    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")

    # Prefer source headers over stale installed copies so that fixes
    # take effect without requiring a reinstall of the library.
    include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/../lwrcl/fastdds/lwrcl/include)
    include_directories(${LWRCL_INSTALL_PREFIX}/include
                        ${LWRCL_INSTALL_PREFIX}/include/optionparser)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)

    set(DDS_LIBRARIES fastrtps)

elseif(DDS_BACKEND STREQUAL "cyclonedds")
    set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/cyclonedds-libs)
    set(DDS_PREFIX_DEFAULT /opt/cyclonedds)
    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")
    set(DDS_PREFIX "${DDS_PREFIX_DEFAULT}" CACHE PATH "DDS install prefix")

    # Prefer source headers over stale installed copies.
    include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/../lwrcl/cyclonedds/lwrcl/include)
    include_directories(${DDS_PREFIX}/include)
    include_directories(${DDS_PREFIX}/include/ddscxx)
    include_directories(${LWRCL_INSTALL_PREFIX}/include)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)
    link_directories(${DDS_PREFIX}/lib)

    set(DDS_LIBRARIES ddsc ddscxx)

elseif(DDS_BACKEND STREQUAL "vsomeip")
    set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/vsomeip-libs)
    set(VSOMEIP_PREFIX_DEFAULT /opt/vsomeip)
    set(DDS_PREFIX_DEFAULT /opt/cyclonedds)
    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")
    set(VSOMEIP_PREFIX "${VSOMEIP_PREFIX_DEFAULT}" CACHE PATH "vsomeip install prefix")
    set(DDS_PREFIX "${DDS_PREFIX_DEFAULT}" CACHE PATH "DDS install prefix (for CDR)")

    # Prefer source headers over stale installed copies.
    include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/../lwrcl/vsomeip/lwrcl/include)
    include_directories(${VSOMEIP_PREFIX}/include)
    include_directories(${DDS_PREFIX}/include)
    include_directories(${DDS_PREFIX}/include/ddscxx)
    include_directories(${LWRCL_INSTALL_PREFIX}/include)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)
    link_directories(${VSOMEIP_PREFIX}/lib)
    link_directories(${DDS_PREFIX}/lib)

    set(DDS_LIBRARIES vsomeip3 lwrcl_cdr)
else()
    message(FATAL_ERROR "Unknown DDS_BACKEND: ${DDS_BACKEND}. Use 'fastdds', 'cyclonedds', or 'vsomeip'.")
endif()

# --- Common link libraries ---------------------------------------------------
set(COMMON_LIBS ${DDS_LIBRARIES} std_msgs sensor_msgs lwrcl yaml-cpp gtest gtest_main)

# --- Helper macro to add a test executable -----------------------------------
macro(add_lwrcl_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PRIVATE ${COMMON_LIBS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    # Give each test a generous timeout (30 seconds)
    set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT 30)
endmacro()

# ============================================================================
# Common tests (both backends)
# ============================================================================

# 1. Publisher-Subscriber Pub/Sub with std_msgs::String
add_lwrcl_test(test_pubsub_string         src/test_pubsub_string.cpp)

# 2. Executor-based Pub+Sub
add_lwrcl_test(test_pubsub_executor       src/test_pubsub_executor.cpp)

# 3. Timer callback
add_lwrcl_test(test_timer                 src/test_timer.cpp)

# 4. WaitSet
add_lwrcl_test(test_waitset               src/test_waitset.cpp)

# 5. Service Server / Client
add_lwrcl_test(test_service               src/test_service.cpp)

# 6. Node creation and properties
add_lwrcl_test(test_node                  src/test_node.cpp)

# 7. Parameter declare/get
add_lwrcl_test(test_parameter             src/test_parameter.cpp)

# 8. QoS configuration
add_lwrcl_test(test_qos                   src/test_qos.cpp)

# 9. Clock / Time / Duration
add_lwrcl_test(test_clock_time_duration   src/test_clock_time_duration.cpp)

# 10. Rate / WallRate
add_lwrcl_test(test_rate                  src/test_rate.cpp)

# 11. Logger
add_lwrcl_test(test_logger                src/test_logger.cpp)

# 12. Serialization
add_lwrcl_test(test_serialization         src/test_serialization.cpp)

# 13. MultiThreadedExecutor
add_lwrcl_test(test_multi_executor        src/test_multi_executor.cpp)

# 14. Pub/Sub with sensor_msgs::Image
add_lwrcl_test(test_pubsub_image          src/test_pubsub_image.cpp)

# 15. spin_some
add_lwrcl_test(test_spin_some             src/test_spin_some.cpp)

# 16. Namespace support
add_lwrcl_test(test_namespace             src/test_namespace.cpp)

# 17. Timer control (cancel/reset)
add_lwrcl_test(test_timer_control         src/test_timer_control.cpp)

# 18. QoS presets
add_lwrcl_test(test_qos_presets           src/test_qos_presets.cpp)

# ============================================================================
# Fast DDS only tests
# ============================================================================
if(DDS_BACKEND STREQUAL "fastdds")
    # 19. Zero-copy (LoanedMessage)
    add_lwrcl_test(test_zero_copy         src/test_zero_copy.cpp)
endif()
