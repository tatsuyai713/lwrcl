cmake_minimum_required(VERSION 3.16.3)
project(data_types)

set(DDS_BACKEND "fastdds" CACHE STRING "DDS backend to use")
set_property(CACHE DDS_BACKEND PROPERTY STRINGS "fastdds" "cyclonedds" "vsomeip")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif()

if(DDS_BACKEND STREQUAL "fastdds")
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

    if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/qnx/fast-dds-libs)
    else()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/fast-dds-libs)
    endif()

    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")

    set(ROS_DATA_TYPES_INCLUDE_PATH ${LWRCL_INSTALL_PREFIX}/include)
    include_directories(${LWRCL_INSTALL_PREFIX}/include
                        ${LWRCL_INSTALL_PREFIX}/include/optionparser)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)

    add_subdirectory(src/ros-data-types-for-fastdds)

elseif(DDS_BACKEND STREQUAL "cyclonedds")
    add_subdirectory(src/ros-data-types-cyclonedds/src)

elseif(DDS_BACKEND STREQUAL "vsomeip")
    # vsomeip uses CycloneDDS-generated types but links against standalone CDR
    # library (liblwrcl_cdr.a) instead of the full CycloneDDS runtime.
    # The stub CMake configs at ${VSOMEIP_PREFIX}/lib/cmake/ provide
    # CycloneDDS::ddsc and CycloneDDS-CXX::ddscxx targets that redirect
    # to the CDR library.
    add_subdirectory(src/ros-data-types-cyclonedds/src)

else()
    message(FATAL_ERROR "Unknown DDS_BACKEND: ${DDS_BACKEND}. Use 'fastdds', 'cyclonedds', or 'vsomeip'.")
endif()
