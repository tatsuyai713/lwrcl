cmake_minimum_required(VERSION 3.16.3)
project(libraries)

set(DDS_BACKEND "fastdds" CACHE STRING "DDS backend to use")
set_property(CACHE DDS_BACKEND PROPERTY STRINGS "fastdds" "cyclonedds")

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
        set(LWRCL_FASTDDS_INCLUDE_EXTRA_DEFAULT /opt/qnx/fast-dds/usr/include)
    else()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/fast-dds-libs)
        set(LWRCL_FASTDDS_INCLUDE_EXTRA_DEFAULT "")
    endif()

    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")
    set(LWRCL_FASTDDS_INCLUDE_EXTRA "${LWRCL_FASTDDS_INCLUDE_EXTRA_DEFAULT}" CACHE PATH "Extra include dir")

    set(ROS_DATA_TYPES_INCLUDE_PATH ${LWRCL_INSTALL_PREFIX}/include)
    if(LWRCL_FASTDDS_INCLUDE_EXTRA)
        include_directories(${LWRCL_FASTDDS_INCLUDE_EXTRA})
    endif()
    include_directories(${LWRCL_INSTALL_PREFIX}/include
                        ${LWRCL_INSTALL_PREFIX}/include/optionparser)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)

    add_subdirectory(src/domain_participant_counter)

elseif(DDS_BACKEND STREQUAL "cyclonedds")
    if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/qnx/cyclonedds-libs)
        set(DDS_PREFIX_DEFAULT /opt/qnx/cyclonedds)
    else()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/cyclonedds-libs)
        set(DDS_PREFIX_DEFAULT /opt/cyclonedds)
    endif()

    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")
    set(DDS_PREFIX "${DDS_PREFIX_DEFAULT}" CACHE PATH "DDS install prefix")

    include_directories(${DDS_PREFIX}/include)
    include_directories(${DDS_PREFIX}/include/ddscxx)
    include_directories(${LWRCL_INSTALL_PREFIX}/include)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)
    link_directories(${DDS_PREFIX}/lib)

else()
    message(FATAL_ERROR "Unknown DDS_BACKEND: ${DDS_BACKEND}. Use 'fastdds' or 'cyclonedds'.")
endif()

add_subdirectory(src/yaml-cpp)
