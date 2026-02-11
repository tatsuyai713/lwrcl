cmake_minimum_required(VERSION 3.16.3)
project(lwrcl)

set(DDS_BACKEND "fastdds" CACHE STRING "DDS backend to use")
set_property(CACHE DDS_BACKEND PROPERTY STRINGS "fastdds" "cyclonedds")

option(BUILD_FFI "Build FFI bindings for Dart/Flutter" OFF)

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
        set(DDS_PREFIX_DEFAULT /opt/qnx/fast-dds)
        set(LWRCL_FASTDDS_INCLUDE_EXTRA_DEFAULT /opt/qnx/fast-dds/usr/include)
    else()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/fast-dds-libs)
        set(DDS_PREFIX_DEFAULT /opt/fast-dds)
        set(LWRCL_FASTDDS_INCLUDE_EXTRA_DEFAULT "")
    endif()

    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix (include/lib)")
    set(DDS_PREFIX "${DDS_PREFIX_DEFAULT}" CACHE PATH "DDS install prefix")
    set(LWRCL_FASTDDS_INCLUDE_EXTRA "${LWRCL_FASTDDS_INCLUDE_EXTRA_DEFAULT}" CACHE PATH "Optional extra include dir for Fast DDS headers")

    set(ROS_DATA_TYPES_INCLUDE_PATH ${LWRCL_INSTALL_PREFIX}/include)
    if(LWRCL_FASTDDS_INCLUDE_EXTRA)
        include_directories(${LWRCL_FASTDDS_INCLUDE_EXTRA})
    endif()
    include_directories(${LWRCL_INSTALL_PREFIX}/include
                        ${LWRCL_INSTALL_PREFIX}/include/optionparser)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)

    set(DDS_LIBRARIES fastrtps CACHE INTERNAL "DDS link libraries")

    add_subdirectory(fastdds/tf2)
    add_subdirectory(fastdds/tf2_ros)
    add_subdirectory(fastdds/lwrcl)
    
    if(BUILD_FFI)
        add_subdirectory(fastdds/lwrcl_ffi)
    endif()

elseif(DDS_BACKEND STREQUAL "cyclonedds")
    if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/qnx/cyclonedds-libs)
        set(DDS_PREFIX_DEFAULT /opt/qnx/cyclonedds)
    else()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/cyclonedds-libs)
        set(DDS_PREFIX_DEFAULT /opt/cyclonedds)
    endif()

    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix (include/lib)")
    set(DDS_PREFIX "${DDS_PREFIX_DEFAULT}" CACHE PATH "DDS install prefix")

    include_directories(${DDS_PREFIX}/include)
    include_directories(${DDS_PREFIX}/include/ddscxx)
    include_directories(${LWRCL_INSTALL_PREFIX}/include)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)
    link_directories(${DDS_PREFIX}/lib)

    set(DDS_LIBRARIES ddsc ddscxx CACHE INTERNAL "DDS link libraries")

    add_subdirectory(cyclonedds/tf2)
    add_subdirectory(cyclonedds/tf2_ros)
    add_subdirectory(cyclonedds/lwrcl)

else()
    message(FATAL_ERROR "Unknown DDS_BACKEND: ${DDS_BACKEND}. Use 'fastdds' or 'cyclonedds'.")
endif()
