cmake_minimum_required(VERSION 3.16.3)
project("apps")

set(DDS_BACKEND "fastdds" CACHE STRING "DDS backend to use")
set_property(CACHE DDS_BACKEND PROPERTY STRINGS "fastdds" "cyclonedds" "vsomeip" "adaptive-autosar")

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

    set(DDS_LIBRARIES fastrtps CACHE INTERNAL "DDS link libraries")

    # Common apps
    add_subdirectory(lwrcl_example)

    if(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        add_subdirectory(ROSTypeImagePubSubEdge)
        add_subdirectory(ROSTypeImagePubSubMono)
        add_subdirectory(ROSTypeImagePubSubExecutor)
    endif()

    # Fast DDS only apps
    add_subdirectory(ROSTypeImagePubSub)
    add_subdirectory(CustomROSTypeDataPublisher)
    add_subdirectory(HelloWorldExample)

    # Advanced examples
    add_subdirectory(advanced)

elseif(DDS_BACKEND STREQUAL "cyclonedds")
    if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/qnx/cyclonedds-libs)
        set(DDS_PREFIX_DEFAULT /opt/qnx/cyclonedds)
        set(ICEORYX_PREFIX_DEFAULT /opt/qnx/iceoryx)
    else()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/cyclonedds-libs)
        set(DDS_PREFIX_DEFAULT /opt/cyclonedds)
        set(ICEORYX_PREFIX_DEFAULT /opt/iceoryx)
    endif()

    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")
    set(DDS_PREFIX "${DDS_PREFIX_DEFAULT}" CACHE PATH "DDS install prefix")

    include_directories(${DDS_PREFIX}/include)
    include_directories(${DDS_PREFIX}/include/ddscxx)
    # iceoryx headers (required when CycloneDDS is built with ENABLE_SHM=ON)
    set(ICEORYX_PREFIX "${ICEORYX_PREFIX}" CACHE PATH "iceoryx install prefix")
    if(NOT ICEORYX_PREFIX)
        set(ICEORYX_PREFIX "${ICEORYX_PREFIX_DEFAULT}")
    endif()
    if(EXISTS "${ICEORYX_PREFIX}/lib/cmake")
        list(APPEND CMAKE_PREFIX_PATH "${ICEORYX_PREFIX}/lib/cmake")
        find_package(iceoryx_hoofs QUIET)
        find_package(iceoryx_posh QUIET)
        find_package(iceoryx_binding_c QUIET)
    endif()
    if(EXISTS "${ICEORYX_PREFIX}/include")
        include_directories(${ICEORYX_PREFIX}/include)
    endif()
    if(EXISTS "${ICEORYX_PREFIX}/lib")
        link_directories(${ICEORYX_PREFIX}/lib)
    endif()
    file(GLOB ICEORYX_INCLUDE_DIRS "${ICEORYX_PREFIX}/include/iceoryx/v*")
    foreach(dir ${ICEORYX_INCLUDE_DIRS})
        include_directories(${dir})
    endforeach()
    include_directories(${LWRCL_INSTALL_PREFIX}/include)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)
    link_directories(${DDS_PREFIX}/lib)

    # CycloneDDS built with SHM=ON may require explicit iceoryx linkage.
    set(CYCLONEDDS_SHM_LIBRARIES "")
    if(TARGET iceoryx_binding_c::iceoryx_binding_c)
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_binding_c::iceoryx_binding_c)
    elseif(EXISTS "${ICEORYX_PREFIX}/lib/libiceoryx_binding_c.so")
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_binding_c)
    endif()
    if(TARGET iceoryx_posh::iceoryx_posh)
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_posh::iceoryx_posh)
    elseif(EXISTS "${ICEORYX_PREFIX}/lib/libiceoryx_posh.so")
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_posh)
    endif()
    if(TARGET iceoryx_hoofs::iceoryx_hoofs)
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_hoofs::iceoryx_hoofs)
    elseif(EXISTS "${ICEORYX_PREFIX}/lib/libiceoryx_hoofs.so")
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_hoofs)
    endif()
    if(TARGET iceoryx_hoofs::iceoryx_platform)
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_hoofs::iceoryx_platform)
    elseif(EXISTS "${ICEORYX_PREFIX}/lib/libiceoryx_platform.so")
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_platform)
    endif()

    set(DDS_LIBRARIES ddsc ddscxx ${CYCLONEDDS_SHM_LIBRARIES} CACHE INTERNAL "DDS link libraries")

    # Cyclone DDS only apps
    add_subdirectory(ffmpeg_image_transport_msgs)
    add_subdirectory(h265_image_transport)

    # Common apps
    add_subdirectory(lwrcl_example)

    if(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        add_subdirectory(ROSTypeImagePubSubEdge)
        add_subdirectory(ROSTypeImagePubSubMono)
        add_subdirectory(ROSTypeImagePubSubExecutor)
    endif()

    add_subdirectory(ROSCustomTypeImagePubSub)

    # Advanced examples
    add_subdirectory(advanced)

elseif(DDS_BACKEND STREQUAL "vsomeip")
    # vsomeip backend: uses vsomeip for transport + CycloneDDS for CDR serialization
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/qnx/vsomeip-libs)
        set(DDS_PREFIX_DEFAULT /opt/qnx/cyclonedds)
        set(VSOMEIP_PREFIX_DEFAULT /opt/qnx/vsomeip)
        set(ICEORYX_PREFIX_DEFAULT /opt/qnx/iceoryx)
    else()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/vsomeip-libs)
        set(DDS_PREFIX_DEFAULT /opt/cyclonedds)
        set(VSOMEIP_PREFIX_DEFAULT /opt/vsomeip)
        set(ICEORYX_PREFIX_DEFAULT /opt/iceoryx)
    endif()

    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")
    set(DDS_PREFIX "${DDS_PREFIX_DEFAULT}" CACHE PATH "CycloneDDS install prefix (for CDR)")
    set(VSOMEIP_PREFIX "${VSOMEIP_PREFIX_DEFAULT}" CACHE PATH "vsomeip install prefix")

    include_directories(${DDS_PREFIX}/include)
    include_directories(${DDS_PREFIX}/include/ddscxx)
    # iceoryx headers (required when CycloneDDS is built with ENABLE_SHM=ON)
    set(ICEORYX_PREFIX "${ICEORYX_PREFIX}" CACHE PATH "iceoryx install prefix")
    if(NOT ICEORYX_PREFIX)
        set(ICEORYX_PREFIX "${ICEORYX_PREFIX_DEFAULT}")
    endif()
    if(EXISTS "${ICEORYX_PREFIX}/lib/cmake")
        list(APPEND CMAKE_PREFIX_PATH "${ICEORYX_PREFIX}/lib/cmake")
        find_package(iceoryx_hoofs QUIET)
        find_package(iceoryx_posh QUIET)
        find_package(iceoryx_binding_c QUIET)
    endif()
    if(EXISTS "${ICEORYX_PREFIX}/include")
        include_directories(${ICEORYX_PREFIX}/include)
    endif()
    if(EXISTS "${ICEORYX_PREFIX}/lib")
        link_directories(${ICEORYX_PREFIX}/lib)
    endif()
    file(GLOB ICEORYX_INCLUDE_DIRS "${ICEORYX_PREFIX}/include/iceoryx/v*")
    foreach(dir ${ICEORYX_INCLUDE_DIRS})
        include_directories(${dir})
    endforeach()
    include_directories(${LWRCL_INSTALL_PREFIX}/include)
    include_directories(${VSOMEIP_PREFIX}/include)
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)
    link_directories(${DDS_PREFIX}/lib)
    link_directories(${VSOMEIP_PREFIX}/lib)

    # CycloneDDS built with SHM=ON may require explicit iceoryx linkage.
    set(CYCLONEDDS_SHM_LIBRARIES "")
    if(TARGET iceoryx_binding_c::iceoryx_binding_c)
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_binding_c::iceoryx_binding_c)
    elseif(EXISTS "${ICEORYX_PREFIX}/lib/libiceoryx_binding_c.so")
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_binding_c)
    endif()
    if(TARGET iceoryx_posh::iceoryx_posh)
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_posh::iceoryx_posh)
    elseif(EXISTS "${ICEORYX_PREFIX}/lib/libiceoryx_posh.so")
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_posh)
    endif()
    if(TARGET iceoryx_hoofs::iceoryx_hoofs)
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_hoofs::iceoryx_hoofs)
    elseif(EXISTS "${ICEORYX_PREFIX}/lib/libiceoryx_hoofs.so")
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_hoofs)
    endif()
    if(TARGET iceoryx_hoofs::iceoryx_platform)
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_hoofs::iceoryx_platform)
    elseif(EXISTS "${ICEORYX_PREFIX}/lib/libiceoryx_platform.so")
        list(APPEND CYCLONEDDS_SHM_LIBRARIES iceoryx_platform)
    endif()

    set(DDS_LIBRARIES ddsc ddscxx vsomeip3 ${CYCLONEDDS_SHM_LIBRARIES} CACHE INTERNAL "DDS link libraries")

    # Common apps
    add_subdirectory(lwrcl_example)

    if(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        add_subdirectory(ROSTypeImagePubSubEdge)
        add_subdirectory(ROSTypeImagePubSubMono)
        add_subdirectory(ROSTypeImagePubSubExecutor)
    endif()

    add_subdirectory(ROSCustomTypeImagePubSub)

    # Advanced examples
    add_subdirectory(advanced)

elseif(DDS_BACKEND STREQUAL "adaptive-autosar")
    # Adaptive AUTOSAR backend: uses only ara::com interface APIs (C++14).

    if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        set(AUTOSAR_QNX_ARCH_DEFAULT "aarch64le")
        if(DEFINED ENV{AUTOSAR_QNX_ARCH} AND NOT "$ENV{AUTOSAR_QNX_ARCH}" STREQUAL "")
            set(AUTOSAR_QNX_ARCH_DEFAULT "$ENV{AUTOSAR_QNX_ARCH}")
        elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
            set(AUTOSAR_QNX_ARCH_DEFAULT "x86_64")
        endif()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/qnx/autosar-ap-libs)
        set(AUTOSAR_AP_PREFIX_DEFAULT /opt/qnx/autosar_ap/${AUTOSAR_QNX_ARCH_DEFAULT})
        set(DDS_PREFIX_DEFAULT /opt/qnx/cyclonedds)
        set(VSOMEIP_PREFIX_DEFAULT /opt/qnx/vsomeip)
        set(ICEORYX_PREFIX_DEFAULT /opt/qnx/iceoryx)
    else()
        set(LWRCL_INSTALL_PREFIX_DEFAULT /opt/autosar-ap-libs)
        set(AUTOSAR_AP_PREFIX_DEFAULT /opt/autosar_ap)
        set(DDS_PREFIX_DEFAULT /opt/cyclonedds)
        set(VSOMEIP_PREFIX_DEFAULT /opt/vsomeip)
        set(ICEORYX_PREFIX_DEFAULT /opt/iceoryx)
    endif()

    set(LWRCL_INSTALL_PREFIX "${LWRCL_INSTALL_PREFIX_DEFAULT}" CACHE PATH "LWRCL install prefix")
    set(AUTOSAR_AP_PREFIX "${AUTOSAR_AP_PREFIX_DEFAULT}" CACHE PATH "Adaptive AUTOSAR AP install prefix")
    set(DDS_PREFIX "${DDS_PREFIX_DEFAULT}" CACHE PATH "CycloneDDS install prefix (transitive dep)")
    set(VSOMEIP_PREFIX "${VSOMEIP_PREFIX_DEFAULT}" CACHE PATH "vsomeip install prefix (transitive dep)")
    set(AUTOSAR_GENERATED_PROXY_SKELETON_DIR "${AUTOSAR_GENERATED_PROXY_SKELETON_DIR}" CACHE PATH "Generated Adaptive AUTOSAR proxy/skeleton header directory")

    # Find Adaptive AUTOSAR package
    list(APPEND CMAKE_PREFIX_PATH "${AUTOSAR_AP_PREFIX}/lib/cmake/AdaptiveAutosarAP")
    list(APPEND CMAKE_PREFIX_PATH "${DDS_PREFIX}/lib/cmake")
    find_package(AdaptiveAutosarAP REQUIRED)

    # iceoryx headers (transitive CycloneDDS dependency)
    set(ICEORYX_PREFIX "${ICEORYX_PREFIX}" CACHE PATH "iceoryx install prefix")
    if(NOT ICEORYX_PREFIX)
        set(ICEORYX_PREFIX "${ICEORYX_PREFIX_DEFAULT}")
    endif()
    if(EXISTS "${ICEORYX_PREFIX}/lib/cmake")
        list(APPEND CMAKE_PREFIX_PATH "${ICEORYX_PREFIX}/lib/cmake")
        find_package(iceoryx_hoofs QUIET)
        find_package(iceoryx_posh QUIET)
        find_package(iceoryx_binding_c QUIET)
    endif()
    # Add iceoryx include directories (versioned layout)
    file(GLOB ICEORYX_INCLUDE_DIRS "${ICEORYX_PREFIX}/include/iceoryx/v*")
    foreach(dir ${ICEORYX_INCLUDE_DIRS})
        include_directories(${dir})
    endforeach()

    # Adaptive AUTOSAR headers
    include_directories(${AUTOSAR_AP_PREFIX}/include)
    # CycloneDDS headers (transitive dependency)
    include_directories(${DDS_PREFIX}/include)
    include_directories(${DDS_PREFIX}/include/ddscxx)
    # vsomeip headers (required when generated proxy/skeleton support is enabled)
    if(EXISTS "${VSOMEIP_PREFIX}/include")
        # Keep vsomeip include after CycloneDDS includes to avoid dds/* header shadowing.
        include_directories(AFTER SYSTEM ${VSOMEIP_PREFIX}/include)
    endif()
    # Data type headers
    include_directories(${LWRCL_INSTALL_PREFIX}/include)
    if(NOT AUTOSAR_GENERATED_PROXY_SKELETON_DIR OR
       NOT EXISTS "${AUTOSAR_GENERATED_PROXY_SKELETON_DIR}/lwrcl_autosar_proxy_skeleton.hpp")
        message(FATAL_ERROR
            "Adaptive AUTOSAR requires generated proxy/skeleton header: "
            "${AUTOSAR_GENERATED_PROXY_SKELETON_DIR}/lwrcl_autosar_proxy_skeleton.hpp")
    endif()
    include_directories(BEFORE ${AUTOSAR_GENERATED_PROXY_SKELETON_DIR})
    link_directories(${LWRCL_INSTALL_PREFIX}/lib)
    link_directories(${AUTOSAR_AP_PREFIX}/lib)
    link_directories(${DDS_PREFIX}/lib)
    if(EXISTS "${VSOMEIP_PREFIX}/lib")
        link_directories(${VSOMEIP_PREFIX}/lib)
    endif()

    add_compile_definitions(ARA_COM_USE_CYCLONEDDS=1)

    set(DDS_LIBRARIES AdaptiveAutosarAP::ara_com ddsc ddscxx vsomeip3 CACHE INTERNAL "DDS link libraries")

    # Common apps
    add_subdirectory(lwrcl_example)

    if(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
        add_subdirectory(ROSTypeImagePubSubEdge)
        add_subdirectory(ROSTypeImagePubSubMono)
        add_subdirectory(ROSTypeImagePubSubExecutor)
    endif()

    add_subdirectory(ROSCustomTypeImagePubSub)

    # Advanced examples
    add_subdirectory(advanced)

else()
    message(FATAL_ERROR "Unknown DDS_BACKEND: ${DDS_BACKEND}. Use 'fastdds', 'cyclonedds', 'vsomeip', or 'adaptive-autosar'.")
endif()
