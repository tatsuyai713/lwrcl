cmake_minimum_required(VERSION 3.16.3)
project(lwrcl_ffi)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(LWRCL_REPO_ROOT ${PROJECT_SOURCE_DIR}/../../..)
set(LWRCL_IDL_ROOT ${LWRCL_REPO_ROOT}/data_types/src/ros-data-types-for-fastdds/src)
set(LWRCL_FFI_GEN_SCRIPT ${LWRCL_REPO_ROOT}/scripts/generate_lwrcl_ffi.py)
set(LWRCL_FFI_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(LWRCL_FFI_GEN_HEADER ${LWRCL_FFI_GEN_DIR}/lwrcl_ffi_generated.h)
set(LWRCL_FFI_GEN_SOURCE ${LWRCL_FFI_GEN_DIR}/lwrcl_ffi_generated.cpp)
set(LWRCL_DART_GEN ${LWRCL_REPO_ROOT}/packages/lwrcl_dart/lib/src/generated/messages.dart)

add_custom_command(
    OUTPUT ${LWRCL_FFI_GEN_HEADER} ${LWRCL_FFI_GEN_SOURCE} ${LWRCL_DART_GEN}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${LWRCL_FFI_GEN_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${LWRCL_REPO_ROOT}/packages/lwrcl_dart/lib/src/generated
    COMMAND ${Python3_EXECUTABLE} ${LWRCL_FFI_GEN_SCRIPT}
            --idl-root ${LWRCL_IDL_ROOT}
            --out-h ${LWRCL_FFI_GEN_HEADER}
            --out-cpp ${LWRCL_FFI_GEN_SOURCE}
            --out-dart ${LWRCL_DART_GEN}
    DEPENDS ${LWRCL_FFI_GEN_SCRIPT}
    COMMENT "Generating lwrcl FFI bindings"
)

add_custom_target(lwrcl_ffi_generated
    DEPENDS ${LWRCL_FFI_GEN_HEADER} ${LWRCL_FFI_GEN_SOURCE}
)

file(GLOB LWRCL_MSG_PACKAGE_DIRS LIST_DIRECTORIES true ${LWRCL_IDL_ROOT}/*)
set(LWRCL_MSG_LIBS "")
foreach(pkg_dir ${LWRCL_MSG_PACKAGE_DIRS})
    if(IS_DIRECTORY ${pkg_dir})
        get_filename_component(pkg_name ${pkg_dir} NAME)
        list(APPEND LWRCL_MSG_LIBS ${pkg_name})
    endif()
endforeach()

add_library(${PROJECT_NAME} SHARED
    src/lwrcl_ffi.cpp
    ${LWRCL_FFI_GEN_SOURCE}
)

add_dependencies(${PROJECT_NAME} lwrcl_ffi_generated)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${LWRCL_FFI_GEN_DIR}
        ${ROS_DATA_TYPES_INCLUDE_PATH}
)

target_link_libraries(${PROJECT_NAME}
    lwrcl
    ${LWRCL_MSG_LIBS}
)

if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE LWRCL_FFI_EXPORTS)
endif()

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES
    include/lwrcl_ffi.h
    ${LWRCL_FFI_GEN_HEADER}
    DESTINATION include/
)
