project("apps")

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
endif()

add_subdirectory(rcl_like_wrapper_example)
add_subdirectory(CustomROSTypeDataPublisher)
add_subdirectory(CustomROSTypeDataSubscriber)


