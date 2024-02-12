project("apps")

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(rcl_like_wrapper_example)
add_subdirectory(CustomROSTypeDataPublisher)
add_subdirectory(ROSTypeImagePubSub)
add_subdirectory(ROSTypeImagePubSubEdge)
add_subdirectory(ROSTypeImagePubSubMono)
add_subdirectory(ROSTypeImagePubSubExecutor)
add_subdirectory(CustomROSTypeDataPublisherExecutor)
add_subdirectory(CustomROSTypeDataSubscriber)


