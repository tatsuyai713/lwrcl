cmake_minimum_required(VERSION 3.5)
project(CycloneDDS_PubSub)

set(CMAKE_CXX_STANDARD 14)

# CycloneDDSのパスを指定

if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    message("|||||||||QNX||||||||")
    include_directories(/opt/qnx/cyclonedds/include)
    include_directories(/opt/qnx/cyclonedds/include/ddscxx)
    include_directories(/opt/qnx/cyclonedds/include)
    # std_msgsのパスを指定
    include_directories(/opt/qnx/cyclonedds-libs/include)
    link_directories(/opt/qnx/cyclonedds-libs/lib)
    link_directories(/opt/qnx/cyclonedds/lib)
else()
    include_directories(/opt/cyclonedds/include)
    include_directories(/opt/cyclonedds/include/ddscxx)
    link_directories(/opt/cyclonedds/lib)
    # std_msgsのパスを指定
    include_directories(/opt/cyclonedds-libs/include)
    link_directories(/opt/cyclonedds-libs/lib)
    link_directories(/opt/cyclonedds/lib)
endif()


# 実行ファイルの設定
add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ddsc ddscxx std_msgs builtin_interfaces)

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ddsc ddscxx std_msgs builtin_interfaces)