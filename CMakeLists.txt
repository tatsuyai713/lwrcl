cmake_minimum_required(VERSION 3.16.3)
project(rclmodoki)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp )
target_include_directories(${PROJECT_NAME} PUBLIC include)


include_directories(/opt/fast-dds-libs/include)
link_directories(/opt/fast-dds-libs/lib)

add_executable(example src/example.cpp)
target_link_libraries(example PRIVATE pthread ${PROJECT_NAME} fastrtps std_msgs sensor_msgs)
