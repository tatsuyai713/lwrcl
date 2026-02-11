# Processor specification, possible values: x86_64, armv7le, aarach64, ...
set(CMAKE_SYSTEM_PROCESSOR aarch64)
# Specify arch to be aarch64le
set(arch gcc_ntoaarch64le)
set(ARCH_NAME aarch64le)
set(OBJCOPY_COMMAND aarch64-unknown-nto-qnx7.1.0-objcopy)
set(STRIP_COMMAND aarch64-unknown-nto-qnx7.1.0-strip)

# Command settings for QNX
# Cross compilation for QNX
set(CMAKE_SYSTEM_NAME QNX)
set(BUILD_WITH_QNX YES)

# Upstream settings
set(CMAKE_C_COMPILER qcc)
set(CMAKE_C_COMPILER_TARGET ${arch})
set(CMAKE_CXX_COMPILER q++)
set(CMAKE_CXX_COMPILER_TARGET ${arch})

# Suppose you have already set QNX_HOST and QNX_TARGET Add compiler and linker
# flags -std=gnu++14 is needed because QNX header files only cover GNU and POSIX

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# CMAKE_CXX_EXTENSIONS=ON will introduce gnu++14 instead c++14
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options("-c")
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# Enable "stack-protector-strong" explicitly (enabled by QDP default)
add_compile_options(-fstack-protector-strong)

add_compile_definitions(_FORTIFY_SOURCE=2)

# _QNX_SOURCE: See qnx doc for reference.
add_compile_definitions(_QNX_SOURCE)


set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-g -D_DEBUG")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")

if("${ARCH_NAME}" STREQUAL "x86_64")
  option(ENABLE_LIBSTDCXX "Enable build with libstdc++. default=off=libc++. Experimental feature" ON)
  option(ENABLE_QNX_IOSOCK "Enable QNX io-sock(libsocket.so.4), default=on=io-sock" OFF)
else()
  option(ENABLE_LIBSTDCXX "Enable build with libstdc++. default=off=libc++. Experimental feature" OFF)
  option(ENABLE_QNX_IOSOCK "Enable QNX io-sock(libsocket.so.4), default=on=io-sock" ON)
endif()
if (ENABLE_LIBSTDCXX)
add_compile_options(-Y_gpp)
add_link_options(-Y_gpp)
endif()

# Add definitions command for diffent platforms So we can use macro definition
# "#ifdef PLATFORM_QNX" in source code
set(PLATFORM_QNX ON)
add_definitions(-DPLATFORM_QNX)
# Build crossguid library for qnx
set(GUID_LIBUUID ON)
add_definitions(-DGUID_LIBUUID)

if("${CMAKE_SYSTEM_NAME}" STREQUAL "QNX")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DIFM_TOKEN=0x00000040 -DIFM_FDDI=0x00000060")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIFM_TOKEN=0x00000040 -DIFM_FDDI=0x00000060")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
endif()

add_compile_options(
  -Wno-error=pedantic
  -Wno-error=sign-compare
  -Wno-error=unused-variable
  -Wno-error=format
)
if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    add_definitions(-DNO_MCAST_SUPPORT)
endif()