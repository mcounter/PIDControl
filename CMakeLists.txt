project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/pid.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

find_path(UWS_INCLUDE_DIR uWS/uWS.h)
find_library(UWS_LIBRARY uWS)
include_directories(${UWS_INCLUDE_DIR})
link_libraries(${UWS_LIBRARY})

find_path(ZLIB_INCLUDE_DIR zlib.h)
find_library(ZLIB_LIBRARY zlib)
include_directories(${ZLIB_INCLUDE_DIR})
link_libraries(${ZLIB_LIBRARY}) 

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")


add_executable(pid ${sources})

target_link_libraries(pid z ssl uv uWS)
