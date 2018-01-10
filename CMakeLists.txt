project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

## Boost for gnuplot-iostream.h
find_package(Boost REQUIRED COMPONENTS system iostreams)
if(Boost_FOUND)

    message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
    message(STATUS "Boost_VERSION: ${Boost_VERSION}")

    include_directories(${Boost_INCLUDE_DIRS})

endif()

#set(sources src/MPC.cpp src/main.cpp)
set(sources src/MPC.cpp src/main.cpp src/gnuplot-iostream.h)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})

#target_link_libraries(mpc ipopt z ssl uv uWS)

## target_link_libraries for gnuplot-iostream.h
# target_link_libraries(pid z ssl uv uWS)
target_link_libraries(mpc ipopt z ssl uv uWS ${Boost_LIBRARIES})