cmake_minimum_required(VERSION 3.10)
project(Dijikstra_backend)

set(CMAKE_CXX_STANDARD 17)

include_directories(http json/include)

add_executable(Dijikstra_backend src/main.cpp)

if (WIN32)
    target_link_libraries(Dijikstra_backend ws2_32 crypt32)
endif()