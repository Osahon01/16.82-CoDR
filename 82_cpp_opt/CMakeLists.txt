cmake_minimum_required(VERSION 3.8)

project(82_Opt LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG " -g -O0")
# string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O3 -DNDEBUG")
# list(APPEND CMAKE_PREFIX_PATH "./pagmo2")

# find_package(Boost REQUIRED)
find_package(Pagmo REQUIRED)


add_executable(82opt
            82opt.cpp
            fitness.cpp
            82opt.h)


target_link_libraries(82opt PRIVATE Pagmo::pagmo)

target_include_directories(82opt PRIVATE ${Boost_INCLUDE_DIRS})

target_compile_options(82opt PRIVATE -lineinfo)
