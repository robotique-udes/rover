cmake_minimum_required(VERSION 3.16)
project(rover_shared_libs LANGUAGES CXX)

file(GLOB_RECURSE LIB_HEADERS CONFIGURE_DEPENDS
    rover_can2/src/*.hpp
    rover_lib2/src/*.hpp
    rovus_lib/src/*.hpp
)
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    rover_can2/src/*.cpp
    rover_lib2/src/*.cpp
    rovus_lib/src/*.cpp
)

add_library(rover_shared_libs ${LIB_HEADERS} ${LIB_SOURCES})

target_include_directories(rover_shared_libs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/rover_can2/src
    ${CMAKE_CURRENT_SOURCE_DIR}/rover_lib2/src
    ${CMAKE_CURRENT_SOURCE_DIR}/rovus_lib/src
)

target_compile_features(rover_shared_libs PUBLIC cxx_std_23)
target_compile_options(rover_shared_libs PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS rover_shared_libs
        EXPORT rover_shared_libsTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)

install(DIRECTORY rover_can2/src/rover_can2
        DESTINATION include)

install(DIRECTORY rover_lib2/src/rover_lib2
        DESTINATION include)

install(DIRECTORY rovus_lib/src/rovus_lib
        DESTINATION include)
