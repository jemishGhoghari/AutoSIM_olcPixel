cmake_minimum_required(VERSION 3.8)
project(olcPixelGameEngine_Ex)

# Specify the C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include Directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/Configs)
include_directories(${CMAKE_SOURCE_DIR}/Assets)

# Add Source file
add_executable(autonomous_car src/auto_vehicle.cpp)

# link header files to executable
# target_sources(autonomous_car PRIVATE
#     ${CMAKE_SOURCE_DIR}/include/auto_vehicle.h
# )

# target_sources(autonomous_car PRIVATE
#     ${CMAKE_SOURCE_DIR}/include/olcPixelGameEngine.h
# )

target_link_libraries(autonomous_car PRIVATE
    X11
    GL
    pthread
    png
    stdc++
    yaml-cpp
)