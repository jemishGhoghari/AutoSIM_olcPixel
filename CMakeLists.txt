cmake_minimum_required(VERSION 3.8)
project(AutoSIM_olcPixel)

# Specify the C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include Directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/Configs)
include_directories(${CMAKE_SOURCE_DIR}/Assets)
include_directories(${CMAKE_SOURCE_DIR}/libraries/olcPixelGameEngine)

# Add Source file
add_executable(autonomous_car
    src/auto_vehicle.cpp
    src/logger.cpp
    src/vehicle.cpp
)

# Target Link libraries
target_link_libraries(autonomous_car PRIVATE
    X11
    GL
    pthread
    png
    stdc++
    yaml-cpp
)