cmake_minimum_required(VERSION 3.30)
project(racetrack_opt)

set(CMAKE_CXX_STANDARD 20)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(
        racetrack_opt
        construction_heur/main.cpp
        construction_heur/InitPathUtils.cpp
        construction_heur/InitPathUtils.h
        construction_heur/Track.cpp
        construction_heur/Track.h)
