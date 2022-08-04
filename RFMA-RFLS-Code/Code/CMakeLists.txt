cmake_minimum_required(VERSION 3.15)
project(UWCP)

set(CMAKE_CXX_STANDARD 11)

set(INC_DIR ./object/json)
set(LINK_DIR ./object/jsoncpp.lib)

include_directories($ {INC_DIR})
link_directories($ {LINK_DIR})

SET(CMAKE_BUILD_TYPE "Release")

add_executable(UWCP main.cpp object/Point.cpp object/Point.h object/Vehicle.cpp object/Vehicle.h object/DataLoader.cpp
        object/DataLoader.h object/jsoncpp.cpp InitialSolution.cpp InitialSolution.h LocalSearch.cpp LocalSearch.h
        MemeticAlgorithm.cpp MemeticAlgorithm.h)