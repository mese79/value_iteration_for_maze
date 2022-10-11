cmake_minimum_required(VERSION 2.8)
project(policy-assignment)

configure_file("show.py" "show.py" COPYONLY)
configure_file("grid_world.py" "grid_world.py" COPYONLY)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fconcepts")
set(SOURCES main.cpp value_iteration.cpp value_iteration.h utils.h)

add_executable(${PROJECT_NAME} ${SOURCES})
