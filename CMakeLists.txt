cmake_minimum_required(VERSION 3.20)

project(DroneSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML CONFIG REQUIRED COMPONENTS Graphics Window System)

add_executable(DroneSimulator
main.cpp
)

target_include_directories(DroneSimulator PRIVATE .)

target_link_libraries(DroneSimulator PRIVATE
SFML::Graphics
SFML::Window
SFML::System
opengl32
glu32
ws2_32
)
