cmake_minimum_required(VERSION 3.10)
project(bots_arm)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/FTServo
)

file(GLOB_RECURSE SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/jointsCtrl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/max.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FTServo/*.cpp
)

add_library(frodobots-bots_arm STATIC ${SOURCES})

add_executable(${PROJECT_NAME} ${SOURCES} "./main.cpp")
