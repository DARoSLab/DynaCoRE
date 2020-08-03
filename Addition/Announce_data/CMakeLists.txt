project (HUME_Announcer)
cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

set (${PROJECT_NAME}_VERSION_NAME "moon")
set (${PROJECT_NAME}_VERSION_MAJOR 1)
set (${PROJECT_NAME}_VERSION_MINOR 0)
set (${PROJECT_NAME}_VERSION_PATCH 0)
set (${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})

message ("")
message ("-[ MAKE INFORMATION ]-----------------------------------------------------------")
message (" The project name: ${PROJECT_NAME}")
message (" The version: ${${PROJECT_NAME}_VERSION}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${PROJECT_BINARY_DIR}")
message (" The install destination: ${CMAKE_INSTALL_PREFIX}")
message ("--------------------------------------------------------------------------------")
message ("")


set (headers 
IMU_Announcer.h
../../util/Sejong_Thread.h
)

set (sources
IMU_Announcer.cpp
Announcer.cpp
../../util/Sejong_Thread.cpp
)

include_directories ("${PROJECT_SOURCE_DIR}/../../“)
link_directories("${PROJECT_SOURCE_DIR}/../../build/util/")
link_directories("/usr/local/lib/")
link_directories("/usr/lib/")

add_executable(Hume_Announcer ${sources} ${headers})
target_link_libraries(Hume_Announcer pthread)
target_link_libraries(Hume_Announcer util)

