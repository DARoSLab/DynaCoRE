cmake_minimum_required(VERSION 2.8.12)

include_directories(${DART_INCLUDE_DIRS})
set(example_name dart_Mercury)

#include_directories(${DART_INCLUDE_DIRS})
include_directories ("${example_name}")

message("Dart) Mercury is building")
file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} ${DART_LIBRARIES})
target_link_libraries(${example_name} Mercury_Controller dynacore_utils)
 
