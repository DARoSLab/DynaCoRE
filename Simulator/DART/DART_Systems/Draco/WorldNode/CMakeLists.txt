##### TEMPERARY BUILD
#cmake_minimum_required(VERSION 2.8.12)

include_directories(${DART_INCLUDE_DIRS})
set(example_name draco)

#include_directories(${DART_INCLUDE_DIRS})
include_directories ("${example_name}")
include_directories ("${example_name}/StateEstimation")
include_directories ("${example_name}/WorldNode")

file(GLOB ${example_name}_srcs "*.cpp" "*.hpp"
               "WorldNode/*.cpp" "WorldNode/*.hpp"
               "StateEstimation/*.cpp" "StateEstimation/*.hpp")

add_executable(${example_name} ${${example_name}_srcs})
target_link_libraries(${example_name} ${DART_LIBRARIES} )
