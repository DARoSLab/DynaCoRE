##### TEMPERARY BUILD
#cmake_minimum_required(VERSION 2.8.12)

include_directories(${DART_INCLUDE_DIRS})
set(example_name dart_mini_cheetah)

include_directories ("${example_name}")

file(GLOB ${example_name}_srcs "*.cpp" "*.hpp")

add_executable(${example_name} ${${example_name}_srcs})
target_link_libraries(${example_name} ${DART_LIBRARIES})
#target_link_libraries(${example_name} Valkyrie_Controller dynacore_utils)
