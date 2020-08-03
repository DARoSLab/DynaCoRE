include_directories(${DART_INCLUDE_DIRS})
set(example_name dart_wblc_valkyrie)

include_directories ("${example_name}")

file(GLOB ${example_name}_srcs "*.cpp" "*.hpp")

add_executable(${example_name} ${${example_name}_srcs})
target_link_libraries(${example_name} ${DART_LIBRARIES})
target_link_libraries(${example_name} Valkyrie_Controller 
    dynacore_utils)
