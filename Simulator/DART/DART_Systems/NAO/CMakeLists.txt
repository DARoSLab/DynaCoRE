set(example_name nao)
file(GLOB ${example_name}_srcs "*.cpp" "*.hpp")

add_executable(${example_name} ${${example_name}_srcs})
target_link_libraries(${example_name} ${DART_LIBRARIES})
target_link_libraries(${example_name} NAO_Exam_Controller dynacore_utils)
