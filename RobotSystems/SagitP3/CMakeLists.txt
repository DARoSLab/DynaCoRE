FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE headerspp *.hpp)
add_library(SagitP3_model SHARED ${sources} ${headerspp})

target_link_libraries (SagitP3_model dynacore_utils dynacore_rbdl)
