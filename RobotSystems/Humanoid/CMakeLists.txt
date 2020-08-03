FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE headerspp *.hpp)
add_library(Humanoid_model SHARED ${sources} ${headerspp})

target_link_libraries (Humanoid_model dynacore_utils dynacore_rbdl)
