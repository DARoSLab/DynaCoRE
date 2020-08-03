FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(OC3_model SHARED ${sources} ${headers})

target_link_libraries(OC3_model SJutils)
target_link_libraries(OC3_model SJrbdl)
