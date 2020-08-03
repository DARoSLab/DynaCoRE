FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(OC2_model SHARED ${sources} ${headers})

target_link_libraries(OC2_model SJutils SJrbdl)

