FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(Draco_model SHARED ${sources} ${headers})

target_link_libraries(Draco_model SJutils)
target_link_libraries(Draco_model SJrbdl)
