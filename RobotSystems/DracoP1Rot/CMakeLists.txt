FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(${CTRL_SYS}_model SHARED ${sources} ${headers})

target_link_libraries(${CTRL_SYS}_model SJutils SJrbdl)
