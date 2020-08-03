FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(SJplanner SHARED ${sources} ${headers})
target_link_libraries(SJplanner SJutils)
