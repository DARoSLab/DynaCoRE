FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)

add_library (Polyhedron SHARED ${header} ${source})

target_link_libraries (Polyhedron CDDSrc SJgurobi)
