FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(WBDC SHARED ${sources} ${headers})
target_link_libraries(WBDC dynacore_utils dynacore_Goldfarb)

install(TARGETS WBDC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/WBDC")
