FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(WBDC_Relax SHARED ${sources} ${headers})
target_link_libraries(WBDC_Relax dynacore_utils dynacore_gurobi dynacore_Goldfarb)

install(TARGETS WBDC_Relax DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/WBDC_Relax")
