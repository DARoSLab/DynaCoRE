FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(WBDC_Rotor SHARED ${sources} ${headers})
target_link_libraries(WBDC_Rotor dynacore_utils dynacore_Goldfarb)

install(TARGETS WBDC_Rotor DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/WBDC_Rotor")
