FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE cheaders *.h)
FILE(GLOB_RECURSE sources *.cpp)

set(CTRL_SYS Mercury)

add_library (${CTRL_SYS}_Walking SHARED ${headers} ${sources} ../Test.cpp)
target_link_libraries (${CTRL_SYS}_Walking dynacore_utils dynacore_filters)
target_link_libraries (${CTRL_SYS}_Walking ${CTRL_SYS}_model)
target_link_libraries (${CTRL_SYS}_Walking WBDC_Rotor)

target_link_libraries (${CTRL_SYS}_Walking dynacore_param_handler PIPM_planner)

install(TARGETS ${CTRL_SYS}_Walking DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/${CTRL_SYS}_Walking")
install(FILES ${cheaders} DESTINATION
    "${INSTALL_INCLUDE_DIR}/${CTRL_SYS}_Walking")
