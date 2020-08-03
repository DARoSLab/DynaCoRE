FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(hume_model SHARED ${sources} ${headers})

# find_library(util_package utils)
# find_library(rbdl_lib rbdl)
target_link_libraries(hume_model SJutils)
target_link_libraries(hume_model rbdl)

install(TARGETS hume_model DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/ControlSystem/Hume_Controller/Hume_Model")
