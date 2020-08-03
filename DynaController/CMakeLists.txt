#add_subdirectory(NAO_Exam_Controller)
add_subdirectory(NAO_Controller)
add_subdirectory(Valkyrie_Controller)
add_subdirectory(DracoBip_Controller)
add_subdirectory(Atlas_Controller)
add_subdirectory(Mercury_Controller)
add_subdirectory(Humanoid_Controller)
add_subdirectory(TELLO_Controller)
#add_subdirectory(SagitP3_Controller)

install(FILES interface.hpp Test.hpp Controller.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")
