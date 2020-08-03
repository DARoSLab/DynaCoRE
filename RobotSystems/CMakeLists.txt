add_subdirectory(NAO)
add_subdirectory(Valkyrie)
add_subdirectory(Mercury)
add_subdirectory(Atlas)
add_subdirectory(DracoBip)
add_subdirectory(Humanoid)
add_subdirectory(TELLO)
#add_subdirectory(SagitP3)
#add_subdirectory(Quadruped)

install(FILES RobotSystem.hpp RobotSystem2D.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")
