FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_cheetah3 ${header} ${source} )

target_link_libraries (simulation_cheetah3
  common
  LieGroup
  srDyn
  SR_SysGenerator
  srg
  Renderer
  util
  ${OPENGL_LIBRARIES}
  dynacore_urdf
  Cheetah3_Controller
)

if (APPLE)
  target_link_libraries(simulation_cheetah3 "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(simulation_cheetah3 ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)
