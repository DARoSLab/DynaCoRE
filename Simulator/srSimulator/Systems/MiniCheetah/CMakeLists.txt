FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_mini_cheetah ${header} ${source} )

target_link_libraries (simulation_mini_cheetah
  common
  LieGroup
  srDyn
  SR_SysGenerator
  srg
  Renderer
  util
  ${OPENGL_LIBRARIES}
  dynacore_urdf
)

if (APPLE)
  target_link_libraries(simulation_mini_cheetah "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(simulation_mini_cheetah ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)
