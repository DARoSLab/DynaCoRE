FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_manipulator ${header} ${source} )

target_link_libraries (simulation_manipulator
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
  target_link_libraries(simulation_manipulator "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(simulation_manipulator ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)
