#find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui gui-osg CONFIG)
find_package(DART 6.8 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})
add_subdirectory (DART_Systems)

