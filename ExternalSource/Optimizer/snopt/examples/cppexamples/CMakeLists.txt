add_executable(snopt_cpp_catmixa catmixa.cpp)
add_executable(snopt_cpp_hs118 hs118.cpp)
add_executable(snopt_cpp_sntoya sntoya.cpp)
add_executable(snopt_cpp_sntoyb sntoyb.cpp)
add_executable(snopt_cpp_sntoyc sntoyc.cpp)

target_include_directories(snopt_cpp_catmixa PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include/)
target_include_directories(snopt_cpp_hs118 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include/)
target_include_directories(snopt_cpp_sntoya PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include/)
target_include_directories(snopt_cpp_sntoyb PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include/)
target_include_directories(snopt_cpp_sntoyc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include/)

target_link_libraries(snopt_cpp_catmixa dynacore_snopt)
target_link_libraries(snopt_cpp_hs118 dynacore_snopt)
target_link_libraries(snopt_cpp_sntoya dynacore_snopt)
target_link_libraries(snopt_cpp_sntoyb dynacore_snopt)
target_link_libraries(snopt_cpp_sntoyc dynacore_snopt)

