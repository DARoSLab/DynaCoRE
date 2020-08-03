FILE(GLOB_RECURSE c_sources src/*.c)
FILE(GLOB_RECURSE cpp_sources src/*.cpp)
add_library (dynacore_snopt SHARED ${c_sources} ${cpp_sources})
target_include_directories(dynacore_snopt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

   #find_library(snopt7_c++ NAMES libsnopt7_cpp.so.0 libsnopt7_cpp.so.0.0.0 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
   #find_library(snopt7_int NAMES libsnopt7.so.0 libsnopt7.so.0.0.0 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
   find_library(snopt7_c++ NAMES libsnopt7_cpp.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
   find_library(snopt7_int NAMES libsnopt7.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)

target_link_libraries (dynacore_snopt ${snopt7_c++} )
target_link_libraries (dynacore_snopt ${snopt7_int} ) 

add_subdirectory(examples) # Uncomment if you want to compile the snopt examples

