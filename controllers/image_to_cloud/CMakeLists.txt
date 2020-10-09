cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)

project(PROJECT)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# configure webots library
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set(WEBOTS_LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} 
                     ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
set(WEBOTS_CAR_LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}car${CMAKE_SHARED_LIBRARY_SUFFIX}
                     ${CMAKE_SHARED_LIBRARY_PREFIX}CppCar${CMAKE_SHARED_LIBRARY_SUFFIX})
set(WEBOTS_DRIVER_LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}driver${CMAKE_SHARED_LIBRARY_SUFFIX}
                     ${CMAKE_SHARED_LIBRARY_PREFIX}CppDriver${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} ) 

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

file(GLOB files "${CMAKE_SOURCE_DIR}/*.cpp")
foreach(filename ${files})
    get_filename_component(EXEC ${filename} NAME_WE)
    add_executable(${EXEC} ${filename} ${HEADERS})
    target_link_libraries(${EXEC} ${PCL_LIBRARIES} ${WEBOTS_LIBRARIES} ${WEBOTS_DRIVER_LIBRARIES})
endforeach()

