cmake_minimum_required (VERSION 2.6)
project (Route)

include(cmake/mason.cmake)
include(ExternalProject)

add_definitions(-Wno-attributes)
add_definitions(-Wno-deprecated-declarations)

list(APPEND CMAKE_MODULE_PATH "/s2geometry/third_party/cmake")
add_subdirectory(/s2geometry s2geometry)
include_directories(/code)

include_directories(/nomad/src)
include_directories(/nomad/ext/sgtelib/src)
#link_directories(/code/nomad.3.9.1/lib)

mason_use(variant VERSION 1.1.4 HEADER_ONLY)
mason_use(geometry VERSION 1.0.0 HEADER_ONLY)
mason_use(rapidjson VERSION 1.1.0 HEADER_ONLY)
mason_use(geojson VERSION 0.4.3 HEADER_ONLY)
mason_use(boost VERSION 1.67.0 HEADER_ONLY)

add_library(nomad SHARED IMPORTED GLOBAL)
set_property(TARGET nomad PROPERTY IMPORTED_LOCATION "/nomad/builds/release/lib/libnomad.so")

add_library(simulation simulation.cpp)
target_link_libraries(simulation PUBLIC s2)
target_add_mason_package(simulation PUBLIC variant)
target_add_mason_package(simulation PUBLIC geometry)
target_add_mason_package(simulation PUBLIC rapidjson)
target_add_mason_package(simulation PUBLIC geojson)

add_library(opt opt.cpp)
target_link_libraries(opt PUBLIC simulation)
target_link_libraries(opt PUBLIC nomad)

add_library(motion_primitive motion_primitive.cpp)
target_link_libraries(motion_primitive PUBLIC simulation)
target_link_libraries(motion_primitive PUBLIC opt)
target_link_libraries(motion_primitive PUBLIC nomad)
target_add_mason_package(motion_primitive PUBLIC boost)

add_executable(mp mp.cpp)
target_link_libraries(mp PUBLIC motion_primitive)

add_library(astar astar.cpp)
target_add_mason_package(astar PUBLIC boost)
target_link_libraries(astar PUBLIC s2)
target_link_libraries(astar PUBLIC simulation)
target_link_libraries(astar PUBLIC motion_primitive)

add_executable(route main.cpp)

target_link_libraries(route PUBLIC astar)
target_link_libraries(route PUBLIC s2)
target_add_mason_package(route PUBLIC variant)
target_add_mason_package(route PUBLIC geometry)
target_add_mason_package(route PUBLIC rapidjson)
target_add_mason_package(route PUBLIC geojson)
target_add_mason_package(route PUBLIC boost)
target_link_libraries(route PUBLIC simulation)
target_link_libraries(route PUBLIC motion_primitive)

add_executable(sim sim.cpp)
target_link_libraries(sim PUBLIC simulation)
target_link_libraries(sim PUBLIC s2)

# add_executable(opt opt.cpp)
# target_link_libraries(opt PUBLIC simulation)
# target_link_libraries(opt PUBLIC nomad)

