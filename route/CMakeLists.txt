cmake_minimum_required (VERSION 2.6)
project (Route)

find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIRS})

find_package(casadi REQUIRED)
include_directories(${casadi_INCLUDE_DIRS})

include(cmake/mason.cmake)

add_definitions(-Wno-attributes)
add_definitions(-Wno-deprecated-declarations)

list(APPEND CMAKE_MODULE_PATH "/s2geometry/third_party/cmake")
add_subdirectory(/s2geometry s2geometry)
include_directories(/code)

include_directories(/nomad/src)
include_directories(/nomad/ext/sgtelib/src)

mason_use(variant VERSION 1.1.4 HEADER_ONLY)
mason_use(geometry VERSION 1.0.0 HEADER_ONLY)
mason_use(rapidjson VERSION 1.1.0 HEADER_ONLY)
mason_use(geojson VERSION 0.4.3 HEADER_ONLY)

add_library(nomad SHARED IMPORTED GLOBAL)
set_property(TARGET nomad PROPERTY IMPORTED_LOCATION "/nomad/builds/release/lib/libnomad.so")

add_library(simulation simulation.cpp)
target_link_libraries(simulation s2)
target_add_mason_package(simulation variant)
target_add_mason_package(simulation geometry)
target_add_mason_package(simulation rapidjson)
target_add_mason_package(simulation geojson)

add_library(opt opt.cpp)
target_link_libraries(opt
    simulation
    nomad
)

add_library(motion_primitive motion_primitive.cpp)
target_link_libraries(motion_primitive
    simulation
    opt
    nomad
)

add_executable(mp mp.cpp)
target_link_libraries(mp PUBLIC motion_primitive)

add_library(obstacle obstacle.cpp)
target_link_libraries(obstacle s2)
target_add_mason_package(obstacle variant)
target_add_mason_package(obstacle geometry)
target_add_mason_package(obstacle rapidjson)
target_add_mason_package(obstacle geojson)

add_library(graph graph.cpp)
target_link_libraries(graph 
    s2
    motion_primitive
    obstacle
    ${BOOST_LIBRARIES}
)

add_library(hlut hlut.cpp)
target_link_libraries(hlut PUBLIC graph)

add_library(astar astar.cpp)
target_link_libraries(astar PUBLIC graph)

add_library(landing landing.cpp)
target_link_libraries(landing
    obstacle
    s2
    casadi
)

add_executable(gen_hlut gen_hlut.cpp)
target_link_libraries(gen_hlut PUBLIC hlut)

add_executable(route main.cpp)

target_link_libraries(route
    astar
    motion_primitive
    hlut
    obstacle
    landing
    s2
    ${BOOST_LIBRARIES}
    casadi
)
target_add_mason_package(route variant)
target_add_mason_package(route geometry)
target_add_mason_package(route rapidjson)
target_add_mason_package(route geojson)

add_executable(opt_test opt_test.cpp)
target_link_libraries(opt_test 
    simulation
    opt
    nomad
)

add_executable(improve_viz improve_viz.cpp)
target_link_libraries(improve_viz
    simulation
)
