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

add_library(obstacle obstacle.cpp)
target_link_libraries(obstacle PUBLIC s2)
target_add_mason_package(obstacle PUBLIC variant)
target_add_mason_package(obstacle PUBLIC geometry)
target_add_mason_package(obstacle PUBLIC rapidjson)
target_add_mason_package(obstacle PUBLIC geojson)

add_library(graph graph.cpp)
target_link_libraries(graph PUBLIC s2)
target_link_libraries(graph PUBLIC simulation)
target_link_libraries(graph PUBLIC motion_primitive)
target_link_libraries(graph PUBLIC obstacle)
target_link_libraries(graph PUBLIC ${BOOST_LIBRARIES})

add_library(hlut hlut.cpp)
target_link_libraries(hlut PUBLIC graph)

add_library(astar astar.cpp)
target_link_libraries(astar PUBLIC graph)

add_library(landing landing.cpp)
target_link_libraries(landing PUBLIC obstacle)
target_link_libraries(landing PUBLIC s2)
target_link_libraries(landing PUBLIC casadi)

add_executable(gen_hlut gen_hlut.cpp)
target_link_libraries(gen_hlut PUBLIC hlut)

add_executable(route main.cpp)

target_link_libraries(route PUBLIC astar)
target_link_libraries(route PUBLIC s2)
target_add_mason_package(route PUBLIC variant)
target_add_mason_package(route PUBLIC geometry)
target_add_mason_package(route PUBLIC rapidjson)
target_add_mason_package(route PUBLIC geojson)
target_link_libraries(route PUBLIC simulation)
target_link_libraries(route PUBLIC motion_primitive)
target_link_libraries(route PUBLIC hlut)
target_link_libraries(route PUBLIC obstacle)
target_link_libraries(route PUBLIC landing)
target_link_libraries(route PUBLIC ${BOOST_LIBRARIES})
target_link_libraries(route PUBLIC casadi)

add_executable(sim sim.cpp)
target_link_libraries(sim PUBLIC simulation)
target_link_libraries(sim PUBLIC s2)
target_link_libraries(sim PUBLIC opt)

add_executable(opt_test opt_test.cpp)
target_link_libraries(opt_test PUBLIC simulation)
target_link_libraries(opt_test PUBLIC nomad)
target_link_libraries(opt_test PUBLIC opt)

