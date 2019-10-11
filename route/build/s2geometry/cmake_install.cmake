# Install script for directory: /s2geometry

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2" TYPE FILE FILES
    "/s2geometry/src/s2/_fp_contract_off.h"
    "/s2geometry/src/s2/encoded_s2cell_id_vector.h"
    "/s2geometry/src/s2/encoded_s2point_vector.h"
    "/s2geometry/src/s2/encoded_s2shape_index.h"
    "/s2geometry/src/s2/encoded_string_vector.h"
    "/s2geometry/src/s2/encoded_uint_vector.h"
    "/s2geometry/src/s2/id_set_lexicon.h"
    "/s2geometry/src/s2/mutable_s2shape_index.h"
    "/s2geometry/src/s2/r1interval.h"
    "/s2geometry/src/s2/r2.h"
    "/s2geometry/src/s2/r2rect.h"
    "/s2geometry/src/s2/s1angle.h"
    "/s2geometry/src/s2/s1chord_angle.h"
    "/s2geometry/src/s2/s1interval.h"
    "/s2geometry/src/s2/s2boolean_operation.h"
    "/s2geometry/src/s2/s2builder.h"
    "/s2geometry/src/s2/s2builder_graph.h"
    "/s2geometry/src/s2/s2builder_layer.h"
    "/s2geometry/src/s2/s2builderutil_closed_set_normalizer.h"
    "/s2geometry/src/s2/s2builderutil_find_polygon_degeneracies.h"
    "/s2geometry/src/s2/s2builderutil_graph_shape.h"
    "/s2geometry/src/s2/s2builderutil_lax_polygon_layer.h"
    "/s2geometry/src/s2/s2builderutil_s2point_vector_layer.h"
    "/s2geometry/src/s2/s2builderutil_s2polygon_layer.h"
    "/s2geometry/src/s2/s2builderutil_s2polyline_layer.h"
    "/s2geometry/src/s2/s2builderutil_s2polyline_vector_layer.h"
    "/s2geometry/src/s2/s2builderutil_snap_functions.h"
    "/s2geometry/src/s2/s2builderutil_testing.h"
    "/s2geometry/src/s2/s2cap.h"
    "/s2geometry/src/s2/s2cell.h"
    "/s2geometry/src/s2/s2cell_id.h"
    "/s2geometry/src/s2/s2cell_index.h"
    "/s2geometry/src/s2/s2cell_union.h"
    "/s2geometry/src/s2/s2centroids.h"
    "/s2geometry/src/s2/s2closest_cell_query.h"
    "/s2geometry/src/s2/s2closest_cell_query_base.h"
    "/s2geometry/src/s2/s2closest_edge_query.h"
    "/s2geometry/src/s2/s2closest_edge_query_base.h"
    "/s2geometry/src/s2/s2closest_point_query.h"
    "/s2geometry/src/s2/s2closest_point_query_base.h"
    "/s2geometry/src/s2/s2contains_point_query.h"
    "/s2geometry/src/s2/s2contains_vertex_query.h"
    "/s2geometry/src/s2/s2convex_hull_query.h"
    "/s2geometry/src/s2/s2coords_internal.h"
    "/s2geometry/src/s2/s2coords.h"
    "/s2geometry/src/s2/s2crossing_edge_query.h"
    "/s2geometry/src/s2/s2debug.h"
    "/s2geometry/src/s2/s2distance_target.h"
    "/s2geometry/src/s2/s2earth.h"
    "/s2geometry/src/s2/s2edge_clipping.h"
    "/s2geometry/src/s2/s2edge_crosser.h"
    "/s2geometry/src/s2/s2edge_crossings.h"
    "/s2geometry/src/s2/s2edge_distances.h"
    "/s2geometry/src/s2/s2edge_tessellator.h"
    "/s2geometry/src/s2/s2edge_vector_shape.h"
    "/s2geometry/src/s2/s2error.h"
    "/s2geometry/src/s2/s2furthest_edge_query.h"
    "/s2geometry/src/s2/s2latlng.h"
    "/s2geometry/src/s2/s2latlng_rect.h"
    "/s2geometry/src/s2/s2latlng_rect_bounder.h"
    "/s2geometry/src/s2/s2lax_loop_shape.h"
    "/s2geometry/src/s2/s2lax_polygon_shape.h"
    "/s2geometry/src/s2/s2lax_polyline_shape.h"
    "/s2geometry/src/s2/s2loop.h"
    "/s2geometry/src/s2/s2loop_measures.h"
    "/s2geometry/src/s2/s2measures.h"
    "/s2geometry/src/s2/s2metrics.h"
    "/s2geometry/src/s2/s2max_distance_targets.h"
    "/s2geometry/src/s2/s2min_distance_targets.h"
    "/s2geometry/src/s2/s2padded_cell.h"
    "/s2geometry/src/s2/s2point.h"
    "/s2geometry/src/s2/s2point_vector_shape.h"
    "/s2geometry/src/s2/s2point_compression.h"
    "/s2geometry/src/s2/s2point_index.h"
    "/s2geometry/src/s2/s2point_region.h"
    "/s2geometry/src/s2/s2point_span.h"
    "/s2geometry/src/s2/s2pointutil.h"
    "/s2geometry/src/s2/s2polygon.h"
    "/s2geometry/src/s2/s2polyline.h"
    "/s2geometry/src/s2/s2polyline_alignment.h"
    "/s2geometry/src/s2/s2polyline_measures.h"
    "/s2geometry/src/s2/s2polyline_simplifier.h"
    "/s2geometry/src/s2/s2predicates.h"
    "/s2geometry/src/s2/s2projections.h"
    "/s2geometry/src/s2/s2r2rect.h"
    "/s2geometry/src/s2/s2region.h"
    "/s2geometry/src/s2/s2region_term_indexer.h"
    "/s2geometry/src/s2/s2region_coverer.h"
    "/s2geometry/src/s2/s2region_intersection.h"
    "/s2geometry/src/s2/s2region_union.h"
    "/s2geometry/src/s2/s2shape.h"
    "/s2geometry/src/s2/s2shape_index.h"
    "/s2geometry/src/s2/s2shape_index_buffered_region.h"
    "/s2geometry/src/s2/s2shape_index_region.h"
    "/s2geometry/src/s2/s2shape_measures.h"
    "/s2geometry/src/s2/s2shapeutil_build_polygon_boundaries.h"
    "/s2geometry/src/s2/s2shapeutil_coding.h"
    "/s2geometry/src/s2/s2shapeutil_contains_brute_force.h"
    "/s2geometry/src/s2/s2shapeutil_count_edges.h"
    "/s2geometry/src/s2/s2shapeutil_edge_iterator.h"
    "/s2geometry/src/s2/s2shapeutil_get_reference_point.h"
    "/s2geometry/src/s2/s2shapeutil_range_iterator.h"
    "/s2geometry/src/s2/s2shapeutil_shape_edge.h"
    "/s2geometry/src/s2/s2shapeutil_shape_edge_id.h"
    "/s2geometry/src/s2/s2shapeutil_testing.h"
    "/s2geometry/src/s2/s2shapeutil_visit_crossing_edge_pairs.h"
    "/s2geometry/src/s2/s2testing.h"
    "/s2geometry/src/s2/s2text_format.h"
    "/s2geometry/src/s2/s2wedge_relations.h"
    "/s2geometry/src/s2/sequence_lexicon.h"
    "/s2geometry/src/s2/value_lexicon.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/base" TYPE FILE FILES
    "/s2geometry/src/s2/base/casts.h"
    "/s2geometry/src/s2/base/commandlineflags.h"
    "/s2geometry/src/s2/base/integral_types.h"
    "/s2geometry/src/s2/base/log_severity.h"
    "/s2geometry/src/s2/base/logging.h"
    "/s2geometry/src/s2/base/mutex.h"
    "/s2geometry/src/s2/base/port.h"
    "/s2geometry/src/s2/base/spinlock.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/strings" TYPE FILE FILES "/s2geometry/src/s2/strings/ostringstream.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/algorithm" TYPE FILE FILES "/s2geometry/src/s2/third_party/absl/algorithm/algorithm.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/base" TYPE FILE FILES
    "/s2geometry/src/s2/third_party/absl/base/attributes.h"
    "/s2geometry/src/s2/third_party/absl/base/casts.h"
    "/s2geometry/src/s2/third_party/absl/base/config.h"
    "/s2geometry/src/s2/third_party/absl/base/dynamic_annotations.h"
    "/s2geometry/src/s2/third_party/absl/base/log_severity.h"
    "/s2geometry/src/s2/third_party/absl/base/macros.h"
    "/s2geometry/src/s2/third_party/absl/base/optimization.h"
    "/s2geometry/src/s2/third_party/absl/base/policy_checks.h"
    "/s2geometry/src/s2/third_party/absl/base/port.h"
    "/s2geometry/src/s2/third_party/absl/base/thread_annotations.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/base/internal" TYPE FILE FILES
    "/s2geometry/src/s2/third_party/absl/base/internal/identity.h"
    "/s2geometry/src/s2/third_party/absl/base/internal/inline_variable.h"
    "/s2geometry/src/s2/third_party/absl/base/internal/invoke.h"
    "/s2geometry/src/s2/third_party/absl/base/internal/throw_delegate.h"
    "/s2geometry/src/s2/third_party/absl/base/internal/unaligned_access.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/container" TYPE FILE FILES "/s2geometry/src/s2/third_party/absl/container/inlined_vector.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/container/internal" TYPE FILE FILES
    "/s2geometry/src/s2/third_party/absl/container/internal/compressed_tuple.h"
    "/s2geometry/src/s2/third_party/absl/container/internal/container_memory.h"
    "/s2geometry/src/s2/third_party/absl/container/internal/layout.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/memory" TYPE FILE FILES "/s2geometry/src/s2/third_party/absl/memory/memory.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/meta" TYPE FILE FILES "/s2geometry/src/s2/third_party/absl/meta/type_traits.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/numeric" TYPE FILE FILES
    "/s2geometry/src/s2/third_party/absl/numeric/int128.h"
    "/s2geometry/src/s2/third_party/absl/numeric/int128_have_intrinsic.inc"
    "/s2geometry/src/s2/third_party/absl/numeric/int128_no_intrinsic.inc"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/strings" TYPE FILE FILES
    "/s2geometry/src/s2/third_party/absl/strings/numbers.h"
    "/s2geometry/src/s2/third_party/absl/strings/str_cat.h"
    "/s2geometry/src/s2/third_party/absl/strings/string_view.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/types" TYPE FILE FILES "/s2geometry/src/s2/third_party/absl/types/span.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/third_party/absl/utility" TYPE FILE FILES "/s2geometry/src/s2/third_party/absl/utility/utility.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/util/bits" TYPE FILE FILES "/s2geometry/src/s2/util/bits/bits.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/util/coding" TYPE FILE FILES
    "/s2geometry/src/s2/util/coding/coder.h"
    "/s2geometry/src/s2/util/coding/varint.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/util/endian" TYPE FILE FILES "/s2geometry/src/s2/util/endian/endian.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/util/gtl" TYPE FILE FILES
    "/s2geometry/src/s2/util/gtl/btree.h"
    "/s2geometry/src/s2/util/gtl/btree_container.h"
    "/s2geometry/src/s2/util/gtl/btree_map.h"
    "/s2geometry/src/s2/util/gtl/btree_set.h"
    "/s2geometry/src/s2/util/gtl/compact_array.h"
    "/s2geometry/src/s2/util/gtl/container_logging.h"
    "/s2geometry/src/s2/util/gtl/dense_hash_set.h"
    "/s2geometry/src/s2/util/gtl/densehashtable.h"
    "/s2geometry/src/s2/util/gtl/hashtable_common.h"
    "/s2geometry/src/s2/util/gtl/layout.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/util/hash" TYPE FILE FILES "/s2geometry/src/s2/util/hash/mix.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/util/math" TYPE FILE FILES
    "/s2geometry/src/s2/util/math/mathutil.h"
    "/s2geometry/src/s2/util/math/matrix3x3.h"
    "/s2geometry/src/s2/util/math/vector.h"
    "/s2geometry/src/s2/util/math/vector3_hash.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/s2/util/units" TYPE FILE FILES
    "/s2geometry/src/s2/util/units/length-units.h"
    "/s2geometry/src/s2/util/units/physical-units.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libs2.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libs2.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libs2.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/code/build/s2geometry/libs2.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libs2.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libs2.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libs2.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/code/build/s2geometry/libs2testing.a")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/code/build/s2geometry/examples/cmake_install.cmake")

endif()

