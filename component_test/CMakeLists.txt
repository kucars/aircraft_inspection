cmake_minimum_required(VERSION 2.8.3)
project( component_test )


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  )

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../fcl/test/fcl_resources/config.h.in" "${CMAKE_CURRENT_SOURCE_DIR}/../fcl/test/fcl_resources/config.h")
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../fcl/test/
  ${CMAKE_CURRENT_SOURCE_DIR}/../fcl/test/fcl_resources
)
  
add_library(fcl_test ${CMAKE_CURRENT_SOURCE_DIR}/../fcl/test/test_fcl_utility.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../fcl/src/traversal/traversal_node_bvhs.cpp)
add_executable(frustum_test src/frustum_test.cpp)
add_dependencies(frustum_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(frustum_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(discretization src/discretization.cpp)
add_dependencies(discretization ${catkin_EXPORTED_TARGETS})
target_link_libraries(discretization ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(filtering src/filtering.cpp)
add_dependencies(filtering ${catkin_EXPORTED_TARGETS})
target_link_libraries(filtering fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(collision_distance_test src/collision_distance_test.cpp)
add_dependencies(collision_distance_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_distance_test fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(samples_orientation src/samples_orientation.cpp)
add_dependencies(samples_orientation ${catkin_EXPORTED_TARGETS})
target_link_libraries(samples_orientation fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(orientation_testing src/orientation_testing.cpp)
add_dependencies(orientation_testing ${catkin_EXPORTED_TARGETS})
target_link_libraries(orientation_testing fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
