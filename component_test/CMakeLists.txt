cmake_minimum_required(VERSION 2.8.3)
project( component_test )
SET ( CMAKE_CXX_FLAGS "-frounding-math" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  shape_msgs
  std_msgs
  rviz_visual_tools
  )

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)
# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )
find_package(VTK REQUIRED)

#Cuda related Flags
find_package(CUDA QUIET)
#list(APPEND CMAKE_CXX_FLAGS "-std=c++11 -O3")
#list(APPEND CUDA_NVCC_FLAGS "-std=c++11;-O2;-DVERBOSE")
#set(CUDA_PROPAGATE_HOST_FLAGS OFF)

if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  
endif()

include_directories(${PCL_INCLUDE_DIRS} /usr/local/cuda/include/)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES occlusion_culling
   CATKIN_DEPENDS roscpp pcl_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
#  /usr/local/cuda/include/
#  /home/randa/workspace/catkin_ws/src/libigl/include/
)
  
add_library(fcl_test src/fcl_utility.cpp )
add_library(voxel_grid src/voxel_grid.cpp )
target_link_libraries(voxel_grid ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES})

add_library(voxel_grid_occlusion_estimation src/voxel_grid_occlusion_estimation.cpp )

add_library(frustum_culling src/frustum_culling.cpp )
target_link_libraries(frustum_culling voxel_grid ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES})

add_library(occlusion_culling src/occlusion_culling.cpp)
target_link_libraries(occlusion_culling frustum_culling voxel_grid_occlusion_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(visualization_voxelgrid src/visualization_voxelgrid.cpp)
target_link_libraries(visualization_voxelgrid voxel_grid ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES})

add_executable(surface_coverage_evaluation src/surface_coverage_evaluation.cpp)
target_include_directories(surface_coverage_evaluation PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(surface_coverage_evaluation ${catkin_EXPORTED_TARGETS})
target_link_libraries(surface_coverage_evaluation occlusion_culling occlusion_culling_gpu ${rviz_visual_tools_LIBRARIES} ${VTK_LIBRARIES} ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(accuracy_check src/accuracy_check.cpp)
add_dependencies(accuracy_check ${catkin_EXPORTED_TARGETS})
target_link_libraries(accuracy_check occlusion_culling ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(coverage_comparison src/coverage_comparison.cpp)
add_dependencies(coverage_comparison ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_comparison occlusion_culling ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(filtering src/filtering.cpp)
add_dependencies(filtering ${catkin_EXPORTED_TARGETS})
target_link_libraries(filtering occlusion_culling ${catkin_LIBRARIES} ${CGAL_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(frustum_test src/frustum_test.cpp)
add_dependencies(frustum_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(frustum_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(frustum_test_debug src/frustum_test_debug.cpp)
add_dependencies(frustum_test_debug ${catkin_EXPORTED_TARGETS})
target_link_libraries(frustum_test_debug frustum_culling voxel_grid_occlusion_estimation ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(occlusion_culling_test src/occlusion_culling_test.cpp)
add_dependencies(occlusion_culling_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(occlusion_culling_test voxel_grid_occlusion_estimation ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(components_combination src/components_combination.cpp)
add_dependencies(components_combination ${catkin_EXPORTED_TARGETS})
target_link_libraries(components_combination frustum_culling voxel_grid_occlusion_estimation ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(trial_testing src/trial_testing.cpp src/occlusion_culling.cpp src/frustum_culling.cpp src/voxel_grid_occlusion_estimation.cpp)
add_dependencies(trial_testing ${catkin_EXPORTED_TARGETS})
target_link_libraries(trial_testing visualization_voxelgrid voxel_grid ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(coverage_quantification src/coverage_quantification.cpp)
add_dependencies(coverage_quantification ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_quantification occlusion_culling ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})


add_executable(occlusion_culling_debug src/occlusion_culling_debug.cpp)
add_dependencies(occlusion_culling_debug ${catkin_EXPORTED_TARGETS})
target_link_libraries(occlusion_culling_debug voxel_grid_occlusion_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(discretization src/discretization.cpp)
add_dependencies(discretization ${catkin_EXPORTED_TARGETS})
target_link_libraries(discretization ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(filtering_old src/filtering_old.cpp)
add_dependencies(filtering_old ${catkin_EXPORTED_TARGETS})
target_link_libraries(filtering_old fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(collision_distance_test src/collision_distance_test.cpp)
add_dependencies(collision_distance_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_distance_test fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(samples_orientation src/samples_orientation.cpp)
add_dependencies(samples_orientation ${catkin_EXPORTED_TARGETS})
target_link_libraries(samples_orientation fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(orientation_testing src/orientation_testing.cpp)
add_dependencies(orientation_testing ${catkin_EXPORTED_TARGETS})
target_link_libraries(orientation_testing fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(reconstruction src/reconstruction.cpp)
add_dependencies(reconstruction ${catkin_EXPORTED_TARGETS})
target_link_libraries(reconstruction fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(z_buffer_test src/z_buffer_test.cpp)
add_dependencies(z_buffer_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(z_buffer_test fcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(cgal_test src/cgal_test.cpp)
add_dependencies(cgal_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(cgal_test ${catkin_LIBRARIES} ${CGAL_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(cgal_interior_skeleton src/cgal_interior_skeleton.cpp)
add_dependencies(cgal_interior_skeleton ${catkin_EXPORTED_TARGETS})
target_link_libraries(cgal_interior_skeleton -lgmp -lmpfr ${catkin_LIBRARIES} ${CGAL_Core_LIBRARY} ${CGAL_LIBRARY} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(triangulation_mesh src/triangulation_mesh.cpp)
add_dependencies(triangulation_mesh ${catkin_EXPORTED_TARGETS})
target_link_libraries(triangulation_mesh -lgmp -lmpfr ${catkin_LIBRARIES} ${CGAL_Core_LIBRARY} ${CGAL_LIBRARY} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

if (CUDA_FOUND)
    cuda_add_library(frustum_culling_gpu src/frustum_culling_gpu.cu )
    cuda_add_library(voxel_grid_occlusion_estimation_gpu src/voxel_grid_occlusion_estimation_gpu.cu )
    add_library(occlusion_culling_gpu src/occlusion_culling_gpu.cpp)
    target_link_libraries(occlusion_culling_gpu frustum_culling_gpu voxel_grid_occlusion_estimation_gpu ${catkin_LIBRARIES} ${PCL_LIBRARIES})
    cuda_add_executable(test_cuda src/test_cuda.cu)
    add_executable(coverage_quantification_gpu src/coverage_quantification_gpu.cpp)
    add_dependencies(coverage_quantification_gpu ${catkin_EXPORTED_TARGETS})
    target_link_libraries(coverage_quantification_gpu occlusion_culling_gpu ${catkin_LIBRARIES} -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
endif (CUDA_FOUND)
