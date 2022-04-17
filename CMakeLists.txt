cmake_minimum_required(VERSION 3.10)
project(pcl_test)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(OpenCV_DIR /usr/local/openCV-3.4.12/share/OpenCV)
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenMP)

include_directories(include)

SET(EXECUTABLE_OUTPUT_PATH ../bin)

add_executable(pcl_select src/pcl_select.cpp)
target_link_libraries(pcl_select ${PCL_LIBRARIES})

add_executable(pcl_delete src/pcl_delete.cpp)
target_link_libraries(pcl_delete ${PCL_LIBRARIES})

add_executable(pcl_filter src/pcl_filter.cpp)
target_link_libraries(pcl_filter ${PCL_LIBRARIES})

add_executable(pcl_cluster src/pcl_cluster.cpp)
target_link_libraries(pcl_cluster ${PCL_LIBRARIES})
set_target_properties(pcl_cluster PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS})

add_executable(pcl_intensity src/pcl_intensity.cpp)
target_link_libraries(pcl_intensity ${PCL_LIBRARIES})

add_executable(pcl_iter_icp src/pcl_iter_icp.cpp)
target_link_libraries(pcl_iter_icp ${PCL_LIBRARIES})

add_executable(pcl_generate src/pcl_generate.cpp)
target_link_libraries(pcl_generate ${PCL_LIBRARIES})

add_executable(pcl_transform src/pcl_transform.cpp)
target_link_libraries(pcl_transform ${PCL_LIBRARIES})

add_executable(pcl_center src/pcl_center.cpp)
target_link_libraries(pcl_center ${PCL_LIBRARIES})

add_executable(pcl_normal_estimate src/pcl_normal_estimate.cpp)
target_link_libraries(pcl_normal_estimate ${PCL_LIBRARIES})
set_target_properties(pcl_normal_estimate PROPERTIES
  COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
  LINK_FLAGS ${OpenMP_CXX_FLAGS})

add_executable(pcl_normal_diff src/pcl_normal_diff.cpp)
target_link_libraries(pcl_normal_diff ${PCL_LIBRARIES})
set_target_properties(pcl_normal_diff PROPERTIES
  COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
  LINK_FLAGS ${OpenMP_CXX_FLAGS})

add_executable(pcl_origanized src/pcl_origanized.cpp)
target_link_libraries(pcl_origanized ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pcl_pfh src/pcl_pfh.cpp)
target_link_libraries(pcl_pfh ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pcl_fpfh src/pcl_fpfh.cpp)
target_link_libraries(pcl_fpfh ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pcl_correspondences src/pcl_correspondences.cpp)
target_link_libraries(pcl_correspondences ${PCL_LIBRARIES})

add_executable(pcl_bash src/pcl_bash.cpp)
target_link_libraries(pcl_bash ${PCL_LIBRARIES})
set_target_properties(pcl_bash PROPERTIES
  COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
  LINK_FLAGS ${OpenMP_CXX_FLAGS})

add_executable(pcl_concat src/pcl_concat.cpp)
target_link_libraries(pcl_concat ${PCL_LIBRARIES})
set_target_properties(pcl_concat PROPERTIES
  COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
  LINK_FLAGS ${OpenMP_CXX_FLAGS})

add_executable(pcl_bilateral src/pcl_bilateral.cpp)
target_link_libraries(pcl_bilateral ${PCL_LIBRARIES})

add_executable(pcl_pca src/pcl_pca.cpp)
target_link_libraries(pcl_pca ${PCL_LIBRARIES})
set_target_properties(pcl_pca PROPERTIES
  COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
  LINK_FLAGS ${OpenMP_CXX_FLAGS})