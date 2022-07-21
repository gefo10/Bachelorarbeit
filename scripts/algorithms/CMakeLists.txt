cmake_minimum_required(VERSION 3.16)
project(CloudPointMeshGeneration)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

add_library(algorithms SHARED KdTree.cpp ICP.cpp)

target_include_directories(algorithms
    PUBLIC ../openGLPointCloudSystem
    PUBLIC ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(algorithms Eigen3::Eigen ${PCL_LIBRARIES})

