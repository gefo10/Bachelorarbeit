cmake_minimum_required(VERSION 3.16)
project(CloudPointMeshGeneration)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_library(algorithms SHARED KdTree.cpp ICP.cpp)
set_target_properties(algorithms PROPERTIES

    PUBLIC_HEADER scripts/algorithms/ICP.h
    PUBLIC_HEADER scripts/algorithms/KdTree.h
    PUBLIC_HEADER scripts/openGLPointCloudSystem/Point.h

)

target_include_directories(algorithms
    #PUBLIC $<INSTALL_INTERFACE:/../openGLPointCloudSystem/>
    #PUBLIC 
    PUBLIC $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
    PUBLIC $<INSTALL_INTERFACE:${PCL_INCLUDE_DIRS}>
)
#set_target_properties(algorithms PROPERTIES
#PUBLIC_HEADER KdTree.h
#   PUBLIC_HEADER ICP.h)
target_link_libraries(algorithms Eigen3::Eigen ${PCL_LIBRARIES})

