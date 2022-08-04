
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 14)

#option(BUILD_SHARED_LIBS "Build using shared libraries" ON)


if (WIN32)
    set(EXTERNAL_DEPENDENCIES wsock32 ws2_32 ${EXTERNAL_DEPENDENCIES})
else (WIN32)
    set(EXTERNAL_DEPENDENCIES pthread ${EXTERNAL_DEPENDENCIES})
endif (WIN32)

add_subdirectory(openGLPointCloudSystem)
add_subdirectory(algorithms)


add_library(pcl_helper SHARED pcl_helper.cpp)
add_library(scanner SHARED openGLPointCloud.hpp JsonParser.cpp Scanner.cpp)

find_package(PCL REQUIRED) #1.7 REQUIRED)

find_package(OpenCV REQUIRED)
set(EXTERNAL_DEPENDENCIES ${OpenCV_LIBS} ${EXTERNAL_DEPENDENCIES})

find_package(AndreiUtils REQUIRED)
set(EXTERNAL_DEPENDENCIES ${AndreiUtils_LIBRARY} ${EXTERNAL_DEPENDENCIES})

target_include_directories(pcl_helper
    PUBLIC
    $<INSTALL_INTERFACE:${realsense2_INCLUDE_DIR}>
    #PUBLIC $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
    #PUBLIC $<INSTALL_INTERFACE:${PCL_INCLUDE_DIRS}>
    #PUBLIC $<INSTALL_INTERFACE:openGLPointCloudSystem/>
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algortihms>
    $<INSTALL_INTERFACE:algorithms>
    #PUBLIC $<BUILD_INTERFACE:${realsense2_INCLUDE_DIR}>
    #PUBLIC $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
    #PUBLIC $<INSTALL_INTERFACE:${PCL_INCLUDE_DIRS}>
    #PUBLIC $<BUILD_INTERFACE:openGLPointCloudSystem>
    #PUBLIC $<BUILD_INTERFACE:algorithms>
)
target_include_directories(scanner
    PUBLIC $<INSTALL_INTERFACE:${realsense2_INCLUDE_DIR}>
    #PUBLIC $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
    #PUBLIC $<INSTALL_INTERFACE:${PCL_INCLUDE_DIRS}>
    #PUBLIC $<INSTALL_INTERFACE:../external_dependencies/imgui
    #PUBLIC $<INSTALL_INTERFACE:../external_dependencies/imgui/backends
    #PUBLIC $<INSTALL_INTERFACE:openGLPointCloudSystem/>
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algortihms>
    $<INSTALL_INTERFACE:algorithms>
    #PUBLIC $<BUILD_INTERFACE:${realsense2_INCLUDE_DIR}>
    #PUBLIC $<BUILD_INTERFACE:openGLPointCloudSystem>
    #PUBLIC $<BUILD_INTERFACE:algorithms>
)


set(EXTERNAL_DEPENDENCIES ${EXTERNAL_DEPENDENCIES} myLibrary algorithms) #Threads::Threads)
target_link_libraries(pcl_helper algorithms)
target_link_libraries(scanner ${EXTERNAL_DEPENDENCIES} algorithms pcl_helper)



include(GNUInstallDirs)

install(TARGETS algorithms

  ARCHIVE

  PUBLIC_HEADER

  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/algorithms

)
