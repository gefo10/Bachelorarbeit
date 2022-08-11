cmake_minimum_required(VERSION 3.16)

project(ScannerLib)  # sets PROJECT_NAME variable
set(PROJECT_VERSION_STRING "0.0.1")
set(PROJECT_TARGET scanner)
set(PROJECT_INCLUDE_PREFIX ${PROJECT_NAME})


include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 14)
if (WIN32)
    set(EXTERNAL_DEPENDENCIES wsock32 ws2_32 ${EXTERNAL_DEPENDENCIES})
    set(EXTERNAL_DEPENDENCIES_GENERAL wsock32 ws2dd_32 ${EXTERNAL_DEPENDENCIES_GENERAL})
else (WIN32)
    set(EXTERNAL_DEPENDENCIES pthread ${EXTERNAL_DEPENDENCIES})
endif (WIN32)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(AndreiUtils REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIR})
include_directories( ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_subdirectory(external_dependencies)
set(EXTERNAL_DEPENDENCIES glfw glm ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRRARIES} GLEW::GLEW Eigen3::Eigen)
set(EXTERNAL_DEPENDENCIES ${realsense2_LIBRARY} ${EXTERNAL_DEPENDENCIES})
#set(EXTERNAL_DEPENDENCIES ${OPENGL_LIBRARIES} ${EXTERNAL_DEPENDENCIES})
set(EXTERNAL_DEPENDENCIES ${PCL_LIBRARIES} ${EXTERNAL_DEPENDENCIES})
set(EXTERNAL_DEPENDENCIES realsense2::realsense2 ${EXTERNAL_DEPENDENCIES})
set(EXTERNAL_DEPENDENCIES Eigen3::Eigen ${EXTERNAL_DEPENDENCIES})
set(EXTERNAL_DEPENDENCIES ${AndreiUtils_LIBRARY} ${EXTERNAL_DEPENDENCIES})

include_directories("include")




set(VISUALIZER_SRC_FILES src/Window.cpp src/Renderer.cpp src/IndexBuffer.cpp src/VertexBuffer.cpp src/VertexArray.cpp src/VertexBufferLayout.cpp src/Shader.cpp src/Camera.cpp src/Mesh.cpp src/openGLPointCloud.cpp)
set(ALGORITHMS_SRC_FILES src/KdTree.cpp src/ICP.cpp)
set(JSONPARSER_SRC_FILES src/JsonParser.cpp)
set(PCL_HELPERS_SRC_FILES src/pcl_helper.cpp)
set(SCANNER_SRC_FILES src/Scanner.cpp)

set(SRC_FILES ${VISUALIZER_SRC_FILES} ${ALGORITHMS_SRC_FILES}  ${JSONPARSER_SRC_FILES} ${PCL_HELPERS_SRC_FILES} ${SCANNER_SRC_FILES})
# GENERAL_UTILS

add_library(scanner ${SRC_FILES})
target_link_libraries(scanner ${EXTERNAL_DEPENDENCIES})
target_include_directories(scanner
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

message(${EXTERNAL_DEPENDENCIES})

#set(EXTERNAL_DEPENDENCIES scanner ${EXTERNAL_DEPENDENCIES})
message("EXTRN LIBS: ${EXTERNAL_DEPENDENCIES}")
add_executable(main testing.cpp)
target_link_libraries(main scanner)


install(DIRECTORY external_dependencies/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/)
#
#install(TARGETS scanner EXPORT ScannerLibTargets ARCHIVE
#
#  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ScannerLib
#
#)
#
#install(EXPORT ScannerLibTargets
#
#    DESTINATION ${CMAKE_INSTALL_LIBDIR}/ScannerLib/cmake
#
#    NAMESPACE ScannerLib::
#
#)


include(cmake/installConfig.cmake)

