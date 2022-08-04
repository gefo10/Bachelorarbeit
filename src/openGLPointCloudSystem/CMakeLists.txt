cmake_minimum_required(VERSION 3.16)
project(CloudPointMeshGeneration)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(PCL REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS})

add_library(myLibrary SHARED Point.h Window.cpp Renderer.cpp IndexBuffer.cpp VertexBuffer.cpp VertexArray.cpp VertexBufferLayout.h Shader.cpp Camera.cpp Mesh.cpp)

#set_target_properties(myLibrary PROPERTIES
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/Point.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/Window.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/Renderer.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/IndexBuffer.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/VertexBuffer.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/VertexArray.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/VertexBufferLayout.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/Shader.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/Camera.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/Mesh.h
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/PointShader.shader
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/M_basic.shader
#    PUBLIC_HEADER scripts/openGLPointCloudSystem/Shader.shader
#
#
#)


add_subdirectory(external)
#include_directories(external_dependencies/imgui/ external_dependencies/imgui/backends/)

target_include_directories(myLibrary
    #PUBLIC external
    PUBLIC $<INSTALL_INTERFACE:../../external_dependencies>
    PUBLIC $<INSTALL_INTERFACE:${GLEW_INCLUDE_DIRS}>
    #PUBLIC $<INSTALL_INTERFACE:${PCL_INCLUDE_DIRS}>
    PUBLIC ${PCL_INCLUDE_DIRS}
)

set(EXTERNAL_DEPENDENCIES glfw ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${GLEW_LIBRRARIES} GLEW::GLEW Eigen3::Eigen glm::glm) 
target_link_libraries(myLibrary ${EXTERNAL_DEPENDENCIES})


#install(TARGETS myLibrary EXPORT openGLPointCloudSystemTargets ARCHIEVE
#    PUBLIC_HEADER_DESTINATION
#    ${CMAKE_INSTALL_INCLUDEDIR}/openGLPointCloudSystem
#    )
#install(EXPORT openGLPointCloudSystemTargets
#    DESTINATION ${CMAKE_INSTALL_LIBDIR/openGLPointCloudSystem/cmake
    
