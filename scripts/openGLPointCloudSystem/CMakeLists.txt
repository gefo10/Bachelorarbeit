cmake_minimum_required(VERSION 3.16)
project(CloudPointMeshGeneration)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(PCL REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS})

add_library(myLibrary SHARED Point.h Window.cpp Renderer.cpp IndexBuffer.cpp VertexBuffer.cpp VertexArray.cpp VertexBufferLayout.h Shader.cpp Camera.cpp Mesh.cpp)

add_subdirectory(external)
#include_directories(external_dependencies/imgui/ external_dependencies/imgui/backends/)

target_include_directories(myLibrary
    #PUBLIC external
    PUBLIC ../../external_dependencies
    PUBLIC ${GLEW_INCLUDE_DIRS}
    PUBLIC ${PCL_INCLUDE_DIRS}
)

#target_link_directories(myLibrary
#    PRIVATE external/glm
#)

set(EXTERNAL_DEPENDENCIES glfw ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${GLEW_LIBRRARIES} GLEW::GLEW Eigen3::Eigen glm::glm) 
target_link_libraries(myLibrary ${EXTERNAL_DEPENDENCIES})

