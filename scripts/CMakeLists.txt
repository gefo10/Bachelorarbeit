
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 14)

#option(BUILD_SHARED_LIBS "Build using shared libraries" ON)


if (WIN32)
    set(EXTERNAL_DEPENDENCIES wsock32 ws2_32 ${EXTERNAL_DEPENDENCIES})
else (WIN32)
    set(EXTERNAL_DEPENDENCIES pthread ${EXTERNAL_DEPENDENCIES})
endif (WIN32)
#

#include_directories(../external_dependencies/imgui ../external_dependencies/imgui/backends)
add_subdirectory(openGLPointCloudSystem)
add_subdirectory(algorithms)


add_library(pcl_helper SHARED pcl_helper.cpp)
add_library(scanner SHARED openGLPointCloud.hpp JsonParser.cpp Scanner.cpp)
#add_library(jsonParser SHARED JsonParser.cpp)

find_package(PCL REQUIRED) #1.7 REQUIRED)

#set(EXTERNAL_DEPENDENCIES ${GFLW_LIBRARIES} ${EXTERNAL_DEPENDENCIES})

find_package(AndreiUtils REQUIRED)
set(EXTERNAL_DEPENDENCIES ${AndreiUtils_LIBRARY} ${EXTERNAL_DEPENDENCIES})


target_include_directories(pcl_helper
    PUBLIC ${realsense2_INCLUDE_DIR}
    PUBLIC ${EIGEN3_INCLUDE_DIRS}
    PUBLIC ${PCL_INCLUDE_DIRS}
    #   PUBLIC ${GLFW_INCLUDE_DIRS}
    #PUBLIC ${OPENGL_INCLUDE_DIRS}
    #PUBLIC ${GLUT_INCLUDE_DIRS}
    #PUBLIC ${GLM_INCLUDE_DIRS}
    #PUBLIC external/imgui
    #PUBLIC external/imgui/backends
    PUBLIC openGLPointCloudSystem
    PUBLIC algorithms
)
target_include_directories(scanner
    PUBLIC ${realsense2_INCLUDE_DIR}
    PUBLIC ${EIGEN3_INCLUDE_DIRS}
    PUBLIC ${PCL_INCLUDE_DIRS}
    #PUBLIC ${GLFW_INCLUDE_DIRS}
    # PUBLIC ${OPENGL_INCLUDE_DIRS}
    #PUBLIC ${GLM_INCLUDE_DIRS}
    #PUBLIC ${GLUT_INCLUDE_DIRS}
    PUBLIC ../external_dependencies/imgui
    PUBLIC ../external_dependencies/imgui/backends
    PUBLIC openGLPointCloudSystem
    PUBLIC algorithms
)

#set_target_properties(scanner PROPERTIES
#                        POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
#                        )
                    #set_target_properties(pcl_helper PROPERTIES
                    #   POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
                    #   )
#set(EXTERNAL_DEPENDENCIES ${EXTERNAL_DEPENDENCIES} glfw ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} myLibrary) #Threads::Threads)
set(EXTERNAL_DEPENDENCIES ${EXTERNAL_DEPENDENCIES} myLibrary algorithms) #Threads::Threads)
#target_link_libraries(jsonParser ${AndreiUtils_LIBRARY})
target_link_libraries(pcl_helper algorithms)
target_link_libraries(scanner ${EXTERNAL_DEPENDENCIES} pcl_helper)


