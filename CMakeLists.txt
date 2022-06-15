
cmake_minimum_required(VERSION 3.16)
project(CloudPointMeshGeneration)

set(CMAKE_CXX_STANDARD 14)

set(OPENCV_VERSION "" CACHE STRING "The opencv version to use in the project")

if (WIN32)
    set(EXTERNAL_DEPENDENCIES wsock32 ws2_32 ${EXTERNAL_DEPENDENCIES})
else (WIN32)
    set(EXTERNAL_DEPENDENCIES pthread ${EXTERNAL_DEPENDENCIES})
endif (WIN32)



set(GCC_COMPILE_FLAGS "-g -ggdb3")

add_definitions(${GCC_COMPILE_FLAGS})



find_package(Eigen3 3.3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

#include_directories(${GLFW_INCLUDE_DIRS})
set(EXTERNAL_DEPENDENCIES ${GFLW_LIBRARIES} ${EXTERNAL_DEPENDENCIES})

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(EXTERNAL_DEPENDENCIES ${PCL_LIBRARIES} ${EXTERNAL_DEPENDENCIES})



#include_directories(${EIGEN3_INCLUDE_DIRS})
set(EXTERNAL_DEPENDENCIES Eigen3::Eigen ${EXTERNAL_DEPENDENCIES})

find_package(realsense2 REQUIRED)
#include_directories(${realsense2_INCLUDE_DIR})
set(EXTERNAL_DEPENDENCIES realsense2::realsense2 ${EXTERNAL_DEPENDENCIES})

find_package(OpenCV REQUIRED)
set(EXTERNAL_DEPENDENCIES ${OpenCV_LIBS} ${EXTERNAL_DEPENDENCIES})

find_package(AndreiUtils REQUIRED)
#include_directories(${CGAL_INCLUDE_DIRS})
set(EXTERNAL_DEPENDENCIES ${AndreiUtils_LIBRARY} ${EXTERNAL_DEPENDENCIES})

#find_package(CGAL)

#include_directories(${CGAL_INCLUDE_DIRS})
#set(EXTERNAL_DEPENDENCIES ${CGAL_LIBS} ${EXTERNAL_DEPENDENCIES})
#find_package(glm REQUIRED)
#include_directories(${GLM_INCLUDE_DIRS})
#link_directories(${GLM_LIBRARY_DIRS})
#add_definitions(${GLM_DEFINITIONS})


include_directories("include")

#set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
#set(THREADS_PREFER_PTHREAD_FLAG TRUE)
#find_package(Threads REQUIRED)


#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

add_executable(main testing.cpp)
#add_subdirectory(external_dependencies/glfw)

add_subdirectory(scripts)

#include_directories(external_dependencies/imgui/ external_dependencies/imgui/backends/)

target_include_directories(main
    #    PUBLIC external_dependencies/glfw/include/
    #PUBLIC external_dependencies/CGAL/CGAL_Core/include/
    #PUBLIC external_dependencies//CGAL/CGAL_ImageIO/include/
    PUBLIC scripts/
    PUBLIC external_dependencies/imgui/
    PUBLIC external_dependencies/imgui/backends/
    PUBLIC ${realsense2_INCLUDE_DIR}
    PUBLIC ${EIGEN3_INCLUDE_DIRS}
    PUBLIC ${PCL_INCLUDE_DIRS}
#    PUBLIC ${GLFW_INCLUDE_DIRS}
)
target_link_directories(main
    PRIVATE external_dependencies/glfw/src
    # PRIVATE external_dependencies/CGAL
    PRIVATE scripts
    PRIVATE external_dependencies/imgui
    PRIVATE ${PCL_LIBRARY_DIRS}
)


set(EXTERNAL_DEPENDENCIES ${EXTERNAL_DEPENDENCIES}  pcl_helper scanner)
#set(EXTERNAL_DEPENDENCIES ${EXTERNAL_DEPENDENCIES} glfw ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} pcl_helper scanner Threads::Threads)

target_link_libraries(main ${EXTERNAL_DEPENDENCIES})

add_executable(TestJson testJson.cpp)
target_link_libraries(TestJson ${AndreiUtils_LIBRARY})

