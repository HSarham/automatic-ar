project(automatic-ar)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS ON)

#set(CMAKE_PREFIX_PATH "~/local/") #uncomment if you have local libraries compiled in ~/local/

find_package(OpenCV 3.0.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenMP)

if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(PCL)

if(PCL_FOUND)
    INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
    set(pcl_libs "${PCL_VISUALIZATION_LIBRARIES}")
    add_definitions(-DPCL)
endif(PCL_FOUND)

include_directories(libs)
include_directories(3rdparty/eigen3)
include_directories(3rdparty/aruco)
include_directories(3rdparty/sgl/src)

add_subdirectory(3rdparty/sgl)
add_subdirectory(3rdparty/aruco/aruco)
add_subdirectory(libs)
add_subdirectory(apps)












