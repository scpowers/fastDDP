cmake_minimum_required(VERSION 3.19)
project(fastDDP)

set(CMAKE_CXX_STANDARD 17)

set(EIGEN_DIR "/usr/local/include/")
set(SCIPLOT_DIR "/usr/local/include/")
if(NOT EXISTS ${EIGEN_DIR})
    message(FATAL_ERROR "Please check that the set Eigen directory is valid!")
endif()
include_directories(${EIGEN_DIR})
include_directories(${SCIPLOT_DIR})
include_directories(includes)
include_directories(src)

add_executable(fastDDP
        main.cpp src/System.cpp includes/System.h src/util.cpp includes/util.h src/DDP_Engine.cpp includes/DDP_Engine.h)
