cmake_minimum_required(VERSION 3.19)
project(fastDDP)

set(CMAKE_CXX_STANDARD 14)

set(EIGEN_DIR "/usr/local/include")
if(NOT EXISTS ${EIGEN_DIR})
    message(FATAL_ERROR "Please check that the set Eigen directory is valid!")
endif()
include_directories(${EIGEN_DIR})

add_executable(fastDDP
        main.cpp System.cpp System.h util.cpp util.h)
