cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 14)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

project (RealTimeLearning)

set(library_name core)

add_library(${library_name} SHARED "")

add_library(core::core ALIAS ${library_name})

target_sources(core PRIVATE
        			CoordinatesCalculator.h
        			CoordinatesCalculator.cpp
        			Point3D.h
        			Point3D.cpp
        			Utils.h
)
target_include_directories(core
    PUBLIC
        $<INSTALL_INTERFACE:public>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)
