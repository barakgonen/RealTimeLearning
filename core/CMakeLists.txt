cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

project (RealTimeLearning)

set(library_name core)

add_library(${library_name} SHARED "")

add_library(core::core ALIAS ${library_name})

target_sources(core PRIVATE
        			Point.h
        			Point.cpp
)

target_include_directories(core
    PUBLIC
        $<INSTALL_INTERFACE:public>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)
