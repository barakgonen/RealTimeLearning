cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

project (RealTimeLearning)

set(library_name orbslam)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost  1.54.0  	     # Minimum or EXACT version e.g. 1.36.0
  REQUIRED             					# Fail with error if Boost is not found
  COMPONENTS serialization 				# Boost libraries by their canonical name
  )          

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/orb_slam/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${library_name} SHARED
${PROJECT_SOURCE_DIR}/src/System.cc
${PROJECT_SOURCE_DIR}/src/Tracking.cc
${PROJECT_SOURCE_DIR}/src/LocalMapping.cc
${PROJECT_SOURCE_DIR}/src/LoopClosing.cc
${PROJECT_SOURCE_DIR}/src/ORBextractor.cc
${PROJECT_SOURCE_DIR}/src/ORBmatcher.cc
${PROJECT_SOURCE_DIR}/src/FrameDrawer.cc
${PROJECT_SOURCE_DIR}/src/Converter.cc
${PROJECT_SOURCE_DIR}/src/MapPoint.cc
${PROJECT_SOURCE_DIR}/src/KeyFrame.cc
${PROJECT_SOURCE_DIR}/src/Map.cc
${PROJECT_SOURCE_DIR}/src/MapDrawer.cc
${PROJECT_SOURCE_DIR}/src/Optimizer.cc
${PROJECT_SOURCE_DIR}/src/PnPsolver.cc
${PROJECT_SOURCE_DIR}/src/Frame.cc
${PROJECT_SOURCE_DIR}/src/KeyFrameDatabase.cc
${PROJECT_SOURCE_DIR}/src/Sim3Solver.cc
${PROJECT_SOURCE_DIR}/src/Initializer.cc
${PROJECT_SOURCE_DIR}/src/Viewer.cc
)

target_link_libraries(${library_name}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/orb_slam/Thirdparty/DBoW2/lib/libDBoW2.so
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/orb_slam/Thirdparty/orb_slam/g2o/lib/libg2o.so
${Boost_LIBRARIES}
)

add_library(orb_slam::orb_slam ALIAS ${library_name})

# Build tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary
${PROJECT_SOURCE_DIR}/tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${library_name})
