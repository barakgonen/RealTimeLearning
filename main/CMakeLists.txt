add_executable(RealTimeLearningApp main.cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 14)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost  1.54.0 REQUIRED COMPONENTS serialization)   
find_package(PythonLibs REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(/usr/include/python3.8)

target_include_directories(RealTimeLearningApp PUBLIC
${Boost_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
${LBD_MOD_INCLUDE}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
/local/ORB_SLAM2/include/)

target_link_libraries(RealTimeLearningApp PUBLIC 
core::core 
droneLib::droneLib
#orb_slam::orb_slam
/local/ORB_SLAM2/lib/libORB_SLAM2.so
/local/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
/local/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
${Boost_LIBRARIES}
${OpenCV_LIBRARIES}
${PYTHON_LIBRARIES}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PYTHON_LIBRARIES}
)