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
#tello::tello
/local/RealTimeLearning/lib/liborb_slam.so
/local/RealTimeLearning/lib/libDBoW2.so
/local/RealTimeLearning/lib/libg2o.so
${Boost_LIBRARIES}
${OpenCV_LIBRARIES}
${PYTHON_LIBRARIES}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
)