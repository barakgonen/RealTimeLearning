add_executable(RealTimeLearningApp main.cpp)

set(CMAKE_LIBRARY_PATH 
${CMAKE_SOURCE_DIR}/../ORB_SLAM2/Thirdparty/g2o/lib/ 
${CMAKE_SOURCE_DIR}/../ORB_SLAM2/Thirdparty/DBoW2/lib/
${CMAKE_SOURCE_DIR}/../ORB_SLAM2/lib/)

find_package(Threads REQUIRED)
find_library(NAME g2o)
find_library(NAME DBoW2)
find_library(NAME ORB_SLAM2)

       target_include_directories(RealTimeLearningApp PUBLIC
       ${Boost_INCLUDE_DIRS}
       ${OpenCV_INCLUDE_DIRS}
       ${PYTHON_INCLUDE_DIRS}
       ${LBD_MOD_INCLUDE}
       ${EIGEN3_INCLUDE_DIR}
       ${Pangolin_INCLUDE_DIRS}
       /local/ORB_SLAM2/include/
       )
       
       target_link_libraries(RealTimeLearningApp PUBLIC core::core 
#      tello::tello
	   /local/ORB_SLAM2/lib/libORB_SLAM2.so
       /local/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
       /local/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
       ${Boost_LIBRARIES}
       ${OpenCV_LIBRARIES}
       ${PYTHON_LIBRARIES}
       ${EIGEN3_LIBS}
       ${Pangolin_LIBRARIES}
       )