#!/bin/sh

rm -rf build/ CMakeFiles/ cmake_install.cmake CMakeCache.txt Makefile
cd orb_slam
rm -rf build/ CMakeFiles/ cmake_install.cmake CMakeCache.txt Makefile
cd -
cd core
rm -rf build/ CMakeFiles/ cmake_install.cmake CMakeCache.txt Makefile
cd -
cd drone_lib
rm -rf build/ CMakeFiles/ cmake_install.cmake CMakeCache.txt Makefile
cd -
#cd ../ORB_SLAM2
#rm -rf build rm -rf lib/ Thirdparty/DBoW2/lib/ Thirdparty/DBoW2/build/ Thirdparty/g2o/lib/ Thirdparty/g2o/build/
#./build.sh
#cd -
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug .

