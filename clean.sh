#!/bin/sh

rm -rf build/ CMakeFiles/ cmake_install.cmake CMakeCache.txt Makefile
cd orb_slam
rm -rf build/ CMakeFiles/ cmake_install.cmake CMakeCache.txt Makefile
cd -
cd core
rm -rf build/ CMakeFiles/ cmake_install.cmake CMakeCache.txt Makefile
cd -
#cd tello
#rm -rf build/ CMakeFiles/ cmake_install.cmake CMakeCache.txt Makefile
#cd -
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug .
