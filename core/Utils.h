/*
 * Utils.h
 *
 *  Created on: Sep 21, 2021
 *      Author: barakg
 */

#ifndef CORE_UTILS_H_
#define CORE_UTILS_H_
#include <eigen3/Eigen/Core>
#include <fstream>
#include <vector>
#include <iosfwd>

#include "../orb_slam/include/Converter.h"
#include "../orb_slam/include/System.h"
#include <chrono>
#include <utility>

typedef std::chrono::high_resolution_clock::time_point TimeVar;

/**
 * Utility functions
 */

/**
 * Utility function to read csv file written in our format
 * format is:
 * x,y,z
 * original file was the map of the room, for integration we had to store map of points
 */
std::vector<Eigen::Matrix<double, 3, 1>> read_csv(std::string filename){
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<Eigen::Matrix<double, 3, 1>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open())
    	throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;

    // Read data, line by line
    while(std::getline(myFile, line))
    {
    	std::vector<std::string> splittedLine;
    	// Create a stringstream of the current line
		std::stringstream ss(line);
    	while( ss.good() )
    	{
    	    std::string substr;
    	    getline( ss, substr, ',' );
    	    splittedLine.push_back( substr );
    	}

    	double x = std::stod(splittedLine[0]);
    	double y = std::stod(splittedLine[1]);
    	double z = std::stod(splittedLine[2]);

    	result.push_back(Eigen::Matrix<double, 3, 1>(Eigen::Matrix<double, 3, 1>{x, y, z}));
    }


    // Close file
    myFile.close();
    return result;
}

std::vector<Eigen::Matrix<double, 3, 1>> saveMapToFile(ORB_SLAM2::System &SLAM) {
    std::vector<ORB_SLAM2::MapPoint *> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/RoomCoordiantes.csv");
    std::vector<Eigen::Matrix<double, 3, 1>> pointsVector;
    for (auto p: mapPoints) {
        if (p != NULL) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z() << std::endl;
            pointsVector.push_back(v);
        }
    }
    pointData.close();
    return pointsVector;
}

#define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
#define timeNow() std::chrono::high_resolution_clock::now()

template<typename F, typename... Args>
double funcTimeMillis(F func, Args&&... args){
	const auto start = std::chrono::system_clock::now();
    func(std::forward<Args>(args)...);
    const auto end = std::chrono::system_clock::now();
    return duration(end - start);
}

#endif /* CORE_UTILS_H_ */
