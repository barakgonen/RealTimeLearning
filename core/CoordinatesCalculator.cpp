/*
 * CoordinatesCalculator.cpp
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#include "CoordinatesCalculator.h"

#include <cmath>
#include <map>
#include <iostream>
#include <vector>
using namespace std;

Point CoordinatesCalculator::DetectExitCoordinates(int n, const vector<Eigen::Matrix<double,3,1>>& data) {

	// We use map for kind of insertion sort, since it keeps the keys ordered. instead of sorting at the end
	std::map<double, const Eigen::Matrix<double,3,1>> distanceToPoint;
	double rangesSum = 0;
    int processedCoordinates = 1;
	for(const auto point1 : data){
		rangesSum = 0;
		for(const auto point2 : data){
			// Adding the current coordinate range to the main coordinate ranges sum
			rangesSum += std::sqrt(std::pow(point1.x() - point2.x(), 2) + std::pow(point1.y() - point2.y(), 2) +std::pow(point1.z() - point2.z(), 2));
		}
		distanceToPoint.insert(std::pair<double, Eigen::Matrix<double,3,1>>(rangesSum, point1));

        std::cout << processedCoordinates++ << " coordinates has been processed." << std::endl;

	}

	std::map<double, const Eigen::Matrix<double,3,1>>::iterator iter = std::prev(distanceToPoint.end(), n);

    double xSum = 0;
    double ySum = 0;
    double zSum = 0;

    // Summing the x,y,z of the highest ranges coordinates
	while(iter != distanceToPoint.end()) {
		xSum += iter->second.x();
        ySum += iter->second.y();
        zSum += iter->second.z();
		std::move(iter);
	}

    Point exitPoint = Point(xSum/n, ySum/n, zSum/n);
    std::cout << "The exit coordinate: ";
    exitPoint.printPoint();

    // Averaging the final sum
	return exitPoint;
}
