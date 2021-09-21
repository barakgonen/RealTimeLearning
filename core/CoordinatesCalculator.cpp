/*
 * CoordinatesCalculator.cpp
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#include <map>
#include <cmath>
#include <vector>
#include <iostream>

#include "CoordinatesCalculator.h"

using namespace std;

POINT CoordinatesCalculator::detectExitCoordinate(int n, const vector<POINT>& pointsInMapping) {

	// We use map for kind of insertion sort, since it keeps the keys ordered. instead of sorting at the end
	std::map<double, const POINT> distanceToPoint;
	double rangesSum = 0;
    int processedCoordinates = 1;
	for(const auto point1 : pointsInMapping){
		rangesSum = 0;
		for(const auto point2 : pointsInMapping){
			// Adding the current coordinate range to the main coordinate ranges sum
			rangesSum += std::sqrt(std::pow(point1.x() - point2.x(), 2) +
								   std::pow(point1.y() - point2.y(), 2) +
								   std::pow(point1.z() - point2.z(), 2));
		}
		distanceToPoint.insert(std::pair<double, POINT>(rangesSum, point1));

        std::cout << "<CoordinatesCalculator::detectExitCoordinate()> "
        		<< processedCoordinates++ << " coordinates has been processed." << std::endl;

	}

	int numberOfPointsProcessed = 0;
	double xSum, ySum, zSum = 0;

	for (auto iter = distanceToPoint.rbegin();
			iter != distanceToPoint.rend() && numberOfPointsProcessed < n; ++iter) {
		xSum += iter->second.x();
		ySum += iter->second.y();
		zSum += iter->second.z();
		numberOfPointsProcessed++;
	}

    // Averaging the final sum and constructing exit point
	return {xSum/n, ySum/n, zSum/n};
}
