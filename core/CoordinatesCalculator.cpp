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

        if (processedCoordinates % 1000 == 0) {
            std::cout << "<CoordinatesCalculator::detectExitCoordinate()> "
                      << processedCoordinates++ << " coordinates has been processed." << std::endl;

        }

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

	POINT simpleAvg{xSum/n, ySum/n, zSum/n};
	// Standard deviation calculation
	double xSD, ySD, zSD = 0;
	numberOfPointsProcessed = 0;
	for (auto iter = distanceToPoint.rbegin();
			iter != distanceToPoint.rend() && numberOfPointsProcessed < n; ++iter) {
		xSD += std::pow(iter->second.x() - simpleAvg.x(), 2);
		ySD += std::pow(iter->second.y() - simpleAvg.y(), 2);
		zSD += std::pow(iter->second.z() - simpleAvg.z(), 2);
		numberOfPointsProcessed++;
	}

	std::vector<POINT> cleanPoints;
	POINT sd{std::sqrt(xSD/n), std::sqrt(ySD/n), std::sqrt(zSD/n)};
	numberOfPointsProcessed = 0;
	for (auto iter = distanceToPoint.rbegin();
				iter != distanceToPoint.rend() && numberOfPointsProcessed < n; ++iter) {
		if ((std::sqrt(std::pow(iter->second.x() - simpleAvg.x(), 2)) <= sd.x())
		 && (std::sqrt(std::pow(iter->second.y() - simpleAvg.y(), 2)) <= sd.y())
		 && (std::sqrt(std::pow(iter->second.z() - simpleAvg.z(), 2)) <= sd.z())) {
			cleanPoints.push_back(iter->second);
		}
		numberOfPointsProcessed++;
	}

	xSum = 0;
	ySum = 0;
	zSum = 0;
	// Calculating avg after cleanup
	for (const auto p : cleanPoints) {
		xSum += p.x();
		ySum += p.y();
		zSum += p.z();
	}

    // Averaging the final sum and constructing exit point
	return {xSum/cleanPoints.size(), ySum/cleanPoints.size(), zSum/cleanPoints.size()};
}
