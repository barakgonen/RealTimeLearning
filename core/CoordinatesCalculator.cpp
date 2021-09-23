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

#include <thread>
using namespace std;

void calculate_distance_between_points_in_range(const std::vector<POINT>& pointsInMapping, int startIndex, int endIndex, std::map<double, const POINT>& distanceToPoint, int threadNumber, bool isMulti)
{
	std::cout << "<CoordinatesCalculator::calculate_distance_between_points_in_range()> start index = " << startIndex << ", end index = " << endIndex << ", thread number = " << threadNumber << ", is multi? " << std::boolalpha << isMulti << std::endl;
	double rangesSum = 0;
	int processedCoordinates = 1;
	for (int point1Index = startIndex; point1Index < endIndex; point1Index++) {
		rangesSum = 0;
		const auto& point1 = pointsInMapping.at(point1Index);
//		for (int point2Index = startIndex; point2Index < endIndex; point2Index++) {
		for (const auto& point2 : pointsInMapping) {
//			const auto& point2 = pointsInMapping.at(point2Index);
			// Adding the current coordinate range to the main coordinate ranges sum
			rangesSum += std::sqrt(
					std::pow(point1.x() - point2.x(), 2)
							+ std::pow(point1.y() - point2.y(), 2)
							+ std::pow(point1.z() - point2.z(), 2));
		}
		distanceToPoint.insert(std::pair<double, POINT>(rangesSum, point1));

		if (processedCoordinates % 1000 == 0) {
			if (isMulti) {
				std::cout << "<CoordinatesCalculator::calculate_distance_between_points_in_range()> Thread num: " << threadNumber << ", " << processedCoordinates
									<< " coordinates has been processed." << std::endl;

			} else {
				std::cout << "<CoordinatesCalculator::calculate_distance_between_points_in_range()> " << processedCoordinates
						<< " coordinates has been processed." << std::endl;
			}
		}

		processedCoordinates++;
	}
}

POINT CoordinatesCalculator::detectExitCoordinate(int n, const vector<POINT> &pointsInMapping) {
	std::cout << "<CoordinatesCalculator::detectExitCoordinate()> number of points to process: "
			<< pointsInMapping.size() << std::endl;
	// We use map for kind of insertion sort, since it keeps the keys ordered. instead of sorting at the end
	std::map<double, const POINT> distanceToPoint;
	calculate_distance_between_points_in_range(pointsInMapping, 0, pointsInMapping.size(), distanceToPoint, 0, false);
	std::cout << "Single threaded map size after first calculation is: "
			<< distanceToPoint.size() << std::endl;
	return calculate_exit_point(distanceToPoint, n);
}

POINT CoordinatesCalculator::detectExitCoordinateParallelized(int n, const vector<POINT> &pointsInMapping) {
	std::cout
			<< "<CoordinatesCalculator::detectExitCoordinateParallelized()> number of points to process: "
			<< pointsInMapping.size() << std::endl;
	// We use map for kind of insertion sort, since it keeps the keys ordered. instead of sorting at the end
	std::map<double, const POINT> distanceToPoint;
	const size_t nthreads = std::thread::hardware_concurrency();
	std::cout << "parallel (" << nthreads << " threads):" << std::endl;
	std::vector<std::thread> workers(nthreads);
	const auto startMulti = std::chrono::system_clock::now();

	int batchSize = pointsInMapping.size() / workers.size();
	for (int threadNum = 0; threadNum < nthreads; threadNum++) {
		int startIdex = threadNum * batchSize;
		int endIndex = startIdex + batchSize;
		workers.push_back(
				std::thread(calculate_distance_between_points_in_range, pointsInMapping, startIdex, endIndex, std::ref(distanceToPoint), threadNum, true));
	}
	for (std::thread &t : workers) {
		if (t.joinable()) {
			t.join();
		}
	}
	std::cout << "multi threaded map size after first calculation is: "
				<< distanceToPoint.size() << std::endl;
	return calculate_exit_point(distanceToPoint, n);
}

void CoordinatesCalculator::bgTest(int n, const std::vector<POINT>& points) {
	std::map<double, const POINT> singleMap;
	std::map<double, const POINT> multiMap;

	std::cout << "NUMBER OF POINTS: " << points.size() << std::endl;
	/*
	 * TODO: remove
	 */
	const size_t nthreads = std::thread::hardware_concurrency();
	std::cout << "parallel (" << nthreads << " threads):" << std::endl;
	std::vector<std::thread> workers(nthreads);
	const auto startMulti = std::chrono::system_clock::now();

	int batchSize = points.size() / workers.size();
	for (int threadNum = 0; threadNum < nthreads; threadNum++) {
		int startIdex = threadNum * batchSize;
		int endIndex = startIdex + batchSize;
		workers.push_back(
				std::thread(calculate_distance_between_points_in_range, points, startIdex, endIndex, std::ref(multiMap), threadNum, true));
	}
	for (std::thread &t : workers) {
		if (t.joinable()) {
			t.join();
		}
	}
	calculate_distance_between_points_in_range(points, 0, points.size(), singleMap, 0, false);

	const auto parallel =  calculate_exit_point(multiMap, n);
	const auto single = calculate_exit_point(singleMap, n);
	std::cout << "multi map size: " << multiMap.size() << ", single size: " << singleMap.size() << std::endl;

}
POINT CoordinatesCalculator::calculate_exit_point(
		const std::map<double, const POINT> &distanceToPoint, int n) {
	int numberOfPointsProcessed = 0;
	double xSum, ySum, zSum = 0;

	for (auto iter = distanceToPoint.rbegin();
			iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
			++iter) {
		xSum += iter->second.x();
		ySum += iter->second.y();
		zSum += iter->second.z();
		numberOfPointsProcessed++;
	}

	POINT simpleAvg { xSum / n, ySum / n, zSum / n };
	// Standard deviation calculation
	double xSD, ySD, zSD = 0;
	numberOfPointsProcessed = 0;
	for (auto iter = distanceToPoint.rbegin();
			iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
			++iter) {
		xSD += std::pow(iter->second.x() - simpleAvg.x(), 2);
		ySD += std::pow(iter->second.y() - simpleAvg.y(), 2);
		zSD += std::pow(iter->second.z() - simpleAvg.z(), 2);
		numberOfPointsProcessed++;
	}

	std::vector<POINT> cleanPoints;
	POINT sd { std::sqrt(xSD / n), std::sqrt(ySD / n), std::sqrt(zSD / n) };
	numberOfPointsProcessed = 0;
	for (auto iter = distanceToPoint.rbegin();
			iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
			++iter) {
		if ((std::sqrt(std::pow(iter->second.x() - simpleAvg.x(), 2)) <= sd.x())
				&& (std::sqrt(std::pow(iter->second.y() - simpleAvg.y(), 2))
						<= sd.y())
				&& (std::sqrt(std::pow(iter->second.z() - simpleAvg.z(), 2))
						<= sd.z())) {
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
