/*
 * CoordinatesCalculator.cpp
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#include <map>
#include <vector>
#include <iostream>

#include "CoordinatesCalculator.h"

#include <thread>
#include <utility>
using namespace std;

void calculate_distance_between_points_in_range(
		const std::vector<Point3D> &pointsInMapping, int startIndex,
		int endIndex, std::map<double, const Point3D> &distanceToPoint,
		int threadNumber, bool isMulti) {
	std::cout
			<< "<CoordinatesCalculator::calculate_distance_between_points_in_range()> start index = "
			<< startIndex << ", end index = " << endIndex
			<< ", thread number = " << threadNumber << ", is multi? "
			<< std::boolalpha << isMulti << std::endl;
	double rangesSum = 0;
	int processedCoordinates = 1;
	for (int point1Index = startIndex; point1Index < endIndex; point1Index++) {
		rangesSum = 0;
		auto point1 = pointsInMapping.at(point1Index);
		for (const auto &point2 : pointsInMapping) {
			// Adding the current coordinate range to the main coordinate ranges sum
			rangesSum += std::sqrt(
					std::pow(point1.getX() - point2.getX(), 2)
							+ std::pow(point1.getY() - point2.getY(), 2)
							+ std::pow(point1.getZ() - point2.getZ(), 2));
		}
		distanceToPoint.insert(
				std::pair<double, const Point3D>(rangesSum, point1));

		if (processedCoordinates % 1000 == 0) {
			if (isMulti) {
				std::cout
						<< "<CoordinatesCalculator::calculate_distance_between_points_in_range()> Thread num: "
						<< threadNumber << ", " << processedCoordinates
						<< " coordinates has been processed." << std::endl;

			} else {
				std::cout
						<< "<CoordinatesCalculator::calculate_distance_between_points_in_range()> "
						<< processedCoordinates
						<< " coordinates has been processed." << std::endl;
			}
		}

		processedCoordinates++;
	}
}

Point3D CoordinatesCalculator::detectExitCoordinate(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor) {
	std::map<double, const Point3D> multiMap;
	const size_t nthreads = std::thread::hardware_concurrency();
	std::cout << "parallel (" << nthreads << " threads):" << std::endl;
	std::vector<std::thread> workers(nthreads);
	const auto startMulti = std::chrono::system_clock::now();

	int batchSize = mappedPointsFromSensor.size() / workers.size();
	for (int threadNum = 0; threadNum < nthreads; threadNum++) {
		int startIdex = threadNum * batchSize;
		int endIndex = startIdex + batchSize;
		workers.push_back(
				std::thread(calculate_distance_between_points_in_range,
						mappedPointsFromSensor, startIdex, endIndex,
						std::ref(multiMap), threadNum, true));
	}
	for (std::thread &t : workers) {
		if (t.joinable()) {
			t.join();
		}
	}
	return calculate_exit_point(multiMap, numberOfPointsToFilter);
}

Point3D CoordinatesCalculator::calculate_exit_point(
		const std::map<double, const Point3D> &distanceToPoint, int n) {
	int numberOfPointsProcessed = 0;
	double xSum, ySum, zSum = 0;

	for (auto iter = distanceToPoint.rbegin();
			iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
			++iter) {
		xSum += iter->second.getX();
		ySum += iter->second.getY();
		zSum += iter->second.getZ();
		numberOfPointsProcessed++;
	}

	Point3D simpleAvg { xSum / n, ySum / n, zSum / n };
	// Standard deviation calculation
	double xSD, ySD, zSD = 0;
	numberOfPointsProcessed = 0;
	for (auto iter = distanceToPoint.rbegin();
			iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
			++iter) {
		xSD += std::pow(iter->second.getX() - simpleAvg.getX(), 2);
		ySD += std::pow(iter->second.getY() - simpleAvg.getY(), 2);
		zSD += std::pow(iter->second.getZ() - simpleAvg.getZ(), 2);
		numberOfPointsProcessed++;
	}

	std::vector<Point3D> cleanPoints;
	Point3D sd { std::sqrt(xSD / n), std::sqrt(ySD / n), std::sqrt(zSD / n) };
	numberOfPointsProcessed = 0;
	for (auto iter = distanceToPoint.rbegin();
			iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
			++iter) {
		if ((std::sqrt(std::pow(iter->second.getX() - simpleAvg.getX(), 2))
				<= sd.getX())
				&& (std::sqrt(
						std::pow(iter->second.getY() - simpleAvg.getY(), 2))
						<= sd.getY())
				&& (std::sqrt(
						std::pow(iter->second.getZ() - simpleAvg.getZ(), 2))
						<= sd.getZ())) {
			cleanPoints.push_back(iter->second);
		}
		numberOfPointsProcessed++;
	}

	xSum = 0;
	ySum = 0;
	zSum = 0;
	// Calculating avg after cleanup
	for (const auto p : cleanPoints) {
		xSum += p.getX();
		ySum += p.getY();
		zSum += p.getZ();
	}

	// Averaging the final sum and constructing exit point
	return {xSum/cleanPoints.size(), ySum/cleanPoints.size(), zSum/cleanPoints.size()};
}

void CoordinatesCalculator::bgTest(int n, const std::vector<Point3D> &points) {
	std::cout << "NUMBER OF POINTS: " << points.size() << std::endl;
	std::vector<std::pair<std::string, std::pair<Point3D, long int>>> testResults;
	testResults.push_back(runMulti(n, points));
	testResults.push_back(runRegular(n, points));
	for (const auto &result : testResults) {
		std::cout << "Method: " << result.first << ", total time: "
				<< result.second.second << " ms, calculated point: "
				<< result.second.first << std::endl;
	}
}

std::pair<std::string, std::pair<Point3D, long int>> CoordinatesCalculator::runMulti(int numberOfPointsToFilter, const std::vector<Point3D> &mappedPointsFromSensor) {
	std::map<double, const Point3D> multiMap;
	const auto parallelStart = std::chrono::system_clock::now();
	const auto exitPoint = detectExitCoordinate(numberOfPointsToFilter, mappedPointsFromSensor);
	const auto parallelEnd = std::chrono::system_clock::now();
	std::cout << "Parallel total: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(
					parallelEnd - parallelStart).count() << std::endl;
	return std::pair<std::string, std::pair<Point3D, long int>>("MultiThreaded",
			std::make_pair(exitPoint,
					std::chrono::duration_cast<std::chrono::milliseconds>(
							parallelEnd - parallelStart).count()));
}

std::pair<std::string, std::pair<Point3D, long int>> CoordinatesCalculator::runRegular(
		int numberOfPointsToFilter,
		const std::vector<Point3D> &mappedPointsFromSensor) {
	const auto singleStart = std::chrono::system_clock::now();
	std::map<double, const Point3D> singleMap;
	calculate_distance_between_points_in_range(mappedPointsFromSensor, 0,
			mappedPointsFromSensor.size(), singleMap, 0, false);
	const auto single = calculate_exit_point(singleMap, numberOfPointsToFilter);
	const auto singleEnd = std::chrono::system_clock::now();
	return std::pair<std::string, std::pair<Point3D, long int>>("Single, Basic",
			std::make_pair(single,
					std::chrono::duration_cast<std::chrono::milliseconds>(
							singleEnd - singleStart).count()));
}
