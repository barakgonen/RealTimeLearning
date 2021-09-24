/*
 * PerformanceCalculator.cpp
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#include <vector>
#include <iostream>
#include "PerformanceCalculator.h"

#include <chrono>
#include "CoordinatesCalculator.h"

void PerformanceCalculator::runInLoop(int numberOfLoops, const std::vector<Point3D>& points, const int numberOfPointsToFilter, bool multi, bool single) {
	std::cout << "NUMBER OF POINTS: " << points.size() << std::endl;
	long long int totalSingleRuntime = 0;
	long long int totalMultiRuntime = 0;

	if (multi) {
		if (single) {
			// testing both multi and single
		} else {
			// testing multi only
			for (int i = 0; i < numberOfLoops; i++) {
				const auto startTime = std::chrono::system_clock::now();
				const auto exitPoint = CoordinatesCalculator::detectExitCoordinate(numberOfPointsToFilter, points);
				totalMultiRuntime += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime).count();
			}
		}
	} else {
		if (single) {
			// single only
			for (int i = 0; i < numberOfLoops; i++) {
				const auto startTime = std::chrono::system_clock::now();
				const auto exitPoint = CoordinatesCalculator::detectExitCoordinate(numberOfPointsToFilter, points);
				totalMultiRuntime += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime).count();
			}
		} else {
			std::cout << "Must choose at least one true flag, multi or single. current config: " << std::boolalpha << " multi: " << multi << ", single: " << single << std::endl;
		}
	}

	std::cout << "Conclusion: " << std::endl;
	if (multi) {
		if (single) {
			// testing both multi and single
		} else {
			std::cout << "Total runtime multi: " << totalMultiRuntime << ", times: " << numberOfLoops << ", avg: " << totalMultiRuntime / numberOfLoops << std::endl;
		}
	} else {
		if (single) {
			// single only
		} else {
			std::cout << "Must choose at least one true flag, multi or single. current config: " << std::boolalpha << " multi: " << multi << ", single: " << single << std::endl;
		}
	}
//	for (int i = 0; i < numberOfLoops; i++) {
//		std::vector<std::pair<std::string, std::pair<Point3D, long int>>> testResults;
//		testResults.push_back(runMulti(n, points));
//		testResults.push_back(runRegular(n, points));
//		for (const auto &result : testResults) {
//			std::cout << "Method: " << result.first << ", total time: "
//					<< result.second.second << " ms, calculated point: "
//					<< result.second.first << std::endl;
//		}
//	}
}

//
//std::pair<std::string, std::pair<Point3D, long int>> CoordinatesCalculator::runRegular(
//		int numberOfPointsToFilter,
//		const std::vector<Point3D> &mappedPointsFromSensor) {
//	const auto singleStart = std::chrono::system_clock::now();
//	std::map<double, const Point3D> singleMap;
//	calculate_distance_between_points_in_range(mappedPointsFromSensor, 0,
//			mappedPointsFromSensor.size(), singleMap, 0, false);
//	const auto single = calculate_exit_point(singleMap, numberOfPointsToFilter);
//	const auto singleEnd = std::chrono::system_clock::now();
//	return std::pair<std::string, std::pair<Point3D, long int>>("Single, Basic",
//			std::make_pair(single,
//					std::chrono::duration_cast<std::chrono::milliseconds>(
//							singleEnd - singleStart).count()));
//}
