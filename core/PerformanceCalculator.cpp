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

void PerformanceCalculator::runInLoop(int numberOfLoops, const std::vector<Point3D>& mappedPointsFromSensor, const int numberOfPointsToFilter, const bool multi, const bool single, bool printsEnabled) {
	std::cout << "NUMBER OF POINTS: " << mappedPointsFromSensor.size() << std::endl;
	long long int totalSingleRuntime = 0;
	long long int totalMultiRuntime = 0;

	for (int i = 1; i <= numberOfLoops; i++) {
		if (multi) {
			if (single) {
				// testing both multi and single
				totalMultiRuntime += measureRuntime(mappedPointsFromSensor, numberOfPointsToFilter, multi, printsEnabled);
				totalSingleRuntime += measureRuntime(mappedPointsFromSensor, numberOfPointsToFilter, false, printsEnabled);
			} else {
				// testing multi only
					totalMultiRuntime += measureRuntime(mappedPointsFromSensor, numberOfPointsToFilter, multi, printsEnabled);
				}
		} else {
			if (single) {
				// single only
				totalSingleRuntime += measureRuntime(mappedPointsFromSensor, numberOfPointsToFilter, false, printsEnabled);
			} else {
				std::cout << "Must choose at least one true flag, multi or single. current config: " << std::boolalpha << " multi: " << multi << ", single: " << single << std::endl;
			}
		}

		std::cout << "Iteration: " << i << " of: " << numberOfLoops << " has completed! ";
		if (multi) {
				if (single) {
					std::cout << "current avg single: " << (double)(totalSingleRuntime / i) << std::endl;
					std::cout << "current avg multi: " << (double)(totalMultiRuntime / i) << std::endl;
				} else {
					std::cout << "current avg multi: " << (double)(totalMultiRuntime / i) << std::endl;
				}
			} else {
				if (single) {
					// single only
					std::cout << "current avg single: " << (double)(totalSingleRuntime / i) << std::endl;
				}
			}
	}

	std::cout << "Conclusion: [number of runs: " << numberOfLoops << "]" << std::endl;
	if (multi) {
		if (single) {
			std::cout << "Single total time: " << totalSingleRuntime << ", avg: " << (double)(totalSingleRuntime / numberOfLoops) << std::endl;
			std::cout << "multi total time: " << totalMultiRuntime << ", avg: " << (double)(totalMultiRuntime / numberOfLoops) << std::endl;
		} else {
			std::cout << "multi total time: " << totalMultiRuntime << ", avg: " << (double)(totalMultiRuntime / numberOfLoops) << std::endl;
		}
	} else {
		if (single) {
			// single only
			std::cout << "Single total time: " << totalSingleRuntime << ", avg: " << (double)(totalSingleRuntime / numberOfLoops) << std::endl;
		}
	}
}

int PerformanceCalculator::measureRuntime(const std::vector<Point3D>& mappedPointsFromSensor, const int numberOfPointsToFilter, bool multi, bool printsEnabled) {
	const auto startTime = std::chrono::system_clock::now();
	const auto exitPoint = CoordinatesCalculator::detectExitCoordinate(numberOfPointsToFilter, mappedPointsFromSensor, multi, printsEnabled);
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime).count();
}
