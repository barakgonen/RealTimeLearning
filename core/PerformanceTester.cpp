/*
 * PerformanceTester.cpp
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#include "PerformanceTester.h"

#include <chrono>
#include <iosfwd>
#include <fstream>

#include <iostream>
#include "CoordinatesCalculator.h"

PerformanceTester::PerformanceTester(const std::vector<std::string>& config)
: AbstractActivityHandler{config}
, numberOfLoops{stringToInt(config[5])}
, testSingleThreaded{stringToBool(config[4])}
, inputFilePath{config.size() <= 6 ? "/local/RealTimeLearning/input_example.csv" : config[6]}
, outputFilePath{config.size() < 7 ? "/local/RealTimeLearning/output_example.csv" : config[7]}
{
	std::cout << "testSingleThreaded: " << std::boolalpha << testSingleThreaded << std::endl;
	std::cout << "numberOfLoops: " << numberOfLoops << std::endl;
	std::cout << "inputFilePath: " << inputFilePath << std::endl;
	std::cout << "outputFilePath: " << outputFilePath << std::endl;
}

void PerformanceTester::run()
{
	std::cout << "<PerformanceTester::run()> perfomance tester is running!" << std::endl;
	const auto mappedPointsFromFile = read_csv();
	std::cout << "<PerformanceTester::run()> all data read from CSV succussfully!" << std::endl;
	runInLoop(mappedPointsFromFile);
	std::cout << "<PerformanceTester::run()> test finished, writing to output file" << std::endl;


}

void PerformanceTester::exportRawDataToFile(const std::vector<Point3D> pointsVec) {
	std::ofstream pointData;
	pointData.open(outputFilePath);
	for (const auto& p : pointsVec) {
		pointData << p.getX() << "," << p.getY() << "," << p.getZ() << std::endl;
	}
	pointData.close();
}

/**
 * Utility function to read csv file written in our format
 * format is:
 * x,y,z
 * original file was the map of the room, for integration we had to store map of points
 */
std::vector<Point3D> PerformanceTester::read_csv() const {
	std::cout << "<PerformanceTester::read_csv()> Reding points from input line" << std::endl;
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<Point3D> result;

    // Create an input filestream
    std::ifstream myFile(inputFilePath);

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

    	result.push_back({Eigen::Matrix<double, 3, 1>(Eigen::Matrix<double, 3, 1>{x, y, z})});
    }


    // Close file
    myFile.close();
    return result;
}

void PerformanceTester::runInLoop(const std::vector<Point3D>& mappedPointsFromSensor) {
	std::cout << "NUMBER OF POINTS: " << mappedPointsFromSensor.size() << std::endl;
	long long int totalSingleRuntime = 0;
	long long int totalMultiRuntime = 0;

	for (int i = 1; i <= numberOfLoops; i++) {
		if (isMulti) {
			if (testSingleThreaded) {
				// testing both multi and single
				totalMultiRuntime += measureRuntime(mappedPointsFromSensor, numberOfPointsForFiltering, isMulti, isLoggerOn);
				totalSingleRuntime += measureRuntime(mappedPointsFromSensor, numberOfPointsForFiltering, false, isLoggerOn);
			} else {
				// testing multi only
					totalMultiRuntime += measureRuntime(mappedPointsFromSensor, numberOfPointsForFiltering, isMulti, isLoggerOn);
				}
		} else {
			if (testSingleThreaded) {
				// single only
				totalSingleRuntime += measureRuntime(mappedPointsFromSensor, numberOfPointsForFiltering, false, isLoggerOn);
			} else {
				std::cout << "Must choose at least one true flag, multi or single. current config: " << std::boolalpha << " multi: " << isMulti << ", testSingleThreaded: " << testSingleThreaded << std::endl;
			}
		}

		std::cout << "Iteration: " << i << " of: " << numberOfLoops << " has completed! ";
		if (isMulti) {
				if (testSingleThreaded) {
					std::cout << "current avg single: " << (double)(totalSingleRuntime / i) << std::endl;
					std::cout << "current avg multi: " << (double)(totalMultiRuntime / i) << std::endl;
				} else {
					std::cout << "current avg multi: " << (double)(totalMultiRuntime / i) << std::endl;
				}
			} else {
				if (testSingleThreaded) {
					// single only
					std::cout << "current avg single: " << (double)(totalSingleRuntime / i) << std::endl;
				}
			}
	}

	std::cout << "Conclusion: [number of runs: " << numberOfLoops << "]" << std::endl;
	if (isMulti) {
		if (testSingleThreaded) {
			std::cout << "Single total time: " << totalSingleRuntime << ", avg: " << (double)(totalSingleRuntime / numberOfLoops) << std::endl;
			std::cout << "multi total time: " << totalMultiRuntime << ", avg: " << (double)(totalMultiRuntime / numberOfLoops) << std::endl;
		} else {
			std::cout << "multi total time: " << totalMultiRuntime << ", avg: " << (double)(totalMultiRuntime / numberOfLoops) << std::endl;
		}
	} else {
		if (testSingleThreaded) {
			// single only
			std::cout << "Single total time: " << totalSingleRuntime << ", avg: " << (double)(totalSingleRuntime / numberOfLoops) << std::endl;
		}
	}
}

int PerformanceTester::measureRuntime(const std::vector<Point3D>& mappedPointsFromSensor, const int numberOfPointsToFilter, bool multi, bool printsEnabled) {
	const auto startTime = std::chrono::system_clock::now();
	const auto exitPoint = CoordinatesCalculator::detectExitCoordinate(numberOfPointsToFilter, mappedPointsFromSensor, multi, printsEnabled);
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime).count();
}

