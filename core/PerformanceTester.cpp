/*
 * PerformanceTester.cpp
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#include "PerformanceTester.h"
#include <iosfwd>
#include <fstream>

#include "PerformanceCalculator.h"
#include <iostream>
#include "Utils.h"

PerformanceTester::PerformanceTester(bool _isLoggerOn, const int _numberOfPointsForFiltering, bool _testMultiThreaded, const char** argv, int argc)
: isLoggerOn{isLoggerOn}
, numberOfPointsForFiltering{numberOfPointsForFiltering}
, numberOfLoops{stringToInt(argv[6])}
, testMultithreded{_testMultiThreaded}
, testSingleThreaded{stringToBool(argv[5])}
, inputFilePath{argc > 7 ? argv[7] : "/local/RealTimeLearning/input_example.csv"}
, outputFilePath{argc > 8 ? argv[8] : "/local/RealTimeLearning/output_example.csv"}
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
//	PerformanceCalculator::runInLoop(numberOfLoops, mappedPointsFromFile, numberOfPointsToFilter, multi, single, printsEnabled);
	std::cout << "<PerformanceTester::run()> test finished, writing to output file" << std::endl;
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
