/*
 * PerformanceTester.h
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#ifndef CORE_PERFORMANCETESTER_H_
#define CORE_PERFORMANCETESTER_H_
#include <string>
#include <vector>
#include "Point3D.h"

class PerformanceTester {
public:
	PerformanceTester(bool isLoggerOn, const int numberOfPointsForFiltering, bool testMultiThreaded, char** argv, int argc);
	virtual ~PerformanceTester() = default;
	void run();

private:
	std::vector<Point3D> read_csv() const;

	const bool isLoggerOn;
	const int numberOfPointsForFiltering;
	const int numberOfLoops;
	const bool testMultithreded;
	const bool testSingleThreaded;
	const std::string& inputFilePath;
	const std::string& outputFilePath;
};

#endif /* CORE_PERFORMANCETESTER_H_ */
