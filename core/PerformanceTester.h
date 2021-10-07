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

#include "AbstractActivityHandler.h"
#include "Point3D.h"

class PerformanceTester : public AbstractActivityHandler {
public:
	PerformanceTester(const std::vector<std::string>& config);
	virtual ~PerformanceTester() = default;

	void run() override;

private:
	std::vector<Point3D> read_csv() const;
	void exportRawDataToFile(const std::vector<Point3D> pointsVec);
	void runInLoop(const std::vector<Point3D>& mappedPointsFromSensor);
	int measureRuntime(const std::vector<Point3D>& mappedPointsFromSensor, const int numberOfPointsToFilter, bool multi, bool printsEnabled);

	const bool testSingleThreaded;
	const int numberOfLoops;
	std::string inputFilePath;
	std::string outputFilePath;
};

#endif /* CORE_PERFORMANCETESTER_H_ */
