/*
 * PerformanceCalculator.h
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#ifndef CORE_CMAKEFILES_PERFORMANCECALCULATOR_H_
#define CORE_CMAKEFILES_PERFORMANCECALCULATOR_H_
#include <vector>

#include "Point3D.h"
class PerformanceCalculator {
public:
	PerformanceCalculator() = delete;
	virtual ~PerformanceCalculator() = default;

	static void runInLoop(int numberOfLoops, const std::vector<Point3D>& points, const int numberOfPointsToFilter, bool multi, bool single = false);
};

#endif /* CORE_CMAKEFILES_PERFORMANCECALCULATOR_H_ */
