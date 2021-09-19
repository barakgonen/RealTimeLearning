/*
 * CoordinatesCalculator.h
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#ifndef CORE_COORDINATESCALCULATOR_H_
#define CORE_COORDINATESCALCULATOR_H_
#include <set>
#include <vector>

#include "ThreeDimensionPoint.h"

class CoordinatesCalculator {
public:
	CoordinatesCalculator();
	virtual ~CoordinatesCalculator();
	std::set<ThreeDimensionPoint> getTopN(int n, const std::vector<ThreeDimensionPoint>& data);
};

#endif /* CORE_COORDINATESCALCULATOR_H_ */
