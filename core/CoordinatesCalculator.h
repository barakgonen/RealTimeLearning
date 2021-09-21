/*
 * CoordinatesCalculator.h
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#ifndef CORE_COORDINATESCALCULATOR_H_
#define CORE_COORDINATESCALCULATOR_H_
#include <eigen3/Eigen/Core>
#include <set>
#include <vector>

#define POINT Eigen::Matrix<double,3,1>

class CoordinatesCalculator {
public:
	CoordinatesCalculator() = default;
	virtual ~CoordinatesCalculator() = default;
	POINT detectExitCoordinate(int n, const std::vector<POINT>& points);
};

#endif /* CORE_COORDINATESCALCULATOR_H_ */
