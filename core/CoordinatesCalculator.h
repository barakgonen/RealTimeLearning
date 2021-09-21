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

class CoordinatesCalculator {
public:
	CoordinatesCalculator() = default;
	virtual ~CoordinatesCalculator() = default;
	std::vector<std::pair<double, Eigen::Matrix<double, 3, 1>>> getTopN(int n, const std::vector<Eigen::Matrix<double,3,1>>& data);
};

#endif /* CORE_COORDINATESCALCULATOR_H_ */
