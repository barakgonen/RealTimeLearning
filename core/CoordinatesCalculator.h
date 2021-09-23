/*
 * CoordinatesCalculator.h
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#ifndef CORE_COORDINATESCALCULATOR_H_
#define CORE_COORDINATESCALCULATOR_H_
#include <eigen3/Eigen/Core>
#include <map>
#include <set>
#include <vector>

#define POINT Eigen::Matrix<double,3,1>

class CoordinatesCalculator {
public:
	CoordinatesCalculator() = default;
	virtual ~CoordinatesCalculator() = default;
	POINT detectExitCoordinate(int n, const std::vector<POINT>& points);
	POINT detectExitCoordinateParallelized(int n, const std::vector<POINT>& points);
	void bgTest(int n, const std::vector<POINT>& points);

private:

	POINT calculate_exit_point(const std::map<double, const POINT>& mappedPoints, int n);
};

#endif /* CORE_COORDINATESCALCULATOR_H_ */
