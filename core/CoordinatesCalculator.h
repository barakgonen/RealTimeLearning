/*
 * CoordinatesCalculator.h
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#ifndef CORE_COORDINATESCALCULATOR_H_
#define CORE_COORDINATESCALCULATOR_H_

#include <map>
#include <set>
#include <vector>
#include "Point3D.h"

class CoordinatesCalculator {
public:
	CoordinatesCalculator() = delete;
	virtual ~CoordinatesCalculator() = default;
	static Point3D detectExitCoordinate(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);
	static void bgTest(int n, const std::vector<Point3D>& points);

private:
	static Point3D calculate_exit_point(const std::map<double, const Point3D>& mappedPoints, int n);

	static std::pair<std::string, std::pair<Point3D, long int>> runMulti(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);
	static std::pair<std::string, std::pair<Point3D, long int>> runRegular(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);
};

#endif /* CORE_COORDINATESCALCULATOR_H_ */
