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
	static Point3D detectExitCoordinate(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor, bool isMulti, const bool printsEnabled);
private:
	static Point3D calculate_exit_point(const std::map<double, const Point3D>& mappedPoints, int numberOfPointsToFilter);
	static Point3D detectExitCoordianteMulti(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor, const bool printsEnabled);
	static Point3D detectExitCoordianteSingle(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor, bool printsEnabled);
};

#endif /* CORE_COORDINATESCALCULATOR_H_ */
