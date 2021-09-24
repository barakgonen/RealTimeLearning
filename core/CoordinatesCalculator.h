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
	CoordinatesCalculator() = default;
	virtual ~CoordinatesCalculator() = default;
	Point3D detectExitCoordinate(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);
	Point3D detectExitCoordinateParallelized(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);

	Point3D detectExitCoordinateCache(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);
	void bgTest(int n, const std::vector<Point3D>& points);

private:
	Point3D calculate_exit_point(const std::map<double, const Point3D>& mappedPoints, int n);
	void insert_caching(const Point3D& point1, const Point3D& point2, double range, std::map<Point3D, std::map<Point3D, double> > &cache);

	std::pair<std::string, std::pair<Point3D, long int>> runMulti(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);
	std::pair<std::string, std::pair<Point3D, long int>> runCached(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);
	std::pair<std::string, std::pair<Point3D, long int>> runRegular(int numberOfPointsToFilter, const std::vector<Point3D>& mappedPointsFromSensor);
};

#endif /* CORE_COORDINATESCALCULATOR_H_ */
