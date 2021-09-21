/*
 * CoordinatesCalculator.cpp
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#include "CoordinatesCalculator.h"

#include <cmath>
#include <map>
#include <iostream>
#include <vector>
using namespace std;

vector<std::pair<double, Eigen::Matrix<double, 3, 1>>> CoordinatesCalculator::getTopN(int n, const vector<Eigen::Matrix<double,3,1>>& data) {

	// We use map for kind of insertion sort, since it keeps the keys ordered. instead of sorting at the end
	std::map<double, const Eigen::Matrix<double,3,1>> distanceToPoint;
	std::vector<std::pair<double, Eigen::Matrix<double, 3, 1>>> resultVec;
	double rangesSum = 0;
	for(const auto point1 : data){
		rangesSum = 0;
		for(const auto point2 : data){
			// Adding the current coordinate range to the main coordinate ranges sum
			rangesSum += std::sqrt(std::pow(point1.x() - point2.x(), 2) + std::pow(point1.y() - point2.y(), 2) +std::pow(point1.z() - point2.z(), 2));
		}
		distanceToPoint.insert(std::pair<double, Eigen::Matrix<double,3,1>>(rangesSum, point1));

	}

	std::map<double, const Eigen::Matrix<double,3,1>>::iterator iter = distanceToPoint.begin();
	std::map<double, const Eigen::Matrix<double,3,1>>::iterator curr = distanceToPoint.begin();
	std::advance(iter, n);

	while(curr != iter) {
		const auto val = *curr;
		resultVec.push_back(std::move(*curr));
		curr++;
	}

	return resultVec;
}
