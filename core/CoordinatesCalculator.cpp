/*
 * CoordinatesCalculator.cpp
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#include "CoordinatesCalculator.h"

#include <cmath>
#include <set>
#include <vector>
using namespace std;
CoordinatesCalculator::CoordinatesCalculator() {
	// TODO Auto-generated constructor stub

}

CoordinatesCalculator::~CoordinatesCalculator() {
	// TODO Auto-generated destructor stub
}

set<ThreeDimensionPoint> CoordinatesCalculator::getTopN(int n, const vector<ThreeDimensionPoint>& data) {
	set<double> ranges;
	double rangesSum = 0;
	for(const auto point1 : data){
		rangesSum = 0;
		for(const auto point2 : data){
			// Adding the current coordinate range to the main coordinate ranges sum
			rangesSum += std::sqrt(std::pow(point1.get_x() - point2.get_x(), 2) + std::pow(point1.get_y() - point2.get_y(), 2) +std::pow(point1.get_z() - point2.get_z(), 2));
		}
		ranges.insert(rangesSum);
	}

	std::set<double> topN;
	std::set<double>::iterator iter = ranges.begin();
	std::advance(iter, n);
	topN.insert(ranges.begin(), iter);
	return topN;
}
