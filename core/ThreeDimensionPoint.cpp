/*
 * ThreeDimensionPoint.cpp
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#include "ThreeDimensionPoint.h"

ThreeDimensionPoint::ThreeDimensionPoint(const double _x_axis,
										 const double _y_axis,
										 const double _z_axis)
: x_axis{_x_axis}
, y_axis{_y_axis}
, z_axis{_z_axis}
{
	// TODO Auto-generated constructor stub

}

ThreeDimensionPoint::~ThreeDimensionPoint() {
	// TODO Auto-generated destructor stub
}

const double ThreeDimensionPoint::get_x() const {
	return x_axis;
}

const double ThreeDimensionPoint::get_y() const {
	return y_axis;
}

const double ThreeDimensionPoint::get_z() const {
	return z_axis;
}

