/*
 * ThreeDimensionPoint.h
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#ifndef CORE_THREEDIMENSIONPOINT_H_
#define CORE_THREEDIMENSIONPOINT_H_

class ThreeDimensionPoint {
public:
	ThreeDimensionPoint(const double x_axis, const double y_axis, const double z_axis);
	virtual ~ThreeDimensionPoint();

	const double get_x() const;
	const double get_y() const;
	const double get_z() const;
private:
	const double x_axis;
	const double y_axis;
	const double z_axis;
};

#endif /* CORE_THREEDIMENSIONPOINT_H_ */
