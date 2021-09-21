/*
 * Point.h
 *
 *  Created on: Sep 3, 2021
 *      Author: barakg
 */

#ifndef INCLUDE_POINT_H_
#define INCLUDE_POINT_H_

class Point {
public:
	Point(double x, double y, double z);
	virtual ~Point();

	void printPoint();

    double getX() const;

    void setX(double x);

    double getY() const;

    void setY(double y);

    double getZ() const;

    void setZ(double z);

protected:
	double x;
    double y;
    double z;

};

#endif /* INCLUDE_POINT_H_ */
