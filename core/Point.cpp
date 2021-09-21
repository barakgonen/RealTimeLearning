/*
 * Point.cpp
 *
 *  Created on: Sep 3, 2021
 *      Author: barakg
 */

#include "Point.h"

#include <iostream>
Point::Point(double x, double y, double z)
: x{x}
, y{y}
, z{z}
{
}

double Point::getX() const {
    return x;
}

void Point::setX(double x) {
    Point::x = x;
}

double Point::getY() const {
    return y;
}

void Point::setY(double y) {
    Point::y = y;
}

double Point::getZ() const {
    return z;
}

void Point::setZ(double z) {
    Point::z = z;
}

void Point::printPoint() {
	std::cout << "x = " << x << ", y = " << y << std::endl;
}
Point::~Point() {
	// TODO Auto-generated destructor stub
}

