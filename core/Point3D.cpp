/*
 * Point3D.cpp
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#include "Point3D.h"
#include <iostream>
#include <cmath>

Point3D::Point3D(const double _x, const double _y, const double _z)
: x { _x }
, y { _y }
, z { _z }
, hash {x * y * z}
{
}

Point3D::Point3D(const Eigen::Matrix<double, 3, 1> &eigenPoint)
: Point3D {eigenPoint.x(), eigenPoint.y(), eigenPoint.z() }
{
	// TODO Auto-generated constructor stub

}

bool Point3D::operator<(const Point3D &rhs) const {
//	return x < rhs.x || (x == rhs.x && (y < rhs.y || (y == rhs.y && z < rhs.z)));
	return hash <= rhs.getHash();
}

double Point3D::getX() const {
	return x;
}

double Point3D::getY() const {
	return y;
}

bool Point3D::operator ==(const Point3D &rhs) const {
	return x == rhs.x && y == rhs.y && z == rhs.z;
}

double Point3D::getZ() const {
	return z;
}

double Point3D::getHash() const {
	return hash;
}

std::ostream& operator<<(std::ostream& os, const Point3D& dt) {
	return os << "{ x = " << dt.getX() << ", y = " << dt.getY() << ", z = " << dt.getZ() << "}";
}

double Point3D::getDistance(const Point3D& point2) const {
	return std::sqrt(std::pow(x - point2.getX(), 2) + std::pow(y - point2.getY(), 2) + std::pow(z - point2.getZ(), 2));
}
