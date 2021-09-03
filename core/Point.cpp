/*
 * Point.cpp
 *
 *  Created on: Sep 3, 2021
 *      Author: barakg
 */

#include "Point.h"

#include <iostream>
Point::Point(int x, int y)
: x{x}
, y{y}
{
	// TODO Auto-generated constructor stub

}

void Point::printPoint() {
	std::cout << "x = " << x << ", y = " << y << std::endl;
}
Point::~Point() {
	// TODO Auto-generated destructor stub
}

