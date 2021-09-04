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
	Point(int x, int y);
	virtual ~Point();

	void printPoint();

protected:
	int x;
	int y;

};

#endif /* INCLUDE_POINT_H_ */
