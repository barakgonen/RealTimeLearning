/*
 * Point3D.h
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#ifndef CORE_POINT3D_H_
#define CORE_POINT3D_H_

#include <opencv2/core/types.hpp>
#include <eigen3/Eigen/Core>

class Point3D
{
public:
	Point3D(const double _x, const double _y, const double _z);
    Point3D(const Eigen::Matrix<double,3,1>& eigenPoint);

    ~Point3D() = default;

    bool operator< (const Point3D &rhs) const;
    bool operator==(const Point3D& rhs) const;
    friend std::ostream& operator<<(std::ostream& os, const Point3D& c);
    double getX() const;
    double getY() const;
    double getZ() const;
    double getDistance(const Point3D& point2) const;
    double getHash() const;

private:
    const double x;
    const double y;
    const double z;
    const double hash;
};

#endif /* CORE_POINT3D_H_ */
