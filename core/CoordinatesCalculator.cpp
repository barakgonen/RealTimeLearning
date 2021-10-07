/*
 * CoordinatesCalculator.cpp
 *
 *  Created on: Sep 18, 2021
 *      Author: barakg
 */

#include <map>
#include <vector>
#include <iostream>

#include "CoordinatesCalculator.h"

#include <thread>
#include <utility>

using namespace std;

void calculate_distance_between_points_in_range(
        const std::vector<Point3D> &pointsInMapping, int startIndex,
        int endIndex, std::map<double, const Point3D> &distanceToPoint,
        int threadNumber, bool isMulti, bool printsEnabld) {
    if (printsEnabld) {
        std::cout
                << "<CoordinatesCalculator::calculate_distance_between_points_in_range()> start index = "
                << startIndex << ", end index = " << endIndex
                << ", thread number = " << threadNumber << ", is multi? "
                << std::boolalpha << isMulti << std::endl;
    }

    double rangesSum = 0;
    for (int point1Index = startIndex; point1Index < endIndex; point1Index++) {
        rangesSum = 0;
        auto point1 = pointsInMapping.at(point1Index);
        for (const auto &point2: pointsInMapping) {
            // Adding the current coordinate range to the main coordinate ranges sum
            rangesSum += std::sqrt(
                    std::pow(point1.getX() - point2.getX(), 2)
                    + std::pow(point1.getY() - point2.getY(), 2)
                    + std::pow(point1.getZ() - point2.getZ(), 2));
        }
        distanceToPoint.insert(
                std::pair<double, const Point3D>(rangesSum, point1));

        if (printsEnabld && distanceToPoint.size() % 1000 == 0) {
            if (isMulti) {
                std::cout
                        << "<CoordinatesCalculator::calculate_distance_between_points_in_range()> Thread num: "
                        << threadNumber << ", " << distanceToPoint.size()
                        << " coordinates has been processed." << std::endl;

            } else {
                std::cout
                        << "<CoordinatesCalculator::calculate_distance_between_points_in_range()> "
                        << distanceToPoint.size()
                        << " coordinates has been processed." << std::endl;
            }
        }
    }
}

Point3D CoordinatesCalculator::detectExitCoordinate(int numberOfPointsToFilter,
                                                    const std::vector<Point3D> &mappedPointsFromSensor, bool isMulti,
                                                    const bool printsEnabled) {
    if (isMulti)
        return detectExitCoordianteMulti(numberOfPointsToFilter, mappedPointsFromSensor, printsEnabled);
    else
        return detectExitCoordianteSingle(numberOfPointsToFilter, mappedPointsFromSensor, printsEnabled);
}

std::pair<Point3D, Point3D> CoordinatesCalculator::detectExitCoordinateWithSd(const int numberOfPointsToFilter,
                                                                              const std::vector<Point3D> &mappedPointsFromSensor,
                                                                              bool isMulti, const bool printsEnabled) {

    std::map<double, const Point3D> multiMap;
    const size_t nthreads = std::thread::hardware_concurrency();
    if (printsEnabled) {
        std::cout << "parallel (" << nthreads << " threads):" << std::endl;
    }

    std::vector<std::thread> workers(nthreads);
    const auto startMulti = std::chrono::system_clock::now();

    int batchSize = mappedPointsFromSensor.size() / workers.size();
    for (int threadNum = 0; threadNum < nthreads; threadNum++) {
        int startIdex = threadNum * batchSize;
        int endIndex = startIdex + batchSize;
        workers.push_back(
                std::thread(calculate_distance_between_points_in_range,
                            mappedPointsFromSensor, startIdex, endIndex,
                            std::ref(multiMap), threadNum, true, printsEnabled));
    }
    for (std::thread &t: workers) {
        if (t.joinable()) {
            t.join();
        }
    }

    std::cout << "Number of points in map: " << multiMap.size() << std::endl;
    int numberOfPointsProcessed = 0;
    double xSum, ySum, zSum = 0;

    for (auto iter = multiMap.rbegin();
         iter != multiMap.rend() && numberOfPointsProcessed < numberOfPointsToFilter;
         ++iter) {
        xSum += iter->second.getX();
        ySum += iter->second.getY();
        zSum += iter->second.getZ();
        numberOfPointsProcessed++;
    }

    Point3D simpleAvg{xSum / numberOfPointsProcessed, ySum / numberOfPointsProcessed, zSum / numberOfPointsProcessed};
    // Standard deviation calculation
    double xSD = 0;
    double ySD = 0;
	double zSD = 0;

    numberOfPointsProcessed = 0;
    for (auto iter = multiMap.rbegin();
         iter != multiMap.rend() && numberOfPointsProcessed < numberOfPointsToFilter;
         ++iter) {
        xSD += std::pow(iter->second.getX() - simpleAvg.getX(), 2);
        ySD += std::pow(iter->second.getY() - simpleAvg.getY(), 2);
        zSD += std::pow(iter->second.getZ() - simpleAvg.getZ(), 2);
        numberOfPointsProcessed++;
    }

    std::vector<Point3D> cleanPoints;
    Point3D sd{std::sqrt(xSD / numberOfPointsProcessed), std::sqrt(ySD / numberOfPointsProcessed),
               std::sqrt(zSD / numberOfPointsProcessed)};

    // Filter by sd with scaling
    for (double sdScale = 1.1; sdScale <= 1000 && cleanPoints.empty(); ++sdScale) {
        numberOfPointsProcessed = 0;
        std::cout << "sd scale: " << sdScale << std::endl;
        for (auto iter = multiMap.rbegin();
             iter != multiMap.rend() && numberOfPointsProcessed < numberOfPointsToFilter;
             ++iter) {
            if ((std::abs(iter->second.getX() - simpleAvg.getX()) <= sd.getX() * sdScale) &&
                (std::abs(iter->second.getZ() - simpleAvg.getZ()) <= sd.getZ() * sdScale)) {
                cleanPoints.push_back(iter->second);
            }
            numberOfPointsProcessed++;
        }
    }

    xSum = 0;
    ySum = 0;
    zSum = 0;
    // Calculating avg after sd filterring
    for (const auto p: cleanPoints) {
        xSum += p.getX();
        ySum += p.getY();
        zSum += p.getZ();
    }

    std::cout << "Clean points size: " << cleanPoints.size() << std::endl;
    if (cleanPoints.size() == 0) {
        return std::make_pair<Point3D, Point3D>({simpleAvg.getX(), simpleAvg.getY(), simpleAvg.getZ()},
                                                {sd.getX(), sd.getY(), sd.getZ()});
    }
    // Averaging the final sum and constructing exit point
    return std::make_pair<Point3D, Point3D>(
            {xSum / cleanPoints.size(), ySum / cleanPoints.size(), zSum / cleanPoints.size()},
            {sd.getX(), sd.getY(), sd.getZ()});

}

Point3D CoordinatesCalculator::calculate_exit_point(
        const std::map<double, const Point3D> &distanceToPoint, int n) {
    std::cout << "Number of points in map: " << distanceToPoint.size() << std::endl;
    int numberOfPointsProcessed = 0;
    double xSum, ySum, zSum = 0;

    for (auto iter = distanceToPoint.rbegin();
         iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
         ++iter) {
        xSum += iter->second.getX();
        ySum += iter->second.getY();
        zSum += iter->second.getZ();
        numberOfPointsProcessed++;
    }

    Point3D simpleAvg{xSum / n, ySum / n, zSum / n};

    std::cout<< "Average 60 farest points: " << simpleAvg.getX() << ", " << simpleAvg.getY() << ", " << simpleAvg.getZ();

    // Standard deviation calculation
    double xSD, ySD, zSD = 0;
    numberOfPointsProcessed = 0;
    for (auto iter = distanceToPoint.rbegin();
         iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
         ++iter) {
        xSD += std::pow(iter->second.getX() - simpleAvg.getX(), 2);
        ySD += std::pow(iter->second.getY() - simpleAvg.getY(), 2);
        zSD += std::pow(iter->second.getZ() - simpleAvg.getZ(), 2);
        numberOfPointsProcessed++;
    }

    std::vector<Point3D> cleanPoints;
    Point3D sd{std::sqrt(xSD / n), std::sqrt(ySD / n), std::sqrt(zSD / n)};
    numberOfPointsProcessed = 0;
    for (auto iter = distanceToPoint.rbegin();
         iter != distanceToPoint.rend() && numberOfPointsProcessed < n;
         ++iter) {
        if ((std::sqrt(std::pow(iter->second.getX() - simpleAvg.getX(), 2)) <= sd.getX())
            && (std::sqrt(std::pow(iter->second.getY() - simpleAvg.getY(), 2)) <= sd.getY())
            && (std::sqrt(std::pow(iter->second.getZ() - simpleAvg.getZ(), 2)) <= sd.getZ())) {
            cleanPoints.push_back(iter->second);
        }
        numberOfPointsProcessed++;
    }

    xSum = 0;
    ySum = 0;
    zSum = 0;
    // Calculating avg after cleanup
    for (const auto p: cleanPoints) {
        xSum += p.getX();
        ySum += p.getY();
        zSum += p.getZ();
    }

    std::cout << "Clean points size: " << cleanPoints.size() << std::endl;
    if (cleanPoints.size() == 0) {
        return simpleAvg;
    }
    // Averaging the final sum and constructing exit point
    return {xSum / cleanPoints.size(), ySum / cleanPoints.size(), zSum / cleanPoints.size()};
}

Point3D CoordinatesCalculator::detectExitCoordianteMulti(int numberOfPointsToFilter,
                                                         const std::vector<Point3D> &mappedPointsFromSensor,
                                                         const bool printsEnabled) {
    std::map<double, const Point3D> multiMap;
    const size_t nthreads = std::thread::hardware_concurrency();
    if (printsEnabled) {
        std::cout << "parallel (" << nthreads << " threads):" << std::endl;
    }

    std::vector<std::thread> workers(nthreads);
    const auto startMulti = std::chrono::system_clock::now();

    int batchSize = mappedPointsFromSensor.size() / workers.size();
    for (int threadNum = 0; threadNum < nthreads; threadNum++) {
        int startIdex = threadNum * batchSize;
        int endIndex = startIdex + batchSize;
        workers.push_back(
                std::thread(calculate_distance_between_points_in_range,
                            mappedPointsFromSensor, startIdex, endIndex,
                            std::ref(multiMap), threadNum, true, printsEnabled));
    }
    for (std::thread &t: workers) {
        if (t.joinable()) {
            t.join();
        }
    }
    return calculate_exit_point(multiMap, numberOfPointsToFilter);
}

Point3D CoordinatesCalculator::detectExitCoordianteSingle(int numberOfPointsToFilter,
                                                          const std::vector<Point3D> &mappedPointsFromSensor,
                                                          bool printsEnabled) {
    std::map<double, const Point3D> singleMap;
    calculate_distance_between_points_in_range(mappedPointsFromSensor, 0, mappedPointsFromSensor.size(), singleMap, 0,
                                               false, printsEnabled);
    return calculate_exit_point(singleMap, numberOfPointsToFilter);
}
