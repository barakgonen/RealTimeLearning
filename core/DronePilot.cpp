/*
 * DronePilot.cpp
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#include "DronePilot.h"

#include <opencv2/videoio.hpp>
#include <iostream>

#include "../orb_slam/include/System.h"
#include "../drone_lib/include/ctello.h"
#include "../orb_slam/include/Converter.h"
#include "../orb_slam/include/Map.h"
#include "../orb_slam/include/MapPoint.h"
#include "CoordinatesCalculator.h"
#include "Point3D.h"

using ctello::Tello;
using cv::VideoCapture;
using cv::VideoCaptureAPIs::CAP_FFMPEG;
using ORB_SLAM2::System;
using ctello::Tello;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;

DronePilot::DronePilot(const std::vector<std::string> &params)
        : AbstractActivityHandler{params}, telloStreamUrl{"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"},
          slamMatrix{cv::Mat{}} {
}

void DronePilot::sendACommand(const std::string &command) {
    bool receievedResponse = false;
    while (!receievedResponse) {
        tello.SendCommand(command);
        std::cout << "A command sent " << command << std::endl;
        for (int i = 0; i < 40 && !receievedResponse; ++i) {
            if (tello.ReceiveResponse()) {
                receievedResponse = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    std::cout << "received response " << std::endl;
}

Point3D extractDroneLocation(cv::Mat slamMatrix){
    // Todo: Do the XYZ rotation matrix transformation to X axis angle
    return Point3D{0,0,0};
}

double extractDroneXAxisAngle(cv::Mat slamMatrix){
    // Todo: Do the XYZ rotation matrix transformation to X axis angle
    return 0;
}

double calculateVectorAngle(const Point3D &point) {
    if (point.getX() >= 0 && point.getZ() >= 0) {
        return std::atan(std::abs(point.getX() / point.getZ())) * 180 / 3.14;
    } else if (point.getX() <= 0 && point.getZ() >= 0) {
        return -std::atan(std::abs(point.getX() / point.getZ())) * 180 / 3.14;
    } else if (point.getX() <= 0 && point.getZ() <= 0) {
        return -(90 + std::atan(std::abs(point.getZ() / point.getX())) * 180 / 3.14);
    } else if (point.getX() >= 0 && point.getZ() <= 0) {
        return 90 + std::atan(std::abs(point.getZ() / point.getX())) * 180 / 3.14;
    } else {
        exit(-1);
    }
}

void DronePilot::run() {
    if (!tello.Bind()) {
        std::cout << "Failed to connect to drone. exiting app" << std::endl;
        exit(-1);
    }

    bool isFlying = false;
    std::cout << "starting streaming on tello" << std::endl;
    sendACommand("setfps high");
    sendACommand("streamon");
    sendACommand("speed 50");

    ORB_SLAM2::System slam{"/local/RealTimeLearning/orb_slam/Vocabulary/ORBvoc.txt",
                           "/local/RealTimeLearning/orb_slam/config/TELLO.yaml", ORB_SLAM2::System::MONOCULAR, true};
    cv::VideoCapture capture{telloStreamUrl, CAP_FFMPEG};
    capture.set(CV_CAP_PROP_BUFFERSIZE, 5);
    double fps = capture.get(CV_CAP_PROP_FPS);
    std::cout << "Fps rate: " << fps << std::endl;

    std::cout << "tello streaming started" << std::endl;

    bool lostTracking = false;
    bool isFinished = false;
    int frameCount = 0;

    // Slam thread
    std::thread slamThread([&]() {
        while (true) {
            cv::Mat greyMat, colorMat;
            capture.grab();
            capture >> colorMat;
            if (!colorMat.empty() && isFlying) {
                slamMatrix = slam.TrackMonocular(colorMat, frameCount / fps);
                if (slamMatrix.empty()) {
                    lostTracking = true;
                } else {
                    lostTracking = false;
                }
                frameCount++;
            }
        }
    });

    // Saving map thread
    std::thread t2([&]() {
        while (true) {
            std::getchar();
            std::cout << "EMERGENCYYYY" << std::endl;
            sendACommand("land");
            sendACommand("emergency");

        }
    });

    // Drone control thread
    sendACommand("takeoff");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    isFlying = true;
    sendACommand("up 30");
    lostTracking = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    while (lostTracking) {
        std::cout << "Trying to initialize" << std::endl;
        sendACommand("up 30");
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        sendACommand("down 30");
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        if (!slamMatrix.empty()) {
            std::cout << slamMatrix << std::endl;
        }
    }

    std::cout << "Orbslam Initialized." << std::endl;

    int fxDeg;
    // Turn on
    for (int i = 0; i < 12; ++i) {
        if (!slamMatrix.empty()) {
            std::cout << slamMatrix << std::endl;
        }

        while (lostTracking) {
            std::cout << "lost tracking, trying to relocalize." << std::endl;
            int j;
            for (j = 0; j < i && lostTracking; ++j) {
                sendACommand("cw 20");
                std::this_thread::sleep_for(std::chrono::milliseconds(700));
                sendACommand("up 30");
                std::this_thread::sleep_for(std::chrono::milliseconds(700));
                sendACommand("down 30");
            }

            for (int k = 0; k < j; ++k) {
                sendACommand("ccw 20");
                std::this_thread::sleep_for(std::chrono::milliseconds(700));
            }
        }

        // Localizing help
        sendACommand("up 30");
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        sendACommand("down 30");
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        // Turn left
        sendACommand("ccw 30");

        //Wait
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    sendACommand("land");

    bool isExitPointCalculated = false;
    std::thread t([&]() {
        while (!isExitPointCalculated) {
            tello.GetState();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });

    // Sleeping till the slam ends mapping
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    std::cout << "calculating exit point" << std::endl;
    auto pointsVector = transformMapFromSlamToRegularPoint(slam);
    exportAllPointsToFile(pointsVector);
    std::cout << "Map Points Saved." << std::endl;
    auto exitPointWithsd = CoordinatesCalculator::detectExitCoordinateWithSd(numberOfPointsForFiltering,
                                                                        pointsVector,
                                                                        isMulti,
                                                                        isLoggerOn);
    std::cout << "calculated exit point" << std::endl;

    Point3D exitPoint = exitPointWithsd.first;
    auto sd = exitPointWithsd.first;
    isExitPointCalculated = true;

    sendACommand("takeoff");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Initializing to get the drone slam real angle
    while (lostTracking) {
        std::cout << "Trying to initialize" << std::endl;
        sendACommand("up 30");
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        sendACommand("down 30");
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        if (!slamMatrix.empty()) {
            std::cout << slamMatrix << std::endl;
        }
    }

    auto droneLocation = extractDroneLocation(slamMatrix);

    auto droneExitVector = Point3D{exitPoint.getX()-droneLocation.getX(),
                                   exitPoint.getY()-droneLocation.getY(),
                                   exitPoint.getZ()-droneLocation.getZ() };
    int angle =(int)(std::floor(calculateVectorAngle(droneExitVector)) - extractDroneXAxisAngle(slamMatrix));

    std::cout << "flying to x = " << exitPoint.getX() << ", y = " << exitPoint.getY() << ", z = "
              << exitPoint.getZ() << std::endl;
    std::cout << "Exit angle: " << angle << std::endl;

    if (angle > 0) {
        sendACommand("cw " + std::to_string(angle));
    } else {
        sendACommand("ccw " + std::to_string(std::abs(angle)));
    }

    // Time to rotate and fix delay
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    double sdScale = 2;
    double deltaX = 0;
    double deltaZ = 0;
    do {
        std::cout << "Moving forward, you are still far from your exit" << std::endl;
        sendACommand("forward 40");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto droneLocation = extractDroneLocation(slamMatrix);
        deltaX = std::abs(exitPoint.getX() - droneLocation.getX());
        deltaZ = std::abs(exitPoint.getZ() - droneLocation.getZ());
    } while (!(deltaX <=sd.getX() * sdScale && deltaZ <= sd.getZ() * sdScale));

    std::cout << "You have arrived!!," << std::endl;

    sendACommand("emergency");

    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    isFinished = true;
}

void DronePilot::exportAllPointsToFile(const std::vector<Point3D> &pointsVec) {
    std::ofstream pointData;
    pointData.open("/local/rawData.csv");
    for (const auto &point: pointsVec)
        pointData << point.getX() << "," << point.getY() << "," << point.getZ() << "\n";

    pointData.close();
}

std::vector<Point3D> DronePilot::transformMapFromSlamToRegularPoint(ORB_SLAM2::System &slam) {
    std::vector<Point3D> pointsVector;
    for (auto p: slam.GetMap()->GetAllMapPoints()) {
        if (p != NULL) {
            auto point = p->GetWorldPos();
            pointsVector.push_back({ORB_SLAM2::Converter::toVector3d(p->GetWorldPos())});
        }
    }

    return pointsVector;
}
