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
    tello.SendCommand(command);
    std::cout << "sent command " << command << std::endl;
    while (!(tello.ReceiveResponse())) { ;
    }
    std::cout << "received response " << std::endl;
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
//    std::thread t2([&]() {
//        while (true) {
//            std::getchar();
//            std::cout << "EMERGENCYYYY" << std::endl;
//            sendACommand("land");
//            sendACommand("emergency");
//
//        }
//    });

    // Drone control thread
    sendACommand("takeoff");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    sendACommand("up 30");
    lostTracking = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    isFlying = true;


    while(lostTracking){
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

    int deg;
    // Turn on
    for (int i = 0; i < 12; ++i) {
        if (!slamMatrix.empty()) {
            std::cout << slamMatrix << std::endl;
        }

        while (lostTracking) {
            std::cout << "Lost tracking trying to localize" << std::endl;
            sendACommand("cw 20");
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
            sendACommand("up 30");
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
            sendACommand("down 30");
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
            sendACommand("ccw 20");
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    std::cout << "calculating exit point" << std::endl;
    auto pointsVector = transformMapFromSlamToRegularPoint(slam);
    exportAllPointsToFile(pointsVector);
    std::cout << "Map Points Saved." << std::endl;
    auto exitPointt = CoordinatesCalculator::detectExitCoordinateWithSd(numberOfPointsForFiltering, pointsVector, isMulti,
                                                                 isLoggerOn);

    Point3D exitPoint = exitPointt.first;
    isExitPointCalculated = true;
    
    sendACommand("takeoff");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    double angle = std::floor(getExitCoordinatesDegree(exitPoint));

    std::cout << "flying to x = " << exitPoint.getX() << ", y = " << exitPoint.getY() << ", z = "
              << exitPoint.getZ() << std::endl;
    std::cout << "Exit angle: " << angle << std::endl;

    if (angle > 0) {
        sendACommand("cw " + std::to_string(angle));
    } else {
        sendACommand("ccw " + std::to_string(std::abs(angle)));
    }

    for (int i = 0; i < 5; ++i) {
        sendACommand("forward 40");
    }
    sendACommand("emergency");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    isFinished = true;
}

double DronePilot::getExitCoordinatesDegree(const Point3D &point) {
    if (point.getX() >= 0 && point.getZ() >= 0)
    {
        return std::atan(std::abs(point.getX() / point.getZ())) * 180 / 3.14;
    } else if(point.getX() <= 0 && point.getZ() >= 0)
    {
        return -std::atan(std::abs(point.getX() / point.getZ())) * 180 / 3.14;
    }
    else if (point.getX() <= 0 && point.getZ() <= 0)
    {
        return -(90 + std::atan(std::abs(point.getX() / point.getZ())) * 180 / 3.14);
    }
    else if (point.getX() >= 0 && point.getZ() <= 0)
    {
        return 90 + std::atan(std::abs(point.getX() / point.getZ())) * 180 / 3.14;
    }
    else
    {
        exit(-1);
    }
}

void DronePilot::exportCalculatedResultsToFile(const std::pair<Point3D, Point3D>& exitPointt) {
    std::ofstream pointData;
    pointData.open("/tmp/exitPointFile");
    pointData << "CalculatedExit:\n" << exitPointt.first.getX() << "," << exitPointt.first.getY() << "," << exitPointt.first.getZ() << "\n";
    pointData << "SD:\n" << exitPointt.second.getX() << "," << exitPointt.second.getY() << "," << exitPointt.second.getZ() << "\n";

    pointData.close();
}

void DronePilot::exportAllPointsToFile(const std::vector<Point3D>& pointsVec){
    std::ofstream pointData;
    pointData.open("/local/rawData.csv");
    for (const auto& point : pointsVec)
        pointData <<  point.getX() << "," << point.getY() << "," << point.getZ() << "\n";

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
