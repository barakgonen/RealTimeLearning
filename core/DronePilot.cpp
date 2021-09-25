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
    bool droneLanded = false;
    int frameCount = 0;

    // Slam thread
    std::thread slamThread([&]() {
        while (!droneLanded) {
            cv::Mat greyMat, colorMat;
            capture.grab();
            capture >> colorMat;
            if (!colorMat.empty() && isFlying) {
                slamMatrix = slam.TrackMonocular(colorMat, frameCount / 30);
                if (slamMatrix.empty()) {
                    std::cout << "Lost Tracking" << std::endl;
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
    std::thread t([&]() {
        sendACommand("takeoff");
        isFlying = true;
        sendACommand("up 20");
        lostTracking = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        int deg;
        // Turn on
        for (int i = 0; i < 12; ++i) {
            if (!slamMatrix.empty()) {
                std::cout << slamMatrix << std::endl;
            }

            if (lostTracking) {
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

        for (int i = 0; i < 10; i++) {
            tello.GetState();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        std::cout << "calculating exit point" << std::endl;
        auto pointsVector = transformMapFromSlamToRegularPoint(slam);
        auto exitPoint = CoordinatesCalculator::detectExitCoordinate(numberOfPointsForFiltering, pointsVector, isMulti,
                                                                     isLoggerOn);

        std::cout << "flying to x = " << exitPoint.getX() << ", y = " << exitPoint.getY() << ", z = "
                  << exitPoint.getZ() << std::endl;
        sendACommand("takeoff");

        int turnAngle = std::floor(std::atan(exitPoint.getY() / exitPoint.getX()) * 180 / 3.14);

        std::cout << "angle to exit: " << turnAngle << std::endl;
        if (turnAngle >= 0) {
            sendACommand("ccw " + std::to_string(turnAngle));
        } else {
            sendACommand("cw " + std::to_string(-turnAngle));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        for (int i = 0; i < 3; ++i) {
            sendACommand("forward 30");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        sendACommand("Land");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        droneLanded = true;
    });

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
