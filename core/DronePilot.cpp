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

using ctello::Tello;
using cv::VideoCapture;
using cv::VideoCaptureAPIs::CAP_FFMPEG;
using ORB_SLAM2::System;
using ctello::Tello;

DronePilot::DronePilot(const std::vector<std::string>& params)
: AbstractActivityHandler{params}
, telloStreamUrl{"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"}
{
	// TODO Auto-generated constructor stub

}

void DronePilot::sendACommand(const std::string &command) {
	tello.SendCommand(command);
	std::cout << "sent command " << command << std::endl;
	while (!(tello.ReceiveResponse())) {
		;
	}
	std::cout << "received response " << std::endl;
}

void DronePilot::run() {
	if (!tello.Bind()) {
		std::cout << "Failed to connect to drone. exiting app" << std::endl;
		exit(-1);
	}

	std::cout << "starting streaming on tello" << std::endl;
	sendACommand("setfps high");
	sendACommand("streamon");
	sendACommand("speed 30");

	System slam(
			"/local/RealTimeLearning/orb_slam/Vocabulary/ORBvoc.txt",
			"/local/RealTimeLearning/orb_slam/config/TELLO.yaml",
			ORB_SLAM2::System::MONOCULAR, true);

	std::cout << "tello streaming started" << std::endl;

	// Set to whatever video source
	VideoCapture capture { telloStreamUrl, CAP_FFMPEG };
	capture.set(CV_CAP_PROP_BUFFERSIZE, 5);
	double fps = capture.get(CV_CAP_PROP_FPS);
	bool lostTracking = false;
	bool droneLanded = false;
	int frameCount = 0;
	std::cout << fps << std::endl;

	// Drone control thread
	std::thread t([&]() {
		sendACommand("takeoff");
		sendACommand("up 20");
		lostTracking = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		int deg;
		// Turn on
		for (int i = 0; i < 12; ++i) {
			if (lostTracking) {
				std::cout << "Lost tracking trying to localize" << std::endl;
				sendACommand("cw 20");
				std::this_thread::sleep_for(std::chrono::milliseconds(700));
				sendACommand("back 35");
				std::this_thread::sleep_for(std::chrono::milliseconds(700));
				sendACommand("forward 35");
				std::this_thread::sleep_for(std::chrono::milliseconds(700));
				sendACommand("ccw 20");
				std::this_thread::sleep_for(std::chrono::milliseconds(700));
			}
			// Localizing help
			sendACommand("back 35");
			std::this_thread::sleep_for(std::chrono::milliseconds(700));
			sendACommand("forward 35");
			std::this_thread::sleep_for(std::chrono::milliseconds(700));

			/*            // Move left
			 sendACommand(tello, "right 30");
			 std::this_thread::sleep_for(std::chrono::milliseconds(1500));

			 // Move right
			 sendACommand(tello, "left 30");
			 std::this_thread::sleep_for(std::chrono::milliseconds(1500));*/

			// Turn left
			sendACommand("ccw 30");

			//Wait
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		}

		// take off
		sendACommand("land");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		droneLanded = true;
	});

	// Saving map thread
	std::thread t2([&]() {
				while (true) {
					std::getchar();
					std::cout << "EMERGENCYYYY" << std::endl;
					sendACommand("land");
					sendACommand("emergency");

//					CoordinatesCalculator::detectExitCoordinate(60, PointsVector);
//					auto pointsVector = saveMapToFile(slam);
//					std::cout << "Map saved" << std::endl;
//
//					Point3D exitCoordinate = getExitCoordinates(pointsVector);
//					std::cout << "flying to x = " << exitCoordinate.x
//							<< ", y = " << exitCoordinate.y() << ", z = "
//							<< exitCoordinate.z() << std::endl;
				}
			});

	while (true) {
		cv::Mat greyMat, colorMat;
		capture.grab();
		capture >> colorMat;
		//cv::cvtColor(colorMat, greyMat, CV_BGR2GRAY);
		if (!colorMat.empty()) {
			cv::Mat slamMatrix = slam.TrackMonocular(colorMat, frameCount / 30);
			if (slamMatrix.empty()) {
				std::cout << "You are fuckeeeedddddddddddd" << std::endl;
				lostTracking = true;
			} else {
				lostTracking = false;
			}
			frameCount++;
		}
	}

	/*// Drone control thread
	 std::thread escapeThread([&tello, &exitCoordinate]() {
	 sendACommand(tello, "takeoff");
	 std::string command = "go " + std::to_string(exitCoordinate.x()) + " " + std::to_string(exitCoordinate.y()) + " " + std::to_string(exitCoordinate.z()) + " " + std::to_string(10);
	 sendACommand(tello, "land");
	 });*/

}
