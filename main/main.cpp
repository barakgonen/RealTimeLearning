#include<chrono>
#include <iostream>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../core/CoordinatesCalculator.h"
#include "../core/Utils.h"
#include "../drone_lib/include/ctello.h"

#include "../orb_slam/include/System.h"
#include "../orb_slam/include/Converter.h"

using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::line;
using cv::Mat;
using cv::Point2i;
using cv::resize;
using cv::Size;
using cv::Vec3b;
using cv::VideoCapture;
using cv::waitKey;

using namespace std;

using cv::CAP_FFMPEG;
using cv::imread;
using cv::imshow;
using cv::line;
using cv::Mat;
using cv::Point2i;
using cv::resize;
using cv::Size;
using cv::Vec3b;
using cv::VideoCapture;
using g2o::VertexSim3Expmap;

// URL where the Tello sends its video stream to.
const char *const TELLO_STREAM_URL {
		"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000" };

void sendACommand(Tello &tello, const std::string &command) {
	tello.SendCommand(command);
	std::cout << "sent command " << command << std::endl;
	while (!(tello.ReceiveResponse())) {
		;
	}
	std::cout << "received response " << std::endl;
}

POINT getExitCoordinates(const std::vector<POINT> &PointsVector) {
	CoordinatesCalculator calculator;
	return calculator.detectExitCoordinate(60, PointsVector);
}

int main(int argc, char **argv) {
	Tello tello { };
	if (!tello.Bind()) {
		return 0;
	}

	std::cout << "starting streaming on tello" << std::endl;
	sendACommand(tello, "setfps high");
	sendACommand(tello, "streamon");
	sendACommand(tello, "speed 30");

	ORB_SLAM2::System slam(
			"/local/RealTimeLearning/orb_slam/Vocabulary/ORBvoc.txt",
			"/local/RealTimeLearning/orb_slam/config/TELLO.yaml",
			ORB_SLAM2::System::MONOCULAR, true);

	std::cout << "tello streaming started" << std::endl;

	// Set to whatever video source
	VideoCapture capture { TELLO_STREAM_URL, CAP_FFMPEG };
	capture.set(CV_CAP_PROP_BUFFERSIZE, 5);
	double fps = capture.get(CV_CAP_PROP_FPS);
	bool lostTracking = false;
	bool droneLanded = false;
	int frameCount = 0;
	std::cout << fps << std::endl;

	// Drone control thread
	std::thread t([&tello, &lostTracking, &slam, &droneLanded]() {
		sendACommand(tello, "takeoff");
		sendACommand(tello, "up 20");
		lostTracking = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		int deg;
		// Turn on
		for (int i = 0; i < 12; ++i) {
			if (lostTracking) {
				std::cout << "Lost tracking trying to localize" << std::endl;
				sendACommand(tello, "cw 20");
				std::this_thread::sleep_for(std::chrono::milliseconds(700));
				sendACommand(tello, "back 35");
				std::this_thread::sleep_for(std::chrono::milliseconds(700));
				sendACommand(tello, "forward 35");
				std::this_thread::sleep_for(std::chrono::milliseconds(700));
				sendACommand(tello, "ccw 20");
				std::this_thread::sleep_for(std::chrono::milliseconds(700));
			}
			// Localizing help
			sendACommand(tello, "back 35");
			std::this_thread::sleep_for(std::chrono::milliseconds(700));
			sendACommand(tello, "forward 35");
			std::this_thread::sleep_for(std::chrono::milliseconds(700));

			/*            // Move left
			 sendACommand(tello, "right 30");
			 std::this_thread::sleep_for(std::chrono::milliseconds(1500));

			 // Move right
			 sendACommand(tello, "left 30");
			 std::this_thread::sleep_for(std::chrono::milliseconds(1500));*/

			// Turn left
			sendACommand(tello, "ccw 30");

			//Wait
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		}

		// take off
		sendACommand(tello, "land");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		droneLanded = true;
	});

	// Saving map thread
	std::thread t2(
			[&slam, &tello]() {
				while (true) {
					std::getchar();
					std::cout << "EMERGENCYYYY" << std::endl;
					tello.SendCommand("land");
					tello.SendCommand("emergency");

					auto pointsVector = saveMapToFile(slam);
					std::cout << "Map saved" << std::endl;

					POINT exitCoordinate = getExitCoordinates(pointsVector);
					std::cout << "flying to x = " << exitCoordinate.x()
							<< ", y = " << exitCoordinate.y() << ", z = "
							<< exitCoordinate.z() << std::endl;
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

	return 0;
}
