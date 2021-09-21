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
const char *const TELLO_STREAM_URL{"udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"};

void sendACommand(Tello &tello, const std::string &command);

void saveMap(ORB_SLAM2::System &SLAM) {
    std::vector<ORB_SLAM2::MapPoint *> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/RoomCoordiantes.csv");
    std::vector<Eigen::Matrix<double,3,1>> pointsVector;
    for (auto p: mapPoints) {
        if (p != NULL) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z() << std::endl;
            pointsVector.push_back(v);
        }
    }
    pointData.close();
}

Point getExitCoordinates(ORB_SLAM2::System &SLAM){
    std::vector<ORB_SLAM2::MapPoint *> mapPoints = SLAM.GetMap()->GetAllMapPoints();

    std::vector<Eigen::Matrix<double,3,1>> pointsVector;
    CoordinatesCalculator calculator;

    for (auto p: mapPoints) {
        if (p != NULL) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointsVector.push_back(v);
        }
    }
    return calculator.DetectExitCoordinates(60, pointsVector);
}

int main(int argc, char **argv) {
    Tello tello{};
    if (!tello.Bind()) {
        return 0;
    }

    std::cout << "starting streaming on tello" << std::endl;
    sendACommand(tello, "setfps high");
    sendACommand(tello, "streamon");
    sendACommand(tello, "speed 30");

    ORB_SLAM2::System slam("/local/RealTimeLearning/orb_slam/Vocabulary/ORBvoc.txt",
                           "/local/RealTimeLearning/orb_slam/config/TELLO.yaml",
                           ORB_SLAM2::System::MONOCULAR, true);

    std::cout << "tello streaming started" << std::endl;

    // Set to whatever video source
    VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};
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
            if(lostTracking){
                std::cout << "Lost tracking trying to localize" << std::endl;
                sendACommand(tello, "cw 20");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                sendACommand(tello, "back 35");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                sendACommand(tello, "forward 35");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                sendACommand(tello, "ccw 20");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            // Localizing help
            sendACommand(tello, "back 35");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            sendACommand(tello, "forward 35");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

/*            // Move left
            sendACommand(tello, "right 30");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));

            // Move right
            sendACommand(tello, "left 30");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));*/

            // Turn left
            sendACommand(tello, "ccw 30");

            //Wait
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        }

        // take off
        tello.SendCommand("land");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        droneLanded = true;
    });


    // Saving map thread
    std::thread t2([&slam, &tello]() {
        while (true) {
            std::getchar();
            std::cout << "EMERGENCYYYY" << std::endl;
            tello.SendCommand("land");
            tello.SendCommand("emergency");
            saveMap(slam);
        }
    });

    while (!droneLanded) {
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

    Point exitCoordinates = getExitCoordinates(slam);

    // TODO: Fly to the exit

    return 0;
}

void sendACommand(Tello &tello, const std::string &command) {
    tello.SendCommand(command);
    std::cout << "sent command " << command << std::endl;
    while (!(tello.ReceiveResponse())) { ; }
    std::cout << "received response " << std::endl;
}

