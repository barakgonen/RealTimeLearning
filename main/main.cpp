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

#include "../drone_lib/include/ctello.h"

#include "../orb_slam/include/System.h"

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

int main(int argc, char **argv) {

   Tello tello{};
    if (!tello.Bind()) {
        return 0;
    }

    std::cout << "starting streaming on tello" << std::endl;
    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse())) { ; }

    ORB_SLAM2::System slam("/local/RealTimeLearning/orb_slam/Vocabulary/ORBvoc.txt",
                           "/local/RealTimeLearning/orb_slam/config/TELLO.yaml",
						   ORB_SLAM2::System::MONOCULAR, true);


    std::cout << "tello streaming started" << std::endl;

    // Set to whatever video source
    VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};
    capture.set(CV_CAP_PROP_BUFFERSIZE, 5);
    double fps = capture.get(CV_CAP_PROP_FPS);

    int frameCount = 0;
    std::cout << "bla" << std::endl;
    std::thread t([&tello](){
        tello.SendCommand("takeoff");
        while (!(tello.ReceiveResponse())) { ; }
        tello.SendCommand("forward 30");
        while (!(tello.ReceiveResponse())) { ; }
        tello.SendCommand("cw 90");
        tello.SendCommand("right 30");
        while (!(tello.ReceiveResponse())) { ; }
        while (!(tello.ReceiveResponse())) { ; }
        tello.SendCommand("cw 90");
        while (!(tello.ReceiveResponse())) { ; }
        tello.SendCommand("land");
        while (!(tello.ReceiveResponse())) { ; }
    });


    while (true) {
        cv::Mat colorMat;
        capture.grab();
        capture >> colorMat;
        if(!colorMat.empty()) {
            slam.TrackMonocular(colorMat, frameCount / fps);
            frameCount++;
        }
    }

    std::cout << "DONE" << std::endl;
    return 0;
}
