#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

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
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

int main(int argc, char **argv) {

//	 tello SHIT
    ORB_SLAM2::System System("/local/RealTimeLearning/orb_slam/Vocabulary/ORBvoc.txt", "/local/RealTimeLearning/orb_slam/config/TELLO.yaml", ORB_SLAM2::System::RGBD, true);


    std::cout << "BG DONE" << std::endl;
/*    Tello tello { };
    if (!tello.Bind()) {
        return 0;
    }

	tello.SendCommand("streamon");
	while (!(tello.ReceiveResponse()))
		;

	VideoCapture capture { TELLO_STREAM_URL, CAP_FFMPEG };
    capture.set(CV_CAP_PROP_BUFFERSIZE, 3);

    while (true) {
		Mat frame;
        capture.grab();
		capture >> frame;

		resize(frame, frame, Size(), 0.75, 0.75);
		imshow("CTello Stream", frame);
		if (waitKey(1) == 27) {
			break;
		}
	}*/

	return 0;
}
