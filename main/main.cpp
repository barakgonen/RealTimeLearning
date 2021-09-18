#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "../drone_lib/include/ctello.h"

#include "../orb_slam/include/System.h"

#include <python3.8/Python.h>

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
const std::string pythonScriptPath = "/local/RealTimeLearning/main/Main.py";
int main(int argc, char **argv) {

	//Initialize the python instance
	Py_Initialize();
	FILE* PScriptFile = fopen(pythonScriptPath.c_str(), "r");
	if(PScriptFile){
		PyRun_SimpleFile(PScriptFile, pythonScriptPath.c_str());
		fclose(PScriptFile);
	}

    ORB_SLAM2::System System("/local/RealTimeLearning/orb_slam/Vocabulary/ORBvoc.txt", "/local/RealTimeLearning/orb_slam/config/TELLO.yaml", ORB_SLAM2::System::RGBD, true);
    //	 tello SHIT
    Tello tello { };
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
	}

	//Close the python instance
	Py_Finalize();
	return 0;
}
