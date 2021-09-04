#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "../core/Point.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <iostream>
#include <optional>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include <memory>

#include "../orb_slam/include/Map.h"
#include "../orb_slam/include/MapDrawer.h"
//#include "../tello/inc/tello.hpp"
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
using ORB_SLAM2::Map;
using ORB_SLAM2::MapDrawer;
//using ORB_SLAM2::Map;
//using ORB_SLAM2::MapDrawer;

int main(int argc, char **argv) {
	cout << "Hello bgbg! VFVF " << std::endl;

	// Using of core - like our future algorithms
	Point p1{12121, 21212};
	p1.printPoint();


	// Opencv shit
	Mat image = imread("/local/RealTimeLearning/basketball.jpg");
	std::cout << "Image cols: " << image.cols << std::endl;
	std::cout << "image.size.p: " << image.size.p << std::endl;

	// ORB_SLAM2 shit
	Map map;
	const std::string pattern = "ppppppp";
	MapDrawer mapDrawer{&map, pattern};

	mapDrawer.DrawMapPoints();

	std::cout << "mapDrawer.mpMap->GetMaxKFid(): " << mapDrawer.mpMap->GetMaxKFid() << std::endl;

	// CPP 14 stuff

	auto lambda = [](auto x, auto y) {return x + y;};
	std::cout << "How much is 2 + 3? " << lambda(2,3) << std::endl;
	// tello SHIT
//	asio::io_service io_service;
//	  asio::io_service::work work(io_service);
//
//	  std::condition_variable cv_run;

//	#ifdef USE_CONFIG
//
//	  std::map<std::string, std::unique_ptr<Tello>>  m = handleConfig("../config.yaml", io_service, cv_run);
//
//	  if(m.count("0.prime.0") > 0){
//	    Tello& t = *m["0.prime.0"];
//	    t.cs->addCommandToQueue("command");
//	    t.cs->addCommandToQueue("sdk?");
//	    t.cs->addCommandToQueue("command");
//	    t.cs->addCommandToQueue("sdk?");
//	    t.cs->addCommandToQueue("streamon");
//	    t.cs->addCommandToQueue("takeoff");
//	    t.cs->executeQueue();
//	    t.cs->addCommandToQueue("forward 20");
//	    t.cs->addCommandToQueue("back 20");
//	    t.cs->addCommandToQueue("delay 5");
//	    t.cs->addCommandToFrontOfQueue("stop");
//	    // t.cs->stopQueueExecution();
//	    t.cs->doNotAutoLand();
//	    t.cs->addCommandToQueue("land");
//	  }
//	  else{
//	    utils_log::LogErr() << "The requested drone does not exist.";
//	  }
//
//	#else
//
//	  Tello t(io_service, cv_run, "192.168.10.1", "8889", "11111", "8890", "../camera_config.yaml", "../orb_vocab.dbow2");
//
//	  t.cs->addCommandToQueue("command");
//	  t.cs->addCommandToQueue("sdk?");
//	  t.cs->addCommandToQueue("streamon");
//	  t.cs->addCommandToQueue("takeoff");
//	  t.cs->executeQueue();
//	  t.cs->addCommandToQueue("forward 20");
//	  t.cs->addCommandToQueue("back 20");
//	  t.cs->addCommandToQueue("delay 5");
//	  t.cs->addCommandToFrontOfQueue("stop");
//	  // t.cs->stopQueueExecution();
//	  t.cs->doNotAutoLand();
//	  t.cs->addCommandToQueue("land");
//
//	#endif
//
//	  {
//	    std::mutex mtx;
//	    std::unique_lock<std::mutex> lck(mtx);
//	    cv_run.wait_for(lck,std::chrono::seconds(300));
//	  }
//
//	  utils_log::LogWarn() << "----------- Done -----------";
//	  utils_log::LogWarn() << "----------- Landing -----------";
//	  // t.cs->exitAllThreads();
//	  io_service.stop();
//	  usleep(1000000); // Ensure this is greater than timeout to prevent seg faults
//	  utils_log::LogDebug() << "----------- Main thread returns -----------";
	return 0;
}
