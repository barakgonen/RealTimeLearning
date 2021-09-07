#include <boost/optional/optional.hpp>
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

#include "../drone_lib/include/ctello.h"
#include "../orb_slam/include/Map.h"
#include "../orb_slam/include/MapDrawer.h"

#include "../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
using ctello::Tello;
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
using g2o::VertexSim3Expmap;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapDrawer;

// URL where the Tello sends its video stream to.
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

// The frame size is 720x960.
// We assume the Tello's ray of vision to hit 360x480.
const cv::Point2i TELLO_POSITION(480, 360);

// Distance to consider target found.
const int SQUARE_TARTET_DISTANCE{2500};

// Amount of centimeters to move per pixel.
const float CM_PER_PIXEL{0.3};

// Maximum step to move.
const int MIN_STEP{20};

// Minimum step to move.
const int MAX_STEP{60};

namespace
{
bool IsTarget(const uchar B, const uchar G, const uchar R)
{
    // Target is light.
    return B > 250 && G > 250 && R > 250;
}

boost::optional<Point2i> FindTarget(const Mat& frame)
{
    int max_row_count{0};
    int max_row_index{0};
    std::vector<int> cols_count(frame.cols, 0);
    for (int i = 0; i < frame.rows; ++i)
    {
        int row_count{0};
        for (int j = 0; j < frame.cols; ++j)
        {
            const auto& pixel = frame.at<Vec3b>(i, j);
            const bool is_target = IsTarget(pixel[0], pixel[1], pixel[2]);
            row_count += is_target;
            cols_count[j] += is_target;
        }
        if (row_count > max_row_count)
        {
            max_row_count = row_count;
            max_row_index = i;
        }
    }
    const auto max_col_iter =
        std::max_element(std::cbegin(cols_count), std::cend(cols_count));
    const int max_col_index =
        std::distance(std::cbegin(cols_count), max_col_iter);
    if (max_col_index == 0 && max_row_index == 0)
    {
        return {};
    }
    return Point2i(max_col_index, max_row_index);
}

std::pair<std::string, Point2i> Steer(const Point2i& position,
                                      const Point2i& target,
                                      const int square_target_distance,
                                      const float cm_per_pixel,
                                      const int min_step,
                                      const int max_step)
{
    std::string command;
    const Point2i velocity{target - position};
    if (abs(velocity.x) > abs(velocity.y))
    {
        auto step = static_cast<int>(velocity.x * cm_per_pixel);
        step = std::max(std::min(step, max_step), min_step);
        if (velocity.x > 0)
        {
            command = "right " + std::to_string(step);
        }
        else
        {
            command = "left " + std::to_string(step);
        }
    }
    else
    {
        auto step = static_cast<int>(velocity.y * cm_per_pixel);
        step = std::max(std::min(step, max_step), min_step);
        if (velocity.y < 0)
        {
            command = "up " + std::to_string(step);
        }
        else
        {
            command = "down " + std::to_string(step);
        }
    }
    return {command, velocity};
}

void DrawMaxRowAndCol(Mat& image, const Point2i& target)
{
    // Point(x, y) is (col, row)
    // Horizontal line
    const Point2i max_row_start{0, target.y};
    const Point2i max_row_end{image.cols - 1, target.y};
    line(image, max_row_start, max_row_end, {0, 255, 0}, 2);

    // Vertical line
    const Point2i max_col_start{target.x, 0};
    const Point2i max_col_end{target.x, image.rows - 1};
    line(image, max_col_start, max_col_end, {0, 255, 0}, 2);
}

void DrawVelocity(Mat& image,
                  const Point2i& tello_position,
                  const Point2i& velocity)
{
    arrowedLine(image, tello_position, tello_position + velocity, {0, 0, 255},
                2);
}
}  // namespace


int main(int argc, char **argv) {
	cout << "Hello bgbg! VFVF " << std::endl;

	// Using of core - like our future algorithms
	Point p1 { 12121, 21212 };
	p1.printPoint();

	// Opencv shit
	Mat image = imread("/local/RealTimeLearning/basketball.jpg");
	std::cout << "Image cols: " << image.cols << std::endl;
	std::cout << "image.size.p: " << image.size.p << std::endl;

	// ORB_SLAM2 shit
	Map map;
	const std::string pattern = "ppppppp";
	MapDrawer mapDrawer { &map, pattern };

	mapDrawer.DrawMapPoints();

	std::cout << "mapDrawer.mpMap->GetMaxKFid(): "
			<< mapDrawer.mpMap->GetMaxKFid() << std::endl;

	// CPP 14 stuff

	auto lambda = [](auto x, auto y) {
		return x + y;
	};
	std::cout << "How much is 2 + 3? " << lambda(2, 3) << std::endl;

//	 tello SHIT
	Tello tello { };
	if (!tello.Bind()) {
		return 0;
	}

	tello.SendCommand("streamon");
	while (!(tello.ReceiveResponse()))
		;

	VideoCapture capture { TELLO_STREAM_URL, CAP_FFMPEG };

/*	// Take-off first
	tello.SendCommand("takeoff");
	while (!(tello.ReceiveResponse()))
		;*/

	bool busy { false };
	while (true) {
		// See surrounding
		Mat frame;
		capture >> frame;

		// Listen response
		if (const auto response = tello.ReceiveResponse()) {
			std::cout << "Tello: " << *response << std::endl;
			busy = false;
		}

		// Show what the Tello sees
		resize(frame, frame, Size(), 0.75, 0.75);
		imshow("CTello Stream", frame);
		if (waitKey(1) == 27) {
			break;
		}
	}

	// G2o lib
	VertexSim3Expmap vertesSim3{};
	std::cout << "BG DIMENSION: " << vertesSim3.Dimension << std::endl;

	return 0;
}
