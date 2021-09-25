/*
 * DronePilot.h
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#ifndef CORE_DRONEPILOT_H_
#define CORE_DRONEPILOT_H_

#include <opencv2/videoio.hpp>
#include "../drone_lib/include/ctello.h"
#include "../orb_slam/include/System.h"
#include "AbstractActivityHandler.h"
#include "Point3D.h"

using ctello::Tello;
using ORB_SLAM2::System;
using cv::VideoCapture;

class DronePilot : public AbstractActivityHandler {
public:
	DronePilot(const std::vector<std::string>& config);
	virtual ~DronePilot() = default;

	void run() override;

private:
	void sendACommand(const std::string &command);
	std::vector<Point3D> transformMapFromSlamToRegularPoint();
	const std::string telloStreamUrl;
	ctello::Tello tello;
	ORB_SLAM2::System slam;
	cv::VideoCapture capture;
};

#endif /* CORE_DRONEPILOT_H_ */
