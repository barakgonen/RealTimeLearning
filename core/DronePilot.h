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

class DronePilot : public AbstractActivityHandler {
public:
	DronePilot(const std::vector<std::string>& config);
	virtual ~DronePilot() = default;

	void run() override;

private:
	void sendACommand(const std::string &command);
    void exportCalculatedResultsToFile(const std::pair<Point3D, Point3D>& points);
    void exportAllPointsToFile(const std::vector<Point3D>& pointsVec);
	std::vector<Point3D> transformMapFromSlamToRegularPoint(ORB_SLAM2::System& slam);
	const std::string telloStreamUrl;
    cv::Mat slamMatrix;
    ctello::Tello tello;
};

#endif /* CORE_DRONEPILOT_H_ */
