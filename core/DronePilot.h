/*
 * DronePilot.h
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#ifndef CORE_DRONEPILOT_H_
#define CORE_DRONEPILOT_H_

#include "../drone_lib/include/ctello.h"
#include "AbstractActivityHandler.h"

using ctello::Tello;

class DronePilot : public AbstractActivityHandler {
public:
	DronePilot(const std::vector<std::string>& config);
	virtual ~DronePilot() = default;

	void run() override;

private:
	void sendACommand(const std::string &command);
	const std::string telloStreamUrl;
	ctello::Tello tello;
};

#endif /* CORE_DRONEPILOT_H_ */
