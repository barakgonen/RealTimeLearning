/*
 * AbstractActivityHandler.h
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#ifndef CORE_ABSTRACTACTIVITYHANDLER_H_
#define CORE_ABSTRACTACTIVITYHANDLER_H_
#include <string>
#include <vector>

class AbstractActivityHandler {
public:
	AbstractActivityHandler(const std::vector<std::string>& config);
	virtual ~AbstractActivityHandler() = default;

	virtual void run() = 0;
	static bool stringToBool(std::string value);
	static int stringToInt(std::string value);
protected:
	bool isLoggerOn;
	int numberOfPointsForFiltering;
	bool isTestMode;
	bool isMulti;
};

#endif /* CORE_ABSTRACTACTIVITYHANDLER_H_ */
