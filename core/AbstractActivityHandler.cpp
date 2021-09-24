/*
 * AbstractActivityHandler.cpp
 *
 *  Created on: Sep 24, 2021
 *      Author: barakg
 */

#include "AbstractActivityHandler.h"
#include <algorithm>
#include <iostream>

bool AbstractActivityHandler::stringToBool(std::string value) {
	std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return std::tolower(c); });

	if (value == "true")
		return true;
	else if (value == "false")
		return false;
	else
		throw std::invalid_argument{"<stringToBool()> passed value is: " + value + ", support true / false"};
}

int AbstractActivityHandler::stringToInt(std::string value) {
	try {
		int number = std::stoi(value);
		return number;
	} catch (std::exception& e) {
		throw std::invalid_argument{"<stringToInt()> passed value is: " + value + " and it's not a number"};
	}
}


AbstractActivityHandler::AbstractActivityHandler(const std::vector<std::string>& arguments)
: isLoggerOn{stringToBool(arguments[0])}
, numberOfPointsForFiltering{stringToInt(arguments[1])}
, isTestMode{stringToBool(arguments[2])}
, isMulti{stringToBool(arguments[3])}
{
	std::cout << "Initialized with the following configuration: " << std::endl;
	std::cout << "---------------------------------------------\n" << std::endl;
	std::cout << "isLoggerOn: " << std::boolalpha << isLoggerOn << std::endl;
	std::cout << "numberOfPointsForFiltering: " << numberOfPointsForFiltering << std::endl;
	std::cout << "isTestMode: " << std::boolalpha << isTestMode << std::endl;
	std::cout << "isMulti: " << std::boolalpha << isMulti << std::endl;
}
