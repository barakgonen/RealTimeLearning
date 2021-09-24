/*
 * Utils.h
 *
 *  Created on: Sep 21, 2021
 *      Author: barakg
 */

#ifndef CORE_UTILS_H_
#define CORE_UTILS_H_

#include <vector>
#include <iosfwd>
#include <fstream>

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

#include "../orb_slam/include/Converter.h"
#include "../orb_slam/include/System.h"

/**
 * Utility functions
 */

std::vector<Point3D> transformMapFromSlamToRegularPoint(ORB_SLAM2::System &SLAM) {
    std::vector<Point3D> pointsVector;
    for (auto p: SLAM.GetMap()->GetAllMapPoints()) {
        if (p != NULL) {
            pointsVector.push_back({ORB_SLAM2::Converter::toVector3d(p->GetWorldPos())});
        }
    }

    return pointsVector;
}

void exportRawDataToFile(const std::vector<Point3D> pointsVec, std::string pointsIutputFilePath) {
	std::ofstream pointData;
	pointData.open(pointsIutputFilePath);
	for (const auto& p : pointsVec) {
		pointData << p.getX() << "," << p.getY() << "," << p.getZ() << std::endl;
	}
}

#define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
#define timeNow() std::chrono::high_resolution_clock::now()

template<typename F, typename... Args>
double funcTimeMillis(F func, Args&&... args){
	const auto start = std::chrono::system_clock::now();
    func(std::forward<Args>(args)...);
    const auto end = std::chrono::system_clock::now();
    return duration(end - start);
}

std::vector<std::string> splitStringByRegex(std::string const &str, const char* delim = " ")
{
	std::vector<std::string> splitted;
    char *token = strtok(const_cast<char*>(str.c_str()), delim);
    while (token != nullptr)
    {
    	splitted.push_back(std::string(token));
        token = strtok(nullptr, delim);
    }

    return splitted;
}

std::string baseCommands{"LOGGER_ON TOP_N IS_TESTING_MODE IS_MULTI"};
std::string testingModeConstants{"TEST_SINGLE_THREADED NUMBER_OF_LOOPS CSV_INPUT_FILE_PATH CSV_OUTPUT_FILE_PATH"};

std::vector<std::string> baseCommandsSplitted = splitStringByRegex(baseCommands);
std::vector<std::string> testingCommandsSplitted = splitStringByRegex(testingModeConstants);

void printCommands(const std::vector<std::string>& commandsVec, const std::string& prefix = "  ./RealTimeLearningApp ") {
	std::cout << prefix;
	for (const auto & splittedCommand : commandsVec) {
		std::cout << splittedCommand << " ";
	}
}

void printHelp() {
	std::cout << "Usage documentation: " << std::endl;
	std::cout << "--------------------\n" << std::endl;
	std::cout << "There are 2 modes, run mode and test mode.\n" << std::endl;
	std::cout << "* Running mode example: please run the following command:" << std::endl;
	printCommands(baseCommandsSplitted);
	std::cout << "" << std::endl;
	std::cout << "  ./RealTimeLearningApp false 20 true false true" << std::endl;
	std::cout << "  ./RealTimeLearningApp false 20 true false false" << std::endl;
	std::cout << "* Testing mode example: please run the following command:" << std::endl;
	for (int i = 0; i <= testingCommandsSplitted.size() ; i++) {
		printCommands(baseCommandsSplitted);
		printCommands(std::vector<std::string>(testingCommandsSplitted.begin(), testingCommandsSplitted.begin() + i), "");
		std::cout << "" << std::endl;
	}
}

bool stringToBool(std::string value) {
	std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return std::tolower(c); });

	if (value == "true")
		return true;
	else if (value == "false")
		return false;
	else
		throw std::invalid_argument{"<stringToBool()> passed value is: " + value + ", support true / false"};
}

int stringToInt(std::string value) {
	try {
		int number = std::stoi(value);
		return number;
	} catch (std::exception& e) {
		throw std::invalid_argument{"<stringToInt()> passed value is: " + value + " and it's not a number"};
	}
}

#endif /* CORE_UTILS_H_ */
