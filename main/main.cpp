
#include <iostream>
#include "../core/DronePilot.h"
#include "../core/PerformanceTester.h"

#include "../orb_slam/include/System.h"
#include "../orb_slam/include/Converter.h"

using namespace std;

std::vector<std::string> splitStringByRegex(std::string const &str, const char* delim = " ") {
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

int main(int argc, char **argv) {
	if (argc <= baseCommandsSplitted.size() || !((baseCommandsSplitted.size() < argc) && argc < (baseCommands.size() + baseCommandsSplitted.size() - 2))) {
		printHelp();
		return 0;
	}
	std::vector<string> arguments;
	for (int i = 1; i < argc; i++)
		arguments.push_back(argv[i]);

	bool isTestMode = AbstractActivityHandler::stringToBool(arguments[3]);
	if (isTestMode) {
			PerformanceTester tester{arguments};
			tester.run();
		} else {
			DronePilot pilot{arguments};
			pilot.run();
	}

	return 0;
}
