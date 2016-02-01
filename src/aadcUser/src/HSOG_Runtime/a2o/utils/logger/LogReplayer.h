#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>

#include "perception/IPerception.h"
#include "perception/impl/ValuePerceptor.h"
#include "perception/impl/AccelerometerPerceptor.h"
#include "perception/impl/GyroPerceptor.h"

class LogReplayer{

	public:
		LogReplayer(std::string logPath, A2O::IPerception::Ptr perception);
		
		void load();

		bool next(int time);

		int getFirstTime();

	private:
		std::vector<A2O::IPerceptor::ConstPtr> perceptors;
		std::ifstream _log;
		char delim = ';';
		A2O::IPerception::Ptr _perception;

		bool _loaded = false;

		A2O::IPerceptor::ConstPtr createPerceptor(std::string line);

		A2O::AccelerometerPerceptor* generateAccelerometerPerceptor(std::string name, long time, double value);
		A2O::GyroPerceptor* generateGyroPerceptor(std::string name, long time, double value);

		std::vector<std::string>& split(const std::string &s, std::vector<std::string> &elems);
		std::vector<std::string> split(const std::string &s);
};
