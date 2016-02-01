#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <mutex>

#include <boost/shared_ptr.hpp>

#include "utils/logger/IEventLogger.h"

class EventLogger : public virtual A2O::IEventLogger {

public:
	static EventLogger* getLogger(std::string filename)
	{
	  if(logger)
	    return logger;
	  
	  logger = new EventLogger(filename);
	  return logger;
	}

	void logDecodeEvent(uint32_t timestamp, std::string name, double value) const;
	void logPose(const std::string& name, const A2O::Pose2D& pose) const;
	void logTimeTaken(const std::string& name, long duration) const;

	
	void start() const;
	void stop() const;

	~EventLogger(){
		logger = nullptr;
		stop();
	}

protected:
	EventLogger(std::string filename)
	{
		_filename = filename;
	}

private:

	void writeToLog(const std::string& toWrite) const;

	static EventLogger* logger;

	std::string _filename;
	const char* del = ";";

	mutable bool _started = false;
	mutable std::ofstream _log;
	mutable std::mutex writeLock;	
	EventLogger(const EventLogger& );
};
