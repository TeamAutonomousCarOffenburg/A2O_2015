#include "WheelTickOdometry.h"

#include "utils/value/FuzzyCompare.h"
#include <cmath>


using namespace A2O;

WheelTickOdometry::WheelTickOdometry()
      : _pose(0, 0, Angle::Zero()), _time(0), _driveCalculator() , _wheelDiameter(0.1)
{
}

WheelTickOdometry::WheelTickOdometry(DriveCalculator::ConstPtr driveCalculator, const double& wheelDiameter)
      : _pose(0, 0, Angle::Zero()), _time(0), _driveCalculator(driveCalculator), _wheelDiameter(wheelDiameter)
{
}

WheelTickOdometry::~WheelTickOdometry()
{
}

void WheelTickOdometry::setDriveCalculator(DriveCalculator::ConstPtr driveCalculator)
{
  _driveCalculator = driveCalculator;
}

const Pose2D& WheelTickOdometry::getCurrentPose()
{
  return _pose;
}

const Pose2D WheelTickOdometry::update(const double& leftWheelTicks,
				       const double& rightWheelTicks,
				       const long int& time)
{
  // Update tick history queues
  if (_leftTickHistory.empty() || _rightTickHistory.empty()) {
    // Ensire both hirtory queues are empty
    _leftTickHistory.clear();
    _rightTickHistory.clear();
    
    // Push back initial measurement
    _leftTickHistory.push_back(std::pair<long, double>(time, leftWheelTicks));
    _rightTickHistory.push_back(std::pair<long, double>(time, rightWheelTicks));
    _time = time;
    _pose = Pose2D();
  } else {
    // Check if left wheel tick count increased since last update
    if (leftWheelTicks > _leftTickHistory.back().second) {
      _leftTickHistory.push_back(std::pair<long, double>(time, leftWheelTicks));
    }
    
    // Check if right wheel tick count increased since last update
    if (rightWheelTicks > _rightTickHistory.back().second) {
      _rightTickHistory.push_back(std::pair<long, double>(time, rightWheelTicks));
    }
  }
  
  // As soon as we received two measurements to each side, we can properly integrate the speeds
  if (_leftTickHistory.size() > 1 && _rightTickHistory.size() > 1) {
    long newTime = std::min(_leftTickHistory.back().first, _rightTickHistory.back().first);
    long currentTime = _time;
    long nextTime;
    size_t leftIdx = 1;
    size_t rightIdx = 1;
    
    // Piecewise interpolate speeds and resulting poses
    while (currentTime < newTime) {
      nextTime = std::min(_leftTickHistory[leftIdx].first, _rightTickHistory[rightIdx].first);
      long deltaT = nextTime - currentTime;
      
      // Progress pose
      double leftWheelDistance = _wheelDiameter * M_PI * getRPMS(_leftTickHistory[leftIdx - 1], _leftTickHistory[leftIdx]) * deltaT;
      double rightWheelDistance = _wheelDiameter * M_PI * getRPMS(_rightTickHistory[rightIdx - 1], _rightTickHistory[rightIdx]) * deltaT;
      _pose *= _driveCalculator->calculateCurvePose(leftWheelDistance, rightWheelDistance);
      
      if (_leftTickHistory[leftIdx].first == nextTime) {
	// We just interpolated up to this measurement, so try the next one
	leftIdx++;
      }
      
      if (_rightTickHistory[rightIdx].first == nextTime) {
	// We just interpolated up to this measurement, so try the next one
	rightIdx++;
      }
      
      // Progress to new time slot
      currentTime = nextTime;
    }
    
    _time = currentTime;
    
    // Clear all recent measurements that are of no use anymore
    while (_leftTickHistory.size() > 1 && _leftTickHistory[1].first <= _time) {
      _leftTickHistory.pop_front();
    }
    
    while (_rightTickHistory.size() > 1 && _rightTickHistory[1].first <= _time) {
      _rightTickHistory.pop_front();
    }
  }
  
  return _pose;
}

const double WheelTickOdometry::getRPMS(const std::pair<long, double>& tm1,
					const std::pair<long, double>& tm2)
{
  double rounds = (tm2.second - tm1.second) / 8.0; // 8 ticks are ane round
  return rounds / (tm2.first - tm1.first);
}

void WheelTickOdometry::reset(const Pose2D& zeroPose, const long int& time)
{
  _leftTickHistory.clear();
  _rightTickHistory.clear();
  _time = 0;
  _pose = Pose2D();
}
