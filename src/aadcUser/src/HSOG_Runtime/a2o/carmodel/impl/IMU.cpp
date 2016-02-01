#include "IMU.h"

#include "utils/geometry/Geometry.h"
#include "perception/IIMUPerceptor.h"


using namespace A2O;
using namespace Eigen;

IMU::IMU(ISensorConfig::ConstPtr config)
        : Sensor(config), _gyro(), _orientation(), _initialOrientation(), _distribution(new Distribution(40))
{
    _gyro.setIdentity();
    _orientation.setIdentity();
    _initialOrientation.setIdentity();
}

IMU::~IMU()
{
    if (_distribution != nullptr) {
        delete _distribution;
        _distribution = nullptr;
    }
}

const Eigen::Quaterniond& IMU::getGyro() const
{
    return _orientation;
}

const Angle IMU::getHorizontalAngle() const
{
    return Geometry::getHorizontalAngle(_orientation);
}

const Eigen::Vector3d& IMU::getAcceleration() const
{
    return _acc;
}

const bool IMU::update(IPerception::ConstPtr perception)
{
    IIMUPerceptor::ConstPtr imuPerceptor = perception->getIMUPerceptor(_perceptorName);

    if (imuPerceptor.get()) {
        _acc = _pose.getOrientation() * imuPerceptor->getAcceleration();
        _gyro = _pose.getOrientation() * imuPerceptor->getGyro() * _pose.getOrientation().inverse();
        _orientation = _initialOrientation * _gyro;
        _lastMeasurementTime = imuPerceptor->getTime();

        // Check if the gyro sensor is initialized
        if (_distribution != nullptr) {
            _distribution->addValue(Geometry::getHorizontalAngle(_gyro).deg());
            if (_distribution->isValid() && _distribution->getVarianz() < 0.05) {
                reset();
                delete _distribution;
                _distribution = nullptr;
            }
        }

        return true;
    }

    return false;
}

const bool IMU::isInitialized() const
{
    return _distribution == nullptr;
}

void IMU::reset()
{
    _initialOrientation = _gyro.inverse();
}
