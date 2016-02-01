#pragma once

#include "Sensor.h"
#include "carmodel/IIMU.h"
#include "meta/ISensorConfig.h"
#include "utils/value/Distribution.h"

#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * The IMU class represents an accelerometer and gyroscope sensor.
 * 
 * \author Stefan Glaser
 */
class IMU : public Sensor, public virtual IIMU {
public:
    typedef boost::shared_ptr<IMU> Ptr;
    typedef boost::shared_ptr<const IMU> ConstPtr;

    IMU(ISensorConfig::ConstPtr config);
    virtual ~IMU();

    virtual const Eigen::Quaterniond& getGyro() const;
    virtual const Angle getHorizontalAngle() const;

    virtual const Eigen::Vector3d& getAcceleration() const;

    virtual const bool update(IPerception::ConstPtr perception);
    virtual const bool isInitialized() const;
    virtual void reset();

protected:
    Eigen::Vector3d _acc;

    Eigen::Quaterniond _gyro;

    Eigen::Quaterniond _orientation;

    Eigen::Quaterniond _initialOrientation;

    Distribution* _distribution;
};

}
