#pragma once

#include "IPerceptor.h"
#include "IAccelerometerPerceptor.h"
#include "IGyroPerceptor.h"

#include <eigen3/Eigen/Dense>


namespace A2O {

/**
 * Interface for an IMU perceptor.
 * 
 * \author Stefan Glaser
 */
    class IIMUPerceptor : public virtual IAccelerometerPerceptor, public virtual IGyroPerceptor {
    public:
        typedef boost::shared_ptr<IIMUPerceptor> Ptr;
        typedef boost::shared_ptr<const IIMUPerceptor> ConstPtr;

        virtual ~IIMUPerceptor(){};
    };

}
