#pragma once

#include "IAccelerometer.h"
#include "IGyroscope.h"

#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Interface for a Inertial Measurement Unit sensor.
 * 
 * \author Stefan Glaser
 */
class IIMU : public virtual IAccelerometer, public virtual IGyroscope {
public:
    typedef boost::shared_ptr<IIMU> Ptr;
    typedef boost::shared_ptr<const IIMU> ConstPtr;

    virtual ~IIMU(){};
};

}
