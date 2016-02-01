#pragma once

#include "Decoder.h"
#include "meta/ISensorConfig.h"

#include <vector>

namespace A2O {

/**
 * The IMUDecoder manages the decoding a IMU-pin.
 *
 * \author Stefan Glaser
 */
class IMUDecoder : public Decoder {
public:
    IMUDecoder(ISensorConfig::ConstPtr config, IEventLogger::ConstPtr logger);
    virtual ~IMUDecoder();

    virtual std::vector<IPerceptor::ConstPtr> decode(const A2O::InputPin& pin,
                                                     adtf::IMediaTypeDescription* mediaTypeDescription,
                                                     adtf::IMediaSample* mediaSample);

protected:
    /** The name of the represented gyro perceptor. */
    std::string _perceptiorName;
};
}

