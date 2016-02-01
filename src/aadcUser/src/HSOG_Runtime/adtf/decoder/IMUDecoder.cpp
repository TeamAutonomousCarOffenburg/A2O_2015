#include "IMUDecoder.h"
#include "perception/IIMUPerceptor.h"
#include "perception/impl/IMUPerceptor.h"

using namespace A2O;

IMUDecoder::IMUDecoder(ISensorConfig::ConstPtr config,
                       IEventLogger::ConstPtr logger)
        : Decoder(logger), _perceptiorName(config->getPerceptorName())
{
    _pins.push_back(InputPin(_perceptiorName, "tInerMeasUnitData"));
}

IMUDecoder::~IMUDecoder()
{

}

std::vector<IPerceptor::ConstPtr> IMUDecoder::decode(const InputPin& pin,
                                                     adtf::IMediaTypeDescription* mediaTypeDescription,
                                                     adtf::IMediaSample* mediaSample)
{
    int pinIndex = indexOfPin(pin);

    if (pinIndex >= 0) {
        __adtf_sample_read_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);

        // Extract acceleration values
        tFloat32 acc_x = 0;
        mediaCoder->Get("f32A_x", (tVoid*)&acc_x);
        tFloat32 acc_y = 0;
        mediaCoder->Get("f32A_y", (tVoid*)&acc_y);
        tFloat32 acc_z = 0;
        mediaCoder->Get("f32A_z", (tVoid*)&acc_z);

        // Extract gyroscope values
        tFloat32 gyro_w = 0;
        mediaCoder->Get("f32Q_w", (tVoid*)&gyro_w);
        tFloat32 gyro_x = 0;
        mediaCoder->Get("f32Q_x", (tVoid*)&gyro_x);
        tFloat32 gyro_y = 0;
        mediaCoder->Get("f32Q_y", (tVoid*)&gyro_y);
        tFloat32 gyro_z = 0;
        mediaCoder->Get("f32Q_z", (tVoid*)&gyro_z);

        // Extract timestamp
        tUInt32 time = 0;
        mediaCoder->Get("ui32ArduinoTimestamp", (tVoid*)&time);

        std::string name = _pins.at(pinIndex).name;
        _logger->logDecodeEvent(time, name + "_ACC_X", acc_x);
        _logger->logDecodeEvent(time, name + "_ACC_Y", acc_y);
        _logger->logDecodeEvent(time, name + "_ACC_Z", acc_z);
        _logger->logDecodeEvent(time, name + "_GYRO_W", gyro_w);
        _logger->logDecodeEvent(time, name + "_GYRO_X", gyro_x);
        _logger->logDecodeEvent(time, name + "_GYRO_Y", gyro_y);
        _logger->logDecodeEvent(time, name + "_GYRO_Z", gyro_z);

        std::vector<IPerceptor::ConstPtr> perceptors;
        perceptors.push_back(boost::make_shared<IMUPerceptor>(_perceptiorName, long(time),
                                                              double(acc_x), double(acc_y), double(acc_z),
                                                              double(gyro_w), double(gyro_x), double(gyro_y), double(gyro_z)));

        return perceptors;
    }

    return std::vector<IPerceptor::ConstPtr>();
}
