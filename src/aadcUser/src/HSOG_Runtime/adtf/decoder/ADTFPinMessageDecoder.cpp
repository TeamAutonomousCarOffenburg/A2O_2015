#include "ADTFPinMessageDecoder.h"

#include "GyroDecoder.h"
#include "AccelerometerDecoder.h"
#include "IMUDecoder.h"
#include "ValueGroupDecoder.h"
#include "DepthImageDecoder.h"
#include "CameraDecoder.h"
#include "JuryDecoder.h"
#include "WheelTickDecoder.h"

#include <boost/iterator/iterator_concepts.hpp>

using namespace A2O;

ADTFPinMessageDecoder::ADTFPinMessageDecoder(IPerception::Ptr perception,
					     ICarMetaModel::ConstPtr carMetaModel,
					     IEventLogger::ConstPtr logger)
      : _perception(perception)
{
  // Add Sensor perceptor decoders
  std::vector<ISensorConfig::ConstPtr> sensorConfigs = carMetaModel->getIMUConfigs();
  for (unsigned int i = 0; i < sensorConfigs.size(); i++) {
    _decoder.push_back(boost::make_shared<IMUDecoder>(sensorConfigs[i], logger));
  }

  sensorConfigs = carMetaModel->getGyroConfigs();
  for (unsigned int i = 0; i < sensorConfigs.size(); i++) {
    _decoder.push_back(boost::make_shared<GyroDecoder>(sensorConfigs[i], logger));
  }
  
  sensorConfigs = carMetaModel->getAccelerometerConfigs();
  for (unsigned int i = 0; i < sensorConfigs.size(); i++) {
    _decoder.push_back(boost::make_shared<AccelerometerDecoder>(sensorConfigs[i], logger));
  }
  
  std::vector<IDistanceSensorConfig::ConstPtr> dsConfigs = carMetaModel->getInfraRedConfigs();
  if (!dsConfigs.empty()) {
    _decoder.push_back(boost::make_shared<ValueGroupDecoder>(dsConfigs, logger, 0.01));
  }
  
  dsConfigs = carMetaModel->getUltraSoundConfigs();
  if (!dsConfigs.empty()) {
    _decoder.push_back(boost::make_shared<ValueGroupDecoder>(dsConfigs, logger, 0.01));
  }
  
  sensorConfigs = carMetaModel->getRotationConfigs();
  for (unsigned int i = 0; i < sensorConfigs.size(); i++) {
    _decoder.push_back(boost::make_shared<WheelTickDecoder>(sensorConfigs[i], logger));
  }
  
  sensorConfigs = carMetaModel->getVoltageConfigs();
  if (!sensorConfigs.empty()) {
    _decoder.push_back(boost::make_shared<ValueGroupDecoder>(sensorConfigs, logger));
  }
  
  std::vector<ICameraConfig::ConstPtr> depthCameraConfigs = carMetaModel->getDepthCameraConfigs();
  for (unsigned int i = 0; i < depthCameraConfigs.size(); i++) {
    _decoder.push_back(boost::make_shared<DepthImageDecoder>(depthCameraConfigs[i], logger));
  }

  std::vector<ICameraConfig::ConstPtr> cameraConfigs = carMetaModel->getCameraConfigs();
  for (unsigned int i = 0; i < cameraConfigs.size(); i++) {
    _decoder.push_back(boost::make_shared<CameraDecoder>(cameraConfigs[i], logger));
  }


  _decoder.push_back(boost::make_shared<JuryDecoder>(logger));

  // Add Actuator perceptor decoders
  std::vector<IServoDriveConfig::ConstPtr> servoDriveConfigs = carMetaModel->getServoDriveConfigs();
   if (!servoDriveConfigs.empty()) {
    _decoder.push_back(boost::make_shared<ValueGroupDecoder>(servoDriveConfigs, logger));
  }
}

ADTFPinMessageDecoder::~ADTFPinMessageDecoder()
{

}

const vector<A2O::InputPin> ADTFPinMessageDecoder::getPinConfigs()
{
  std::vector<InputPin> pinConfigs;
  
  for (auto it : _decoder) {
    const std::vector<InputPin>& newPinConfigs = it->getPinConfig();
    if (!newPinConfigs.empty()) {
      pinConfigs.insert(pinConfigs.begin(), newPinConfigs.begin(), newPinConfigs.end());
    }
  }
  
  return pinConfigs;
}

bool ADTFPinMessageDecoder::decode(const A2O::InputPin& pin,
				   adtf::IMediaTypeDescription* mediaTypeDescription,
				   adtf::IMediaSample* mediaSample)
{ 
  for (auto it : _decoder) {
    if (it->indexOfPin(pin) >= 0) {
      std::vector<IPerceptor::ConstPtr> perceptors = it->decode(pin, mediaTypeDescription, mediaSample);
      
      if (!perceptors.empty()) {
	// Create perceptor map
	std::map<std::string, IPerceptor::ConstPtr> perceptorMap;
	for (unsigned int i = 0; i < perceptors.size(); i++) {
	  perceptorMap.insert(make_pair(perceptors[i]->getName(), perceptors[i]));
	}
	
	// Publish new perceptions
	_perception->updatePerceptors(perceptorMap);
      }
      
      return true;
    }
  }
  
  return false;
}

