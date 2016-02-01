#include "ADTFPinMessageEncoder.h"


using namespace A2O;

ADTFPinMessageEncoder::ADTFPinMessageEncoder(IAction::Ptr action,
					     ICarMetaModel::ConstPtr carMetaModel)
      : _action(action)
{
  // Create output pins
  const std::vector<IServoDriveConfig::ConstPtr>& servoDriveConfigs = carMetaModel->getServoDriveConfigs();
  for (unsigned int i = 0; i < servoDriveConfigs.size(); i++) {
    _pins.push_back(OutputPin(servoDriveConfigs[i]->getEffectorName(), "tSignalValue"));
  }
  
  std::vector<IActuatorConfig::ConstPtr> actuatorConfigs = carMetaModel->getMotorConfigs();
  for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
    _pins.push_back(OutputPin(actuatorConfigs[i]->getEffectorName(), "tSignalValue"));
  }
  
  actuatorConfigs = carMetaModel->getLightConfigs();
  for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
    _pins.push_back(OutputPin(actuatorConfigs[i]->getEffectorName(), "tBoolSignalValue"));
  }

  actuatorConfigs = carMetaModel->getManeuverStatusConfigs();
  for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
    _pins.push_back(OutputPin(actuatorConfigs[i]->getEffectorName(), "tDriverStruct"));
  }
}

ADTFPinMessageEncoder::~ADTFPinMessageEncoder()
{

}

int ADTFPinMessageEncoder::indexOfPin(OutputPin pin) const
{
  for (unsigned int i = 0; i < _pins.size(); i++) {
    if (_pins[i] == pin) {
      return i;
    }
  }
  
  return -1;
}

const std::vector<OutputPin>& ADTFPinMessageEncoder::getOutputPins()
{
  return _pins;
}

bool ADTFPinMessageEncoder::encode(const OutputPin& pin,
				   adtf::IMediaTypeDescription* mediaTypeDescription,
				   adtf::IMediaSample* mediaSample)
{
  int pinIndex = indexOfPin(pin);
  bool toTransmit = false;

  if (pinIndex >= 0) {
	  cObjectPtr<adtf::IMediaSerializer> serializer;
	  mediaTypeDescription->GetMediaSampleSerializer(&serializer);

	  tInt size = serializer->GetDeserializedSize();
	  mediaSample->AllocBuffer(size);

	  cObjectPtr<adtf::IMediaCoder> mediaCoder;

	  tUInt32 timestamp = 0;

	  if (pin.signalType == "tBoolSignalValue") {
		  IBoolValueEffector::Ptr boolEffector = _action->getLightEffector(pin.name);
		  if (boolEffector) {
			  __adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
	  		  if(!mediaCoder)
			  {
				  return false;
			  }
			  tBool value = boolEffector->getValue();
			  mediaCoder->Set("bValue", (tVoid*)&value);
			  mediaCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	  		 
			  toTransmit = true;
		  }
	  } else if (pin.signalType == "tSignalValue") {
		  IDoubleValueEffector::Ptr valueEffector = boost::dynamic_pointer_cast<IDoubleValueEffector>(_action->getEffector(pin.name));
		  if (valueEffector) {
			  __adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
	  		  if(!mediaCoder)
			  {
				  return false;
			  }
			  tFloat32 value = valueEffector->getValue();	
			  mediaCoder->Set("f32Value", (tVoid*)&value);
			  mediaCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	  		 
			  toTransmit = true;
		  }
	  } else if (pin.signalType == "tDriverStruct") {
		IManeuverStatusEffector::Ptr valueEffector = boost::dynamic_pointer_cast<IManeuverStatusEffector>(_action->getEffector(pin.name));
		if (valueEffector)
		{
			  __adtf_sample_write_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
			if(!mediaCoder)
			{
				  return false;
			}
			int state = valueEffector->getStatus();
			int maneuverId = valueEffector->getManeuverId();
			mediaCoder->Set("i8StateID", (tVoid*)&state);
			mediaCoder->Set("i16ManeuverEntry", (tVoid*)&maneuverId);

			toTransmit = true;
		}
	  }
  }

  return toTransmit;
}
