#include "JuryDecoder.h"
#include "perception/impl/JuryPerceptor.h"
#include "AADCCar.h"

using namespace A2O;

JuryDecoder::JuryDecoder(IEventLogger::ConstPtr logger)
      : Decoder(logger), _perceptiorName(AADC_Car::JURY_COMMAND)
{
  _pins.push_back(InputPin(AADC_Car::JURY_COMMAND, "tJuryStruct"));
}

JuryDecoder::~JuryDecoder()
{

}

std::vector<IPerceptor::ConstPtr> JuryDecoder::decode(const InputPin& pin,
						      adtf::IMediaTypeDescription* mediaTypeDescription,
						      adtf::IMediaSample* mediaSample)
{
  int pinIndex = indexOfPin(pin);
  
  if (pinIndex >= 0) {
    // Extract value and timestamp
    __adtf_sample_read_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);
    
    tInt8 i8ActionID;
    mediaCoder->Get("i8ActionID", (tVoid*)&i8ActionID);
    
    tInt16 i16entry;
    mediaCoder->Get("i16ManeuverEntry", (tVoid*)&i16entry);
   
    long time = mediaSample->GetTime();

    std::string name = _pins.at(pinIndex).name;
    _logger->logDecodeEvent(time, name, i16entry);
    
    std::vector<IPerceptor::ConstPtr> perceptors;
    perceptors.push_back(boost::make_shared<JuryPerceptor>(_perceptiorName, time, i8ActionID, i16entry));
      
    return perceptors;
    }
  
  return std::vector<IPerceptor::ConstPtr>();
}
