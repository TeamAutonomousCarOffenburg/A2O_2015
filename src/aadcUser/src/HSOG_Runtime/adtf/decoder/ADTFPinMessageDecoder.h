#pragma once

#include "perception/IPerception.h"
#include "meta/ICarMetaModel.h"
#include "InputPin.h"
#include "Decoder.h"

#include "utils/logger/IEventLogger.h"

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <vector>
#include <boost/smart_ptr.hpp>

namespace A2O {

/**
 * The ADTFPinMessageDecoder class provides the mapping from input pins
 * of the ADTF-Filter to internal Perceptor objects.
 *
 * \author Stefan Glaser
 */
class ADTFPinMessageDecoder {
public:
  ADTFPinMessageDecoder(IPerception::Ptr perception,
			ICarMetaModel::ConstPtr carMetaModel,
			IEventLogger::ConstPtr logger);
  virtual ~ADTFPinMessageDecoder();

  typedef boost::shared_ptr<ADTFPinMessageDecoder> Ptr;
  typedef boost::shared_ptr<const ADTFPinMessageDecoder> ConstPtr;
  
  virtual const std::vector<A2O::InputPin> getPinConfigs();
  virtual bool decode(const A2O::InputPin& pin,
		      adtf::IMediaTypeDescription* mediaTypeDescription,
		      adtf::IMediaSample* mediaSample);
  
protected:
  A2O::IPerception::Ptr _perception;
  std::vector<A2O::DecoderPtr> _decoder;
};

}
