#pragma once

#include "../common/framework.h"
#include "../encoder/ADTFPinMessageEncoder.h"

#include "general/A2ORuntime.h"
#include "general/IComponentFactory.h"
#include "general/ComponentFactory.h"

#include <chrono>

#define OID_ADTF_HSOG_RATE "adtf.hsog.rateLimiter"

class rateLimiter : public adtf::cFilter {
  ADTF_DECLARE_FILTER_VERSION(OID_ADTF_HSOG_RATE, "HSOG limiter",
                              adtf::OBJCAT_DataFilter, "rateLimiterVersion", 0, 0, 1, "1")

 public:
  rateLimiter(const tChar* __info);
  virtual ~rateLimiter();
  tResult Init(tInitStage eStage, __exception = NULL) override;
  tResult OnPinEvent(adtf::IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, adtf::IMediaSample* pMediaSample) override;

 protected:
  tResult Start(__exception = NULL) override;
  tResult Stop(__exception = NULL) override;
  tResult Shutdown(tInitStage eStage, __exception = NULL) override;

 private:
  tResult CreatePins(__exception = NULL);
  tResult CreateDescriptors();
  
  bool longEnoughSinceLastSent(adtf::IPin *pSource);

  std::map<adtf::IPin*, A2O::OutputPin> _inputPinMap;
  std::map<adtf::IPin*, adtf::cOutputPin*> _inputPinToOutputMap;
  std::map<adtf::IPin*, tFloat32> _tsignalLastValue;
  std::map<adtf::IPin*, tBool> _boolLastValue;
  std::map<adtf::IPin*, std::pair<tInt8,tInt16>> _driverStructLastValue;

  std::map<adtf::IPin*, std::chrono::system_clock::time_point> _lastSentTime;
  std::map<adtf::IPin*, cObjectPtr<adtf::IMediaTypeDescription>> _descriptors;
  
  std::vector<A2O::OutputPin> _carOutputPins;
  adtf::IMediaDescriptionManager* _pDescManager;

};
