#pragma once

#include "../common/framework.h"

#define OID_ADTF_HSOG_TSIGNALVALUE "adtf.hsog.tsignalvaluegenerator"

class tsignalValueGenerator : public adtf::cTimeTriggeredFilter {
  ADTF_DECLARE_FILTER_VERSION(OID_ADTF_HSOG_TSIGNALVALUE, "HSOG tSignalValue Generator Filter",
                              adtf::OBJCAT_DataFilter, "tsignalValueGeneratorVersion", 0, 0, 1, "1")

 public:
  tsignalValueGenerator(const tChar* __info);
  virtual ~tsignalValueGenerator();
  tResult Init(tInitStage eStage, __exception = NULL) override;

 protected:
  tResult Start(__exception = NULL) override;
  tResult Stop(__exception = NULL) override;
  tResult Shutdown(tInitStage eStage, __exception = NULL) override;
  tResult Cycle(__exception) override;
 
 private:
  adtf::cOutputPin m_oOutput;

  tResult sendData();
  tResult CreateOutputPins(__exception = NULL);
  cObjectPtr<adtf::IMediaTypeDescription> m_pCoderDescSignal;

};
