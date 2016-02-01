#pragma once

#include <thread>

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>

#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QKeyEvent>

#include "../../a2o/general/A2ORuntime.h"
#include "../common/framework.h"
#include "../decoder/ADTFPinMessageDecoder.h"
#include "../encoder/ADTFPinMessageEncoder.h"
#include "../../tools/world/WorldViewer.h"

#define OID_HSOG_RUNTIME_FILTER "adtf.hsog.runtime_filter"


class RuntimeFilter : public QObject, public adtf_graphics::cBaseQtFilter, public IKeyEventHandler{
  ADTF_DECLARE_FILTER_VERSION(OID_HSOG_RUNTIME_FILTER, "HSOG runtime filter", OBJCAT_DataFilter,
                              "hsogRuntimeFilterVersion", 0, 0, 1, "1")

	  Q_OBJECT
 public:
  RuntimeFilter(const tChar* __info);
  virtual ~RuntimeFilter();

  tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample) override;

  tResult Init(tInitStage eStage, __exception = NULL) override;
  tResult Start(__exception = NULL) override;
  tResult Stop(__exception = NULL) override;
  tResult Shutdown(tInitStage eStage, __exception = NULL) override;
  tResult OnKeyEvent(tKeyEventCode eCode, tInt nParam1=0, tInt nParam2=0, tInt nFlags=0, tVoid* pEventData = NULL);

  adtf::IGlobalKeyEventManager* m_pKeyEventManager;

 private:

  A2O::ADTFPinMessageDecoder::Ptr _decoder;
  A2O::ADTFPinMessageEncoder::Ptr _encoder;
  A2O::A2ORuntime* _A2ORuntime;

  tResult CreateInputPins(__exception = NULL);
  tResult CreateOutputPins(__exception = NULL);
  
  tResult CreateDescriptors();
  tResult SendToActuators();
 
  std::map<IPin*, A2O::InputPin> _inputPinMap;
  std::map<IPin*, A2O::OutputPin> _outputPinMap;
  std::map<IPin*, cObjectPtr<IMediaTypeDescription>> _descriptors;

  std::thread _actuatorthread;
  bool _running = false;
  void actuatorThread();

  void CreateRuntime();
  void setVisualization();
  WorldViewer* _visWidget = nullptr;

 protected:
  tHandle CreateView() override;
  tResult ReleaseView() override;

};
