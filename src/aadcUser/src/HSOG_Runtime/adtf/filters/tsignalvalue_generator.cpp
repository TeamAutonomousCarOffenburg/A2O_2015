#include "tsignalvalue_generator.h"

#include <iostream>
using namespace adtf;

ADTF_FILTER_PLUGIN("HSOG tsignalValueGenerator", OID_ADTF_HSOG_TSIGNALVALUE, tsignalValueGenerator);

tsignalValueGenerator::tsignalValueGenerator(const tChar* __info) : cTimeTriggeredFilter(__info) {}

tsignalValueGenerator::~tsignalValueGenerator() {}

tResult tsignalValueGenerator::CreateOutputPins(__exception) {
	cObjectPtr<IMediaDescriptionManager> pDescManager;

	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
				(tVoid**)&pDescManager, __exception_ptr));

	tChar const* strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
	RETURN_IF_POINTER_NULL(strDescSignalValue);
	cObjectPtr<IMediaType> pTypeSignalValue =
		new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(
			pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

	RETURN_IF_FAILED(
			m_oOutput.Create("tSignalValueOutputPin", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&m_oOutput));

	RETURN_NOERROR;
}

tResult tsignalValueGenerator::Init(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {
		std::cout << "Creating pin for tsignalvaluegeneratorfilter" << std::endl;
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
	}
	if (eStage == StageGraphReady) {
		RETURN_IF_FAILED(SetInterval(50000));
	}
	RETURN_NOERROR;
}

tResult tsignalValueGenerator::Start(__exception) { return cTimeTriggeredFilter::Start(__exception_ptr); }

tResult tsignalValueGenerator::Stop(__exception) { return cTimeTriggeredFilter::Stop(__exception_ptr); }

tResult tsignalValueGenerator::Shutdown(tInitStage eStage, __exception) {
	return cTimeTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult tsignalValueGenerator::Cycle(__exception) { return sendData(); }

tResult tsignalValueGenerator::sendData() {
	static float value = 0;

	value++;

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample(&pMediaSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	RETURN_IF_FAILED(m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer));

	tInt nSize = pSerializer->GetDeserializedSize();
	pMediaSample->AllocBuffer(nSize);

	__adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pMediaSample, pCoderOutput);

	pCoderOutput->Set("f32Value", (tVoid*)&(value));

	tUInt32 timeStamp = (tUInt32) value;
	pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&(timeStamp));

	pMediaSample->SetTime(pMediaSample->GetTime());
	m_oOutput.Transmit(pMediaSample);

	RETURN_NOERROR;
}
