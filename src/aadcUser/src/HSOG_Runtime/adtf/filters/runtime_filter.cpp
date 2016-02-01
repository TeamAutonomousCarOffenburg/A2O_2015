#include <iostream>

#include "runtime_filter.h"

#include "general/IComponentFactory.h"
#include "general/ComponentFactory.h"
#include "general/LogComponentFactory.h"

#ifdef __linux__

#include <sys/prctl.h>

#endif 

ADTF_FILTER_PLUGIN("HSOG RuntimeFilter", OID_HSOG_RUNTIME_FILTER, RuntimeFilter);

RuntimeFilter::RuntimeFilter(const tChar* __info) : QObject(), cBaseQtFilter(__info) {

#ifdef __linux__
	prctl(PR_SET_NAME, "RuntimeFilt", 0, 0, 0);
#endif 
}


void RuntimeFilter::CreateRuntime()
{
	//A2O::IComponentFactory::ConstPtr factory(new A2O::ComponentFactory());
	A2O::IComponentFactory::ConstPtr factory(new A2O::LogComponentFactory());

	_A2ORuntime = new A2O::A2ORuntime(factory);
	_A2ORuntime->init();

	A2O::ICarMetaModel::Ptr carMetaModel = _A2ORuntime->getCarMetaModel();
	A2O::IAction::Ptr action = _A2ORuntime->getAction();
	A2O::IPerception::Ptr perception = _A2ORuntime->getPerception();

	A2O::IEventLogger::ConstPtr logger = _A2ORuntime->getEventLogger();

	_encoder = A2O::ADTFPinMessageEncoder::Ptr(new A2O::ADTFPinMessageEncoder(action, carMetaModel));

	_decoder = A2O::ADTFPinMessageDecoder::Ptr(new A2O::ADTFPinMessageDecoder(perception, carMetaModel, logger));
	setVisualization();
}

void RuntimeFilter::setVisualization()
{
	if(_visWidget)
	{
		_visWidget->setWorldModel(_A2ORuntime->getWorldModel());
		_visWidget->setCarModel(_A2ORuntime->getCarModel());
	}
}

tResult RuntimeFilter::OnKeyEvent(tKeyEventCode eCode, tInt nParam1, tInt nParam2, tInt nFlags, tVoid* pEventData)
{
	if(eCode == adtf::IKeyEventHandler::tKeyEventCode::EC_KeyDown && nParam1 == adtf::IKeyEventHandler::tKey::KEY_L)
	{
		QKeyEvent *event = new QKeyEvent(QEvent::KeyPress, Qt::Key_L, Qt::NoModifier);
		QCoreApplication::postEvent(_visWidget, event);
	}
	RETURN_NOERROR;
}

tResult RuntimeFilter::ReleaseView()
{
	if(_visWidget != nullptr)
	{
		delete _visWidget;
		_visWidget = nullptr;
	}
	RETURN_NOERROR;
}

tHandle RuntimeFilter::CreateView()
{
	QWidget* parent = (QWidget*)m_pViewport->VP_GetWindow();
	_visWidget = new WorldViewer(parent);
	_visWidget->setWorldModel(_A2ORuntime->getWorldModel());
	_visWidget->setCarModel(_A2ORuntime->getCarModel());

	return (tHandle) _visWidget;
}


RuntimeFilter::~RuntimeFilter() {  }

tResult RuntimeFilter::CreateInputPins(__exception) {
	std::vector<A2O::InputPin> inputPins = _decoder->getPinConfigs();

	int index = 0;
	for(auto it = inputPins.begin(); it != inputPins.end(); ++it, index++) {
		std::string sensorname = (*it).name;
		std::string sensorSignalType = (*it).signalType;

		std::cout << "Creating input pin " << sensorname << " with type " << sensorSignalType << std::endl;

		cPin* pin;
		if (sensorSignalType != "VIDEO") {
			pin = new cInputPin();
			RETURN_IF_FAILED(((cInputPin*)pin)->Create(sensorname.c_str(),
						new cMediaType(0, 0, 0, sensorSignalType.c_str()),
						static_cast<IPinEventSink*>(this)));
		} else {
			pin = new cVideoPin();
			RETURN_IF_FAILED(((cVideoPin*)pin)->Create(sensorname.c_str(), adtf::IPin::PD_Input,
						static_cast<IPinEventSink*>(this)));
		}


		RETURN_IF_FAILED(RegisterPin(pin));
		_inputPinMap.insert(make_pair(pin, *it));
	}
	RETURN_NOERROR;
}

tResult RuntimeFilter::CreateDescriptors(){

	IMediaDescriptionManager* descManager;
	_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER , (tVoid**)&descManager, NULL);

	for(auto it = _inputPinMap.begin(); it!= _inputPinMap.end(); ++it)
	{
		A2O::InputPin pin = (*it).second;
		tChar const* strDescSignalValue = descManager->GetMediaDescription(pin.signalType.c_str());
		IMediaType* pTypeSignalValue = new cMediaType(0, 0, 0, pin.signalType.c_str(), strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
		pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal);

		_descriptors[(*it).first] = m_pCoderDescSignal;

	}

	for(auto it = _outputPinMap.begin(); it!= _outputPinMap.end(); ++it)
	{
		A2O::OutputPin pin = (*it).second;
		tChar const* strDescSignalValue = descManager->GetMediaDescription(pin.signalType.c_str());
		IMediaType* pTypeSignalValue = new cMediaType(0, 0, 0, pin.signalType.c_str(), strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
		pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal);

		_descriptors[(*it).first] = m_pCoderDescSignal;

	}

	RETURN_NOERROR;
}

tResult RuntimeFilter::CreateOutputPins(__exception) {
	std::vector<A2O::OutputPin> outputPins = _encoder->getOutputPins();

	int index = 0;
	for(auto it = outputPins.begin(); it != outputPins.end(); ++it, index++) {
		std::string sensorname = (*it).name + "_output";
		std::string sensorSignalType = (*it).signalType;

		std::cout << "Creating pin " << sensorname << " with type " << sensorSignalType << std::endl;

		cOutputPin* pin = new cOutputPin();
		RETURN_IF_FAILED(((cOutputPin*)pin)->Create(sensorname.c_str(),
					new cMediaType(0, 0, 0, sensorSignalType.c_str()),
					static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(pin));

		_outputPinMap.insert(make_pair(pin, *it));
	}
	RETURN_NOERROR;
}

tResult RuntimeFilter::Init(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {
		std::cout << "Creating pins for runtimefilter " << std::endl;
		CreateRuntime();

		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
	}

	if (eStage == StageGraphReady) {
		CreateDescriptors();
		std::cout << "Runtime service filter started" << std::endl;
		cObjectPtr<IApplication> pApplication;
		if(IS_OK(_runtime->GetObject(OID_ADTF_APPLICATION, IID_ADTF_APPLICATION, (tVoid**) &pApplication)))
		{
			tHandle hKeyEventManager = NULL;
			cException oLocalEx;
			tResult nResult = pApplication->GetInternalHandle(ADTF_APPLICATION_KEYEVENTMANAGER_HANDLE, &hKeyEventManager, &oLocalEx);
			if(IS_OK(nResult))
			{
				m_pKeyEventManager = static_cast<IGlobalKeyEventManager*>(hKeyEventManager);
			}

		}
	}
	RETURN_NOERROR;
}


void RuntimeFilter::actuatorThread(){

	std::chrono::system_clock::time_point beforeCycle;
	std::chrono::system_clock::time_point afterCycle;


	while(_running)
	{
		long elapsedTimeMs;
		beforeCycle = std::chrono::system_clock::now();
		SendToActuators();	
		afterCycle = std::chrono::system_clock::now();

		elapsedTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(afterCycle - beforeCycle).count();

		chrono::milliseconds dura(20 - elapsedTimeMs);
		this_thread::sleep_for(dura);
		while(_A2ORuntime->update());
	}

}


tResult RuntimeFilter::Start(__exception) {
	std::cout << "starting  runtime filter" << std::endl;
	
	CreateRuntime();
	_A2ORuntime->start();
	_running = true;
	_actuatorthread = std::thread(&RuntimeFilter::actuatorThread, this);

#ifdef __linux__
	auto handle = _actuatorthread.native_handle();
	pthread_setname_np(handle, "ActuatorThread");
#endif 

	if(m_pKeyEventManager)
	{
		ucom::cException oLocalEx;
		if(IS_FAILED(m_pKeyEventManager->RegisterKeyEventHandler(this, 0, &oLocalEx)))
		{
			__catch_exception(oLocalEx);
			{}
		}

	}
	return cBaseQtFilter::Start(__exception_ptr);
}
tResult RuntimeFilter::Stop(__exception) {

	try{

		std::cout << "stopping  runtime filter" << std::endl;
		_running = false;
		_A2ORuntime->stop();
		_actuatorthread.join();

		delete _A2ORuntime;
		_decoder.reset();
		_encoder.reset();
	}
	catch(const std::exception& ex)
	{
		std::cout << " exeption: " << ex.what() << endl;
	}
	catch(...)
	{
		std::exception_ptr eptr = std::current_exception();

		try{
			rethrow_exception(eptr);
		}
		catch(const std::exception& e)
		{
			std::cout << " exeption: " << e.what() << endl;
		}
	}

	if(m_pKeyEventManager)
	{
		m_pKeyEventManager->UnregisterKeyEventHandler(this);
	}
	return cBaseQtFilter::Stop(__exception_ptr);
}

tResult RuntimeFilter::Shutdown(tInitStage eStage, __exception) {
	if (eStage == StageGraphReady) {
	}
	return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult RuntimeFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived && _running) {

		auto pin = _inputPinMap.find(pSource);
		if(pin!=_inputPinMap.end())
		{
			IMediaTypeDescription* mediaTypeDescription = _descriptors[pSource];
			_decoder->decode(pin->second, mediaTypeDescription, pMediaSample);		
		}
		else
		{
			std::cerr << "could not find matching pin " << std::endl;
		}

	}
	RETURN_NOERROR;
}

tResult RuntimeFilter::SendToActuators(){

	_A2ORuntime->act();
	for(auto it = _outputPinMap.begin(); it!= _outputPinMap.end(); ++it)
	{
		IMediaTypeDescription* descriptor = _descriptors[it->first];

		cObjectPtr<IMediaSample> mediaSample;
		AllocMediaSample(&mediaSample);

		if(_encoder->encode(it->second,descriptor, mediaSample))
		{
			it->first->Transmit(mediaSample);
		}

	}
	RETURN_NOERROR;
}

