// arduinofilter.cpp : Definiert die exportierten Funktionen für die DLL-Anwendung.
//
#include "stdafx.h"
#include "cWatchdogTrigger.h"

ADTF_FILTER_PLUGIN("AADC WatchDog TRIGGER", OID_ADTF_WATCHDOGTRIGGER_FILTER, cWatchdogTrigger)

    cWatchdogTrigger::cWatchdogTrigger(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
{
    SetPropertyInt("Transmit rate in ms", 250);
    SetPropertyStr("Transmit rate in ms" NSSUBPROP_DESCRIPTION, "Sets the interval between two sent watchdog triggers in msec"); 
    SetPropertyInt("Transmit rate in ms" NSSUBPROP_MIN, 10); 
    SetPropertyInt("Transmit rate in ms" NSSUBPROP_MAX, 500);
}

cWatchdogTrigger::~cWatchdogTrigger()
{
}

tResult cWatchdogTrigger::CreateOutputPins(__exception)
{    
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);        
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescOutput)); 

    RETURN_IF_FAILED(m_oOutputPin.Create("WatchdogAliveSignal",pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputPin));

    RETURN_NOERROR;
}

tResult cWatchdogTrigger::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

        if (eStage == StageFirst)
        {
            tResult nResult = CreateOutputPins();
            if (IS_FAILED(nResult))
            {
                THROW_ERROR_DESC(nResult, "Failed to create Output Pins");
            }

        }
        else if (eStage == StageNormal)
        {
            tUInt32 t = GetPropertyInt("Transmit rate in ms");
            this->SetInterval(t * 1000);  //cycle time 250 ms

        }
        else if (eStage == StageGraphReady)
        {
            // no ids were set so far
            m_bIDsBoolSet = tFalse;
        }
        RETURN_NOERROR;
}

tResult cWatchdogTrigger::Cycle(__exception)
{
    //write values with zero
    tBool value = tTrue;
    tUInt32 timeStamp = 0;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescOutput,pMediaSample,pCoderOutput);

        // set the ids if not already done
        if(!m_bIDsBoolSet)
        {
            pCoderOutput->GetID("bValue", m_szIDbValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDArduinoTimestamp);
            m_bIDsBoolSet = tTrue;
        }      

        //get values from media sample        
        pCoderOutput->Set(m_szIDbValue, (tVoid*)&(value));    
        pCoderOutput->Set(m_szIDArduinoTimestamp, (tVoid*)&timeStamp);
    }    

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputPin.Transmit(pMediaSample);


    RETURN_NOERROR;
}








