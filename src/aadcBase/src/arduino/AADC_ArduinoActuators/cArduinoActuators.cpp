/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-21 12:31:24#$ $Rev:: 35328   $
**********************************************************************/


#include "stdafx.h"
#include "cArduinoActuators.h"

#define INITIAL_TIME_SPEEDCONTROLLER_SEC 6


ADTF_FILTER_PLUGIN("AADC Actuators", OID_ADTF_ACTUATOR_FILTER, cArduinoActuators)

cArduinoActuators::cArduinoActuators(const tChar* __info) : cFilter(__info), m_bDebugModeEnabled(tFalse),m_bDebugModeEnabledWatchdog(tFalse), m_bEmergencyStopBlock(tFalse), m_ui8lightMask(0)
{
    // creating the filter properties
    SetPropertyBool("Debug Output to Console", m_bDebugModeEnabled);	
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)"); 

    SetPropertyBool("Omit Watchdog Signal from Console Output",tTrue);    
    SetPropertyStr("Omit Watchdog Signal from Console Output" NSSUBPROP_DESCRIPTION, "If disabled all the watchdog sample are also printed to console (Warning: decreases performance)"); 
}

cArduinoActuators::~cArduinoActuators()
{
}

//****************************
tResult cArduinoActuators::CreateInputPins(__exception)
{
    //get the media description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
        (tVoid**)&pDescManager, __exception_ptr));

    //get description for bool signal value input pin
    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);

    //get mediatype for tBoolSignalValue data    
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get mediatype description for bool signal value data type
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolSignalInput));

    //get description for tSignalValue input pin
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescSignalValue);

    //get mediatype for tSignalValue data    
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get media description for tBoolSignalValue data    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalInput));

    //get description for tJuryEmergencyStop input pin
    tChar const * strDescEmergencyStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescEmergencyStop);

    //get media description for tJuryEmergencyStop data    
    cObjectPtr<IMediaType> pTypeEmergencyStop = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescEmergencyStop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get description for tJuryEmergencyStop input pin
    RETURN_IF_FAILED(pTypeEmergencyStop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));

    // create input pins

    // speed controller input pin
    RETURN_IF_FAILED(m_oAccelerateInputPin.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oAccelerateInputPin));

    // Steer Angle Input pin
    RETURN_IF_FAILED(m_oSteerInputPin.Create("SteeringController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteerInputPin));


    // Beam Input pin
    RETURN_IF_FAILED(m_oHeadlightInputPin.Create("headLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oHeadlightInputPin));

    // Indicator Input pin
    RETURN_IF_FAILED(m_oTurnSignalLeftInputPin.Create("turnSignalLeftEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalLeftInputPin));

    RETURN_IF_FAILED(m_oTurnSignalRightInputPin.Create("turnSignalRightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalRightInputPin));

    // hazzard input pin
    RETURN_IF_FAILED(m_oHazardLightsInputPin.Create("hazzardLightsEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oHazardLightsInputPin));

    // Brakelight Input pin
    RETURN_IF_FAILED(m_oBrakeLightInputPin.Create("brakeLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oBrakeLightInputPin));

    // Reversinglights Input pin
    RETURN_IF_FAILED(m_oReverseLightInputPin.Create("reverseLightsEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oReverseLightInputPin));

    // watchdog Input pin
    RETURN_IF_FAILED(m_oWatchdogInputPin.Create("Watchdog_Alive_Flag", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oWatchdogInputPin));

    // Emergency_Stop input pin
    RETURN_IF_FAILED(m_oEmergencyStopInputPin.Create("Emergency_Stop", pTypeEmergencyStop, this));
    RETURN_IF_FAILED(RegisterPin(&m_oEmergencyStopInputPin));       

    RETURN_NOERROR;
}

//*****************************************************************************

tResult cArduinoActuators::CreateOutputPins(__exception)
{    
    //get the media description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    //get description for arduino output pin
    tChar const * strDescArduino = pDescManager->GetMediaDescription("tArduinoData");

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescArduino);

    //get mediatype for arduino data    
    cObjectPtr<IMediaType> pTypeArduinoData = new cMediaType(0, 0, 0, "tArduinoData", strDescArduino,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    

    //get mediatype description for arduino data type
    RETURN_IF_FAILED(pTypeArduinoData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionArduinoData)); 

    // creates the output pin
    RETURN_IF_FAILED(m_oArduinoCOMOutPin.Create("ArduinoCOM_output", pTypeArduinoData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oArduinoCOMOutPin));

    RETURN_NOERROR;    
}

tResult cArduinoActuators::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

        if (eStage == StageFirst)
        {
            // creates the output pins
            tResult nResult = CreateOutputPins();
            if (IS_FAILED(nResult))
            {
                THROW_ERROR_DESC(nResult, "Failed to create Output Pins");
            }

            // creates the input pins
            nResult = CreateInputPins(__exception_ptr);
            if (IS_FAILED(nResult))
            {
                THROW_ERROR_DESC(nResult, "Failed to create Input Pins");
            }
        }
        else if (eStage == StageNormal)
        {
            // get the properties and set them to the class member variables
            m_bDebugModeEnabledWatchdog = GetPropertyBool("Omit Watchdog Signal from Console Output");
            m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
        }
        else if (eStage == StageGraphReady)
        {
            m_bIDsSignalInputSet=tFalse;
            m_bIDsEmergencyStopSet = tFalse;
            m_bIDsBoolSignalInputSet = tFalse;
            m_bIDsArduinoSet = tFalse;
			m_ui8lightMask = 0;

            // no sample was received so far
            m_timestampFirstSpeedSample = -1;
        }
        RETURN_NOERROR;
}

tResult cArduinoActuators::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cArduinoActuators::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cArduinoActuators::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}


tResult cArduinoActuators::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    // something received
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        // switch depending on what input pin was used
        if (pSource == &m_oHeadlightInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_HEAD);
        }        

        if (pSource == &m_oTurnSignalLeftInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_TURNLEFT);
        }

        if (pSource == &m_oTurnSignalRightInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_TURNRIGHT);    
        }

        if (pSource == &m_oHazardLightsInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_HAZZARD);    
        }

        if (pSource == &m_oBrakeLightInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_BRAKE);
        }

        if (pSource == &m_oReverseLightInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_REVERSE);
        }

        if (pSource == &m_oEmergencyStopInputPin)
        {
            ProcessEmergencyStop(pMediaSample);
        }

        if (pSource == &m_oSteerInputPin)
        {
            ProcessActuatorValue(pMediaSample, ID_ARD_ACT_STEER_SERVO);
        }

        if (pSource == &m_oAccelerateInputPin)
        {
            ProcessActuatorValue(pMediaSample, ID_ARD_ACT_SPEED_CONTR);
        }

        if (pSource == &m_oWatchdogInputPin)
        {
            ProcessWatchdog(pMediaSample);
        }
    }
    RETURN_NOERROR;
}

tResult cArduinoActuators::ProcessEmergencyStop(IMediaSample* pMediaSample)
{
    if (pMediaSample != NULL && m_pDescriptionEmergencyStop != NULL)
    {
        tBool bValue = tFalse;

        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pDescriptionEmergencyStop,pMediaSample,pCoder);      

            // set the id if not already done
            if(!m_bIDsEmergencyStopSet)
            {
                pCoder->GetID("bEmergencyStop", m_szIDEmergencyStopBEmergencyStop);
                m_bIDsEmergencyStopSet = tTrue;
            }      

            // get value from sample
            pCoder->Get(m_szIDEmergencyStopBEmergencyStop, (tVoid*)&bValue);     
        }

        //set watchdog active in arduino to tFalse
        if(bValue == tTrue)
        {
            m_bEmergencyStopBlock = tTrue;
            tUInt8 ui8Data = 0x01;
            // build and send arduino frame
            BuildArduinoFrame(ID_ARD_ACT_EMERGENCY_STOP,1,&ui8Data);
            LOG_WARNING(cString::Format("Emergency_Stop now!")); 
        }
    }
    RETURN_NOERROR;
}


tResult cArduinoActuators::ProcessWatchdog(IMediaSample* pMediaSample)
{        
    //enters critical section
    __synchronized_obj(m_oWatchDogCritSection);

    if (pMediaSample != NULL && m_pDescriptionBoolSignalInput != NULL && m_bEmergencyStopBlock == tFalse)
    {
        //write values with zero
        tBool bValue = tFalse;
        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pDescriptionBoolSignalInput,pMediaSample,pCoder);
            // set the ids if not already done
            if(!m_bIDsBoolSignalInputSet)
            {
                pCoder->GetID("bValue", m_szIDBoolSignalInputF32Value);
                pCoder->GetID("ui32ArduinoTimestamp", m_szIDBoolSignalInputArduinoTimestamp);
                m_bIDsBoolSignalInputSet = tTrue;
            }      

            //get values from media sample        
            pCoder->Get(m_szIDBoolSignalInputF32Value, (tVoid*)&bValue); 
        }

        // variable for flag in arduino frame
        tUInt8 ui8Data = static_cast<tUInt8> (bValue);
        if (bValue == tTrue)
        {
            BuildArduinoFrame(ID_ARD_ACT_WATCHDOG,1,&ui8Data);
        }
    }
    RETURN_NOERROR;
}


tResult cArduinoActuators::ProcessActuatorValue( IMediaSample* pMediaSample, tUInt8 ui8ChID )
{  
    // enters the critical section
    __synchronized_obj(m_oActuatorCritSection);

    if (pMediaSample != NULL && m_pDescriptionSignalInput != NULL)
    {
        //write values with 90: zero position for steering and wheel
        tFloat32 f32Value = 90;
        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalInput,pMediaSample,pCoder);  

            // set the ids if not already done
            if(!m_bIDsSignalInputSet)
            {
                pCoder->GetID("f32Value", m_szIDSignalInputF32Value);
                pCoder->GetID("ui32ArduinoTimestamp", m_szIDSignalInputArduinoTimestamp);
                m_bIDsSignalInputSet = tTrue;
            }      

            //get values from media sample        
            pCoder->Get(m_szIDSignalInputF32Value, (tVoid*)&f32Value);    
        }


        // because the ARDUINO protocol defines the range between 0...180 for speed controller and 60...120 for steering calibration
        // the values must be limited 
        switch (ui8ChID)
        {
        case (ID_ARD_ACT_SPEED_CONTR):
            if (m_timestampFirstSpeedSample==-1)
            {
                // first frame for speed controller is now received
                // set timestamp and transmit 90.0f
                m_timestampFirstSpeedSample = _clock->GetStreamTime();
                f32Value = 90.0f;
            }
            else if (_clock->GetStreamTime() - m_timestampFirstSpeedSample < INITIAL_TIME_SPEEDCONTROLLER_SEC*1e6)
            {
                // the speed controller needs in initial phase of startup the zero position of servo which is at 90.0°
                // transmit 90.0f
                f32Value = 90.0f;
            }
            else if (f32Value > 180.0f)
            {
                // the speed controller is initialized
                f32Value = 180.0f;
                LOG_WARNING("Actuator value out of range. Setting to max. 180");
            }
            else if (f32Value < 0.0f)
            {
                // the speed controller is initialized
                f32Value = 0.0f;
                LOG_WARNING("Actuator value out of range. Setting to min. 0");
            }

            break;
        case (ID_ARD_ACT_STEER_SERVO):            
            if (f32Value > 120.0f)
            {
                f32Value = 120.0f;
                LOG_WARNING("Actuator value out of range. Setting to max. 120");
            }
            else if (f32Value < 60.0f)
            {
                f32Value = 60.0f;
                LOG_WARNING("Actuator value out of range. Setting to min. 60");
            }
            break;
        default:
            break;
        }            

        // cast from float to tUInt8 with correct rounding
        tUInt8 ui8Data = static_cast<tUInt8> (f32Value + 0.5f);

        // build and send arduino data
        BuildArduinoFrame(ui8ChID,1,&ui8Data);
    }
    RETURN_NOERROR;
}

tResult cArduinoActuators::ProcessLights(IMediaSample* pMediaSample, tUInt8 ui8LightID)
{   
    // enters critical section
    __synchronized_obj(m_oLightsCritSection); 

    if (pMediaSample != NULL && m_pDescriptionBoolSignalInput != NULL)
    {
        // temp variables for receiving data
        tBool bValue = tFalse;

        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pDescriptionBoolSignalInput,pMediaSample,pCoder);

            // set the ids if not already done
            if(!m_bIDsBoolSignalInputSet)
            {
                pCoder->GetID("bValue", m_szIDBoolSignalInputF32Value);
                pCoder->GetID("ui32ArduinoTimestamp", m_szIDBoolSignalInputArduinoTimestamp);
                m_bIDsBoolSignalInputSet = tTrue;
            }      

            //get values from media sample        
            pCoder->Get(m_szIDBoolSignalInputF32Value, (tVoid*)&bValue); 
        }

		//Special case: If turn signal switch from left to right. Disable other turn signal.
		if(ui8LightID == ID_ARD_ACT_LIGHT_MASK_TURNLEFT){
			 m_ui8lightMask &= ~ID_ARD_ACT_LIGHT_MASK_TURNRIGHT;
		}
		
		if(ui8LightID == ID_ARD_ACT_LIGHT_MASK_TURNRIGHT){
			 m_ui8lightMask &= ~ID_ARD_ACT_LIGHT_MASK_TURNLEFT;
		}

        // set enable or disable in frame
        if (bValue==tTrue)
        {
            m_ui8lightMask |= ui8LightID;
            if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Actuatorfilter Light: ID %d switched on",ui8LightID));
        }
        else 
        {
            m_ui8lightMask &= ~ui8LightID;
            if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Actuatorfilter Light: ID %d switched off",ui8LightID));
        }       

        // build and send arduino frame
        BuildArduinoFrame(ID_ARD_ACT_LIGHT,2,&m_ui8lightMask);
    }

    RETURN_NOERROR; 
}

tResult cArduinoActuators::BuildArduinoFrame( tUInt8 ch8Id, tUInt8 i8DataLength, tUInt8 *pChData )
{    
    // enters critical section
    __synchronized_obj(m_oTransmitCritSection); 

    // build empty frame
    tUInt8 aChFrameData[25];
    cMemoryBlock::MemSet(aChFrameData, 0x00, sizeof(aChFrameData));
    cMemoryBlock::MemCopy(aChFrameData, pChData, i8DataLength);

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionArduinoData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionArduinoData,pMediaSample,pCoder);

        tUInt8 i8SOF = ID_ARD_SOF; 
        tUInt32 ui32ArduinoTimestamp = 0;  

        // set the ids if not already done
        if(!m_bIDsArduinoSet)
        {
            pCoder->GetID("ui8SOF", m_szIDArduinoUi8SOF);
            pCoder->GetID("ui8ID", m_szIDArduinoUi8ID);
            pCoder->GetID("ui32ArduinoTimestamp", m_szIDArduinoUi32ArduinoTimestamp);
            pCoder->GetID("ui8DataLength", m_szIDArduinoUi8DataLength);
            pCoder->GetID("ui8Data", m_szIDArduinoUi8Data);
            m_bIDsArduinoSet = tTrue;
        }

        pCoder->Set(m_szIDArduinoUi8SOF, (tVoid*)&i8SOF);    
        pCoder->Set(m_szIDArduinoUi8ID, (tVoid*)&ch8Id);
        pCoder->Set(m_szIDArduinoUi32ArduinoTimestamp, (tVoid*)&(ui32ArduinoTimestamp));
        pCoder->Set(m_szIDArduinoUi8DataLength, (tVoid*)&i8DataLength);        
        // TODO: setting value with m_szIDArduinoUi8Data is not working here
        pCoder->Set("ui8Data", (tVoid*)&(aChFrameData));
    }     

    // set the time of the sample
    pMediaSample->SetTime(_clock->GetStreamTime());

    //transmit media sample over output pin
    RETURN_IF_FAILED(m_oArduinoCOMOutPin.Transmit(pMediaSample));       

    // check if output is enabled and id is not watchdog   OR         id is watchdog and watchdog output is enabled and output is enabled
    if (i8DataLength > 0)
    {    
        if ((m_bDebugModeEnabled && ch8Id != ID_ARD_ACT_WATCHDOG) 
            || (ch8Id == ID_ARD_ACT_WATCHDOG && !(m_bDebugModeEnabledWatchdog) && m_bDebugModeEnabled) ) 
            LOG_INFO(cString::Format("Actuatorfilter sent: ID %x DataLength %d First Value %d",ch8Id,i8DataLength,aChFrameData[0]));   
    }
    RETURN_NOERROR;
}




