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
* $Author:: spiesra $  $Date:: 2015-05-20 10:31:32#$ $Rev:: 35239   $
**********************************************************************/

#include "stdafx.h"
#include "cArduinoSensors.h"

// define some constants for calculations
#define CONVERT_FACTOR_QUATERNION 1/16384.0f
#define CONVERT_FACTOR_ACCELERATION 9.80665f/8000.0f
#define CONVERT_FACTOR_VOLTAGE 1/1024*5*25.f/10.f
#define CONVERT_FACTOR_ULTRASONIC 1/100.f

ADTF_FILTER_PLUGIN("AADC Arduino Sensors", OID_ARDUINO_SENSORS_FILTER, cArduinoSensors)

cArduinoSensors::cArduinoSensors(const tChar* __info) : cFilter(__info)
{    
    m_bDebugModeEnabled = tFalse;
    
    // create the filter properties
    SetPropertyBool("Debug Output to Console", m_bDebugModeEnabled);
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)"); 
}

cArduinoSensors::~cArduinoSensors()
{
}

//****************************
tResult cArduinoSensors::CreateInputPins(__exception)
{    
    //get the media description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    //get description for arduino input pin
    tChar const * strDescArduino = pDescManager->GetMediaDescription("tArduinoData");
    
    // checks if exists
    RETURN_IF_POINTER_NULL(strDescArduino);
    
    //get mediatype for arduino data    
    cObjectPtr<IMediaType> pTypeArduinoData = new cMediaType(0, 0, 0, "tArduinoData", strDescArduino,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    
    //get mediatype description for arduino data type
    RETURN_IF_FAILED(pTypeArduinoData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionArduinoData)); 

    // creates the input pin
    RETURN_IF_FAILED(m_oArduinoCOMInputPin.Create("ArduinoCOM_input", pTypeArduinoData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oArduinoCOMInputPin));

    RETURN_NOERROR;
}

//*****************************************************************************

tResult cArduinoSensors::CreateOutputPins(__exception)
{
    //get the description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


    //get description for sensor data pins
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    
    //get mediatype for ultrasonic sensor data pins
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    



    //get description for inertial measurement sensor data pin
    tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");    

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescInerMeasUnit);

    //get mediatype for ultrasonic sensor data pins
    cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        


    //get description for wheel sensors data pins
    tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
    
    // checks if exists
    RETURN_IF_POINTER_NULL(strDescWheelData);    

    //get mediatype for ultrasonic sensor data pins
    cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        
        

    //get description for ultrasonic struct data pins
    tChar const * strDescUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
    
    // checks if exists
    RETURN_IF_POINTER_NULL(strDescUltrasonicStruct);    

    //get mediatype for ultrasonic sensor data pins
    cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        


    //get mediatype description for ultrasonic sensor data type
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsData));

    //get mediatype description for voltage sensor data type
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionVoltData));

    //get mediatype description for inertial measurement unit sensor data type
    RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));

    //get mediatype description for inertial measurement unit sensor data type
    RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelData));

    //get mediatype description for ultrasonic data struct data type
    RETURN_IF_FAILED(pTypeUltrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStructData));

    //create pins for ultrasonic sensor data
    RETURN_IF_FAILED(m_oOutputUsFrontLeft.Create("Ultrasonic_Front_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontLeft));
        
    RETURN_IF_FAILED(m_oOutputUsFrontCenterLeft.Create("Ultrasonic_Front_Center_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontCenterLeft));
        
    RETURN_IF_FAILED(m_oOutputUsFrontCenter.Create("Ultrasonic_Front_Center", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontCenter));
        
    RETURN_IF_FAILED(m_oOutputUsFrontCenterRight.Create("Ultrasonic_Front_Center_Right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontCenterRight));
        
    RETURN_IF_FAILED(m_oOutputUsFrontRight.Create("Ultrasonic_Front_Right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontRight));
        
    RETURN_IF_FAILED(m_oOutputUsSideLeft.Create("Ultrasonic_Side_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsSideLeft));
        
    RETURN_IF_FAILED(m_oOutputUsSideRight.Create("Ultrasonic_Side_Right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsSideRight));
        
    RETURN_IF_FAILED(m_oOutputUsRearLeft.Create("Ultrasonic_Rear_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsRearLeft));
        
    RETURN_IF_FAILED(m_oOutputUsRearCenter.Create("Ultrasonic_Rear_Center", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsRearCenter));
        
    RETURN_IF_FAILED(m_oOutputUsRearRight.Create("Ultrasonic_Rear_Right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsRearRight));

    RETURN_IF_FAILED(m_oOutputUsStruct.Create("Ultrasonic", pTypeUltrasonicStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsStruct));

    //create pins for voltage sensor data
    RETURN_IF_FAILED(m_oOutputVoltMeas.Create("Voltage_Measurement", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputVoltMeas));
        
    RETURN_IF_FAILED(m_oOutputVoltSpeedCtr.Create("Voltage_SpeedCntrl", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputVoltSpeedCtr));


    //create pin for inertial measurement unit data
    RETURN_IF_FAILED(m_oOutputInerMeasUnit.Create("InerMeasUnit_Struct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputInerMeasUnit));


    //create pin for wheel left sensor data
    RETURN_IF_FAILED(m_oOutputWheelLeft.Create("WheelLeft_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputWheelLeft));

    //create pin for wheel right data
    RETURN_IF_FAILED(m_oOutputWheelRight.Create("WheelRight_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputWheelRight));
    RETURN_NOERROR;
}

//*****************************************************************************

tResult cArduinoSensors::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        // creates the output pins
        tResult nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");
        }

        // creates the input pins
        nResult = CreateInputPins();
        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");
        }        
    }
    else if (eStage == StageNormal)
    {
      // get the filter properties and set to member variables
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");  
    }    
    else if (eStage == StageGraphReady)
    {
       // the ids for the items in the structs of the mediasamples still have to be set
       m_bIDsArduinoSet = tFalse;
       m_bIDsInerMeasUnitSet = tFalse;
       m_bIDsVoltageSet = tFalse;
       m_bIDsUltrasonicSet = tFalse;
       m_bIDsWheelDataSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult cArduinoSensors::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cArduinoSensors::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cArduinoSensors::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cArduinoSensors::TransmitInerMeasUnitData(tTimeStamp inputTimeStamp, const tUInt32 ui32ArduinoTimestamp,const tUInt8* pFrameData)
{
    //cast to imu struct
    tImuData *sArduinoData =  (tImuData*) pFrameData;
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionInerMeasUnitData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    
    //divide by 16384.0f (maximum value from raw value from sensor)
    tFloat32 f32Q_w =tFloat32(sArduinoData->i16Q_w)*CONVERT_FACTOR_QUATERNION;
    tFloat32 f32Q_x =tFloat32(sArduinoData->i16Q_x)*CONVERT_FACTOR_QUATERNION;
    tFloat32 f32Q_y =tFloat32(sArduinoData->i16Q_y)*CONVERT_FACTOR_QUATERNION;
    tFloat32 f32Q_z =tFloat32(sArduinoData->i16Q_z)*CONVERT_FACTOR_QUATERNION;
     
    /*
     * AFS_SEL | Full Scale Range | LSB Sensitivity
     * --------+------------------+----------------
     * 0       | +/- 2g           | 8192 LSB/mg
     * 1       | +/- 4g           | 4096 LSB/mg
     * 2       | +/- 8g           | 2048 LSB/mg  <- selected
     * 3       | +/- 16g          | 1024 LSB/mg
     ->
     int(8000) =^ 1g
     so the value has to be calculated as following:
     value [m/sec^2] = value * 9,81m/sec^2 /8000 = value * 
     
     */ 

    //multiplicate incoming data with factor to get correct value
    tFloat32 f32A_x =tFloat32(sArduinoData->i16A_x)*CONVERT_FACTOR_ACCELERATION;
    tFloat32 f32A_y =tFloat32(sArduinoData->i16A_y)*CONVERT_FACTOR_ACCELERATION;
    tFloat32 f32A_z =tFloat32(sArduinoData->i16A_z)*CONVERT_FACTOR_ACCELERATION;
    
    {   // focus for sample write lock
       
        __adtf_sample_write_lock_mediadescription(m_pDescriptionInerMeasUnitData,pMediaSample,pCoder);
        
        // get the IDs for the items in the media sample 
        if(!m_bIDsInerMeasUnitSet)
        {
            pCoder->GetID("f32Q_w", m_szIDInerMeasUnitF32Q_w);
            pCoder->GetID("f32Q_x", m_szIDInerMeasUnitF32Q_x);
            pCoder->GetID("f32Q_y", m_szIDInerMeasUnitF32Q_y);
            pCoder->GetID("f32Q_z", m_szIDInerMeasUnitF32Q_z);
            pCoder->GetID("f32A_x", m_szIDInerMeasUnitF32A_x);
            pCoder->GetID("f32A_y", m_szIDInerMeasUnitF32A_y);
            pCoder->GetID("f32A_z", m_szIDInerMeasUnitF32A_z);
            pCoder->GetID("ui32ArduinoTimestamp", m_szIDInerMeasUnitArduinoTimestamp);
            m_bIDsInerMeasUnitSet = tTrue;
        }
        //write date to the media sample with the coder of the descriptor
        pCoder->Set(m_szIDInerMeasUnitF32Q_w, (tVoid*)&f32Q_w);
        pCoder->Set(m_szIDInerMeasUnitF32Q_x, (tVoid*)&f32Q_x);
        pCoder->Set(m_szIDInerMeasUnitF32Q_y, (tVoid*)&f32Q_y);
        pCoder->Set(m_szIDInerMeasUnitF32Q_z, (tVoid*)&f32Q_z);
        pCoder->Set(m_szIDInerMeasUnitF32A_x, (tVoid*)&f32A_x);
        pCoder->Set(m_szIDInerMeasUnitF32A_y, (tVoid*)&f32A_y);
        pCoder->Set(m_szIDInerMeasUnitF32A_z, (tVoid*)&f32A_z);
        pCoder->Set(m_szIDInerMeasUnitArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);
    } 
    // set the correct timestamp for sample 
    pMediaSample->SetTime(inputTimeStamp);
    // transmit media sample over output pin
    RETURN_IF_FAILED(m_oOutputInerMeasUnit.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitUltrasonicData(tTimeStamp inputTimeStamp, const tUInt32 ui32ArduinoTimestamp,const tUInt8* pFrameData, SensorDefinition::ultrasonicSensor usSensor)
{   
    // enter critical section
    __synchronized_obj(m_oTransmitUltraSonicDataCritSection);
    
    //cast to ultrasonic data struct
    tUsData *sArduinoData =  (tUsData*) pFrameData;
    
    // convert from centimeter to meter
    tFloat32 f32data = sArduinoData->ui16UsData*CONVERT_FACTOR_ULTRASONIC;
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionUsData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock       
        __adtf_sample_write_lock_mediadescription(m_pDescriptionUsData,pMediaSample,pCoder);
        
        // get the IDs for the items in the media sample 
        if(!m_bIDsUltrasonicSet)
        {
            pCoder->GetID("f32Value", m_szIDUltrasonicF32Value);
            pCoder->GetID("ui32ArduinoTimestamp", m_szIDUltrasonicArduinoTimestamp);
            m_bIDsUltrasonicSet = tTrue;
        }      
        //write data to the media sample with the coder of the descriptor
        pCoder->Set(m_szIDUltrasonicArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);
        pCoder->Set(m_szIDUltrasonicF32Value, (tVoid*)&f32data);
    }    

    // set timestamp of media sample
    pMediaSample->SetTime(inputTimeStamp);
    
    //transmit media sample over output pin specified by usSensor
    switch (usSensor)
    {
        case SensorDefinition::usFrontLeft:
            m_ultrasonicDataStruct.tFrontLeft.f32Value = f32data;
            m_ultrasonicDataStruct.tFrontLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsFrontLeft.Transmit(pMediaSample));
            RETURN_IF_FAILED(TransmitUltrasonicStructData());
            break;
        case SensorDefinition::usFrontCenterLeft:
            m_ultrasonicDataStruct.tFrontCenterLeft.f32Value = f32data;
            m_ultrasonicDataStruct.tFrontCenterLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsFrontCenterLeft.Transmit(pMediaSample));
            break;
        case SensorDefinition::usFrontCenter:
            m_ultrasonicDataStruct.tFrontCenter.f32Value = f32data;
            m_ultrasonicDataStruct.tFrontCenter.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsFrontCenter.Transmit(pMediaSample));
            break;
        case SensorDefinition::usFrontCenterRight:
            m_ultrasonicDataStruct.tFrontCenterRight.f32Value = f32data;
            m_ultrasonicDataStruct.tFrontCenterRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsFrontCenterRight.Transmit(pMediaSample));
            break;
        case SensorDefinition::usFrontRight:
            m_ultrasonicDataStruct.tFrontRight.f32Value = f32data;
            m_ultrasonicDataStruct.tFrontRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsFrontRight.Transmit(pMediaSample));
            break;
        case SensorDefinition::usSideLeft:
            m_ultrasonicDataStruct.tSideLeft.f32Value = f32data;
            m_ultrasonicDataStruct.tSideLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsSideLeft.Transmit(pMediaSample));  
            break;
        case SensorDefinition::usSideRight:
            m_ultrasonicDataStruct.tSideRight.f32Value = f32data;
            m_ultrasonicDataStruct.tSideRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsSideRight.Transmit(pMediaSample)); 
            break;
        case SensorDefinition::usRearLeft:
            m_ultrasonicDataStruct.tRearLeft.f32Value = f32data;
            m_ultrasonicDataStruct.tRearLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsRearLeft.Transmit(pMediaSample)); 
            break;
        case SensorDefinition::usRearCenter:
            m_ultrasonicDataStruct.tRearCenter.f32Value = f32data;
            m_ultrasonicDataStruct.tRearCenter.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsRearCenter.Transmit(pMediaSample));
            break;
        case SensorDefinition::usRearRight:
            m_ultrasonicDataStruct.tRearRight.f32Value = f32data;
            m_ultrasonicDataStruct.tRearRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
            RETURN_IF_FAILED(m_oOutputUsRearRight.Transmit(pMediaSample));
            break;
    }
    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitUltrasonicStructData()
{    
    // create new pointer for media sample
    cObjectPtr<IMediaSample> pMediaSample = new cMediaSample();
    
    // update media sample with whole struct and current time
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetTime(), &m_ultrasonicDataStruct,sizeof(m_ultrasonicDataStruct),0));
    
    // transmit media sample
    RETURN_IF_FAILED(m_oOutputUsStruct.Transmit(pMediaSample));
    
    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitWheelData(tTimeStamp inputTimeStamp, const tUInt32 ui32ArduinoTimestamp,const tUInt8* pFrameData, SensorDefinition::wheelSensor wheelSensorType)
{       
    // enters critical section
    __synchronized_obj(m_oTransmitWheelDataCritSection);
    
    //cast to struct
    tSensWheelData *sArduinoData =  (tSensWheelData*) pFrameData;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionWheelData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock        
        __adtf_sample_write_lock_mediadescription(m_pDescriptionWheelData,pMediaSample,pCoder);
        
        // get the IDs for the items in the media sample 
        if(!m_bIDsWheelDataSet)
        {
            pCoder->GetID("i8WheelDir", m_szIDWheelDataI8WheelDir);
            pCoder->GetID("ui32WheelTach", m_szIDWheelDataUi32WheelTach);
            pCoder->GetID("ui32ArduinoTimestamp", m_szIDWheelDataArduinoTimestamp);
            m_bIDsWheelDataSet = tTrue;
        }        
        
        //write date to the media sample with the coder of the descriptor   
        pCoder->Set(m_szIDWheelDataI8WheelDir, (tVoid*)&(sArduinoData->i8WheelDir));
        pCoder->Set(m_szIDWheelDataUi32WheelTach, (tVoid*)&(sArduinoData->ui32WheelTach));
        pCoder->Set(m_szIDWheelDataArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);
    }    

    //transmit media sample over output pin
    pMediaSample->SetTime(inputTimeStamp);

    //transmits media sample defined by wheelSensorType
    switch (wheelSensorType)
    {
    case SensorDefinition::wheelLeft:
            RETURN_IF_FAILED(m_oOutputWheelLeft.Transmit(pMediaSample));
            break;
    case SensorDefinition::wheelRight:
            RETURN_IF_FAILED(m_oOutputWheelRight.Transmit(pMediaSample));
            break;
    }

    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitVoltData(tTimeStamp inputTimeStamp, const tUInt32 ui32ArduinoTimestamp,const tUInt8* pFrameData, SensorDefinition::voltageSensor voltageSensor)
{    
    //cast to struct with voltage data
    tVoltageData *arduinoData =  (tVoltageData*) pFrameData;
    
    tFloat32 f32Data = static_cast<tFloat32>(arduinoData->ui16VoltageData);
    
    // on arduino 1023 is 5V, and the voltage is measured over 15kOhm and 10kOhm
    f32Data = f32Data*CONVERT_FACTOR_VOLTAGE;
    
    // doing some sensor specified calculation
    switch (voltageSensor)
    {
    case SensorDefinition::measurementCircuit:
            // dc voltage dissipation over diode: 0.3V
            f32Data = f32Data +0.3f;
            break;
    case SensorDefinition::speedCntrlCircuit: 
            break;
    }

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionVoltData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock       
        __adtf_sample_write_lock_mediadescription(m_pDescriptionVoltData,pMediaSample,pCoder);
        
        // get the IDs for the items in the media sample 
        if(!m_bIDsVoltageSet)
        {
            pCoder->GetID("f32Value", m_szIDVoltageF32Value);
            pCoder->GetID("ui32ArduinoTimestamp", m_szIDVoltageArduinoTimestamp);
            m_bIDsVoltageSet = tTrue;
        }      
        
        //write date to the media sample with the coder of the descriptor    
        pCoder->Set(m_szIDVoltageArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);
        pCoder->Set(m_szIDVoltageF32Value, (tVoid*)&f32Data);
    }    
    
    //set time of media sample
    pMediaSample->SetTime(inputTimeStamp);
    
    //transmit media sample over output pin
    switch (voltageSensor)
    {
    case SensorDefinition::measurementCircuit:
            RETURN_IF_FAILED(m_oOutputVoltMeas.Transmit(pMediaSample));
            break;
    case SensorDefinition::speedCntrlCircuit:
            RETURN_IF_FAILED(m_oOutputVoltSpeedCtr.Transmit(pMediaSample));
            break;
    }
    
    
    RETURN_NOERROR;
}

tResult cArduinoSensors::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    // new media sample was received
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {            
        RETURN_IF_POINTER_NULL(pMediaSample);
        // new media sample with arduino data, pass it to processing
        if (pSource == &m_oArduinoCOMInputPin)
        {
            return Process_ArduinoCOMInputPin(pMediaSample);
        }
    } 
    RETURN_NOERROR;
}

tResult cArduinoSensors::Process_ArduinoCOMInputPin(IMediaSample *pMediaSample)
{
    //enters critical section, check if really necessary
    __synchronized_obj(m_oProcess_ArduinoCOMCritSection);
    
    if (pMediaSample != NULL && m_pDescriptionArduinoData != NULL)
    {
        //write values with zero
        tUInt8 ui8SOF = 0;
        tUInt8 ui8ID = 0;
        tUInt32 ui32ArduinoTimestamp = 0;
        tUInt8 ui8DataLength = 0;
        // arduino data can be maximum 25 bytes
        tUInt8 aFrameData[25];
        // set array to zero
        cMemoryBlock::MemSet(aFrameData, 0x00, sizeof(aFrameData));
        
        {   // focus for sample read lock
            // read-out the incoming Media Sample        
            __adtf_sample_read_lock_mediadescription(m_pDescriptionArduinoData,pMediaSample,pCoder);
            
            //set ids if not already done 
            if(!m_bIDsArduinoSet)
                        {
                        pCoder->GetID("ui8SOF", m_szIDArduinoUi8SOF);
                        pCoder->GetID("ui8ID", m_szIDArduinoUi8ID);
                        pCoder->GetID("ui32ArduinoTimestamp", m_szIDArduinoUi32ArduinoTimestamp);
                        pCoder->GetID("ui8DataLength", m_szIDArduinoUi8DataLength);
                        pCoder->GetID("ui8Data", m_szIDArduinoUi8Data);
                        m_bIDsArduinoSet = tTrue;
                        }
            //get values from media sample 
            pCoder->Get(m_szIDArduinoUi8SOF, (tVoid*)&ui8SOF);
            pCoder->Get(m_szIDArduinoUi8ID, (tVoid*)&ui8ID);
            pCoder->Get(m_szIDArduinoUi32ArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);
            pCoder->Get(m_szIDArduinoUi8DataLength, (tVoid*)&ui8DataLength);
            // TODO: getting value with m_szIDArduinoUi8Data is not working here
            pCoder->Get("ui8Data", (tVoid*)&(aFrameData));
        }

        //print to console if enabled
        if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Sensorfilter received: ID %x DataLength %d First Byte %f",ui8ID,ui8DataLength, static_cast<tFloat32>(aFrameData[0])));   
        
        // checing ui8ID and send to specific transmit function
        switch(ui8ID)
        {
            case ID_ARD_SENS_US_FRONT_LEFT:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usFrontLeft));
                    break;
            case ID_ARD_SENS_US_FRONT_RIGHT:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usFrontRight));
                    break;
            case ID_ARD_SENS_US_FRONT_CENTER_RIGHT:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usFrontCenterRight));
                    break;
            case ID_ARD_SENS_US_FRONT_CENTER_LEFT:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usFrontCenterLeft));
                    break;
            case ID_ARD_SENS_US_FRONT_CENTER:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usFrontCenter));
                    break;
            case ID_ARD_SENS_US_SIDE_LEFT:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usSideLeft));
                    break;
            case ID_ARD_SENS_US_SIDE_RIGHT:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usSideRight));
                    break;
            case ID_ARD_SENS_US_BACK_LEFT:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usRearLeft));
                    break;
            case ID_ARD_SENS_US_BACK_RIGHT:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usRearRight));
                    break;
            case ID_ARD_SENS_US_BACK_CENTER:
                    RETURN_IF_FAILED(TransmitUltrasonicData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::usRearCenter));
                    break;
            case ID_ARD_SENS_VOLT_MEAS:
                    RETURN_IF_FAILED(TransmitVoltData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::measurementCircuit));
                    break;
            case ID_ARD_SENS_VOLT_SPEEDCONT:
                    RETURN_IF_FAILED(TransmitVoltData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::speedCntrlCircuit));
                    break;
            case ID_ARD_SENS_IMU:
                    RETURN_IF_FAILED(TransmitInerMeasUnitData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData));
                    break;
            case ID_ARD_SENS_WHEEL_RIGHT:
                    RETURN_IF_FAILED(TransmitWheelData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::wheelRight));
                    break;
            case ID_ARD_SENS_WHEEL_LEFT:
                    RETURN_IF_FAILED(TransmitWheelData(pMediaSample->GetTime(),ui32ArduinoTimestamp,aFrameData,SensorDefinition::wheelLeft));
                    break;
            default:
                break;
        }
    }
    
    RETURN_NOERROR;
}

