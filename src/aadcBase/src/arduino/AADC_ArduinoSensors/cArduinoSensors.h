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

#ifndef _SENSOR_FILTER_H_
#define _SENSOR_FILTER_H_

#include "stdafx.h"
#define OID_ARDUINO_SENSORS_FILTER "adtf.aadc.arduinoSensors"

/*! /brief cArduinoSensors
 *  This filter receives all the media samples from the Arduino Communication Filter and sends them depending on their content (i.e. ID) to the different output pins. All outputs of all four Arduino Communication Filters communicating with the four different Arduinos mounted in the car should be connected with this filter to provide all sensor data on the output pins.
The samples on the pin Ultrasonic contain structs with all sensor data from all ultrasonic sensors up to the update time of the sample. 

*/
class cArduinoSensors : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ARDUINO_SENSORS_FILTER, "AADC Arduino Sensors", OBJCAT_SensorDevice, "Arduino Sensors", 1, 0,0, "BFFT");    
    
    protected:
        /*! Output pin for the ultrasonic front left data */
        cOutputPin       m_oOutputUsFrontLeft;
        /*! Output pin for the ultrasonic front center left data */
        cOutputPin       m_oOutputUsFrontCenterLeft;
        /*! Output pin for the ultrasonic front center data */
        cOutputPin       m_oOutputUsFrontCenter;
        /*! Output pin for the ultrasonic front center right data */
        cOutputPin       m_oOutputUsFrontCenterRight;
        /*! Output pin for the ultrasonic front right data */
        cOutputPin       m_oOutputUsFrontRight;
        /*! Output pin for the ultrasonic side left data */
        cOutputPin      m_oOutputUsSideLeft;
        /*! Output pin for the ultrasonic side right data */
        cOutputPin      m_oOutputUsSideRight;
        /*! Output pin for the ultrasonic rear left data */
        cOutputPin      m_oOutputUsRearLeft;
        /*! Output pin for the ultrasonic rear center data */
        cOutputPin      m_oOutputUsRearCenter;
        /*! Output pin for the ultrasonic rear right data */
        cOutputPin      m_oOutputUsRearRight;

        /*! Output pin for the ultrasonic struct data */
        cOutputPin      m_oOutputUsStruct;

        /*! Output pin for the voltage data of measurement circuit */
        cOutputPin      m_oOutputVoltMeas;
        /*! Output pin for the voltage data of speed controller circuit */
        cOutputPin      m_oOutputVoltSpeedCtr;

        /*! Output pin for the inertial measurement unit data */
        cOutputPin      m_oOutputInerMeasUnit;        
        
        /*! Output Pin for left wheel struct*/        
        cOutputPin      m_oOutputWheelLeft;            
        
        /*! Output Pin for right wheel struct*/        
        cOutputPin      m_oOutputWheelRight;            
        
        /*! input pin for data from arduino communications filter */
        cInputPin       m_oArduinoCOMInputPin;              
        
        /*! boolean which inidicates if debug outputs are plotted to the console */
        tBool m_bDebugModeEnabled;

    public:
        cArduinoSensors(const tChar* __info);
        virtual ~cArduinoSensors();

    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    protected:        
        /*! splits the arduino gyro package and transmits new sample to the output pin
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data with the frame of the inertial measurement unit data
        */
        tResult TransmitInerMeasUnitData(tTimeStamp inputTimeStamp, const tUInt32 ui32ArduinoTimestamp,const tUInt8* pFrameData);
                
        /*! splits one of the arduino ultrasonic package and transmits new sample to the output pin
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        @param usSensor the type of the ultrasonic sensor
        */
        tResult TransmitUltrasonicData(tTimeStamp inputTimeStamp, const tUInt32 ui32ArduinoTimestamp,const tUInt8* pFrameData, SensorDefinition::ultrasonicSensor usSensor);
        
        /*! transmits the ultrasonic data struct saved in m_ultrasonicDataStruct
        */
        tResult TransmitUltrasonicStructData();

        /*! splits one of the arduino wheel encoder data package and transmits new sample to the output pin
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        @param wheelSensorType defines if left or right data is sent
        */
        tResult TransmitWheelData(tTimeStamp inputTimeStamp, const tUInt32 ui32ArduinoTimestamp,const tUInt8* pFrameData, SensorDefinition::wheelSensor wheelSensorType);
        
        /*! splits one of the arduino voltage data package and transmits new sample to the output pin
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        @param voltageSensor the type of the voltage sensor
        */
        tResult TransmitVoltData(tTimeStamp inputTimeStamp, const tUInt32 ui32ArduinoTimestamp,const tUInt8* pFrameData, SensorDefinition::voltageSensor voltageSensor);

        /*! resolves the incoming mediasample and filters to the different transmit functions            
        @param pMediaSample incoming mediasample
        */
        tResult Process_ArduinoCOMInputPin(IMediaSample *pMediaSample);
        
        
        /*!< descriptor for wheelEncoderData*/
        cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelData; 
        /*! the id for the ui32WheelTach of the media description for output pin of the WheelData data */
        tBufferID m_szIDWheelDataUi32WheelTach; 
        /*! the id for the i8WheelDir of the media description for output pin of the WheelData data */
        tBufferID m_szIDWheelDataI8WheelDir; 
        /*! the id for the arduino time stamp of the media description for output pin of the WheelData data */
        tBufferID m_szIDWheelDataArduinoTimestamp;         
        /*! indicates if bufferIDs were set */
        tBool m_bIDsWheelDataSet;
        
        /*! descriptor for input data from Arduino */        
        cObjectPtr<IMediaTypeDescription> m_pDescriptionArduinoData;         
        /*! the id for the ui8SOF of the media description of input pin for the arduino data */
        tBufferID m_szIDArduinoUi8SOF; 
        /*! the id for the ui8ID of the media description of input pin for the arduino data */
        tBufferID m_szIDArduinoUi8ID; 
        /*! the id for the ui32ArduinoTimestamp of the media description of input pin for the arduino data */
        tBufferID m_szIDArduinoUi32ArduinoTimestamp; 
        /*! the id for the ui8DataLength of the media description of input pin for the arduino data */
        tBufferID m_szIDArduinoUi8DataLength; 
        /*! the id for the arduino ui8Data of the media description for input pin for the measured speed */
        tBufferID m_szIDArduinoUi8Data; 
        /*! indicates if bufferIDs were set */
        tBool m_bIDsArduinoSet;
        
        /*! descriptor for ultrasonic sensor data */        
        cObjectPtr<IMediaTypeDescription> m_pDescriptionUsData;         
        /*! the id for the f32value of the media description for output pin of the ultrasoncic data */
        tBufferID m_szIDUltrasonicF32Value; 
        /*! the id for the arduino time stamp of the media description for output pin of the ultrasoncic data */
        tBufferID m_szIDUltrasonicArduinoTimestamp;         
        /*! indicates if bufferIDs were set */
        tBool m_bIDsUltrasonicSet;


        /*! descriptor for ultrasonic sensor data */        
        cObjectPtr<IMediaTypeDescription> m_pDescriptionUsStructData;         
        /*! the id for the f32value of the media description for output pin of the ultrasoncic data */
        tBufferID m_szIDUltrasonicStructFrontLeftF32Value; 
        /*! the id for the arduino time stamp of the media description for output pin of the ultrasoncic data */
        tBufferID m_szIDUltrasonicStructFrontLeftArduinoTimestamp;         
        /*! indicates if bufferIDs were set */
        tBool m_bIDsUltrasonicStructSet;
    
        /*! descriptor for voltage sensor data */        
        cObjectPtr<IMediaTypeDescription> m_pDescriptionVoltData;        
        /*! the id for the f32value of the media description for output pin of the voltage data */
        tBufferID m_szIDVoltageF32Value; 
        /*! the id for the arduino time stamp of the media description for output pin of the voltage data */
        tBufferID m_szIDVoltageArduinoTimestamp;         
        /*! indicates if bufferIDs were set */
        tBool m_bIDsVoltageSet;
            
        /*! descriptor for intertial measurement unit sensor data */        
        cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;          
        /*! the id for the f32Q_w of the media description for output pin of the imu data */
        tBufferID m_szIDInerMeasUnitF32Q_w; 
        /*! the id for the f32Q_x of the media description for output pin of the imu data */
        tBufferID m_szIDInerMeasUnitF32Q_x; 
        /*! the id for the f32Q_y of the media description for output pin of the imu data */
        tBufferID m_szIDInerMeasUnitF32Q_y; 
        /*! the id for the f32Q_z of the media description for output pin of the imu data */
        tBufferID m_szIDInerMeasUnitF32Q_z; 
        /*! the id for the f32A_x of the media description for output pin of the imu data */
        tBufferID m_szIDInerMeasUnitF32A_x; 
        /*! the id for the f32A_y of the media description for output pin of the imu data */
        tBufferID m_szIDInerMeasUnitF32A_y; 
        /*! the id for the f32A_z of the media description for output pin of the imu data */
        tBufferID m_szIDInerMeasUnitF32A_z; 
        /*! the id for the arduino time stamp of the media description for output pin of the imu data */
        tBufferID m_szIDInerMeasUnitArduinoTimestamp;         
        /*! indicates if bufferIDs were set */
        tBool m_bIDsInerMeasUnitSet;

        /*! critical section for the processing of the wheel data samples because function can be called from different onPinEvents */
        cCriticalSection    m_oTransmitWheelDataCritSection;

        /*! critical section for the processing of the ultrasonic data samples because function can be called from different onPinEvents */
        cCriticalSection    m_oTransmitUltraSonicDataCritSection;

        /*! critical section for the processing of the ultrasonic data samples because function can be called from different threads */
        cCriticalSection    m_oProcess_ArduinoCOMCritSection;

        /*! in this struct all the ultrasonic signals are collected and transmitted togeher */
        tUltrasonicStruct m_ultrasonicDataStruct;

    private:
        /*! creates all the output pins for the filter 
        @param __exception exception pointer
        */
        inline tResult CreateOutputPins(__exception = NULL);
        
        /*! creates all the output pins for the filter 
        @param __exception exception pointer
        */
        inline tResult CreateInputPins(__exception = NULL);
};

//*************************************************************************************************

#endif // _SENSOR_FILTER_H_

