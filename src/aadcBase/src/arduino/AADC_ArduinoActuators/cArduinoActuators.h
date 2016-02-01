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


#ifndef _ACTUATORS_FILTER_H_
#define _ACTUATORS_FILTER_H_


#define OID_ADTF_ACTUATOR_FILTER "adtf.aadc.arduinoActuators"

/*! /brief cArduinoActuators
To this filter the user can send the data which should be send to the Arduino. With input pins the speed and steering controller can be controlled, the light can be switched on and off, the watchdog can be triggered or the Emergency stop can be called.
The lights can be switched with media sample of the type tBoolSignalValue. The value “true” in bValue means lights on, the value “false” in bValue means lights off.
The pin Watchdog_Alive_Flag also waits for a media sample of type tBoolSignalValue. If the media sample contains a “true” in bValue a trigger is send to the Arduino. If this trigger is not received for more than a half second the relais on the board toggles and the Speed Controller is turned off.
When the speed controller is first switched on it needs some seconds to initialize. During this time the speed controller needs the “Zero Position” which corresponds to an input value of 90.0. To prevent troubles with the initialization phase the media samples received on the input pin SpeedController are set to 90.0 during this time period. Nevertheless it needs samples in this time to trigger the messages sent to the Arduino.
The range for the samples for the input pin Speed Controller is between 0.0 and 180.0, resulting the already mentioned zero position at 90.0. This values set the angle of the servo motor between 0.0° and 180.0° corresponding to full speed reverse and forward. The direction and also the real car speed depends on the calibration of the speed controller itself. For further information look at Chapter Calibration.
The range for the samples for the input pin SteeringController is between 60.0 and 120.0 and corresponds to an angle of the steering servo between 60.0° and 120.0°. This results in a steering angle between -30.0° and 30.0° around the straight ahead position.
*/
class cArduinoActuators : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ACTUATOR_FILTER, "AADC Arduino Actuators", OBJCAT_SensorDevice, "Arduino Actuators", 1, 0, 1,"BFFT GmbH");
protected:
    /*! output pin to arduino  */
    cOutputPin               m_oArduinoCOMOutPin;                

    /*! Input Pin for accelerate values; values must be between 0...180*/
    cInputPin                m_oAccelerateInputPin;              
    /*! Input Pin for reverse light messages */
    cInputPin                m_oReverseLightInputPin;
    /*! Input Pin for head light */
    cInputPin                m_oHeadlightInputPin;                
    /*! Input Pin for indicator turn signal left  */
    cInputPin                m_oTurnSignalLeftInputPin;            
    /*! Input Pin for indicator turn singal right  */
    cInputPin                m_oTurnSignalRightInputPin;            
    /*! Input pin for the hazzard light*/
    cInputPin                m_oHazardLightsInputPin;                     
    /*! Input Pin for brake light */
    cInputPin                m_oBrakeLightInputPin;                
    /*! Input Pin for steer angle messages; values must be between 0...180 */
    cInputPin                m_oSteerInputPin;                   
    /*! Input Pin for watchdog flags */
    cInputPin                m_oWatchdogInputPin;                
    /*! Input Pin for emergency stop */  
    cInputPin                m_oEmergencyStopInputPin;

    /*!processes the packages for the lights
    @param pMediaSample the incoming mediasample containing the data
    @param ui8LightID the id of the selecet light
    */
    tResult ProcessLights(IMediaSample* pMediaSample, tUInt8 ui8LightID);

    /*! processes the emergency stop data
    @param pMediaSample the incoming mediasample containing the data
    */
    tResult ProcessEmergencyStop(IMediaSample* pMediaSample);

    /*! processes the watchdog data
    @param pMediaSample the incoming mediasample containing the data
    */
    tResult ProcessWatchdog(IMediaSample* pMediaSample);

    /*! processes the general actuator value
    @param pMediaSample the incoming mediasample containing the data
    @param ui8ChID the ID of the signal to be transmitted to the arduino
    */    
    tResult ProcessActuatorValue(IMediaSample* pMediaSample, tUInt8 ui8ChID);


    /*! builds the arduino frame to be transmitted to the arduino
    @param ui8ChID the ID of the signal to be transmitted to the arduino
    @param i8DataLength length of the data package
    @param pChData pointer to the data package
    */
    tResult BuildArduinoFrame(tUInt8 ui8ChID, tUInt8 i8DataLength, tUInt8* pChData);

public:
    cArduinoActuators(const tChar* __info);
    virtual ~cArduinoActuators();

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:
    /*! creates all the output pins
    @param __exception exception pointer
    */
    tResult CreateOutputPins(__exception = NULL);

    /*! creates all the input pins
    @param __exception exception pointer
    */
    tResult CreateInputPins(__exception = NULL);

    // Media Descriptionr for the output pin         

    /*! media description for output data*/
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

    // Media Description for the input pins 

    /*! media description for emergency stop data*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyStop;           
    /*! the id for the bEmergencyStop of the media description for the emergency stop input pin */
    tBufferID m_szIDEmergencyStopBEmergencyStop; 
    /*! indicates if bufferIDs were set */
    tBool m_bIDsEmergencyStopSet;


    /*! media description for input data*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalInput;      
    /*! the id for the f32value of the media description for the signal value input pins */
    tBufferID m_szIDSignalInputF32Value; 
    /*! the id for the arduino time stamp of the media description for the signal value input pins */
    tBufferID m_szIDSignalInputArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSignalInputSet;

    /*! media description for tBool input data*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolSignalInput;  
    /*! the id for the f32value of the media description for the bool signal value input pins */
    tBufferID m_szIDBoolSignalInputF32Value; 
    /*! the id for the arduino time stamp of the media description for the bool signal value input pins */
    tBufferID m_szIDBoolSignalInputArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsBoolSignalInputSet;

    /*! if debug console output is enabled */
    tBool m_bDebugModeEnabled;
    /*! if watchdog debug to console is enabled*/
    tBool m_bDebugModeEnabledWatchdog;
    /*! if emergency stop was requested the watchdog has to be blocked to disable reactivation of the arduino */
    tBool m_bEmergencyStopBlock;

    /*! the timestamp when the first sample for the speed controller was send */
    tTimeStamp m_timestampFirstSpeedSample;

    /*! critical section for the processing of the  actuator samples */
    cCriticalSection        m_oActuatorCritSection;
    /*! critical section for the processing of the lights samples */
    cCriticalSection        m_oLightsCritSection;
    /*! critical section for the processing of the watchdog samples */
    cCriticalSection        m_oWatchDogCritSection;
    /*! critical section for the transmitting of output samples */
    cCriticalSection        m_oTransmitCritSection;

    /*! holds the last light mask containing all LED states */
    tUInt8 m_ui8lightMask ;

};


//*************************************************************************************************

#endif // _ACTUATORS_FILTER_H_
