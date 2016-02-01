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
* $Author:: forchhe#$  $Date:: 2015-05-13 15:10:07#$ $Rev:: 35043   $
**********************************************************************/


#ifndef _ARDUINO_COMM_FILTER_H_
#define _ARDUINO_COMM_FILTER_H_

#define OID_ADTF_ARDUINOCOM_FILTER "adtf.aadc.arduinoCommunication"
/*! \brief cArduinoCom
This is the filter which implements the communication with the Arduino. It sends the package data of the media samples given on the input pin to Arduino and transmits all the received packages from the Arduino to the output pin. With the property COM Port the serial port to be used can be set. On Linux it should be something line /dev/ttyACM0 to /dev/ttyACM3 and in Windows it corresponds to the number of the COM Port given by the system, preceded by a \\.\\ .
For this filter it does not matter what Arduino of the car is bounded to the port, there is only this filter for the Arduino. To Read or Write all Arduinos you must add four instances of this filter to the ADTF Configuration.
*/
class cArduinoCom : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ARDUINOCOM_FILTER, "AADC Arduino Communication", OBJCAT_BridgeDevice, "Arduino Communication", 1, 0, 0, "BFFT");    

public:
    /*! constructor */
    cArduinoCom(const tChar* __info);
    virtual ~cArduinoCom();

    //Receiver Thread
private:
    class cReceiveThread: public cKernelThread
    {
    protected:
        cArduinoCom*    m_pParent; /**< the parent of the thread*/

    public:
        /*! the receiving thread */
        cReceiveThread();
        /*! function to set the parent of the thread
        @param pParent parent for thread
        */
        tResult SetParent(cArduinoCom* pParent);

    protected: //overwrite ThreadFunc of cKernelThread
        /*! the thread function */ 
        tResult ThreadFunc();
    private:

    };
    /*! the receive thread */
    cReceiveThread m_oReceiveThread;



protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:
    /*! Output Pin for sensor data */ 
    cOutputPin        m_oCOMOutputPin;            
    /*! input Pin for actuator commands */
    cInputPin         m_oCOMInputPin;                

    /*! descriptor for output data*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionArduinoOutput;       
    /*! the id for the ui8SOF of the media description of output pin for the arduino data */
    tBufferID m_szIDArduinoOutputUi8SOF; 
    /*! the id for the ui8ID of the media description of output pin for the arduino data */
    tBufferID m_szIDArduinoOutputUi8ID; 
    /*! the id for the ui32ArduinoTimestamp of the media description of output pin for the arduino data */
    tBufferID m_szIDArduinoOutputUi32ArduinoTimestamp; 
    /*! the id for the ui8DataLength of the media description of output pin for the arduino data */
    tBufferID m_szIDArduinoOutputUi8DataLength; 
    /*! the id for the arduino ui8Data of the media description for output pin for the measured speed */
    tBufferID m_szIDArduinoOutputUi8Data; 
    /*! indicates if bufferIDs were set */
    tBool m_bIDsArduinoOutputSet;

    /*! descriptor for input data*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionArduinoInput;
    /*! the id for the ui8SOF of the media description of input pin for the arduino data */
    tBufferID m_szIDArduinoInputUi8SOF; 
    /*! the id for the ui8ID of the media description of input pin for the arduino data */
    tBufferID m_szIDArduinoInputUi8ID; 
    /*! the id for the ui32ArduinoTimestamp of the media description of input pin for the arduino data */
    tBufferID m_szIDArduinoInputUi32ArduinoTimestamp; 
    /*! the id for the ui8DataLength of the media description of input pin for the arduino data */
    tBufferID m_szIDArduinoInputUi8DataLength; 
    /*! the id for the arduino ui8Data of the media description for input pin for the measured speed */
    tBufferID m_szIDArduinoInputUi8Data; 
    /*! indicates if bufferIDs were set */
    tBool m_bIDsArduinoInputSet;

    /*! creates the input pins
    @param __exception pointer to exception handler
    */
    tResult CreateInputPins(__exception = NULL);

    /*! creates the output pins
    @param __exception pointer to exception handler
    */        
    tResult CreateOutputPins(__exception = NULL);

    /*! receives the frames on the serial interface */
    tResult ReceiveFrameFromSerial();

    /*! sends the media samples with the data from the frames on the serial interface */
    tResult SendMediaSample();

    /*! receives the media sample on the input pin
    @param pMediaSample the incoming media sample
    */
    tResult ReceiveMediaSample(IMediaSample* pMediaSample);

    /*! Write Error Message to ADTF LOG Console*/
    inline void CheckForArduinoErrorMessage();

    inline void DetectArduinoAddress();
    /*! Arduino Address **/
    tUInt8 m_ArduinoAddress; 


    /*! sends the frame with the data received on the input pins to the serial interface
    @param ui8Id the id of the data
    @param ui8Data the data to send
    */
    tResult WriteFrameToSerial(tUInt8 ui8Id, tUInt8 ui8Data);

    /*! gets the current stream time */
    tTimeStamp GetStreamTime();

    /*! checks whether the id is valid
    @param ui8Id the id which has to be checked
    */
    tBool isSensorIdValid(tUInt8 ui8Id);

    /*! Very Fast checksum calculation (like CRC16). Optimized for 8-bit CPUs.
    @param data pointer to data frame
    @param len length of data frame
    */
    tUInt16 fletcher16( tUInt8 const *pData, tUInt8 ui8Len);

    /*! the serial device */
    Serial::cRawSerialDevice m_oSerialDevice;    

    /*! the frame received from the arduino */
    tArduinoSensorFrame m_oReceivedFrame;           

    /*! the timestamp of the last received frame from the arduino */
    tTimeStamp m_timeLastDataReceived;         

    /*! the ammount of seconds to check whether data is received from arduino */
    tInt m_timeOutSeconds;

    tBool m_bDebugModeEnabled;
};


//*************************************************************************************************

#endif // _ARDUINO_COMM_FILTER_H_

