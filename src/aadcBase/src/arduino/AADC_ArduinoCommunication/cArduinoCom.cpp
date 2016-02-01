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
* $Author:: spiesra $  $Date:: 2015-05-21 12:25:11#$ $Rev:: 35324   $
**********************************************************************/


#include "stdafx.h"
#include "cRawSerialDevice.h"
#include "cArduinoCom.h"
#include "arduinoProtocol.h"


ADTF_FILTER_PLUGIN("AADC Arduino Communication", OID_ADTF_ARDUINOCOM_FILTER, cArduinoCom)

const tInt MAX_COMPORT_BUFFER_SIZE = 1024;

cArduinoCom::cArduinoCom(const tChar* __info) : cFilter (__info), m_timeLastDataReceived(0)
{
	m_ArduinoAddress = ARD_ADDRESS_UNKNOWN;
    SetPropertyStr("COM Port", "\\\\.\\COM0");
    SetPropertyStr("COM Port" NSSUBPROP_DESCRIPTION, "The device name for the COM port of the arduino. In Windows the notation is typically \\\\.\\COMx, in Linux it is /dev/ttyACM0."); 

    
    SetPropertyInt("Connection Timeout in millisec", 3000);
    SetPropertyStr("Connection Timeout in millisec" NSSUBPROP_DESCRIPTION, "The value for the timeout how long is waited for the arduino to send something on serial interface"); 
    SetPropertyInt("Connection Timeout in millisec" NSSUBPROP_MIN, 0); 
    SetPropertyInt("Connection Timeout in millisec" NSSUBPROP_MAX, 5000); 
    m_bDebugModeEnabled = tFalse;    
    SetPropertyBool("Debug Output to Console", m_bDebugModeEnabled);    
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)"); 

}

cArduinoCom::~cArduinoCom()
{
    m_oReceiveThread.Release();
}

cArduinoCom::cReceiveThread::cReceiveThread(): cKernelThread(), m_pParent(NULL)
{
}

tResult cArduinoCom::cReceiveThread::SetParent(cArduinoCom* pParent)
{
    m_pParent = pParent;
    RETURN_NOERROR;
}

tResult cArduinoCom::cReceiveThread::ThreadFunc()
{
    RETURN_IF_POINTER_NULL(m_pParent);
    return m_pParent->ReceiveFrameFromSerial();
}

tResult cArduinoCom::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        tResult nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to create Input Pins");
    }
    else if (eStage == StageNormal)
    {
        m_timeOutSeconds = GetPropertyInt("Connection Time Out in sec");      
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");      
        //open Serial Device
        if (IS_OK(m_oSerialDevice.CheckForDevice(GetPropertyStr("COM Port"))))
        {
            if(!m_oSerialDevice.IsConnected())
            {
                if IS_FAILED(m_oSerialDevice.Open(GetPropertyStr("COM Port"),
                                        115200, cSerialDevice::SER_NOPARITY, 8, cSerialDevice::SER_ONESTOPBIT,
                                        MAX_COMPORT_BUFFER_SIZE, MAX_COMPORT_BUFFER_SIZE, m_timeOutSeconds * 1000))
                    THROW_ERROR_DESC(ERR_DEVICE_IO, "Serial Communication: Cannot open device on com port");
            }
            else
            {
                 THROW_ERROR_DESC(ERR_DEVICE_IO, "Serial Communication: com port is in use");
            }
#ifndef WIN32
            m_oSerialDevice.SetRawMode();
#endif
        }
        else
        {
            THROW_ERROR_DESC(ERR_DEVICE_IO, "Serial Communication: Cannot find device on com port");
        }
        m_oReceiveThread.SetParent(this);       
    }
    else if (eStage == StageGraphReady)
    {
        m_bIDsArduinoOutputSet = tFalse;
        m_bIDsArduinoInputSet = tFalse;
    }
    RETURN_NOERROR;
}


tResult cArduinoCom::Start(__exception)
{
    if(m_oReceiveThread.GetState() == cKernelThread::TS_NotInitialized){
        m_oReceiveThread.Create();
    }
    
    m_oReceiveThread.Run(true);
    return cFilter::Start(__exception_ptr);
}

tResult cArduinoCom::Stop(__exception)
{

    m_oReceiveThread.Suspend(true);
    return cFilter::Stop(__exception_ptr);
}

tResult cArduinoCom::Shutdown(tInitStage eStage, __exception)
{
    if(eStage == StageNormal)
    {    
        m_oSerialDevice.Close();
        m_oReceiveThread.Suspend(true);
    }
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cArduinoCom::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &m_oCOMInputPin)
            ReceiveMediaSample(pMediaSample);            
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged && pSource != NULL)
    {
        cObjectPtr<IMediaType> pType;
        pSource->GetMediaType(&pType);
        if (pType != NULL)
        {
            cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));
            m_pDescriptionArduinoInput = pMediaTypeDesc;                        
        }        
    }    
    RETURN_NOERROR;
}

tResult cArduinoCom::CreateOutputPins(__exception)
{    
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescArduino = pDescManager->GetMediaDescription("tArduinoData");
    RETURN_IF_POINTER_NULL(strDescArduino);

    cObjectPtr<IMediaType> pTypeArduinoData = new cMediaType(0, 0, 0, "tArduinoData", strDescArduino,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeArduinoData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionArduinoOutput)); 
    
    RETURN_IF_FAILED(m_oCOMOutputPin.Create("COM_output", pTypeArduinoData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oCOMOutputPin));

    RETURN_NOERROR;
}

tResult cArduinoCom::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oCOMInputPin.Create("COM_input", new cMediaType(0, 0, 0, "tArduinoData" ), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oCOMInputPin));

        RETURN_NOERROR;
}

bool cArduinoCom::isSensorIdValid(tUInt8 ui8Id){
    switch(ui8Id)
    {
        case ID_ARD_SENS_STEERING:    
        case ID_ARD_SENS_WHEEL_RIGHT:
        case ID_ARD_SENS_WHEEL_LEFT:
        case ID_ARD_SENS_IMU:
        case ID_ARD_SENS_US_FRONT_LEFT:
        case ID_ARD_SENS_US_FRONT_RIGHT:
        case ID_ARD_SENS_US_FRONT_CENTER:
        case ID_ARD_SENS_US_FRONT_CENTER_LEFT :
        case ID_ARD_SENS_US_FRONT_CENTER_RIGHT:
        case ID_ARD_SENS_US_SIDE_LEFT:
        case ID_ARD_SENS_US_SIDE_RIGHT:
        case ID_ARD_SENS_US_BACK_LEFT:
        case ID_ARD_SENS_US_BACK_RIGHT:
        case ID_ARD_SENS_US_BACK_CENTER:
        case ID_ARD_SENS_VOLT_SPEEDCONT:
        case ID_ARD_SENS_VOLT_MEAS:
        case ID_ARD_SENS_RESERVED_FRONT:
        case ID_ARD_SENS_RESERVED_BACK:
        case ID_ARD_SENSOR_INFO:
        case ID_ARD_SENS_ERROR:
            return true;
        default:
            //Invalid id or ID_ARD_SENS_NEVER_USE_IT or ID_ARD_DUMMY    
            return false;
        }

}

tResult cArduinoCom::ReceiveFrameFromSerial()
{
    memset(&m_oReceivedFrame,0xFF,sizeof(m_oReceivedFrame));

    //a few pointers for comfortable access
    tUInt8 *header = (tUInt8*) &m_oReceivedFrame.sSensorHeader;
    tUInt8 *data = (tUInt8*) &m_oReceivedFrame.sData;

    int bytesRead = 0;
    int bytesToRead = 1;
    // some of the arduino don`t send data at all, so the receive thread will be quited here
    if (m_oSerialDevice.BytesAvailable()<sizeof(m_oReceivedFrame))
        {
        adtf_util::cSystem::Sleep(1000);
        RETURN_NOERROR;
        }   
    
    //Search for SOF
    do
         {  
            int ret = m_oSerialDevice.Read(&header[0], bytesToRead);
            if(ret < 0){
             LOG_WARNING("Serial Read Timeout");
             RETURN_NOERROR;
            }
            bytesRead += ret;
            //Message only at second try, because of LOG spamming
            if(bytesRead == 2){
                LOG_WARNING("SOF not found, try again..");
            }
         } while(header[0] != ID_ARD_SOF);

    bytesToRead = sizeof(tArduinoSensorHeader) - sizeof(ID_ARD_SOF);
    bytesRead = 0;

    //Read Header
    do
         {
            tInt32 ret =  m_oSerialDevice.Read(&header[sizeof(ID_ARD_SOF)], bytesToRead - bytesRead);
            if(ret < 0){
             LOG_WARNING("Serial Read Timeout");
             RETURN_NOERROR;
            }
            bytesRead += ret;
         } while(bytesRead < bytesToRead);

    //Check ID
    if(!isSensorIdValid(m_oReceivedFrame.sSensorHeader.ui8ID))
        {
        LOG_INFO(cString::Format("Invalid Arduino ID %x", m_oReceivedFrame.sSensorHeader.ui8ID));
        }
    else
        {
         tUInt8 len =  m_oReceivedFrame.sSensorHeader.ui8DataLength;
         if(len > sizeof(tArduinoSensDataUnion) || len == 0)
            {
            LOG_WARNING(cString("Length mismatch"));
            }
         else 
            {
            bytesToRead = len;
            bytesRead = 0;
            //Read Data
            do
                {
                    tInt32 ret = m_oSerialDevice.Read(&data[bytesRead], bytesToRead - bytesRead);
                    if(ret < 0){
                        LOG_WARNING("Serial Read Timeout");
                        RETURN_NOERROR;
                    }
                    bytesRead += ret;
                } while(bytesRead < bytesToRead);

            bytesToRead = sizeof(INIT_CRC);
            bytesRead = 0;

            //Read CRC
            do
                {
                    tInt32 ret = m_oSerialDevice.Read(&m_oReceivedFrame.ui16CRC, bytesToRead - bytesRead);
                    if(ret < 0){
                        LOG_WARNING("Read Timeout");
                        RETURN_NOERROR;
                    }
                    bytesRead += ret;
                } while(bytesRead < bytesToRead);


            //Calc CRC
            tUInt16 crc = fletcher16(header,sizeof(tArduinoSensorHeader) + len);
            if(m_oReceivedFrame.ui16CRC != crc)
                {
                LOG_WARNING(cString::Format("CRC Error %x != %x",m_oReceivedFrame.ui16CRC,crc)); 
                }
            else
                {
                //is valid message a arduino error messages
                CheckForArduinoErrorMessage();

                //Update Arduino Address
				DetectArduinoAddress();
                

                //transmit Media Sample with the received data
                if(!IS_OK(SendMediaSample()))
                    {
                        LOG_ERROR("Could not send MediaSample from serial data");
                    }
                  
                RETURN_NOERROR;
                }
            }
        }
    
    RETURN_NOERROR;
}


void cArduinoCom::DetectArduinoAddress(){
 if(m_oReceivedFrame.sSensorHeader.ui8ID == ID_ARD_SENSOR_INFO){
            
        tInfoData version = m_oReceivedFrame.sData.sInfoData;
		tUInt8 tmp = version.ui8ArduinoAddress;
	    if(tmp!= m_ArduinoAddress){
			m_ArduinoAddress = tmp;
			cString arduinoName = "";
			switch(m_ArduinoAddress){
				case ARD_ADDRESS_SENSORSFRONT:
					arduinoName = "SensorsFront";
					break;
				case ARD_ADDRESS_SENSORSBACK:
					arduinoName = "SensorsBack";
					break;
				case ARD_ADDRESS_SENSORSSIDE:
					arduinoName = "SensorsSide";
					break;
				case ARD_ADDRESS_ACTORS:
					arduinoName = "Actors";
					break;
				default:
					arduinoName = "UNKNOWN";
			};
 			LOG_INFO(cString("Found Arduino ") + arduinoName + cString::Format(" on %s",GetPropertyStr("COM Port")));
	    }
	
	}
	
}


void cArduinoCom::CheckForArduinoErrorMessage(){
    if(m_oReceivedFrame.sSensorHeader.ui8ID == ID_ARD_SENS_ERROR){
        tErrorData error = m_oReceivedFrame.sData.sErrorData;
        switch(error.ui16ErrorNr){
        case ERROR_SOF_NOT_FOUND:
            LOG_WARNING("ERROR_SOF_NOT_FOUND");
            break;
        case ERROR_CRC_INVALID:
            LOG_WARNING("ERROR_CRC_INVALID");
            break;
        case ERROR_REMOTE_DETECTED:
            LOG_WARNING("ERROR_REMOTE_DETECTED");
            break;
        case ERROR_NO_GYRO_DETECTED:
            LOG_WARNING("ERROR_NO_GYRO_DETECTED");
            break;
        default:
            LOG_WARNING(cString::Format("UNKNOWN ERROR NR %i",error.ui16ErrorNr));
        }
        
    }
}

tResult cArduinoCom::SendMediaSample()
{    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionArduinoOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionArduinoOutput,pMediaSample,pCoderOutput);

        
        // set the ids if not already done
        if(!m_bIDsArduinoOutputSet)
                        {
                        pCoderOutput->GetID("ui8SOF", m_szIDArduinoOutputUi8SOF);
                        pCoderOutput->GetID("ui8ID", m_szIDArduinoOutputUi8ID);
                        pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDArduinoOutputUi32ArduinoTimestamp);
                        pCoderOutput->GetID("ui8DataLength", m_szIDArduinoOutputUi8DataLength);
                        pCoderOutput->GetID("ui8Data", m_szIDArduinoOutputUi8Data);
                        m_bIDsArduinoOutputSet = tTrue;
                        }

        pCoderOutput->Set(m_szIDArduinoOutputUi8SOF, (tVoid*)&(m_oReceivedFrame.sSensorHeader.ui8SOF));    
        pCoderOutput->Set(m_szIDArduinoOutputUi8ID, (tVoid*)&(m_oReceivedFrame.sSensorHeader.ui8ID));
        pCoderOutput->Set(m_szIDArduinoOutputUi32ArduinoTimestamp, (tVoid*)&((m_oReceivedFrame.sSensorHeader.ui32ArduinoTimestamp)));
        //ToDo: Check if m_oReceivedFrame.sData size is 25 byte or edit DDL
        pCoderOutput->Set(m_szIDArduinoOutputUi8DataLength, (tVoid*)&(m_oReceivedFrame.sSensorHeader.ui8DataLength));        
        // TODO: setting value with m_szIDArduinoUi8Data is not working here
        pCoderOutput->Set("ui8Data", (tVoid*)&((m_oReceivedFrame.sData)));
    }  
    
    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oCOMOutputPin.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cArduinoCom::ReceiveMediaSample(IMediaSample *pMediaSample)
{     
    if (pMediaSample != NULL && m_pDescriptionArduinoInput != NULL)
    {
        //write values with zero
        tUInt8 ui8SOF = 0;
        tUInt8 ui8ID = 0;
        tUInt32 ui32ArduinoTimestamp = 0;
        tUInt8 ui8DataLength = 0;
        //ToDo: Check if size is 25 byte or edit DDL // sizeof(tArduinoSensDataUnion)
        tUInt8 aChFrameData[25];

        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pDescriptionArduinoInput,pMediaSample,pCoder);
            
            //set ids if not already done 
            if(!m_bIDsArduinoInputSet)
                        {
                        pCoder->GetID("ui8SOF", m_szIDArduinoInputUi8SOF);
                        pCoder->GetID("ui8ID", m_szIDArduinoInputUi8ID);
                        pCoder->GetID("ui32ArduinoTimestamp", m_szIDArduinoInputUi32ArduinoTimestamp);
                        pCoder->GetID("ui8DataLength", m_szIDArduinoInputUi8DataLength);
                        pCoder->GetID("ui8Data", m_szIDArduinoInputUi8Data);
                        m_bIDsArduinoInputSet = tTrue;
                        }

            //get values from media sample 
            pCoder->Get(m_szIDArduinoInputUi8SOF, (tVoid*)&ui8SOF);
            pCoder->Get(m_szIDArduinoInputUi8ID, (tVoid*)&ui8ID);
            pCoder->Get(m_szIDArduinoInputUi32ArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);
            pCoder->Get(m_szIDArduinoInputUi8DataLength, (tVoid*)&ui8DataLength);
            // TODO: getting value with m_szIDArduinoUi8Data is not working here
            pCoder->Get("ui8Data", (tVoid*)&(aChFrameData));
        }
        //actuator data cant be more than 1 bytes
        if (!(IS_OK(WriteFrameToSerial(ui8ID,aChFrameData[0]))))
        {
            LOG_ERROR("Serial Communcation: Could not send frame on serial interface");            
            RETURN_ERROR(ERR_DEVICE_IO);
        }
    }

    RETURN_NOERROR;
}

tResult cArduinoCom::WriteFrameToSerial(tUInt8 ui8Id, tUInt8 ui8Data)
{
	if(m_ArduinoAddress != ARD_ADDRESS_ACTORS){
		//don t send data to sensors
		RETURN_NOERROR;
    }


    tArduinoActorFrame frame;
    tUInt8 *buffer = (tUInt8*) &frame;

    frame.sActorHeader.ui8SOF = ID_ARD_SOF;
    frame.sActorHeader.ui8ID = ui8Id;
    frame.sData = ui8Data;
    frame.ui16CRC = fletcher16(buffer,sizeof(tArduinoActorFrame) - sizeof(INIT_CRC));

    if(m_oSerialDevice.Write(buffer, sizeof(tArduinoActorFrame)) == sizeof(tArduinoActorFrame)) 
	{
		RETURN_NOERROR;
	}

  
	RETURN_ERROR(ERR_DEVICE_IO);
}

tTimeStamp cArduinoCom::GetStreamTime()
{
    return (_clock != NULL) ? _clock->GetStreamTime() : cSystem::GetTime();
}

tUInt16 cArduinoCom::fletcher16( tUInt8 const *pData, tUInt8 ui8Len )
{
        tUInt16 sum1 = 0xff, sum2 = 0xff;
 
        while (ui8Len) {
                tUInt8 tlen = ui8Len > 20 ? 20 : ui8Len;
                ui8Len -= tlen;
                do {
                        sum2 += sum1 += *pData++;
                } while (--tlen);
                sum1 = (sum1 & 0xff) + (sum1 >> 8);
                sum2 = (sum2 & 0xff) + (sum2 >> 8);
        }
        /* Second reduction step to reduce sums to 8 bits */
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
        return sum2 << 8 | sum1;
}


