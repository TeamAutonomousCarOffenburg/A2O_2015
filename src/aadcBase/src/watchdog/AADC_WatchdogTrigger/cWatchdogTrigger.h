/**********************************************************************
* Copyright (c) 2013 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.
* All rights reserved.
**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/



#ifndef _WATCHDOG_FILTER_H_
#define _WATCHDOG_FILTER_H_

#define OID_ADTF_WATCHDOGTRIGGER_FILTER "adtf.aadc.watchdogTrigger"

/*! \brief cWatchdogTrigger
The filter generates the samples of type tBoolSignalValue which can be passed to the Arduino Actuators filter to keep the Speed Controller alive. The Transmit Rate can be altered in the properties but normally does not need any adjustments.
*/
class cWatchdogTrigger : public adtf::cTimeTriggeredFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_WATCHDOGTRIGGER_FILTER, "AADC Watchdog Trigger", OBJCAT_Tool,  "AADC Watchdog Trigger", 1, 0,0, "BFFT");

protected:
    /*!outputpin which sends periodically the value tTrue */
    cOutputPin        m_oOutputPin;

public:
    cWatchdogTrigger(const tChar* __info);
    virtual ~cWatchdogTrigger();

protected: // overwrites cFilter
    tResult Cycle(__exception=NULL);
    tResult Init(tInitStage eStage, __exception = NULL);


private:
    inline tResult CreateOutputPins(__exception=NULL);

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescOutput;
    /*! the id for the bValue of the media description for the bool signal value input pins */
    tBufferID m_szIDbValue; 
    /*! the id for the arduino time stamp of the media description for the bool signal value input pins */
    tBufferID m_szIDArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsBoolSet;
};

//*************************************************************************************************

#endif // _WATCHDOG_FILTER_H_

