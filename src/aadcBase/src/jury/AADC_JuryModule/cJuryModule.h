/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/


#ifndef _JURYMODULE_HEADER
#define _JURYMODULE_HEADER

#define __guid "adtf.aadc.juryModule"

#include "stdafx.h"
#include "juryEnums.h"
#include "displaywidget.h"


/*! /brief cJuryModule
This filter is used by the jury to control the vehicles during the competition.
The Module sends samples to a driver module which has to be implemented by the teams but can be based on the Demo filters. The sent samples contain the tJuryStruct and are used to check or set the state of the cars. The struct is defined as following:
typedef struct
{
tInt8 i8ActionID;
tInt16 i16ManeuverEntry;
} tJuryStruct;

The i8ActionID contain one of the following actions:
•	isReady: The vehicle is asked if it is ready to start the maneuver given in i16ManeuverEntry.
•	start: After receiving this sample the car should start the maneuver given in i16ManeuverEntry. Before that the sample the same maneuver is asked if it is ready.
•	stop: After receiving this sample the car should stop the maneuver given in i16ManeuverEntry. If the car is in a different maneuver it should be stopped as well.

The output pin Emergency_Stop should be connected directly to the pin Emergency_Stop of the Arduino Actuator Filter. It forces the Speed Controller to be switched of immediately.

The struct tJuryStruct is defined in aadc_structs.h in src\aadcBase\include and the used enums are defined in juryEnums.h in src\aadcBase\include

In the upper part of the gui the user can send messages to car. If an maneuver file was loaded successfully you can select a section or an maneuver index and press on of the three buttons “Request State”, “Start” or “Stop” to transmit the corresponding sample to the vehicle.
The last received state from the driver is shown in the middle of the gui and also in a console view. 
*/

class cJuryModule : public QObject, public cBaseQtFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Jury Module", OBJCAT_Tool, "Jury Module", 1, 0, 0,"");    

    Q_OBJECT

signals:


public: // construction
    cJuryModule(const tChar *);
    virtual ~cJuryModule();

    // overrides cFilter
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    virtual tResult Start(__exception = NULL);
    virtual tResult Stop(__exception = NULL);
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);

    /*! */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);
private:
    /*! this functions transmits the jury struct with the given data over the output pin
    @param actionID the actionID of the struct; -1: stop, 0: getReady, 1: Run
    @param maneuverEntry number of the maneuver of which the action is requested
    */
    tResult sendJuryStruct(juryActions actionID, tInt16 maneuverEntry);
protected: // Implement cBaseQtFilter

    /*! Creates the widget instance*/
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();

    public slots:
        /*! send the sample for the Not Aus*/
        tResult OnNotAus();

        /*! send the sample for stop in section
        @param entryId the current id of the maneuver
        */
        tResult OnStop(tInt16 entryId);

        /*! send the sample for start section
        @param entryId the current id of the maneuver
        */
        tResult OnStart(tInt16 entryId);

        /*! send the sample for request ready 
        @param entryId the current id of the maneuver
        */
        tResult OnRequestReady(tInt16 entryId);

signals:
        /*! send new state to widget 
        @param state the state of the input
        @param entryId the id of the current maneuver
        */
        void sendDriverState(int state, int entryId); 

        /*! send new message to text field 
        @param text the text which should be plotted to gui*/
        void sendMessage(QString text);

private:
    /*! this functions loads the maneuver list given in the properties*/
    tResult loadManeuverList();

    /*! send emergency stop*/
    tResult sendEmergencyStop();

protected:

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;

    /*! Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescNotAus;    
    /*! the id for the bEmergencyStop of the media description */
    tBufferID m_szIDNotAusbEmergencyStop; 
    /*! indicates if bufferIDs were set */
    tBool m_bIDNotAusSet;

    /*! Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescJuryStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDJuryStructI8ActionID; 
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDJuryStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsJuryStructSet;

    /*! Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescDriverStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDDriverStructI8StateID; 
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDDriverStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsDriverStructSet;

    /*! output pin for the Not Aus*/
    cOutputPin            m_NotAusOutputPin;
    /*! output pin for the stop command*/
    cOutputPin            m_JuryStructOutputPin;
    /*! input pin for state from driver*/
    cInputPin            m_DriverStructInputPin;
    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;    
    /*! last maneuver id*/
    tInt16 m_ilastEntryId;
    /*! timestamp of last maneuver id */
    tTimeStamp m_last_timestamp;
    /*! this is the filename of the maneuver list*/
    cFilename m_maneuverListFile;
    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

    /*! handle for timer for not aus*/
    tHandle m_hTimerNotAus;
    /*! counter for not aus */
    tInt m_iNotAusCounter;
    /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerNotAus;

    /*! timer for stop event*/
    tHandle m_hTimerStop;    
    /*! counter for stop event*/
    tInt m_iStopCounter;
    /*! id for current stop id*/
    tInt m_iStopEntryId;
    /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerStop;

    /*! timer for start event*/
    tHandle m_hTimerStart;    
    /*! counter for start event*/
    tInt m_iTimerStartCounter;
    /*! id for current start event*/
    tInt m_iStartEntryId;
    /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerStart;

    /*! timer for request ready event*/
    tHandle m_hTimerRequestReady;
    /*! counter for request ready events */
    tInt m_iRequestReadyCounter;
    /*! id for current request ready*/
    tInt m_iRequestReadyEntryId;
    /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerRequestReady;

};

#endif
