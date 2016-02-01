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



#include "stdafx.h"
#include "juryEnums.h"
#include "ui_jury_module.h"
#include "QTime"
#include "QProcess"
#include "QThread"

// define some string constants to avoid typos
#define CONLIB_OUT_PORT_NAME_EMERGENCY_STOP "Emergency_Stop_raw"
#define CONLIB_IN_PORT_NAME_EMERGENCY_STOP_LOOPBACK "Emergency_Stop_Loopback_raw"
#define CONLIB_STREAM_NAME_EMERGENCY_STOP "stream_tJuryEmergencyStop"

#define CONLIB_OUT_PORT_NAME_JURY_STRUCT "Jury_Struct_raw"
#define CONLIB_IN_PORT_NAME_JURY_STRUCT_LOOPBACK "Jury_Struct_Loopback_raw"
#define CONLIB_STREAM_NAME_JURY_STRUCT "stream_tJuryStruct"

#define CONLIB_OUT_PORT_NAME_MANEUVER_LIST "Jury_ManeuverList_raw"
#define CONLIB_IN_PORT_NAME_MANEUVER_LIST "Jury_ManeuverList_Loopback_raw"
#define CONLIB_STREAM_NAME_MANEUVER_LIST "stream_tManeuverList"

#define CONLIB_IN_PORT_NAME_DRIVER_STRUCT "Driver_Struct_raw"
#define CONLIB_STREAM_NAME_DRIVER_STRUCT "stream_tDriverStruct"

class cJuryModule; // forward declaration

/*! this is just a helper class to connect/disconnect with another thread. */
class cJuryConnectionThread : public QThread
{
public:
    cJuryConnectionThread();
    tResult RegisterJuryModule(cJuryModule* pJuryModule);
private:
    cJuryModule* m_pJuryModule;
private:
    void run();
};

/*! /brief cJuryModule
This class is used by the jury to control the vehicles during the competition.
The Module sends samples with the connection library to a driver module which has to be implemented by the teams but can be based on the Demo filters. The sent samples contain the tJuryStruct and are used to check or set the state of the cars. The struct is defined as following:
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

class cJuryModule : public QWidget, public IPlainDataReceiver
{

    Q_OBJECT

    friend class cJuryConnectionThread;
public: // construction
    explicit cJuryModule(QWidget *parent = 0);
    virtual ~cJuryModule();

    // overrides cFilter
    virtual tResult Init();
    
    // implements IPlainDataReceiver
    tResult OnDataUpdate(const tString& strName, const tVoid* pData, const tSize& szSize);
    tResult OnPlainDataDescriptionUpdate(const tChar* strDataName, const tChar* strStructName, const tChar* strDDL);

    /*! reads the ip adress settings from ui. If the localhost checkbox is checked "localhost" will be returend. */
    tString GetIpAdress();


public slots:
    /*! send the sample for the emergency stop*/
    tResult OnEmergencyStop();

    /*! send the sample for stop in section
    */
    tResult OnStop();

    /*! send the sample for start section
    */
    tResult OnStart();

    /*! send the sample for request ready 
    */
    tResult OnRequestReady();

    /*! sets the current state of the driver
    @param state state to be sent
    @param entryId current entry to be sent
    */
    void OnDriverState(int state, int entryId);
        
    /*! appends a new line to the textfield
    @param text new text for the log line
    */
    void OnAppendText(QString text);

    /*! slot which will be executed when combobox is changed*/
    void OnComboActionBoxChanged(int index);
    /*! slot which will be executed when combobox is changed*/
    void OnComboSectionBoxChanged(int index);

    /*! slot for the maneuverlist send button */
    void OnDescriptionFileButton();
    /*! slot for the maneuverlist send button */
    void OnManeuverListButton();

    /*! slot for the maneuverlist file selection entry in text field (press enter or finish editing by leaving the field) */
    void OnManeuverListSelected();

    /*! slot for the connect/disconnect button */
    void OnConnectDisconnect();

    /*! slot for the localhost checkbox */
    tResult OnLocalhostCheckChanged(int nState);

    /*! slot for the open terminal button */
    tResult OnOpenTerminalProcess();

    /*! slot for the process start event */
    void OnProcessStarted();
    /*! slot for the process finish event */
    void OnProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    /*! slot for the process error event */
    void OnProcessError(QProcess::ProcessError error);


signals:
    /*! send new state to widget 
    @param state the state of the input
    @param entryId the id of the current maneuver
    */
    void SetDriverState(int state, int entryId); 

    /*! send new message to text field 
    @param text the text which should be plotted to gui*/
    void SetLogText(QString text);

    /*! trigger the connection state setting in the ui */
    void SetConnectionState();
    
    /*! trigger the control state setting in the ui */
    void SetControlState();
        

private slots:
    /*! slot for close event */
    void OnCloseWindows();
    /*! slot for refreshing the connect/disconnect button background */
    void OnConnectionStateChange();

    /*! slot for connect/disconnect the connection lib (currently not used as slot, just as method)*/
    tResult ConnectDisconnectNetworkClients();
    /*! slot for transmitting the emergency stop */
    tResult SendEmergencyStop();
    /*! slot for transmitting the jury struct */
    tResult SendJuryStruct();
    /*! slot for transmitting the maneuver list */
    tResult SendManeuverList();
    /*! slot for setting the control state in the ui */
    void OnControlState();

private:
    /*! this method loads the maneuver list given in the file dialog*/
    tResult LoadManeuverList();

    /*! this method just cleans up the network connection */
    tResult CleanUpNetwork();

    /*! set the error ui state*/
    void SetDriverStateError();

    /*! set the run ui state*/
    void SetDriverStateRun();

    /*! set the ready ui state*/
    void SetDriverStateReady();

    /*! set the ready ui state*/
    void SetDriverStateComplete();

    
    /*! set the maneuver id 
    @param id id of maneuver
    */
    void SetDriverManeuverID(tInt16 id);


    /*! this function fills comboboxes */
    void FillComboBoxes();

    

protected:

    /*! The displayed widget*/
    Ui::cDisplayWidget *m_pWidget;

    
    /*! last maneuver id from driver*/
    tInt16 m_i16LastDriverEntryId;
    /*! timestamp of last maneuver id */
    QTime m_oLastReceiveTimestamp;
    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_lstSections;

    /*! counter for emergency transmission */
    tInt m_nEmergencyCounter;
    
    /*! counter for jury struct transmission*/
    tInt m_iJuryStructCounter;
        
    /*! the maneuver file path */
    QString m_strManeuverFile;

    /*! the ip adress from ui */
    tString m_strIPAdress;

    /*! data server for transmission */
    IDataServer*    m_pDataSystemDataSender;
    /*! data server for receiving */
    IDataServer*    m_pDataRemoteSystemData;
    /*! the network object */
    cNetwork*       m_pNetwork;
    /*! system for transmission */
    ISystem*        m_pLocalSystemSender;
    /*! system for receiving */
    ISystem*        m_pRemoteSystem;

    /*! the current action id to transmit */
    juryActions     m_actionID; 
    /*! the current maneuver entry to transmit */ 
    tInt16          m_i16ManeuverEntry;

    /*! the process object for terminal application */
    QProcess        m_oTerminalProcess;

    /*! the current connection state */
    tBool           m_bConnected;

    /*! the connectionlib connect/disconnect thread */
    cJuryConnectionThread   m_oConnectionThread;

    /*! the maneuverlist as xml string (from file) */
    tString         m_strManeuverList;
};



#endif
