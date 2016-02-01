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

#include "stdafx.h"
#include "coder_description.h"
#include "cJuryModule.h"

#include "QFileDialog"
#include "QMessageBox"
#include "QDomDocument"
#include "QTimer"


cJuryConnectionThread::cJuryConnectionThread() : m_pJuryModule(NULL)
{

}

void cJuryConnectionThread::run()
{
    if (m_pJuryModule)
    {
        // if jury module is set, call the connect/disconnect method
        m_pJuryModule->ConnectDisconnectNetworkClients();
    }
}

tResult cJuryConnectionThread::RegisterJuryModule(cJuryModule* pJuryModule)
{
    m_pJuryModule = pJuryModule;
    RETURN_NOERROR;
}




cJuryModule::cJuryModule(QWidget *parent) : 
    QWidget(parent),
    m_i16LastDriverEntryId(-1),
    m_pDataRemoteSystemData(NULL),
    m_pDataSystemDataSender(NULL),
    m_pLocalSystemSender(NULL),
    m_pRemoteSystem(NULL),
    m_pNetwork(NULL),
    m_bConnected(false),
    m_oLastReceiveTimestamp(QTime()),
    m_pWidget(new Ui::cDisplayWidget),
    m_strManeuverList("")
{
    // setup the widget
    m_pWidget->setupUi(this);

    // initialize the Qt connections
    Init();
    
    // set the jury module pointer
    m_oConnectionThread.RegisterJuryModule(this);
}

void cJuryModule::OnCloseWindows()
{
    close();
}

cJuryModule::~cJuryModule()
{
    // delete the widget
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    // cleanup any network conection
    CleanUpNetwork();

}



tResult cJuryModule::Init()
{
    // connect the widgets and methods
    connect(m_pWidget->m_btnFileSelection, SIGNAL(released()), this, SLOT(OnManeuverListButton()));
    connect(m_pWidget->m_btnFileSelectionDescr, SIGNAL(released()), this, SLOT(OnDescriptionFileButton()));
    connect(m_pWidget->m_edtManeuverFile, SIGNAL(returnPressed()), this, SLOT(OnManeuverListSelected()));
    connect(m_pWidget->m_edtManeuverFile, SIGNAL(editingFinished()), this, SLOT(OnManeuverListSelected()));
    connect(m_pWidget->m_btnEmergencyStop, SIGNAL(released()), this, SLOT(OnEmergencyStop()));
    connect(m_pWidget->m_btnStart, SIGNAL(released()), this, SLOT(OnStart()));
    connect(m_pWidget->m_btnStop, SIGNAL(released()), this, SLOT(OnStop()));
    connect(m_pWidget->m_btnRequest, SIGNAL(released()), this, SLOT(OnRequestReady()));
    connect(m_pWidget->m_btnConnectDisconnect, SIGNAL(released()), this, SLOT(OnConnectDisconnect()));
    connect(m_pWidget->m_btnManeuverlist, SIGNAL(released()), this, SLOT(SendManeuverList()));
    connect(this, SIGNAL(SetDriverState(int, int)), this, SLOT(OnDriverState(int, int)));    
    connect(this, SIGNAL(SetLogText(QString)), this, SLOT(OnAppendText(QString)));
    connect(this, SIGNAL(SetConnectionState()), this, SLOT(OnConnectionStateChange()));
    connect(this, SIGNAL(SetControlState()), this, SLOT(OnControlState()));
    connect(m_pWidget->m_comboSector, SIGNAL(currentIndexChanged(int)), this, SLOT(OnComboSectionBoxChanged(int)));
    connect(m_pWidget->m_comboManeuver, SIGNAL(currentIndexChanged(int)), this, SLOT(OnComboActionBoxChanged(int)));
    connect(m_pWidget->m_cbxLocalHost, SIGNAL(stateChanged(int)), this, SLOT(OnLocalhostCheckChanged(int)));
    connect(m_pWidget->m_btnOpenTerminal, SIGNAL(released()), this, SLOT(OnOpenTerminalProcess()));
    connect(&m_oTerminalProcess, SIGNAL(started()), this, SLOT(OnProcessStarted()));
    connect(&m_oTerminalProcess, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(OnProcessFinished(int,QProcess::ExitStatus)));
    connect(&m_oTerminalProcess, SIGNAL(error(QProcess::ProcessError)), this, SLOT(OnProcessError(QProcess::ProcessError)));

    // set the connection and control state
    SetConnectionState();
    SetControlState();
    
    RETURN_NOERROR;
}

void cJuryModule::OnConnectionStateChange()
{
    if(m_pWidget)
    {
        // control the button text and background
        if (m_bConnected)
        {
            m_pWidget->m_btnConnectDisconnect->setText("disconnect");
            m_pWidget->m_btnConnectDisconnect->setStyleSheet("background:green");
        }
        else
        {
            m_pWidget->m_btnConnectDisconnect->setText("connect");
            m_pWidget->m_btnConnectDisconnect->setStyleSheet("");
        }
        SetControlState();
    }

    
}

void cJuryModule::OnControlState()
{
    // control the emergency button and the maneuver control group
    tBool bEnabled = false;
    if (m_bConnected && !m_strManeuverList.empty())
    {
        bEnabled = true;
    }

    m_pWidget->grpManeuverCtrl->setEnabled(bEnabled);
    m_pWidget->m_btnEmergencyStop->setEnabled(m_bConnected);
}

tResult cJuryModule::CleanUpNetwork()
{
    // delete the network
    m_pLocalSystemSender = NULL;
    m_pRemoteSystem = NULL;
    m_pDataSystemDataSender = NULL;
    m_pDataRemoteSystemData = NULL;

    if (m_pNetwork)
    {
        delete m_pNetwork;
        m_pNetwork = NULL;
    }

    RETURN_NOERROR;
}

tResult cJuryModule::ConnectDisconnectNetworkClients()
{
    if (m_pNetwork)
    {
        // network is present, so we have to disconnect
        QString strText = QString("Disconnecting...");
        SetLogText(strText);
        m_bConnected = false;
        SetConnectionState();
        
        m_pNetwork->DestroySystem(m_pLocalSystemSender);
        m_pNetwork->DestroySystem(m_pRemoteSystem);
        CleanUpNetwork();
        
        strText = QString("Disconnected");
        SetLogText(strText);

        RETURN_NOERROR;
    }

    // network is not present, connect
    QString strText = QString("Trying to connect to ") + QString(GetIpAdress().c_str()) + QString(" ...");
    SetLogText(strText);
    
    // create new network
    m_pNetwork = new cNetwork("");
    cNetwork::SetLogLevel(cNetwork::eLOG_LVL_ERROR);

    //Initialize the network
    tResult nRes = m_pNetwork->Init();
    if (ERR_NOERROR != nRes) //Never forget the error check
    {
        CleanUpNetwork();
        return nRes;
    }

    const tInt nFlags = 0;

    tString strIp = GetIpAdress();
    
    //Create one local system which is part of the network
    //this is to send data to the world !! 
    const tString strDataSendUrl("tcp://" + strIp + ":5000");
    m_pLocalSystemSender = NULL;                 //Gets assigned while created
    const tString strLocalSystemNameData("JuryModuleData"); //Name of the first system
    nRes = m_pNetwork->CreateLocalSystem(&m_pLocalSystemSender,
        strLocalSystemNameData,
        strDataSendUrl,
        nFlags);//if not given, default = 0
    if (ERR_NOERROR != nRes)
    {
        CleanUpNetwork();
        return nRes;
    }

    //the system can also have some data available
    nRes = m_pLocalSystemSender->GetDataServer(&m_pDataSystemDataSender);
    if (ERR_NOERROR != nRes)
    {
        CleanUpNetwork();
        return nRes;
    }

    // check if there is a description file set, otherwise use the coded description
    QString strDescriptionFile = m_pWidget->m_edtDescriptionFile->text();
    tString strMyDDLDefinitionAsString;
    if (!QFile::exists(strDescriptionFile))
    {
        SetLogText("No valid media description file set. Will use the internal description.");
        strMyDDLDefinitionAsString = CONNLIB_JURY_MODULE_DDL_AS_STRING;
    }
    else
    {
        SetLogText(QString("Using media description file ") + strDescriptionFile);
        strMyDDLDefinitionAsString = strDescriptionFile.toStdString();
    }
        
    nRes = m_pDataSystemDataSender->AddDDL(strMyDDLDefinitionAsString);
    if (ERR_NOERROR != nRes)
    {
        SetLogText("Media description not valid. Please set a valid file.");
        CleanUpNetwork();
        return nRes;
    }

    // define the description of outgoing data
    tDataDesc sDescEmergencyStopRaw;
    sDescEmergencyStopRaw.strName = CONLIB_OUT_PORT_NAME_EMERGENCY_STOP;
    sDescEmergencyStopRaw.strDescriptionComment = "The emergency stop struct in raw mode";
    sDescEmergencyStopRaw.strType = "plainraw";
    nRes = m_pDataSystemDataSender->PublishPlainData(CONLIB_STREAM_NAME_EMERGENCY_STOP, sDescEmergencyStopRaw);
    if (ERR_NOERROR != nRes)
    {
        CleanUpNetwork();
        return nRes;
    }

    tDataDesc sDescJuryStruct;
    sDescJuryStruct.strName = CONLIB_OUT_PORT_NAME_JURY_STRUCT;
    sDescJuryStruct.strDescriptionComment = "the jury data as struct";
    sDescJuryStruct.strType = "plainraw";
    nRes = m_pDataSystemDataSender->PublishPlainData(CONLIB_STREAM_NAME_JURY_STRUCT, sDescJuryStruct);
    if (ERR_NOERROR != nRes)
    {
        CleanUpNetwork();
        return nRes;
    }


    tDataDesc sDescManeuverList;
    sDescManeuverList.strName = CONLIB_OUT_PORT_NAME_MANEUVER_LIST;
    sDescManeuverList.strDescriptionComment = "the Manuverlist as string";
    sDescManeuverList.strType = "plainraw";
    nRes = m_pDataSystemDataSender->PublishPlainData(CONLIB_STREAM_NAME_MANEUVER_LIST, sDescManeuverList);
    if (ERR_NOERROR != nRes)
    {
        CleanUpNetwork();
        return nRes;
    }

    //this is to receive data from a specific sender!!

    //Create one local system which is part of the network
    const tString strDataUrl("tcp://" + strIp + ":5000");
    m_pRemoteSystem = NULL;               //Gets assigned while created
    const tString strSystemNameData("ADTFData"); //Name of the first system
    nRes = m_pNetwork->CreateRemoteSystem(&m_pRemoteSystem,
        strSystemNameData,
        strDataUrl,
        nFlags);//if not given, default = 0
    if (ERR_NOERROR != nRes)
    {
        CleanUpNetwork();
        return nRes;
    }

    nRes = m_pRemoteSystem->GetDataServer(&m_pDataRemoteSystemData);
    if (ERR_NOERROR != nRes)
    {
        CleanUpNetwork();
        return nRes;
    }

    //this is only required if you subscribe to raw data, otherwise the DDL will be transmitted by the sender.
    m_pDataRemoteSystemData->AddDDL(strMyDDLDefinitionAsString);

    m_pDataRemoteSystemData->SubscribeForPlainDataRaw(CONLIB_IN_PORT_NAME_DRIVER_STRUCT, CONLIB_STREAM_NAME_DRIVER_STRUCT, this);

    m_pDataRemoteSystemData->SubscribeForPlainDataRaw(CONLIB_IN_PORT_NAME_JURY_STRUCT_LOOPBACK, CONLIB_STREAM_NAME_JURY_STRUCT, this);

    m_pDataRemoteSystemData->SubscribeForPlainDataRaw(CONLIB_IN_PORT_NAME_EMERGENCY_STOP_LOOPBACK, CONLIB_STREAM_NAME_EMERGENCY_STOP, this);

    m_pDataRemoteSystemData->SubscribeForPlainDataRaw(CONLIB_IN_PORT_NAME_MANEUVER_LIST, CONLIB_STREAM_NAME_MANEUVER_LIST, this);

    // set the connection state in ui
    strText = QString("Connected to ") + QString(GetIpAdress().c_str());
    SetLogText(strText);
    
    m_bConnected = true;
    SetConnectionState();

    RETURN_NOERROR;
}



void cJuryModule::OnManeuverListButton()
{
    // open file dialog to get the maneuver list file
    QString strManeuverFile = QFileDialog::getOpenFileName(this, tr("Open Maneuver List"), m_strManeuverFile, tr("AADC Maneuver List (*.xml)"));
    if(!strManeuverFile.isEmpty())
    {
        m_pWidget->m_edtManeuverFile->setText(strManeuverFile);
        emit OnManeuverListSelected();    
    }
    
}

void cJuryModule::OnDescriptionFileButton()
{
    // open a file dialog to get the description file
    QString strDescriptionFile = QFileDialog::getOpenFileName(this, tr("Open media description file"), "", tr("ADTF media description file (*.description)"));
    if(!strDescriptionFile.isEmpty())
    {
        m_pWidget->m_edtDescriptionFile->setText(strDescriptionFile);
    }

}

tResult cJuryModule::OnOpenTerminalProcess()
{
    // start the given terminal process (detached because otherwise no terminal window will be opened
    if(m_oTerminalProcess.startDetached(m_pWidget->m_edtTerminalProgram->text() ))
    {
        OnProcessStarted();
    }
    else
    {
        OnProcessError(m_oTerminalProcess.error());
    }

    RETURN_NOERROR;
}


void cJuryModule::OnProcessStarted()
{
    // set text in log
    SetLogText("Terminal program started");
}

void cJuryModule::OnProcessFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    // set text in log
    SetLogText(QString("Process finished with ") + QString::number(exitCode) + QString("and state ") + QString(exitStatus));
}

void cJuryModule::OnProcessError(QProcess::ProcessError error)
{
    // set text in log
    SetLogText(QString("Process failed with error code ") + QString::number(error) + QString(": ") + m_oTerminalProcess.errorString());
}

void cJuryModule::OnManeuverListSelected()
{
    // get the maneuver file from edit field
    QString strManeuverFile = m_pWidget->m_edtManeuverFile->text();
    if (!strManeuverFile.isEmpty())
    {
        // check if file exists
        if (QFile::exists(strManeuverFile) && strManeuverFile.contains(".xml"))
        {
            // load the file
            m_strManeuverFile = strManeuverFile;
            LoadManeuverList();
        }
        else
        {
            // file not found, show message box
            QMessageBox::critical(this, tr("File not found or valid"), tr("The given Maneuver List can not be found or is not valid."));
            m_pWidget->m_edtManeuverFile->setFocus();
        }
    }
    else
    {
        // do nothing
    }
    
}

tResult cJuryModule::OnDataUpdate(const tString& strName, const tVoid* pData, const tSize& szSize)
{
    // called from the connection lib on new data

    // check data size
    RETURN_IF_POINTER_NULL(pData);
    if (szSize <= 0)
    {
        RETURN_ERROR(ERR_INVALID_ARG);
    }

    // check the name of the data
    if (strName == CONLIB_IN_PORT_NAME_DRIVER_STRUCT)
    {     
        // data from driver
        const tDriverStruct* sDriverData = (const tDriverStruct*) pData;
        
        //update the gui
        emit SetDriverState(sDriverData->i8StateID, sDriverData->i16ManeuverEntry);


        //update the timeline
        if (m_i16LastDriverEntryId < sDriverData->i16ManeuverEntry || m_i16LastDriverEntryId == -1 || sDriverData->i8StateID == -1 || sDriverData->i8StateID == 2)
        {
            // set the first time manually
            if (m_oLastReceiveTimestamp.isNull())
            {
                m_oLastReceiveTimestamp = QTime::currentTime();
                m_i16LastDriverEntryId = sDriverData->i16ManeuverEntry;
            }

            // calc the time diff
            tTimeStamp timeDiff = (m_oLastReceiveTimestamp.msecsTo(QTime::currentTime()));  //time in milliseconds
            QString Message;
            switch (stateCar(sDriverData->i8StateID))
            {
            case stateCar_READY:
                Message = "from " + QString::number(m_i16LastDriverEntryId) + " to " + QString::number(sDriverData->i16ManeuverEntry) + ": " + QString::number(timeDiff) + " msec: Ready";
                break;
            case stateCar_RUNNING:
                Message = "from " + QString::number(m_i16LastDriverEntryId) + " to " + QString::number(sDriverData->i16ManeuverEntry) + ": " + QString::number(timeDiff) + " msec: OK";
                break;
            case stateCar_COMPLETE:
                Message = "from " + QString::number(m_i16LastDriverEntryId) + " to " + QString::number(sDriverData->i16ManeuverEntry) + ": " + QString::number(timeDiff) + " msec: Complete";
                break;
            case stateCar_ERROR:
                Message = "from " + QString::number(m_i16LastDriverEntryId) + " to " + QString::number(sDriverData->i16ManeuverEntry) + ": " + QString::number(timeDiff) + " msec: Error";
                break;
            case stateCar_STARTUP:
                break;
            }
            // set the last time
            m_oLastReceiveTimestamp = QTime::currentTime();             
            m_i16LastDriverEntryId = sDriverData->i16ManeuverEntry;
            //update the ui
            SetLogText(Message);
        }

    }
    else if (strName == CONLIB_IN_PORT_NAME_JURY_STRUCT_LOOPBACK)
    {
        // data from jury loopback
        const tJuryStruct* sJuryLoopbackData = (const tJuryStruct*) pData;

        QString strText = QString("Loopback Jury: Received jury struct loopback with action ");
        switch(juryActions(sJuryLoopbackData->i8ActionID))
        {
        case action_GETREADY:
            strText += "GETREADY ";
            break;
        case action_START:
            strText += "START ";
            break;
        case action_STOP:
            strText += "STOP ";
            break;
        default:
            strText += "UNKNOWN ";
            break;
        }

        strText += QString(" and maneuver ") + QString::number(sJuryLoopbackData->i16ManeuverEntry);

        // write text to log (just for checking the connection)
        SetLogText(strText);
        //QMetaObject::invokeMethod(this, "OnAppendText", Qt::AutoConnection, Q_ARG(QString, strText));
    }
    else if (strName == CONLIB_IN_PORT_NAME_EMERGENCY_STOP_LOOPBACK)
    {
        // data from emergency loopback
        const tJuryEmergencyStop* sJuryLoopbackData = (const tJuryEmergencyStop*) pData;

        QString strText = QString("Loopback Emergency: Received emergency loopback with "); 
        strText += (sJuryLoopbackData->bEmergencyStop == true) ? QString("true") : QString("false");
        
        // write text to log (just for checking the connection)
        SetLogText(strText);
        //QMetaObject::invokeMethod(this, "OnAppendText", Qt::AutoConnection, Q_ARG(QString, strText));
    }
    else if (strName == CONLIB_IN_PORT_NAME_MANEUVER_LIST)
    {
        // data from maneuverlist loopback
        const tInt32* i32DataSize = (const tInt32*) pData;
        const tChar* pcString = (const tChar*) ((tChar*)pData + sizeof(tInt32));
        tString strManeuver(pcString);
        QString strText = QString("Loopback ManeuverList received"); 

        // check the data content
        if(*i32DataSize != (m_strManeuverList.size() + sizeof(char)))
        {
            strText += QString(" size does not match"); 
        }
        if(strManeuver != m_strManeuverList)
        {
            strText += QString(" content does not match"); 
        }
        
        strText += "!!!";

        // write text to log (just for checking the connection)
        SetLogText(strText);
        //QMetaObject::invokeMethod(this, "OnAppendText", Qt::AutoConnection, Q_ARG(QString, strText));
    }
    RETURN_NOERROR;

}

tResult cJuryModule::OnPlainDataDescriptionUpdate(const tChar* strDataName, const tChar* strStructName, const tChar* strDDL)
{
    // will be called, if we are using NON-RAW-Mode
    // write text to log (just for checking the connection)
    SetLogText(QString("Received description update for ") + QString(strDataName));

    RETURN_NOERROR;
}

tResult cJuryModule::OnEmergencyStop()
{
    // emergency stop button pressed
    // send emergency stop 3 times
    m_nEmergencyCounter = 3;
    QTimer::singleShot(0, this, SLOT(SendEmergencyStop()));

    RETURN_NOERROR;
}

tResult cJuryModule::OnRequestReady()
{    
    // request button pressed
    // setup jury struct and send 3 times
    m_i16ManeuverEntry = m_pWidget->m_comboManeuver->currentIndex();
    m_actionID = action_GETREADY;
    m_iJuryStructCounter = 3;
    QTimer::singleShot(0, this, SLOT(SendJuryStruct()));

    RETURN_NOERROR;
}

tResult cJuryModule::OnStop()
{
    // stop button pressed
    // setup jury struct and send 3 times
    m_i16ManeuverEntry = m_pWidget->m_comboManeuver->currentIndex();
    m_actionID = action_STOP;
    m_iJuryStructCounter = 3;
    QTimer::singleShot(0, this, SLOT(SendJuryStruct()));
    
    RETURN_NOERROR;
}

tResult cJuryModule::OnStart()
{
    // start button pressed
    // setup jury struct and send 3 times
    m_i16ManeuverEntry = m_pWidget->m_comboManeuver->currentIndex();
    m_actionID = action_START;
    m_iJuryStructCounter = 3;
    QTimer::singleShot(0, this, SLOT(SendJuryStruct()));
    
    RETURN_NOERROR;
}

tResult cJuryModule::OnLocalhostCheckChanged(int nState)
{
    // localhost checkbox state changed
    bool bEnable = false;
    if (nState == 0)
    {
        bEnable = true;
    }

    // enable/disable ip settings
    m_pWidget->m_spinBoxIP1->setEnabled(bEnable);
    m_pWidget->m_spinBoxIP2->setEnabled(bEnable);
    m_pWidget->m_spinBoxIP3->setEnabled(bEnable);
    m_pWidget->m_spinBoxIP4->setEnabled(bEnable);

    RETURN_NOERROR;
}


tResult cJuryModule::LoadManeuverList()
{
    // load the maneuver list
    // clear old list
    m_strManeuverList.clear();
    // use QDom for parsing
    QDomDocument oDoc("maneuver_list");
    QFile oFile(m_strManeuverFile);

    // open file
    if(!oFile.open(QIODevice::ReadOnly))
    {
        RETURN_ERROR(ERR_FILE_NOT_FOUND);
    }
    if (!oDoc.setContent(&oFile))
    {
        oFile.close();
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    // get the root element
    QDomElement oRoot = oDoc.documentElement();

    // get all sectors from DOM
    QDomNodeList lstSectors = oRoot.elementsByTagName("AADC-Sector");

    // iterate over all sectors
    for (int nIdxSectors = 0; nIdxSectors < lstSectors.size(); ++nIdxSectors)
    {
        // get the ids and maneuver from sector
        QDomNode oNodeSector = lstSectors.item(nIdxSectors);
        QDomElement oElementSector = oNodeSector.toElement();
        QString strIdSector = oElementSector.attribute("id");
        tSector sSector;
        sSector.id = strIdSector.toInt();
        QDomNodeList lstManeuver = oElementSector.elementsByTagName("AADC-Maneuver");
        // iterate over all maneuver
        for (int nIdxManeuver = 0; nIdxManeuver < lstManeuver.size(); ++nIdxManeuver)
        {
            // get the id and the action
            QDomNode oNodeManeuver = lstManeuver.item(nIdxManeuver);
            QDomElement oElementManeuver = oNodeManeuver.toElement();
            QString strIdManeuver = oElementManeuver.attribute("id");
            tAADC_Maneuver sManeuver;
            sManeuver.nId = strIdManeuver.toInt();
            sManeuver.action = oElementManeuver.attribute("action").toStdString();
            // fill the internal maneuver list
            sSector.lstManeuvers.push_back(sManeuver);
        }

        // fill the internal sector list
        m_lstSections.push_back(sSector);
    }
   
    // check sector list
    if (m_lstSections.size() > 0)
    {
        SetLogText("Jury Module: Loaded Maneuver file successfully.");
    }
    else
    {
        SetLogText("Jury Module: no valid Maneuver Data found!");
    }

    // fill the combo boxes
    FillComboBoxes();
    // get the xml string for transmission
    m_strManeuverList = oDoc.toString().toStdString();
    // set the controls (enable/disable)
    SetControlState();
    
    RETURN_NOERROR;
}


tResult cJuryModule::SendEmergencyStop()
{
    // the signal was transmitted as long as emergency counter is set
    if(m_nEmergencyCounter > 0)
    {
        tJuryEmergencyStop sEmergencyStop;
        sEmergencyStop.bEmergencyStop = true;
        m_pDataSystemDataSender->UpdateData(CONLIB_OUT_PORT_NAME_EMERGENCY_STOP, &sEmergencyStop, sizeof(sEmergencyStop));
        --m_nEmergencyCounter;   
        QTimer::singleShot(1000, this, SLOT(SendEmergencyStop()));
    }

    RETURN_NOERROR;
}


tResult cJuryModule::SendJuryStruct()
{
    // the signal was transmitted as long as jury struct counter is set
    if (m_iJuryStructCounter > 0)
    {
        tJuryStruct sJury;
        sJury.i8ActionID = m_actionID;
        sJury.i16ManeuverEntry = m_i16ManeuverEntry;
        m_pDataSystemDataSender->UpdateData(CONLIB_OUT_PORT_NAME_JURY_STRUCT, &sJury, sizeof(sJury));
        --m_iJuryStructCounter;
        QTimer::singleShot(1000, this, SLOT(SendJuryStruct()));
    }
    

    RETURN_NOERROR;
}

tResult cJuryModule::SendManeuverList()
{
    // only if maneuver file is set it will be transmitted
    if (!m_strManeuverList.empty())
    {
        // hack to transmit data with dynamic size
        tInt32 i32StringSize = (tInt32) (m_strManeuverList.size() + sizeof(tChar)); // plus one more char for terminating /0
        tSize szBufferSize = i32StringSize * sizeof(char) + sizeof(i32StringSize); // buffer size is string size plus also transmitted size-value
        
        // create a buffer
        tUInt8 *pui8Buffer = new tUInt8[szBufferSize];
        tInt32* pi32SizeInBuffer = (tInt32*) pui8Buffer;
        // set the string size (needed for serialization/deserialization)
        *pi32SizeInBuffer = i32StringSize;
        // copy the maneuverlist string into the buffer
        memcpy(pui8Buffer + sizeof(i32StringSize), m_strManeuverList.c_str(), i32StringSize * sizeof(char));
        
        // transmit the data buffer
        m_pDataSystemDataSender->UpdateData(CONLIB_OUT_PORT_NAME_MANEUVER_LIST, pui8Buffer, szBufferSize);

        // cleanup
        delete [] pui8Buffer;
        
        //QTimer::singleShot(1000, this, SLOT(SendManeuverList()));
    }


    RETURN_NOERROR;
}


void cJuryModule::OnDriverState(int state, int entryId)
{
    // set the driver state
    SetDriverManeuverID(entryId);    
    switch (stateCar(state))
    {
    case stateCar_ERROR:
        SetDriverStateError();
        break;
    case stateCar_READY:        
        SetDriverStateReady();
        break;
    case stateCar_RUNNING:
        SetDriverStateRun();
        break;
    case stateCar_COMPLETE:
        SetDriverStateComplete();
        break;
    case stateCar_STARTUP:
        break;
    }
}

void cJuryModule::OnAppendText(QString text)
{
    // append the given text to the log field
    if(m_pWidget)
    {
        m_pWidget->m_edtLogField->append(text);
    }
}

void cJuryModule::OnComboActionBoxChanged(int index)
{
    // search for the right section entry
    for(unsigned int nIdxSection = 0; nIdxSection < m_lstSections.size(); nIdxSection++)
    {
        for(unsigned int nIdxManeuver = 0; nIdxManeuver < m_lstSections[nIdxSection].lstManeuvers.size(); nIdxManeuver++)
        {
            if(index == m_lstSections[nIdxSection].lstManeuvers[nIdxManeuver].nId)
            {            
                m_pWidget->m_comboSector->setCurrentIndex(nIdxSection);
                break;
            }
        }
    }    
}

void cJuryModule::OnComboSectionBoxChanged(int index)
{
    //get the action id from selected section and write it to action combo box
    m_pWidget->m_comboManeuver->setCurrentIndex(m_lstSections[index].lstManeuvers[0].nId);
}

void cJuryModule::SetDriverStateError()
{
    // set the background and text
    m_pWidget->m_lblState->setStyleSheet("background-color: rgb(255,0,0);");
    m_pWidget->m_lblState->setText("Error");
}

void cJuryModule::SetDriverStateRun()
{
    // set the background and text
    m_pWidget->m_lblState->setStyleSheet("background-color: rgb(0,255,0);");
    m_pWidget->m_lblState->setText("Running");
}

void cJuryModule::SetDriverStateReady()
{
    // set the background and text
    m_pWidget->m_lblState->setStyleSheet("background-color: rgb(255,255,0);");
    m_pWidget->m_lblState->setText("Ready");
}

void cJuryModule::SetDriverStateComplete()
{
    // set the background and text
    m_pWidget->m_lblState->setStyleSheet("background-color: rgb(0,0,255);");
    m_pWidget->m_lblState->setText("Complete");
}

tString cJuryModule::GetIpAdress()
{
    // get the ip adress from the spinners or if localhost checkbox is checked use "localhost"
    if (m_pWidget->m_cbxLocalHost->isChecked())
    {
        m_strIPAdress = "localhost";
    }
    else
    {
        QString strIp1 = m_pWidget->m_spinBoxIP1->text();
        QString strIp2 = m_pWidget->m_spinBoxIP2->text();
        QString strIp3 = m_pWidget->m_spinBoxIP3->text();
        QString strIp4 = m_pWidget->m_spinBoxIP4->text();

        m_strIPAdress = strIp1.toStdString() + "." + strIp2.toStdString() + "." + strIp3.toStdString() + "." + strIp4.toStdString();
    }
    
    return m_strIPAdress;
}

void cJuryModule::OnConnectDisconnect()
{
    // connect / disconnect button pressed
    // check for already running connection thread
    if (!m_oConnectionThread.isRunning())
    {
        m_oConnectionThread.start();
    }
    
    // the timer will use the main thread which freezes the UI while connecting
    //QTimer::singleShot(0, this, SLOT(ConnectDisconnectNetworkClients()));
}

void cJuryModule::SetDriverManeuverID(tInt16 id)
{
    // set the id from received driver struct
    m_pWidget->m_lblDriverManeuverID->setText(QString::number(id));    
}


void cJuryModule::FillComboBoxes()
{
    int nLastManeuverId = 0;
    int nLastSectionId = 0;

    // vector of maneuver names to fill the combobox also with names
    std::vector<std::string> lstManeuverNames;

    // find the last maneuver and section id
    for(unsigned int i = 0; i < m_lstSections.size(); i++)
    {
        nLastSectionId++;
        for(unsigned int j = 0; j < m_lstSections[i].lstManeuvers.size(); j++)
        {
            nLastManeuverId++;
            lstManeuverNames.push_back(m_lstSections[i].lstManeuvers[j].action);
        }
    }

    // clear the old entries
    m_pWidget->m_comboManeuver->clear();
    m_pWidget->m_comboSector->clear();

    // fill the combo boxes
    for(int i = 0; i < nLastManeuverId; i++)
    {
        m_pWidget->m_comboManeuver->addItem(QString::number(i) + QString(" - ") + QString(lstManeuverNames[i].c_str()));
    }

    for(int i = 0; i < nLastSectionId; i++)
    {
        m_pWidget->m_comboSector->addItem(QString::number(i));
    }
}
